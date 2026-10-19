//===================================================================================================
// bench/link_bench/main/link_bench_main.c —— 链路 v2 的 P0 台架固件(遥控器端)
//
// 独立 ESP-IDF 工程, 不属于主工程(不进 main/CMakeLists.txt)。目的: 在**没有 USB 日志**的条件下
// 当"射频对端", 让飞控 bench 量 空速/F(n)/收↔发切换/抖动。
//
// 三条设计约束(照着协议规格 §11.2 来):
//   1. **日志经 E220 回传**: 本端 UART0/USB 可能没接(同时插 USB 和 E220 供电有风险), 所以所有
//      结果都用一行 ASCII 经射频发给飞控 bench 打印。格式 `<tag> key=value ...\n`,
//      以 '#' 开头; 命令以 '@' 开头(飞控 → 本端)。测量帧是二进制, 靠首字节 <0x80 且非 '@'/'#'
//      区分(见 docs/链路层v2_P0台架.md 的"流量区分"一节)。
//   2. **不改 E220 配置**: 只调 radio_e220_init()(它只读寄存器 + AT 查询), 不调
//      radio_e220_boot_ensure()/radio_e220_apply_profile() —— 那两个会写 flash。
//   3. **复用主工程驱动**: 通过 EXTRA_COMPONENT_DIRS 指回 ../../components, 用同一份
//      components/radio 与 components/rc_core/app_config.h(引脚/波特率/信道都是同一份定义)。
//
// 模式(由飞控 bench 用 `@MODE ...` 切):
//   SILENT   不发(只回命令行应答)                    @MODE SILENT
//   RESPOND  收到一帧后等 guard_ms 回发 len 字节       @MODE RESPOND len=8 gap=0
//   PATTERN  每 gap_ms 主动发 len 字节(测定拍抖动)     @MODE PATTERN len=8 gap=22
//   STAT     立刻回一行统计, 然后保持原模式            @STAT
//   PING     回 `#PONG`                               @PING
//
// 回传行:
//   #BOOT fw=2 role=rc_bench baud=.. rate=.. ch=.. aux=..
//   #ACK mode=.. len=.. gap=..
//   #STAT mode=.. tx=.. rx=.. txbusy=.. jmin=.. javg=.. jmax=.. jp99=.. auxbusy=.. uptime_ms=..
//   #PONG
//===================================================================================================
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "app_config.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "radio.h"
#include "radio_e220.h"

// ---- 统计 ---------------------------------------------------------------------------------------
#define JIT_CAP 256
typedef struct
{
    int32_t  jitter_us[JIT_CAP];
    uint16_t n;
    int32_t  jmin;
    int32_t  jmax;
    int64_t  jsum;
    uint32_t tx;
    uint32_t rx;
    uint32_t txbusy;
    uint32_t auxbusy_sum_us;
    uint32_t auxbusy_n;
} bench_stat_t;

static bench_stat_t s_st;

static void st_reset(void)
{
    memset(&s_st, 0, sizeof(s_st));
    s_st.jmin = INT32_MAX;
    s_st.jmax = INT32_MIN;
}

static void st_add_jitter(int32_t j)
{
    if (s_st.n < JIT_CAP)
    {
        s_st.jitter_us[s_st.n++] = j;
    }
    if (j < s_st.jmin) s_st.jmin = j;
    if (j > s_st.jmax) s_st.jmax = j;
    s_st.jsum += j;
}

/* qsort 比较器必须返回 int（ESP-IDF v6.1 用 -std=gnu23 + picolibc：
 * `__compar_fn_t` 是 `int (*)(const void *, const void *)`；xtensa 上 int32_t 是 long，
 * 返回 int32_t 会以 -Wincompatible-pointer-types（-Werror）直接编译失败）。 */
static int cmp_i32(const void *a, const void *b)
{
    const int32_t x = *(const int32_t *)a;
    const int32_t y = *(const int32_t *)b;
    return (x < y) ? -1 : (x > y) ? 1 : 0;
}

static int32_t st_p99(void)
{
    if (s_st.n == 0) return 0;
    int32_t tmp[JIT_CAP];
    memcpy(tmp, s_st.jitter_us, sizeof(int32_t) * s_st.n);
    qsort(tmp, s_st.n, sizeof(int32_t), cmp_i32);
    uint32_t idx = (uint32_t)(((uint64_t)s_st.n * 99u) / 100u);
    if (idx >= s_st.n) idx = s_st.n - 1u;
    return tmp[idx];
}

// ---- 链路状态 -----------------------------------------------------------------------------------
typedef enum { MODE_SILENT = 0, MODE_RESPOND, MODE_PATTERN } bench_mode_t;

static radio_e220_t s_dev;
static radio_t      *s_radio;
static bench_mode_t s_mode       = MODE_SILENT;
static uint16_t     s_len        = 8;
static uint16_t     s_gap_ms     = 22;
static uint16_t     s_guard_ms   = 0;
static uint32_t     s_pat_dur_ms = 0;   /* PATTERN 自动停止时长(0=一直发): 台架切模式靠它, 避免 75% 占空下命令收不到 */
static int64_t      s_pat_start_us = 0;
static char         s_line[96];
static uint16_t     s_line_len   = 0;
static bool         s_ready      = false;
static int64_t      s_next_tx_us = 0;

// ---- 发送 ---------------------------------------------------------------------------------------
/** 发一行 ASCII 给飞控 bench。line 必须已含 '\n'。 */
static void send_line(const char *text)
{
    if (s_radio == NULL) return;
    const size_t n = strlen(text);
    if (n == 0 || n > 200) return;
    (void)radio_send(s_radio, (const uint8_t *)text, (uint8_t)n);
    s_st.tx++;
}

/** 发 n 字节测量载荷(台阶字节, 与飞控 bench 的 sweep/tx 载荷一致: 0xA0+i)。 */
static void send_payload(uint16_t n, int64_t planned_us)
{
    uint8_t buf[64];
    if (n == 0 || n > sizeof(buf) || s_radio == NULL) return;
    for (uint16_t i = 0; i < n; ++i) buf[i] = (uint8_t)(0xA0u + (i & 0x1Fu));
    const int64_t now = esp_timer_get_time();
    const bool    ok  = radio_send(s_radio, buf, (uint8_t)n);
    if (ok)
    {
        s_st.tx++;
        if (planned_us > 0) st_add_jitter((int32_t)(now - planned_us));
    }
    else
    {
        s_st.txbusy++;
    }
}

// ---- 命令 ---------------------------------------------------------------------------------------
static int parse_kv(const char *line, const char *key, int fallback)
{
    const char *p = strstr(line, key);
    if (p == NULL) return fallback;
    p += strlen(key);
    int v = 0;
    bool any = false;
    while (*p >= '0' && *p <= '9')
    {
        v = v * 10 + (*p - '0');
        ++p;
        any = true;
    }
    return any ? v : fallback;
}

static void reply_stat(void)
{
    char line[220];
    const char *m = (s_mode == MODE_SILENT) ? "SILENT" : (s_mode == MODE_RESPOND) ? "RESPOND" : "PATTERN";
    const int32_t jmin = (s_st.jmin == INT32_MAX) ? 0 : s_st.jmin;
    const int32_t jmax = (s_st.jmax == INT32_MIN) ? 0 : s_st.jmax;
    const int32_t javg = (s_st.n == 0) ? 0 : (int32_t)(s_st.jsum / s_st.n);
    snprintf(line, sizeof(line),
             "#STAT mode=%s len=%u gap=%u guard=%u tx=%lu rx=%lu txbusy=%lu jmin=%ld javg=%ld jmax=%ld jp99=%ld "
             "auxbusy=%lu auxn=%lu uptime_ms=%lu\n",
             m, (unsigned)s_len, (unsigned)s_gap_ms, (unsigned)s_guard_ms, (unsigned long)s_st.tx,
             (unsigned long)s_st.rx, (unsigned long)s_st.txbusy, (long)jmin, (long)javg, (long)jmax,
             (long)st_p99(), (unsigned long)s_st.auxbusy_sum_us, (unsigned long)s_st.auxbusy_n,
             (unsigned long)(esp_timer_get_time() / 1000));
    send_line(line);
}

static void handle_command(char *line)
{
    // 允许 @MODE RESPOND len=8 gap=22 guard=0 / @MODE PATTERN len=8 gap=22 / @MODE SILENT / @STAT / @PING
    for (char *p = line; *p != '\0'; ++p)
    {
        if (*p >= 'a' && *p <= 'z') *p = (char)(*p - 'a' + 'A');
    }
    if (strncmp(line, "@MODE", 5) == 0)
    {
        const int len_v   = parse_kv(line, "LEN=", s_len);
        const int gap_v   = parse_kv(line, "GAP=", s_gap_ms);
        const int guard_v = parse_kv(line, "GUARD=", s_guard_ms);
        const int dur_v   = parse_kv(line, "DUR=", (int)s_pat_dur_ms);
        if (guard_v >= 0 && guard_v <= 200) s_guard_ms = (uint16_t)guard_v;
        if (gap_v >= 1 && gap_v <= 1000) s_gap_ms = (uint16_t)gap_v;
        if (len_v >= 1 && len_v <= 32) s_len = (uint16_t)len_v;
        if (dur_v >= 0 && dur_v <= 600000) s_pat_dur_ms = (uint32_t)dur_v;

        if (strstr(line, "RESPOND") != NULL)
        {
            s_mode = MODE_RESPOND;
        }
        else if (strstr(line, "PATTERN") != NULL)
        {
            s_mode        = MODE_PATTERN;
            s_pat_start_us = esp_timer_get_time();
            s_next_tx_us   = s_pat_start_us + (int64_t)s_gap_ms * 1000;
        }
        else if (strstr(line, "SILENT") != NULL)
        {
            s_mode = MODE_SILENT;
        }
        else
        {
            send_line("#ERR mode\n");
            return;
        }
        char ack[80];
        const char *m = (s_mode == MODE_SILENT) ? "SILENT" : (s_mode == MODE_RESPOND) ? "RESPOND" : "PATTERN";
        snprintf(ack, sizeof(ack), "#ACK mode=%s len=%u gap=%u guard=%u dur=%u\n", m, (unsigned)s_len,
                 (unsigned)s_gap_ms, (unsigned)s_guard_ms, (unsigned)s_pat_dur_ms);
        send_line(ack);
    }
    else if (strncmp(line, "@STAT", 5) == 0)
    {
        reply_stat();
    }
    else if (strncmp(line, "@PING", 5) == 0)
    {
        send_line("#PONG\n");
    }
    else if (strncmp(line, "@RESET", 6) == 0)
    {
        st_reset();
        send_line("#ACK reset\n");
    }
    else
    {
        send_line("#ERR cmd\n");
    }
}

// ---- 收包 ---------------------------------------------------------------------------------------
/**
 * 收包与"帧边界"判定。
 *
 * radio_poll_recv() 给的是**字节流**(E220 透传), 没有帧边界。台架里两种流量混在一条链路上:
 *   - ASCII 命令行: 以 '@' 开头、以 '\n' 结尾(飞控 bench 的 rc/send 命令) —— 靠 '\n' 定界;
 *   - 二进制测量帧: 任意字节, 没有 '\n' —— 靠**空闲间隔**(>15ms)定界。
 * 两者都不做 CRC, 台架只要"能对上节奏"就够; 真正的帧层留给 P1 的 v2 帧表。
 */
static void poll_rx(int64_t now_us)
{
    static int64_t last_byte_us = 0;
    uint8_t        buf[64];
    uint8_t        n = 0;

    while (radio_poll_recv(s_radio, buf, sizeof(buf), &n))
    {
        s_st.rx++;
        for (uint8_t i = 0; i < n; ++i)
        {
            const char c = (char)buf[i];
            if (c == '\r')
            {
                continue;
            }
            if (c == '\n')
            {
                s_line[s_line_len] = '\0';
                if (s_line_len > 0 && s_line[0] == '@')
                {
                    handle_command(s_line);
                }
                s_line_len = 0;
                continue;
            }
            if (s_line_len < sizeof(s_line) - 1u)
            {
                s_line[s_line_len++] = c;
            }
        }
        last_byte_us = now_us;
    }

    // 空闲 ≥15ms ⇒ 上一段突发结束(22ms 节拍下不会误判; 32B@57600 只占 6ms)
    if (s_line_len > 0 && last_byte_us != 0 && (now_us - last_byte_us) > 15000)
    {
        if (s_line[0] != '@' && s_mode == MODE_RESPOND)
        {
            const uint16_t len = (s_len > 0) ? s_len : s_line_len;
            const int64_t  planned = esp_timer_get_time() + (int64_t)s_guard_ms * 1000;
            while (esp_timer_get_time() < planned)
            {
                // 忙等: guard 只有几 ms, 让出 tick 会引入 ms 级抖动
            }
            send_payload(len, 0);
        }
        s_line_len = 0;
    }
}

// ---- app_main -----------------------------------------------------------------------------------
void app_main(void)
{
    // 无 USB 日志: 整体静音(radio 组件内部的 ESP_LOGI 也不出来)
    esp_log_level_set("*", ESP_LOG_NONE);
    st_reset();

    const uint8_t addr[2] = {E220_ADDR_H, E220_ADDR_L};
    const esp_err_t err   = radio_e220_init(&s_dev, addr, E220_CHANNEL);
    s_radio               = radio_e220_get(&s_dev);
    s_ready               = (err == ESP_OK) && s_radio->caps != NULL;

    if (s_ready)
    {
        // 只读诊断: 寄存器快照(不写!)
        uint8_t regs[8] = {0};
        (void)radio_e220_dump_regs(regs, sizeof(regs));
        char line[200];
        snprintf(line, sizeof(line),
                 "#BOOT fw=2 role=rc_bench baud=%lu rate=%u ch=%u aux=%u reg=%02X%02X%02X%02X\n",
                 (unsigned long)s_dev.baud, (unsigned)s_dev.rate_code, (unsigned)s_dev.channel,
                 (unsigned)(radio_e220_aux_is_idle(&s_dev) ? 1u : 0u), regs[2], regs[3], regs[4], regs[5]);
        send_line(line);
    }

    int64_t last_stat_us = esp_timer_get_time();

    while (true)
    {
        const int64_t now = esp_timer_get_time();
        poll_rx(now);

        if (s_ready && s_mode == MODE_PATTERN)
        {
            if (s_pat_dur_ms > 0u && now - s_pat_start_us >= (int64_t)s_pat_dur_ms * 1000)
            {
                // 自动停: PATTERN 占空 ~75%, 用命令切回 SILENT 大概率撞包收不到, 所以靠时长自停。
                s_mode = MODE_SILENT;
                send_line("#ACK mode=SILENT auto\n");
            }
            else if (now >= s_next_tx_us)
            {
                send_payload(s_len, s_next_tx_us);
                s_next_tx_us += (int64_t)s_gap_ms * 1000;
                if (now - s_next_tx_us > (int64_t)s_gap_ms * 2000)
                {
                    s_next_tx_us = now + (int64_t)s_gap_ms * 1000; // 落后太多: 重新对齐
                }
            }
            else
            {
                const int64_t remain = s_next_tx_us - now;
                if (remain > 2000)
                {
                    esp_rom_delay_us(1000);
                }
                else if (remain > 0)
                {
                    esp_rom_delay_us(50);
                }
            }
        }
        else
        {
            esp_rom_delay_us(200); // 空闲: 200us 轮询一次(收包延迟 ≤ 200us)
            // 每 5s 主动回一行心跳, 让飞控侧知道"对端活着、现在什么模式"(SILENT 下也能看见)
            if (now - last_stat_us > 5000000)
            {
                last_stat_us = now;
                if (s_ready) reply_stat();
            }
        }
    }
}
