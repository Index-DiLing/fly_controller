/*===================================================================================================
 * test/link_v2/sim/rc_host_checks.c —— 遥控器 RoundMaster 口径断言（验证者独立主机检查）
 *
 * 只调用被测实现 components/fc_link/fc_round.c（不修改它），针对 Lead 拍板的偏差项做回归：
 *   R1 RUN 且下行新鲜：命令可入队；连续 2 轮无合法下行 -> 脱离 RUN，命令一律拒绝
 *   R2 命令 600ms 超时按**日历时间**：入队后 ~600ms 放弃并产出 RES_TIMEOUT；
 *      待确认期间 U2 重发同一 cmd_id、seq = roundCount*4+pos
 *   R3 版本不一致 -> MISMATCH，命令被拒且不再发 0x8–0xD
 *
 * 构建（在飞控仓库根目录，RC 路径由 -I 指定）:
 *   gcc -std=c11 -O2 -I <RC>/components/fc_link/include \
 *       test/link_v2/sim/rc_host_checks.c <RC>/components/fc_link/fc_round.c \
 *       <RC>/components/fc_link/fc_frame.c -o rc_host_checks
 *==================================================================================================*/
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "fc_round.h"
#include "fc_frame.h"

static void check_impl(const char *name, bool ok, const char *detail);
#define CHK_2(n, o)        check_impl((n), (o), "")
#define CHK_3(n, o, d)     check_impl((n), (o), (d))
#define CHK_PICK(_1, _2, _3, NAME, ...) NAME
#define check(...)         CHK_PICK(__VA_ARGS__, CHK_3, CHK_2)(__VA_ARGS__)

static int g_fail;

static void check_impl(const char *name, bool ok, const char *detail)
{
    printf("%-58s %s\n", name, ok ? "PASS" : "FAIL");
    if (!ok) {
        if (detail && *detail) printf("    %s\n", detail);
        ++g_fail;
    }
}

/* 发送回调：记录每一帧（帧头 + 长度），按类型统计 */
typedef struct {
    int      n;
    uint8_t  type[256];
    uint8_t  seq[256];
    uint8_t  head[256];   /* 完整帧头字节: (TYPE<<4)|(SEQ&0xF) */
    uint8_t  byte1[256];
    uint8_t  len[256];
} txlog_t;

static bool tx_cb(void *ctx, const uint8_t *frame, uint8_t len)
{
    txlog_t *log = (txlog_t *)ctx;
    if (log->n < 256) {
        log->head[log->n]  = frame[0];
        log->type[log->n]  = (uint8_t)(frame[0] >> 4);
        log->seq[log->n]   = (uint8_t)(frame[0] & 0x0F);
        log->byte1[log->n] = (len > 1) ? frame[1] : 0;
        log->len[log->n]   = len;
    }
    ++log->n;
    return true;
}

static int count_type(const txlog_t *log, uint8_t type)
{
    int i, n = 0;
    for (i = 0; i < log->n && i < 256; ++i) if (log->type[i] == type) ++n;
    return n;
}

static bool only_types(const txlog_t *log, const uint8_t *types, int ntypes)
{
    int i, j;
    for (i = 0; i < log->n && i < 256; ++i) {
        bool ok = false;
        for (j = 0; j < ntypes; ++j) if (log->type[i] == types[j]) ok = true;
        if (!ok) return false;
    }
    return true;
}

int main(void)
{
    const int64_t RUN_ENTRY = 100000;
    fc_hello_t hello_match = { .ver = (uint8_t)LinkProtoVersion, .caps = 0u, .nonce = 7u };
    fc_hello_t hello_bad   = { .ver = (uint8_t)(LinkProtoVersion + 1u), .caps = 0u, .nonce = 8u };

    /* ---- R1: RUN 命令门控 / 连续 2 轮无下行脱离 RUN ---- */
    {
        fc_round_t r;
        txlog_t log;
        int64_t t0, i;
        memset(&log, 0, sizeof(log));
        fc_round_init(&r, tx_cb, &log);
        fc_round_start(&r, 0);
        fc_round_on_hello(&r, &hello_match, RUN_ENTRY);
        check("R1 版本匹配 -> RUN", r.state == FC_ROUND_RUN);
        t0 = r.t0_us;
        /* 本轮内给一个合法下行，随后入队命令 */
        fc_round_on_downlink(&r, FC_T_DL_ATT, t0 + 50000);
        check("R1 RUN+新鲜下行：命令可入队", fc_round_queue_cmd(&r, FC_OP_ARM, 0u, t0 + 60000));
        check("R1 同一时刻只保留一条", !fc_round_queue_cmd(&r, FC_OP_DISARM, 0u, t0 + 60000));
        /* 连续 2 轮无下行：第 1 轮判丢 -> 开始下一轮 -> 第 2 轮判丢 -> 脱离 RUN */
        fc_round_poll(&r, t0 + 117000);
        check("R1 第 1 轮 DL 丢失仍在 RUN", r.state == FC_ROUND_RUN);
        fc_round_poll(&r, t0 + 120500);
        fc_round_poll(&r, t0 + 237000);
        check("R1 第 2 轮 DL 丢失仍在 RUN（计数 1）", r.state == FC_ROUND_RUN);
        fc_round_poll(&r, t0 + 240500);
        fc_round_poll(&r, t0 + 357000);
        check("R1 连续 2 轮 DL 丢失 -> 脱离 RUN", r.state != FC_ROUND_RUN);
        check("R1 非 RUN 时命令被拒", !fc_round_queue_cmd(&r, FC_OP_ARM, 0u, t0 + 360000));
    }

    /* ---- R2: 命令 600ms 日历超时 + 同 cmd_id 重发 + seq 规则 ---- */
    {
        fc_round_t r;
        txlog_t log;
        int64_t t0, k, drop_t = -1;
        int first_cmd_id = -1;
        bool same_id = true;
        uint8_t first_u0_seq = 0;
        bool seq_ok = true;
        memset(&log, 0, sizeof(log));
        fc_round_init(&r, tx_cb, &log);
        fc_round_start(&r, 0);
        fc_round_on_hello(&r, &hello_match, RUN_ENTRY);
        t0 = r.t0_us;
        fc_round_on_downlink(&r, FC_T_DL_ATT, t0 + 50000);
        check("R2 命令入队", fc_round_queue_cmd(&r, FC_OP_SET_MODE, 1u, t0 + 60000));
        /* 每轮喂一个合法下行（防止 HANDSHAKE 抢先），按 1ms 步进跑 800ms */
        for (k = 0; k <= 800; ++k) {
            int64_t now = t0 + 60000 + k * 1000;
            if (((now - t0) % 120000) >= 50000 && ((now - t0) % 120000) < 51000) {
                fc_round_on_downlink(&r, FC_T_DL_ATT, now);
            }
            fc_round_poll(&r, now);
            if (!fc_round_cmd_pending(&r) && drop_t < 0) {
                drop_t = now;
            }
        }
        check("R2 600ms 内未放弃（>=580ms）", drop_t < 0 || (drop_t - (t0 + 60000)) >= 580000);
        check("R2 660ms 前放弃（按日历”）", drop_t > 0 && (drop_t - (t0 + 60000)) <= 660000);
        {
            uint8_t op = 0, res = 0;
            bool got = fc_round_take_cmd_result(&r, &op, &res);
            check("R2 结果 = RES_TIMEOUT", got && res == FC_RES_TIMEOUT);
            (void)op;
        }
        /* 帧日志：U2 发过 UP_CMD 且 cmd_id 一致；U0 的 seq 低 2 位=0，轮间相差 4(mod 16) */
        {
            int i, first_cmd_id = -1, prev_u0 = -1, n_u0 = 0;
            bool same_id = true, seq_ok = true;
            for (i = 0; i < log.n && i < 256; ++i) {
                if (log.type[i] == FC_T_UP_CMD) {
                    if (first_cmd_id < 0) first_cmd_id = log.byte1[i];
                    else if (log.byte1[i] != first_cmd_id) same_id = false;
                }
                if (log.type[i] == FC_T_UP_ROUND) {
                    ++n_u0;
                    if ((log.seq[i] & 3) != 0) seq_ok = false;
                    if (prev_u0 >= 0 && ((log.seq[i] - prev_u0) & 0x0F) != 4) seq_ok = false;
                    prev_u0 = log.seq[i] & 0x0F;
                }
                /* 每轮 U1/U2/U3 的 seq&3 必须等于位置 */
                if (log.type[i] == FC_T_UP_CTRL) { /* 位置由 head 低位无法区分 U1/U2/U3, 略 */ }
            }
            check("R2 UP_CMD 用同一 cmd_id 重发", count_type(&log, FC_T_UP_CMD) >= 1 && same_id);
            check("R2 U0 每轮 seq 相差 4(mod16)", n_u0 >= 3 && seq_ok);
        }
    }

    /* ---- R3: 版本不一致 -> MISMATCH，命令拒绝、不发控制帧 ---- */
    {
        fc_round_t r;
        txlog_t log;
        const uint8_t allowed[] = { FC_T_HELLO };
        memset(&log, 0, sizeof(log));
        fc_round_init(&r, tx_cb, &log);
        fc_round_start(&r, 0);
        fc_round_on_hello(&r, &hello_match, RUN_ENTRY);
        fc_round_on_hello(&r, &hello_bad, RUN_ENTRY + 150000);
        check("R3 版本不一致 -> MISMATCH", r.state == FC_ROUND_MISMATCH);
        check("R3 MISMATCH 命令被拒",
              !fc_round_queue_cmd(&r, FC_OP_ARM, 0u, RUN_ENTRY + 160000));
        /* 再跑 300ms，期间只允许发 HELLO */
        {
            int64_t k;
            for (k = 0; k < 300; ++k) fc_round_poll(&r, RUN_ENTRY + 160000 + k * 1000);
        }
        check("R3 MISMATCH 只发 HELLO（无 0x8–0xD）", only_types(&log, allowed, 1));
        check("R3 MISMATCH 不发 UP_ROUND", count_type(&log, FC_T_UP_ROUND) == 0);
    }

    if (g_fail == 0) { printf("\nRC RoundMaster 口径断言：全部 PASS\n"); return 0; }
    printf("\nRC RoundMaster 口径断言：%d 项 FAIL\n", g_fail);
    return 1;
}
