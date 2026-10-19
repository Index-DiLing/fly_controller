/*===================================================================================================
 * tools/link_v2/../../test/link_v2/sim/rc_probe.c —— 帧层对拍探针（遥控器端实现）
 *
 * 由独立参考实现（ref_frame.py）驱动，只调用被测实现 components/fc_link/fc_frame.c：
 *   encode  : 读语料（线上整数域），逐例输出编码字节 hex
 *   decode  : 读 "<name> <hex>"，逐例解码并输出 字段=线上整数
 *   resync  : 读随机坏字节流（hex），按"接收端逐字节喂入 + fc_frame_next"模型输出找回的帧
 *
 * 构建（在遥控器仓库根目录）:
 *   gcc -std=c11 -O2 -I components/fc_link/include tools/test_fc_frame.c components/fc_link/fc_frame.c
 * 本探针放在飞控仓库 test/link_v2/sim/ 下，用 -I 指向遥控器仓库的 components/fc_link/include 编译。
 * 本文件只做命令行/字段映射，不含协议常量。
 *==================================================================================================*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "fc_frame.h"

#define MAX_LINE 8192

static long g_kv_val[32];
static char g_kv_key[32][24];
static int  g_kv_n;
static int  g_first;

static void kv_reset(void) { g_kv_n = 0; }
static void kv_add(const char *k, long v)
{
    if (g_kv_n >= 32) return;
    strncpy(g_kv_key[g_kv_n], k, sizeof(g_kv_key[0]) - 1);
    g_kv_key[g_kv_n][sizeof(g_kv_key[0]) - 1] = 0;
    g_kv_val[g_kv_n++] = v;
}
static long kv(const char *k)
{
    int i;
    for (i = 0; i < g_kv_n; ++i) if (strcmp(g_kv_key[i], k) == 0) return g_kv_val[i];
    return 0;
}
static void emit_kv(const char *k, long v)
{
    printf("%s%s=%ld", g_first ? "" : " ", k, v);
    g_first = 0;
}

static int parse_case(char *line, int *type, int *seq)
{
    char *tok, *save = NULL;
    kv_reset();
    tok = strtok_r(line, " \t", &save);
    if (!tok) return 0;
    *type = (int)strtol(tok, NULL, 16);
    tok = strtok_r(NULL, " \t", &save);
    if (!tok) return 0;
    *seq = (int)strtol(tok, NULL, 16);
    while ((tok = strtok_r(NULL, " \t", &save)) != NULL) {
        char *eq = strchr(tok, '=');
        if (!eq) continue;
        *eq = 0;
        kv_add(tok, strtol(eq + 1, NULL, 10));
    }
    return 1;
}

static void print_hex(const uint8_t *p, size_t n)
{
    size_t i;
    for (i = 0; i < n; ++i) printf("%s%02X", i ? " " : "", p[i]);
    printf("\n");
}

static int do_encode(const char *path)
{
    FILE *fp = fopen(path, "r");
    char line[MAX_LINE];
    uint8_t frame[FC_FRAME_MAX + 4];
    if (!fp) { fprintf(stderr, "cannot open %s\n", path); return 2; }
    while (fgets(line, sizeof(line), fp)) {
        int type, seq;
        size_t n = 0;
        if (line[0] == '#' || line[0] == '\n' || line[0] == 0) continue;
        if (!parse_case(line, &type, &seq)) continue;
        switch (type) {
        case FC_T_DL_ATT: {
            fc_dl_att_t p; memset(&p, 0, sizeof(p));
            p.q1 = (int16_t)kv("q1"); p.q2 = (int16_t)kv("q2"); p.q3 = (int16_t)kv("q3");
            p.vx = (int8_t)kv("vx"); p.vy = (int8_t)kv("vy"); p.vz = (int8_t)kv("vz");
            p.status = (uint8_t)kv("status"); p.error = (uint8_t)kv("error");
            p.arming = (uint8_t)kv("arming"); p.mode = (uint8_t)kv("mode");
            n = fc_encode_dl_att(frame, (uint8_t)seq, &p);
            break;
        }
        case FC_T_DL_SYS: {
            fc_dl_sys_t p; memset(&p, 0, sizeof(p));
            p.x = (int16_t)kv("x"); p.y = (int16_t)kv("y"); p.z = (int16_t)kv("z");
            p.current = (uint8_t)kv("current"); p.voltage = (uint8_t)kv("voltage");
            p.motor[0] = (uint16_t)kv("motor0"); p.motor[1] = (uint16_t)kv("motor1");
            p.motor[2] = (uint16_t)kv("motor2"); p.motor[3] = (uint16_t)kv("motor3");
            n = fc_encode_dl_sys(frame, (uint8_t)seq, &p);
            break;
        }
        case FC_T_DL_GPS: {
            fc_dl_gps_t p; memset(&p, 0, sizeof(p));
            p.lat = (int32_t)kv("lat"); p.lon = (int32_t)kv("lon"); p.alt = (int32_t)kv("alt");
            p.gspeed = (int16_t)kv("gspeed"); p.course = (uint16_t)kv("course");
            p.sats = (uint8_t)kv("sats"); p.fix = (uint8_t)kv("fix"); p.hdop = (uint8_t)kv("hdop");
            n = fc_encode_dl_gps(frame, (uint8_t)seq, &p);
            break;
        }
        case FC_T_DL_STATE: {
            fc_dl_state_t p; memset(&p, 0, sizeof(p));
            p.lifecycle = (uint8_t)kv("lifecycle"); p.arming = (uint8_t)kv("arming");
            p.mode = (uint8_t)kv("mode"); p.task = (uint8_t)kv("task");
            p.failsafe_lvl = (uint8_t)kv("failsafe_lvl"); p.failsafe_rsn = (uint8_t)kv("failsafe_rsn");
            p.health = (uint8_t)kv("health"); p.ul_loss = (uint8_t)kv("ul_loss");
            p.dl_loss = (uint8_t)kv("dl_loss"); p.rssi = (int8_t)kv("rssi");
            p.ul_pos_mask = (uint8_t)kv("posmask");
            n = fc_encode_dl_state(frame, (uint8_t)seq, &p);
            break;
        }
        case FC_T_DL_ACK: {
            fc_dl_ack_t p; memset(&p, 0, sizeof(p));
            p.cmd_id = (uint8_t)kv("cmd_id"); p.result = (uint8_t)kv("result");
            n = fc_encode_dl_ack(frame, (uint8_t)seq, &p);
            break;
        }
        case FC_T_UP_ROUND: {
            fc_up_round_t p; memset(&p, 0, sizeof(p));
            p.roll = (int16_t)kv("roll"); p.pitch = (int16_t)kv("pitch"); p.yaw = (int16_t)kv("yaw");
            p.thr = (uint16_t)kv("thr"); p.flags = (uint8_t)kv("flags");
            p.session = (uint8_t)kv("session"); p.flags8 = (uint8_t)kv("flags8");
            n = fc_encode_up_round(frame, (uint8_t)seq, &p);
            break;
        }
        case FC_T_HELLO: {
            fc_hello_t p; memset(&p, 0, sizeof(p));
            p.ver = (uint8_t)kv("ver"); p.caps = (uint8_t)kv("caps"); p.nonce = (uint8_t)kv("nonce");
            n = fc_encode_hello(frame, (uint8_t)seq, &p);
            break;
        }
        case FC_T_UP_CTRL: {
            fc_up_ctrl_t p; memset(&p, 0, sizeof(p));
            p.roll = (int16_t)kv("roll"); p.pitch = (int16_t)kv("pitch"); p.yaw = (int16_t)kv("yaw");
            p.thr = (uint16_t)kv("thr"); p.flags = (uint8_t)kv("flags");
            n = fc_encode_up_ctrl(frame, (uint8_t)seq, &p);
            break;
        }
        case FC_T_UP_CMD: {
            fc_up_cmd_t p; memset(&p, 0, sizeof(p));
            p.cmd_id = (uint8_t)kv("cmd_id"); p.op = (uint8_t)kv("op"); p.arg = (uint8_t)kv("arg");
            n = fc_encode_up_cmd(frame, (uint8_t)seq, &p);
            break;
        }
        case FC_T_UP_VEL: {
            fc_up_vel_t p; memset(&p, 0, sizeof(p));
            p.vx = (int8_t)kv("vx"); p.vy = (int8_t)kv("vy"); p.vz = (int8_t)kv("vz");
            n = fc_encode_up_vel(frame, (uint8_t)seq, &p);
            break;
        }
        case FC_T_UP_POS: {
            fc_up_pos_t p; memset(&p, 0, sizeof(p));
            p.x = (int16_t)kv("x"); p.y = (int16_t)kv("y");
            n = fc_encode_up_pos(frame, (uint8_t)seq, &p);
            break;
        }
        case FC_T_UP_SETPOINT: {
            fc_up_setpoint_t p; memset(&p, 0, sizeof(p));
            p.alt_cm = (int16_t)kv("alt_cm"); p.flags = (uint8_t)kv("flags");
            n = fc_encode_up_setpoint(frame, (uint8_t)seq, &p);
            break;
        }
        case FC_T_UP_MOTOR: {
            fc_up_motor_t p; memset(&p, 0, sizeof(p));
            p.throttle[0] = (uint16_t)kv("motor0"); p.throttle[1] = (uint16_t)kv("motor1");
            p.throttle[2] = (uint16_t)kv("motor2"); p.throttle[3] = (uint16_t)kv("motor3");
            n = fc_encode_up_motor(frame, (uint8_t)seq, &p);
            break;
        }
        default: printf("ERR\n"); continue;
        }
        print_hex(frame, n);
    }
    fclose(fp);
    return 0;
}

static int do_decode(const char *path)
{
    FILE *fp = fopen(path, "r");
    char line[MAX_LINE];
    if (!fp) { fprintf(stderr, "cannot open %s\n", path); return 2; }
    while (fgets(line, sizeof(line), fp)) {
        char *save = NULL, *tok, name[128];
        uint8_t fr[FC_FRAME_MAX + 4];
        size_t fn = 0, flen = 0;
        int type, ok = 0;
        if (line[0] == '#' || line[0] == '\n' || line[0] == 0) continue;
        tok = strtok_r(line, " \t\r\n", &save);
        if (!tok) continue;
        strncpy(name, tok, sizeof(name) - 1); name[sizeof(name) - 1] = 0;
        while ((tok = strtok_r(NULL, " \t\r\n", &save)) != NULL && fn < sizeof(fr))
            fr[fn++] = (uint8_t)strtol(tok, NULL, 16);
        (void)name;
        if (fn < 5) { printf("ERR\n"); continue; }
        flen = fn;
        type = fr[0] >> 4;
        g_first = 1;
        switch (type) {
        case FC_T_DL_ATT: {
            fc_dl_att_t p;
            if ((ok = fc_decode_dl_att(fr, flen, &p))) {
                emit_kv("q1", p.q1); emit_kv("q2", p.q2); emit_kv("q3", p.q3);
                emit_kv("vx", p.vx); emit_kv("vy", p.vy); emit_kv("vz", p.vz);
                emit_kv("status", p.status); emit_kv("error", p.error);
                emit_kv("arming", p.arming); emit_kv("mode", p.mode);
            }
            break;
        }
        case FC_T_DL_SYS: {
            fc_dl_sys_t p;
            if ((ok = fc_decode_dl_sys(fr, flen, &p))) {
                emit_kv("x", p.x); emit_kv("y", p.y); emit_kv("z", p.z);
                emit_kv("current", p.current); emit_kv("voltage", p.voltage);
                emit_kv("motor0", p.motor[0]); emit_kv("motor1", p.motor[1]);
                emit_kv("motor2", p.motor[2]); emit_kv("motor3", p.motor[3]);
            }
            break;
        }
        case FC_T_DL_GPS: {
            fc_dl_gps_t p;
            if ((ok = fc_decode_dl_gps(fr, flen, &p))) {
                emit_kv("lat", p.lat); emit_kv("lon", p.lon); emit_kv("alt", p.alt);
                emit_kv("gspeed", p.gspeed); emit_kv("course", p.course);
                emit_kv("sats", p.sats); emit_kv("fix", p.fix); emit_kv("hdop", p.hdop);
            }
            break;
        }
        case FC_T_DL_STATE: {
            fc_dl_state_t p;
            if ((ok = fc_decode_dl_state(fr, flen, &p))) {
                emit_kv("lifecycle", p.lifecycle); emit_kv("arming", p.arming);
                emit_kv("mode", p.mode); emit_kv("task", p.task);
                emit_kv("failsafe_lvl", p.failsafe_lvl); emit_kv("failsafe_rsn", p.failsafe_rsn);
                emit_kv("health", p.health); emit_kv("ul_loss", p.ul_loss);
                emit_kv("dl_loss", p.dl_loss); emit_kv("rssi", p.rssi);
                emit_kv("posmask", p.ul_pos_mask);
            }
            break;
        }
        case FC_T_DL_ACK: {
            fc_dl_ack_t p;
            if ((ok = fc_decode_dl_ack(fr, flen, &p))) { emit_kv("cmd_id", p.cmd_id); emit_kv("result", p.result); }
            break;
        }
        case FC_T_UP_ROUND: {
            fc_up_round_t p;
            if ((ok = fc_decode_up_round(fr, flen, &p))) {
                emit_kv("roll", p.roll); emit_kv("pitch", p.pitch); emit_kv("yaw", p.yaw);
                emit_kv("thr", p.thr); emit_kv("flags", p.flags);
                emit_kv("session", p.session); emit_kv("flags8", p.flags8);
            }
            break;
        }
        case FC_T_HELLO: {
            fc_hello_t p;
            if ((ok = fc_decode_hello(fr, flen, &p))) {
                emit_kv("ver", p.ver); emit_kv("caps", p.caps); emit_kv("nonce", p.nonce);
            }
            break;
        }
        case FC_T_UP_CTRL: {
            fc_up_ctrl_t p;
            if ((ok = fc_decode_up_ctrl(fr, flen, &p))) {
                emit_kv("roll", p.roll); emit_kv("pitch", p.pitch); emit_kv("yaw", p.yaw);
                emit_kv("thr", p.thr); emit_kv("flags", p.flags);
            }
            break;
        }
        case FC_T_UP_CMD: {
            fc_up_cmd_t p;
            if ((ok = fc_decode_up_cmd(fr, flen, &p))) {
                emit_kv("cmd_id", p.cmd_id); emit_kv("op", p.op); emit_kv("arg", p.arg);
            }
            break;
        }
        case FC_T_UP_VEL: {
            fc_up_vel_t p;
            if ((ok = fc_decode_up_vel(fr, flen, &p))) {
                emit_kv("vx", p.vx); emit_kv("vy", p.vy); emit_kv("vz", p.vz);
            }
            break;
        }
        case FC_T_UP_POS: {
            fc_up_pos_t p;
            if ((ok = fc_decode_up_pos(fr, flen, &p))) { emit_kv("x", p.x); emit_kv("y", p.y); }
            break;
        }
        case FC_T_UP_SETPOINT: {
            fc_up_setpoint_t p;
            if ((ok = fc_decode_up_setpoint(fr, flen, &p))) {
                emit_kv("alt_cm", p.alt_cm); emit_kv("flags", p.flags);
            }
            break;
        }
        case FC_T_UP_MOTOR: {
            fc_up_motor_t p;
            if ((ok = fc_decode_up_motor(fr, flen, &p))) {
                emit_kv("motor0", p.throttle[0]); emit_kv("motor1", p.throttle[1]);
                emit_kv("motor2", p.throttle[2]); emit_kv("motor3", p.throttle[3]);
            }
            break;
        }
        default: break;
        }
        printf("%s\n", ok ? "" : "ERR");
    }
    fclose(fp);
    return 0;
}

static size_t hex2bin(const char *hex, uint8_t *out, size_t cap)
{
    size_t n = 0;
    while (hex[0] && hex[1] && n < cap) {
        char b[3] = {hex[0], hex[1], 0};
        out[n++] = (uint8_t)strtol(b, NULL, 16);
        hex += 2;
    }
    return n;
}

static int do_resync(const char *path)
{
    FILE *fp = fopen(path, "r");
    char line[MAX_LINE];
    if (!fp) { fprintf(stderr, "cannot open %s\n", path); return 2; }
    while (fgets(line, sizeof(line), fp)) {
        static uint8_t stream[4096];
        uint8_t buf[FC_FRAME_MAX * 2];
        size_t slen, buf_len = 0, base = 0, k;
        size_t offs[512], types[512], seqs[512];
        size_t nfound = 0;
        char *nl = strchr(line, '\n');
        if (nl) *nl = 0;
        nl = strchr(line, '\r'); if (nl) *nl = 0;
        if (line[0] == '#' || line[0] == 0) continue;
        slen = hex2bin(line, stream, sizeof(stream));
        for (k = 0; k < slen; ++k) {
            buf_len++;
            buf[buf_len - 1] = stream[k];
            for (;;) {
                fc_frame_t out;
                size_t consumed;
                if (buf_len == 0) break;
                consumed = fc_frame_next(buf, buf_len, &out);
                if (consumed == 0) break;
                if (out.crc_ok) {
                    if (nfound < 512) {
                        offs[nfound] = base;
                        types[nfound] = out.type;
                        seqs[nfound] = out.seq;
                        nfound++;
                    }
                }
                memmove(buf, buf + consumed, buf_len - consumed);
                buf_len -= consumed;
                base += consumed;
            }
        }
        printf("%zu", nfound);
        for (k = 0; k < nfound; ++k) printf(" %zu:%zu:%zu", offs[k], types[k], seqs[k]);
        printf("\n");
    }
    fclose(fp);
    return 0;
}

int main(int argc, char **argv)
{
    if (argc < 3) { fprintf(stderr, "usage: %s encode|decode|resync FILE\n", argv[0]); return 2; }
    if (strcmp(argv[1], "encode") == 0) return do_encode(argv[2]);
    if (strcmp(argv[1], "decode") == 0) return do_decode(argv[2]);
    if (strcmp(argv[1], "resync") == 0) return do_resync(argv[2]);
    fprintf(stderr, "unknown mode %s\n", argv[1]);
    return 2;
}
