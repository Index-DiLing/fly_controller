//===================================================================================================
// test/link_v2/sim/fc_probe.cpp —— 帧层对拍探针（飞控端实现）
//
// 由独立参考实现（ref_frame.py）驱动，只调用被测实现 DL_LIB/link/dlx_link_frame.hpp：
//   encode  : 读语料（线上整数域），逐例输出编码字节 hex
//   decode  : 读 "<name> <hex>"，逐例解码并输出 字段=线上整数
//   resync  : 读随机坏字节流（hex），按"接收端逐字节喂入 + tryDecode"模型输出找回的帧
//
// 构建: g++ -std=c++11 -O2 -I test/stub -I DL_LIB -I . test/link_v2/sim/fc_probe.cpp -o fc_probe
// 本文件只做命令行/字段映射，不含任何协议常量（协议在实现与参考实现里各有一份）。
//===================================================================================================
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <string>
#include <vector>
#include <map>
#include <fstream>
#include <sstream>

#include "link/dlx_link_frame.hpp"

using namespace dlx::link;

static std::map<std::string, long> g_kv;

static void parse_case(const std::string &line, int &type, int &seq)
{
    g_kv.clear();
    std::istringstream iss(line);
    std::string tok;
    iss >> tok; type = (int)std::strtol(tok.c_str(), nullptr, 16);
    iss >> tok; seq = (int)std::strtol(tok.c_str(), nullptr, 16);
    while (iss >> tok) {
        auto eq = tok.find('=');
        if (eq == std::string::npos) continue;
        g_kv[tok.substr(0, eq)] = std::strtol(tok.c_str() + eq + 1, nullptr, 10);
    }
}

static long kv(const char *name) { auto it = g_kv.find(name); return it == g_kv.end() ? 0 : it->second; }
static float kf(const char *name, float scale) { return (float)kv(name) / scale; }

static void print_hex(const uint8_t *p, int n)
{
    for (int i = 0; i < n; ++i) std::printf("%s%02X", i ? " " : "", p[i]);
    std::printf("\n");
}

static int do_encode(const char *path)
{
    std::ifstream in(path);
    if (!in) { std::fprintf(stderr, "cannot open %s\n", path); return 2; }
    uint8_t frame[kMaxFrameBytes + 4];
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        int type = 0, seq = 0;
        parse_case(line, type, seq);
        uint16_t n = 0;
        switch ((FrameType)type) {
        case FrameType::DlAtt: {
            DlAttPayload p{};
            p.q1 = kf("q1", 2047.0f); p.q2 = kf("q2", 2047.0f); p.q3 = kf("q3", 2047.0f);
            p.vx = kf("vx", 10.0f); p.vy = kf("vy", 10.0f); p.vz = kf("vz", 10.0f);
            p.status = (uint8_t)kv("status"); p.error = (uint8_t)kv("error");
            p.arming = (uint8_t)kv("arming"); p.mode = (uint8_t)kv("mode");
            n = encodeDlAtt(p, (uint8_t)seq, frame);
            break;
        }
        case FrameType::DlSys: {
            DlSysPayload p{};
            p.x = kf("x", 10.0f); p.y = kf("y", 10.0f); p.z = kf("z", 10.0f);
            p.currentA = kf("current", 10.0f); p.voltageV = kf("voltage", 10.0f);
            for (int i = 0; i < 4; ++i) p.motor[i] = (uint16_t)kv(("motor" + std::to_string(i)).c_str());
            n = encodeDlSys(p, (uint8_t)seq, frame);
            break;
        }
        case FrameType::DlGps: {
            DlGpsPayload p{};
            p.latE7 = (int32_t)kv("lat"); p.lonE7 = (int32_t)kv("lon");
            p.altMslMm = (int32_t)kv("alt");
            p.gspeedCms = (int16_t)kv("gspeed"); p.courseCdeg = (uint16_t)kv("course");
            p.sats = (uint8_t)kv("sats"); p.fix = (uint8_t)kv("fix");
            p.hdop = kf("hdop", 10.0f);
            n = encodeDlGps(p, (uint8_t)seq, frame);
            break;
        }
        case FrameType::DlState: {
            DlStatePayload p{};
            p.lifecycle = (uint8_t)kv("lifecycle"); p.arming = (uint8_t)kv("arming");
            p.mode = (uint8_t)kv("mode"); p.task = (uint8_t)kv("task");
            p.failsafeLvl = (uint8_t)kv("failsafe_lvl"); p.failsafeRsn = (uint8_t)kv("failsafe_rsn");
            p.health = (uint8_t)kv("health"); p.ulLoss = (uint8_t)kv("ul_loss");
            p.dlLoss = (uint8_t)kv("dl_loss"); p.rssiDbm = (int8_t)kv("rssi");
            p.rxBitmap = (uint8_t)kv("posmask");
            n = encodeDlState(p, (uint8_t)seq, frame);
            break;
        }
        case FrameType::DlAck: {
            DlAckPayload p{};
            p.cmdId = (uint8_t)kv("cmd_id"); p.result = (uint8_t)kv("result");
            n = encodeDlAck(p, (uint8_t)seq, frame);
            break;
        }
        case FrameType::UpRound: {
            UpRoundPayload p{};
            p.roll = (int16_t)kv("roll"); p.pitch = (int16_t)kv("pitch"); p.yaw = (int16_t)kv("yaw");
            p.thr = (uint16_t)kv("thr"); p.flags = (uint8_t)kv("flags");
            p.session = (uint8_t)kv("session"); p.flags8 = (uint8_t)kv("flags8");
            n = encodeUpRound(p, (uint8_t)seq, frame);
            break;
        }
        case FrameType::Hello: {
            HelloPayload p{};
            p.ver = (uint8_t)kv("ver"); p.caps = (uint8_t)kv("caps"); p.nonce = (uint8_t)kv("nonce");
            n = encodeHello(p, (uint8_t)seq, frame);
            break;
        }
        case FrameType::UpCtrl: {
            UpCtrlPayload p{};
            p.roll = (int16_t)kv("roll"); p.pitch = (int16_t)kv("pitch"); p.yaw = (int16_t)kv("yaw");
            p.thr = (uint16_t)kv("thr"); p.flags = (uint8_t)kv("flags");
            n = encodeUpCtrl(p, (uint8_t)seq, frame);
            break;
        }
        case FrameType::UpCmd: {
            UpCmdPayload p{};
            p.cmdId = (uint8_t)kv("cmd_id"); p.op = (uint8_t)kv("op"); p.arg = (uint8_t)kv("arg");
            n = encodeUpCmd(p, (uint8_t)seq, frame);
            break;
        }
        case FrameType::UpVel: {
            UpVelPayload p{};
            p.vx = kf("vx", 20.0f); p.vy = kf("vy", 20.0f); p.vz = kf("vz", 20.0f);
            n = encodeUpVel(p, (uint8_t)seq, frame);
            break;
        }
        case FrameType::UpPos: {
            UpPosPayload p{};
            p.x = kf("x", 10.0f); p.y = kf("y", 10.0f);
            n = encodeUpPos(p, (uint8_t)seq, frame);
            break;
        }
        case FrameType::UpSetpoint: {
            UpSetpointPayload p{};
            p.altM = kf("alt_cm", 100.0f); p.flags = (uint8_t)kv("flags");
            n = encodeUpSetpoint(p, (uint8_t)seq, frame);
            break;
        }
        case FrameType::UpMotor: {
            UpMotorPayload p{};
            for (int i = 0; i < 4; ++i) p.throttle[i] = (uint16_t)kv(("motor" + std::to_string(i)).c_str());
            n = encodeUpMotor(p, (uint8_t)seq, frame);
            break;
        }
        default:
            std::printf("ERR\n");
            continue;
        }
        print_hex(frame, n);
    }
    return 0;
}

static void print_fields(int type) { (void)type; }

static int g_first = 1;
static void emit_kv(const char *k, long v) { std::printf("%s%s=%ld", g_first ? "" : " ", k, v); g_first = 0; }

static int do_decode(const char *path)
{
    std::ifstream in(path);
    if (!in) { std::fprintf(stderr, "cannot open %s\n", path); return 2; }
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream iss(line);
        std::string name; iss >> name;
        std::vector<uint8_t> fr;
        std::string h;
        while (iss >> h) fr.push_back((uint8_t)std::strtol(h.c_str(), nullptr, 16));
        if (fr.size() < 5) { std::printf("ERR\n"); continue; }
        int type = fr[0] >> 4;
        g_first = 1;
        bool ok = false;
        switch ((FrameType)type) {
        case FrameType::DlAtt: {
            DlAttPayload p{};
            if ((ok = decodeDlAtt(fr.data(), p))) {
                emit_kv("q1", std::lround(p.q1 * 2047.0f)); emit_kv("q2", std::lround(p.q2 * 2047.0f));
                emit_kv("q3", std::lround(p.q3 * 2047.0f));
                emit_kv("vx", std::lround(p.vx * 10.0f)); emit_kv("vy", std::lround(p.vy * 10.0f));
                emit_kv("vz", std::lround(p.vz * 10.0f));
                emit_kv("status", p.status); emit_kv("error", p.error);
                emit_kv("arming", p.arming); emit_kv("mode", p.mode);
            }
            break;
        }
        case FrameType::DlSys: {
            DlSysPayload p{};
            if ((ok = decodeDlSys(fr.data(), p))) {
                emit_kv("x", std::lround(p.x * 10.0f)); emit_kv("y", std::lround(p.y * 10.0f));
                emit_kv("z", std::lround(p.z * 10.0f));
                emit_kv("current", std::lround(p.currentA * 10.0f));
                emit_kv("voltage", std::lround(p.voltageV * 10.0f));
                for (int i = 0; i < 4; ++i) emit_kv(("motor" + std::to_string(i)).c_str(), p.motor[i]);
            }
            break;
        }
        case FrameType::DlGps: {
            DlGpsPayload p{};
            if ((ok = decodeDlGps(fr.data(), p))) {
                emit_kv("lat", p.latE7); emit_kv("lon", p.lonE7); emit_kv("alt", p.altMslMm);
                emit_kv("gspeed", p.gspeedCms); emit_kv("course", p.courseCdeg);
                emit_kv("sats", p.sats); emit_kv("fix", p.fix);
                emit_kv("hdop", std::lround(p.hdop * 10.0f));
            }
            break;
        }
        case FrameType::DlState: {
            DlStatePayload p{};
            if ((ok = decodeDlState(fr.data(), p))) {
                emit_kv("lifecycle", p.lifecycle); emit_kv("arming", p.arming);
                emit_kv("mode", p.mode); emit_kv("task", p.task);
                emit_kv("failsafe_lvl", p.failsafeLvl); emit_kv("failsafe_rsn", p.failsafeRsn);
                emit_kv("health", p.health); emit_kv("ul_loss", p.ulLoss);
                emit_kv("dl_loss", p.dlLoss); emit_kv("rssi", p.rssiDbm);
                emit_kv("posmask", p.rxBitmap);
            }
            break;
        }
        case FrameType::DlAck: {
            DlAckPayload p{};
            if ((ok = decodeDlAck(fr.data(), p))) { emit_kv("cmd_id", p.cmdId); emit_kv("result", p.result); }
            break;
        }
        case FrameType::UpRound: {
            UpRoundPayload p{};
            if ((ok = decodeUpRound(fr.data(), p))) {
                emit_kv("roll", p.roll); emit_kv("pitch", p.pitch); emit_kv("yaw", p.yaw);
                emit_kv("thr", p.thr); emit_kv("flags", p.flags);
                emit_kv("session", p.session); emit_kv("flags8", p.flags8);
            }
            break;
        }
        case FrameType::Hello: {
            HelloPayload p{};
            if ((ok = decodeHello(fr.data(), p))) {
                emit_kv("ver", p.ver); emit_kv("caps", p.caps); emit_kv("nonce", p.nonce);
            }
            break;
        }
        case FrameType::UpCtrl: {
            UpCtrlPayload p{};
            if ((ok = decodeUpCtrl(fr.data(), p))) {
                emit_kv("roll", p.roll); emit_kv("pitch", p.pitch); emit_kv("yaw", p.yaw);
                emit_kv("thr", p.thr); emit_kv("flags", p.flags);
            }
            break;
        }
        case FrameType::UpCmd: {
            UpCmdPayload p{};
            if ((ok = decodeUpCmd(fr.data(), p))) {
                emit_kv("cmd_id", p.cmdId); emit_kv("op", p.op); emit_kv("arg", p.arg);
            }
            break;
        }
        case FrameType::UpVel: {
            UpVelPayload p{};
            if ((ok = decodeUpVel(fr.data(), p))) {
                emit_kv("vx", std::lround(p.vx * 20.0f)); emit_kv("vy", std::lround(p.vy * 20.0f));
                emit_kv("vz", std::lround(p.vz * 20.0f));
            }
            break;
        }
        case FrameType::UpPos: {
            UpPosPayload p{};
            if ((ok = decodeUpPos(fr.data(), p))) {
                emit_kv("x", std::lround(p.x * 10.0f)); emit_kv("y", std::lround(p.y * 10.0f));
            }
            break;
        }
        case FrameType::UpSetpoint: {
            UpSetpointPayload p{};
            if ((ok = decodeUpSetpoint(fr.data(), p))) {
                emit_kv("alt_cm", std::lround(p.altM * 100.0f)); emit_kv("flags", p.flags);
            }
            break;
        }
        case FrameType::UpMotor: {
            UpMotorPayload p{};
            if ((ok = decodeUpMotor(fr.data(), p))) {
                for (int i = 0; i < 4; ++i) emit_kv(("motor" + std::to_string(i)).c_str(), p.throttle[i]);
            }
            break;
        }
        default: break;
        }
        std::printf("%s", ok ? "" : "ERR");
        std::printf("\n");
    }
    return 0;
}

// 接收端逐字节喂入模型：buffer 里 while(true) tryDecode；NeedMore 则等下一字节；Reject 前进 1
static int do_resync(const char *path)
{
    std::ifstream in(path);
    if (!in) { std::fprintf(stderr, "cannot open %s\n", path); return 2; }
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::vector<uint8_t> stream;
        for (size_t i = 0; i + 1 < line.size(); i += 2)
            stream.push_back((uint8_t)std::strtol(line.substr(i, 2).c_str(), nullptr, 16));
        std::vector<uint8_t> buf;
        std::vector<std::pair<int, std::pair<int, int> > > found; // (offset, (type, seq))
        size_t base = 0;
        for (size_t k = 0; k < stream.size(); ++k) {
            buf.push_back(stream[k]);
            for (;;) {
                if (buf.empty()) break;
                Frame f{};
                uint16_t consumed = 0;
                int r = tryDecode(buf.data(), (uint16_t)buf.size(), f, consumed);
                if (r == kTryDecoded) {
                    found.push_back(std::make_pair((int)(base + 0),
                                                   std::make_pair((int)(uint8_t)f.type, (int)f.seq)));
                    buf.erase(buf.begin(), buf.begin() + consumed);
                    base += consumed;
                } else if (r == kTryReject) {
                    buf.erase(buf.begin(), buf.begin() + consumed);
                    base += consumed;
                } else {
                    break; // NeedMore
                }
            }
        }
        std::printf("%d", (int)found.size());
        for (size_t i = 0; i < found.size(); ++i)
            std::printf(" %d:%d:%d", found[i].first, found[i].second.first, found[i].second.second);
        std::printf("\n");
    }
    (void)print_fields;
    return 0;
}

int main(int argc, char **argv)
{
    if (argc < 3) {
        std::fprintf(stderr, "usage: %s encode|decode|resync FILE\n", argv[0]);
        return 2;
    }
    std::string mode = argv[1];
    if (mode == "encode") return do_encode(argv[2]);
    if (mode == "decode") return do_decode(argv[2]);
    if (mode == "resync") return do_resync(argv[2]);
    std::fprintf(stderr, "unknown mode %s\n", mode.c_str());
    return 2;
}
