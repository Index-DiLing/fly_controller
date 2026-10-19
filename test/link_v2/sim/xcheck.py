#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""链路 v2 帧层两端独立对拍（验证者工具）

流程：
  1) 编译两端探针（只编译被测实现，不含本仓库算法）：
       飞控 g++ -std=c++11 -I test/stub -I DL_LIB -I .  test/link_v2/sim/fc_probe.cpp
       遥控器 gcc -std=c11 -I <RC>/components/fc_link/include  rc_probe.c <RC>/components/fc_link/fc_frame.c
  2) 独立参考实现 ref_frame.py 生成"规格向量 + 边界值 + 随机 N 组"语料与期望字节
  3) 三方逐字节对比：fc_probe / rc_probe / 参考 oracle  必须完全一致
  4) 解码回环：两端分别解码同一批（参考实现产生的）帧，字段值必须等于输入
  5) 坏字节流：两端流式重同步结果必须与参考 oracle 一致，且不越界

退出码：0 = 全绿；1 = 有差异（打印前若干条，含用例名/期望/实际）。
"""
import argparse, json, os, shutil, subprocess, sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))       # 飞控仓库根
sys.path.insert(0, HERE)
import ref_frame as R


def run(cmd, cwd=None):
    p = subprocess.run(cmd, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    return p.returncode, p.stdout.decode("utf-8", "replace"), p.stderr.decode("utf-8", "replace")


def build(fc_exe, rc_exe, rc_root, out_dir):
    logs = []
    cmd = ["g++", "-std=c++11", "-O2", "-I", os.path.join(ROOT, "test", "stub"),
           "-I", os.path.join(ROOT, "DL_LIB"), "-I", ROOT,
           os.path.join(ROOT, "test", "link_v2", "sim", "fc_probe.cpp"), "-o", fc_exe]
    rc, so, se = run(cmd)
    logs.append(("fc_probe build", rc, se.strip()))
    if rc != 0:
        return False, logs
    rc_inc = os.path.join(rc_root, "components", "fc_link", "include")
    cmd = ["gcc", "-std=c11", "-O2", "-I", rc_inc,
           os.path.join(ROOT, "test", "link_v2", "sim", "rc_probe.c"),
           os.path.join(rc_root, "components", "fc_link", "fc_frame.c"), "-o", rc_exe]
    rc, so, se = run(cmd)
    logs.append(("rc_probe build", rc, se.strip()))
    return rc == 0, logs


def load_cases(corpus_path):
    """读语料，返回 [(name, type, seq, raw dict)]"""
    cases = []
    name = ""
    with open(corpus_path, encoding="utf-8") as fp:
        for line in fp:
            line = line.strip()
            if not line:
                continue
            if line.startswith("#"):
                name = line[1:].strip()
                continue
            parts = line.split()
            ftype, seq = int(parts[0], 16), int(parts[1], 16)
            raw = {}
            for tok in parts[2:]:
                k, v = tok.split("=")
                raw[k] = int(v)
            cases.append((name, ftype, seq, raw))
    return cases


def read_lines(path):
    with open(path, encoding="utf-8") as fp:
        return [ln.rstrip("\n") for ln in fp if ln.strip()]


def hex_to_bytes(s):
    return bytes(int(x, 16) for x in s.split())


def parse_kv(s):
    out = {}
    for tok in s.split():
        k, v = tok.split("=")
        out[k] = int(v)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rc-root", default=os.environ.get("RC_ROOT", r"E:\ESP_PROJECT\controller"))
    ap.add_argument("--seed", type=int, default=20261018)
    ap.add_argument("--nrandom", type=int, default=1000)
    ap.add_argument("--fuzz-n", type=int, default=400)
    ap.add_argument("--out-dir", default=os.path.join(HERE, "out"))
    ap.add_argument("--skip-build", action="store_true")
    ap.add_argument("--json", default=None)
    args = ap.parse_args()

    out_dir = os.path.abspath(args.out_dir)
    os.makedirs(out_dir, exist_ok=True)
    fc_exe = os.path.join(out_dir, "fc_probe.exe")
    rc_exe = os.path.join(out_dir, "rc_probe.exe")
    corpus = os.path.join(out_dir, "corpus.txt")
    expected = os.path.join(out_dir, "expected.txt")
    fuzz = os.path.join(out_dir, "fuzz.txt")

    result = {"corpus_cases": 0, "encode_mismatch": 0, "decode_mismatch": 0,
              "resync_mismatch": 0, "fuzz_streams": 0, "pass": False, "details": []}

    if not args.skip_build:
        ok, logs = build(fc_exe, rc_exe, args.rc_root, out_dir)
        for name, rc, err in logs:
            print("[build] %-16s %s%s" % (name, "OK" if rc == 0 else "FAIL", (" : " + err[:400]) if err else ""))
        if not ok:
            result["details"].append("构建失败（见上）")
            print("FAIL: 探针构建失败")
            return 1
    if not (os.path.exists(fc_exe) and os.path.exists(rc_exe)):
        print("FAIL: 找不到探针可执行文件，先用不带 --skip-build 的方式运行")
        return 1

    # ---- 1. 语料 ----
    cases = R.build_corpus(seed=args.seed, nrandom=args.nrandom)
    R.write_corpus(corpus, cases)
    R.write_expected(expected, cases)
    result["corpus_cases"] = len(cases)
    # 解码输入：名字不含空格（探针按空白切分）
    decode_in = os.path.join(out_dir, "decode_in.txt")
    with open(decode_in, "w", encoding="utf-8", newline="\n") as fp:
        for i, (name, ftype, seq, raw) in enumerate(cases):
            fp.write("case_%04d %s\n" % (i, " ".join("%02X" % b for b in R.encode_raw(ftype, seq, raw))))

    # ---- 2. 编码对拍 ----
    rc, fc_out, ferr = run([fc_exe, "encode", corpus])
    if rc != 0:
        print("FAIL: fc_probe encode 退出码 %d %s" % (rc, ferr[:400]))
        return 1
    rc, rc_out, rerr = run([rc_exe, "encode", corpus])
    if rc != 0:
        print("FAIL: rc_probe encode 退出码 %d %s" % (rc, rerr[:400]))
        return 1
    fc_lines = fc_out.splitlines()
    rc_lines = rc_out.splitlines()
    if not (len(fc_lines) == len(rc_lines) == len(cases)):
        print("FAIL: 输出行数不一致 cases=%d fc=%d rc=%d"
              % (len(cases), len(fc_lines), len(rc_lines)))
        return 1
    for i, (name, ftype, seq, raw) in enumerate(cases):
        exp_b = R.encode_raw(ftype, seq, raw)
        exp_hex = " ".join("%02X" % b for b in exp_b)
        fc_b = hex_to_bytes(fc_lines[i]) if fc_lines[i] and fc_lines[i] != "ERR" else b""
        rc_b = hex_to_bytes(rc_lines[i]) if rc_lines[i] and rc_lines[i] != "ERR" else b""
        if not (fc_b == rc_b == exp_b):
            result["encode_mismatch"] += 1
            if len(result["details"]) < 12:
                result["details"].append(
                    "encode %s: exp=%s fc=%s rc=%s"
                    % (name, exp_hex, " ".join("%02X" % b for b in fc_b),
                       " ".join("%02X" % b for b in rc_b)))
    print("[encode] %d 例：飞控 vs 遥控器 vs 独立 oracle  不一致 %d"
          % (len(cases), result["encode_mismatch"]))

    # ---- 3. 解码回环 ----
    rc, fc_dec, ferr = run([fc_exe, "decode", decode_in])
    rc, rc_dec, rerr = run([rc_exe, "decode", decode_in])
    fc_dl = fc_dec.splitlines()
    rc_dl = rc_dec.splitlines()
    if not (len(fc_dl) == len(rc_dl) == len(cases)):
        print("FAIL: 解码输出行数不一致")
        return 1
    for i, (name, ftype, seq, raw) in enumerate(cases):
        fields = R.FRAMES[ftype]["fields"]
        want = {f["name"]: int(raw.get(f["name"], 0)) for f in fields}
        for side, line in (("fc", fc_dl[i]), ("rc", rc_dl[i])):
            if line == "ERR":
                result["decode_mismatch"] += 1
                if len(result["details"]) < 20:
                    result["details"].append("decode %s: %s 返回失败" % (name, side))
                continue
            got = parse_kv(line)
            bad = {k: (want.get(k), got.get(k)) for k in want
                   if k != "spare" and want.get(k) != got.get(k)}
            if bad:
                result["decode_mismatch"] += 1
                if len(result["details"]) < 20:
                    result["details"].append("decode %s: %s 字段不符 %s" % (name, side, bad))
    print("[decode] %d 例 × 两端：回环字段不一致 %d" % (len(cases), result["decode_mismatch"]))

    # ---- 4. 坏字节流重同步 ----
    streams = R.build_fuzz_streams(seed=args.seed * 7 + 1, n=args.fuzz_n)
    with open(fuzz, "w", encoding="utf-8", newline="\n") as fp:
        for _seed, buf in streams:
            fp.write(buf.hex().upper() + "\n")
    result["fuzz_streams"] = len(streams)
    rc, fc_rs, ferr = run([fc_exe, "resync", fuzz])
    rc, rc_rs, rerr = run([rc_exe, "resync", fuzz])
    fc_l = fc_rs.splitlines()
    rc_l = rc_rs.splitlines()
    if not (len(fc_l) == len(rc_l) == len(streams)):
        print("FAIL: 重同步输出行数不一致 fc=%d rc=%d exp=%d" % (len(fc_l), len(rc_l), len(streams)))
        return 1
    for i, (seed, buf) in enumerate(streams):
        oracle = R.scan_streaming(buf)
        want = "%d" % len(oracle) + "".join(" %d:%d:%d" % (o, t, s) for (o, t, s) in oracle)
        for side, line in (("fc", fc_l[i]), ("rc", rc_l[i])):
            if line.strip() != want:
                result["resync_mismatch"] += 1
                if len(result["details"]) < 30:
                    result["details"].append("resync stream#%d %s: want=[%s] got=[%s]"
                                             % (i, side, want, line.strip()))
    print("[resync] %d 条随机坏字节流：重同步结果与 oracle 不一致 %d"
          % (len(streams), result["resync_mismatch"]))

    # ---- 汇总 ----
    total_bad = result["encode_mismatch"] + result["decode_mismatch"] + result["resync_mismatch"]
    result["pass"] = total_bad == 0
    print()
    if result["pass"]:
        print("PASS: 两端帧层与独立 oracle 完全一致（编码/解码回环/重同步）")
    else:
        print("FAIL: 差异 %d 项" % total_bad)
        for d in result["details"]:
            print("  - " + d[:600])
    if args.json:
        with open(args.json, "w", encoding="utf-8") as fp:
            json.dump(result, fp, ensure_ascii=False, indent=1)
    return 0 if result["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
