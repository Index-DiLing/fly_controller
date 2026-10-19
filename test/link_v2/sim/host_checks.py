#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""两端轮次实现的独立口径断言（Lead 拍板的偏差项回归）
  1) 飞控 RoundSlave：fc_host_checks.cpp（链接 DL_LIB/link/dlx_link_round.hpp）
  2) 遥控器 RoundMaster：rc_host_checks.c（链接 RC 的 fc_round.c + fc_frame.c）
两个探针只调用被测实现，不含协议断言以外的逻辑。

用法（仓库根目录）:
  python test/link_v2/sim/host_checks.py [--rc-root E:\\ESP_PROJECT\\controller]
退出码: 0 = 两端全部 PASS；1 = 有 FAIL/构建失败
"""
import argparse, os, subprocess, sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
OUT = os.path.join(HERE, "out")


def run(cmd, cwd=None):
    p = subprocess.run(cmd, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    out = p.stdout.decode("utf-8", "replace").replace("\r\n", "\n").replace("\r", "\n")
    err = p.stderr.decode("utf-8", "replace").replace("\r\n", "\n").replace("\r", "\n")
    return p.returncode, out, err


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rc-root", default=os.environ.get("RC_ROOT", r"E:\ESP_PROJECT\controller"))
    args = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    fails = 0

    fc_exe = os.path.join(OUT, "fc_host_checks.exe")
    cmd = ["g++", "-std=c++11", "-O2", "-I", os.path.join(ROOT, "test", "stub"),
           "-I", os.path.join(ROOT, "DL_LIB"), "-I", ROOT,
           os.path.join(HERE, "fc_host_checks.cpp"), "-o", fc_exe]
    rc, so, se = run(cmd)
    if rc != 0:
        print("[host] FC 探针构建失败:\n" + se[:1500])
        fails += 1
    else:
        rc, so, se = run([fc_exe])
        sys.stdout.write("[host] 飞控 RoundSlave 口径断言\n" + so)
        if se.strip():
            sys.stdout.write(se + "\n")
        if rc != 0:
            fails += 1

    rc_exe = os.path.join(OUT, "rc_host_checks.exe")
    rinc = os.path.join(args.rc_root, "components", "fc_link", "include")
    cmd = ["gcc", "-std=c11", "-O2", "-I", rinc,
           os.path.join(HERE, "rc_host_checks.c"),
           os.path.join(args.rc_root, "components", "fc_link", "fc_round.c"),
           os.path.join(args.rc_root, "components", "fc_link", "fc_frame.c"),
           "-o", rc_exe]
    rc, so, se = run(cmd)
    if rc != 0:
        print("[host] RC 探针构建失败:\n" + se[:1500])
        fails += 1
    else:
        rc, so, se = run([rc_exe])
        sys.stdout.write("[host] 遥控器 RoundMaster 口径断言\n" + so)
        if se.strip():
            sys.stdout.write(se + "\n")
        if rc != 0:
            fails += 1

    print()
    if fails == 0:
        print("PASS: 两端轮次实现口径断言全部通过")
        return 0
    print("FAIL: %d 组断言失败" % fails)
    return 1


if __name__ == "__main__":
    sys.exit(main())
