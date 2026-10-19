#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""check_link_config.py —— 链路 v2 两端常量头对拍。

用法:
    python tools/check_link_config.py
    python tools/check_link_config.py --rc E:/ESP_PROJECT/controller/components/fc_link/include/fc_link_config.h

做什么:
    - 解析飞控 app/link/link_v2_config.hpp 的 `constexpr <type> Name = N;`
    - 解析遥控器 components/fc_link/include/fc_link_config.h 的 enum 项 `Name = N,`
    - 逐名逐值比较: 名字集合相同、值相同; 不一致 -> 非零退出并打印差异。

为什么值得单独一个脚本:
    两个仓库在不同盘上, 常量名/值靠人眼同步必然会漂; 而漂了的第一个表现是
    "轮次在理论上成立, 台架上却总是差半帧" —— 这种问题不该靠飞行去发现。

ASCII-only 输出(Windows 控制台/CI 友好); 约束: 两个头文件里每个常量必须独占一行。
"""
import argparse
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
DEFAULT_FC = HERE.parent / "app" / "link" / "link_v2_config.hpp"
DEFAULT_RC = Path(r"E:/ESP_PROJECT/controller/components/fc_link/include/fc_link_config.h")

# constexpr uint32_t LinkRoundMs = 120;   // 或 /* ... */ 注释
FC_RE = re.compile(r"^\s*constexpr\s+\w+\s+([A-Za-z_]\w*)\s*=\s*(\d+)\s*;")
# LinkRoundMs = 120,   /* ... */
RC_RE = re.compile(r"^\s*([A-Za-z_]\w*)\s*=\s*(\d+)\s*,")


def parse_fc(path: Path) -> dict:
    out = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        m = FC_RE.match(line)
        if m:
            out[m.group(1)] = int(m.group(2))
    return out


def parse_rc(path: Path) -> dict:
    out = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        m = RC_RE.match(line)
        if m:
            out[m.group(1)] = int(m.group(2))
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--fc", type=Path, default=DEFAULT_FC)
    ap.add_argument("--rc", type=Path, default=DEFAULT_RC)
    args = ap.parse_args()

    for p in (args.fc, args.rc):
        if not p.is_file():
            print(f"FAIL: config header not found: {p}")
            return 2

    fc = parse_fc(args.fc)
    rc = parse_rc(args.rc)
    if not fc or not rc:
        print("FAIL: parsed 0 constants from at least one header (regex mismatch?)")
        return 2

    only_fc = sorted(set(fc) - set(rc))
    only_rc = sorted(set(rc) - set(fc))
    mismatch = sorted(n for n in set(fc) & set(rc) if fc[n] != rc[n])

    for n in only_fc:
        print(f"FAIL: only in FC  : {n} = {fc[n]}")
    for n in only_rc:
        print(f"FAIL: only in RC  : {n} = {rc[n]}")
    for n in mismatch:
        print(f"FAIL: value differs: {n}: FC={fc[n]} RC={rc[n]}")

    if only_fc or only_rc or mismatch:
        print("LINK CONFIG CHECK FAIL")
        return 1

    print(f"LINK CONFIG CHECK PASS ({len(fc)} constants identical)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
