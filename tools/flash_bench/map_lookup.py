"""Parse the Keil armlink .map Image Symbol Table and resolve addresses to symbols."""
import re
import sys

MAP = r"build/FLY/dl_fly.map"
LINE_RE = re.compile(r"^\s+([A-Za-z_$][\w$:<>~@\.\/\\]*)\s+0x([0-9a-fA-F]{8})\s+(?:Section\s+|Thumb Code\s+|ARM Code\s+|Data\s+|Code\s+|Number\s+)?(?:([0-9a-fA-Fx]+))?\s+(\S+\.o)\(")


def load(path=MAP):
    syms = []
    start = False
    with open(path, errors="replace") as f:
        for ln in f:
            if "Image Symbol Table" in ln:
                start = True
                continue
            if start and ("Memory Map of the image" in ln or "Image component sizes" in ln):
                break
            if not start:
                continue
            m = LINE_RE.match(ln)
            if not m:
                continue
            name, addr, size, obj = m.group(1), int(m.group(2), 16), m.group(3), m.group(4)
            try:
                sz = int(size, 16) if size else 0
            except ValueError:
                sz = 0
            syms.append((addr, sz, name, obj))
    syms.sort()
    return syms


def find(syms, addr):
    lo, hi, best = 0, len(syms) - 1, None
    while lo <= hi:
        mid = (lo + hi) // 2
        if syms[mid][0] <= addr:
            best = syms[mid]
            lo = mid + 1
        else:
            hi = mid - 1
    return best


if __name__ == "__main__":
    syms = load()
    print("symbols parsed:", len(syms))
    for a in sys.argv[1:]:
        addr = int(a, 0)
        s = find(syms, addr)
        if s:
            print("0x%08X -> %-45s +0x%-5X  size=0x%-5X %s" % (addr, s[2], addr - s[0], s[1], s[3]))
        else:
            print("0x%08X -> ?" % addr)
