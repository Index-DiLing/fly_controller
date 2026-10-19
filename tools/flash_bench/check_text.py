"""Check one captured boot text dump for line-quality problems.

Reports: segments that are empty/bare-CR/space-only, U+FFFD replacement chars
(UTF-8 cut in half), lines over the 160-byte SimpleLog line budget, and the
content of the previously-truncated long lines.
"""
import sys

path = sys.argv[1]
t = open(path, encoding="utf-8").read()
lines = t.split("\n")
print("file:", path)
print("total segments:", len(lines))
print("empty segments:", sum(1 for l in lines if l == ""))
print("bare-CR segments:", sum(1 for l in lines if l == "\r"))
print("space-only segments:", sum(1 for l in lines
                                   if l.strip() == "" and l not in ("", "\r")))
bad = [l for l in lines if "\ufffd" in l]
print("U+FFFD lines:", len(bad))
for l in bad[:5]:
    print("   ", repr(l))
over = [l for l in lines if len(l.encode("utf-8")) > 161]
print("lines >161 bytes:", len(over))
for l in over[:5]:
    print("   ", len(l.encode("utf-8")), repr(l[:90]))
print("--- key lines ---")
for l in lines:
    if (("进入串口调试模式" in l) or ("运行状态" in l) or ("[Link]" in l)
            or ("[Stat]" in l) or ("[Boot] 进入" in l)):
        print(" bytes=%3d %r" % (len(l.encode("utf-8")), l))
