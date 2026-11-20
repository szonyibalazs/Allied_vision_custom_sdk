#!/usr/bin/env python3
"""
Resolve GenICam feature names to device register addresses.

The camera serves its own GenICam XML (see `./probe`). This tool walks that XML and
prints the address / length / type behind a feature, so you can read or write it with
the GenCP helpers in u3v.c without a GenICam runtime.

    python3 genicam_regs.py GenICam_Default.xml ExposureTime Gain PixelFormat
    python3 genicam_regs.py GenICam_Default.xml --grep Balance      # list matching feature names

Limitations: follows simple pValue/Offset chains only. Features computed by SwissKnife
formulas (e.g. selector-indexed registers) are reported without an address - read the XML.
"""
import sys
import xml.etree.ElementTree as ET


def load(path):
    root = ET.parse(path).getroot()
    ns = root.tag.split("}")[0] + "}" if root.tag.startswith("{") else ""
    nodes = {n.get("Name"): n for n in root.iter() if n.get("Name")}
    return ns, nodes


def text(ns, node, tag):
    e = node.find(ns + tag)
    return e.text.strip() if e is not None and e.text else None


def resolve(ns, nodes, node, depth=0):
    """Return (address, length, register_tag) or None."""
    if node is None or depth > 16:
        return None
    addr = text(ns, node, "Address")
    if addr is not None:
        return int(addr, 0), text(ns, node, "Length"), node.tag.replace(ns, "")
    pv = text(ns, node, "pValue")
    if pv and pv in nodes and nodes[pv] is not node:
        base = resolve(ns, nodes, nodes[pv], depth + 1)
        off = text(ns, node, "Offset")
        if base and off:
            return base[0] + int(off, 0), base[1], base[2]
        return base
    return None


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 1
    ns, nodes = load(sys.argv[1])
    if sys.argv[2] == "--grep":
        pat = sys.argv[3].lower() if len(sys.argv) > 3 else ""
        for name in sorted(nodes):
            if pat in name.lower() and not name.startswith("Reg"):
                print(name)
        return 0
    for name in sys.argv[2:]:
        node = nodes.get(name)
        if node is None:
            print(f"{name:28s} not in this camera's XML")
            continue
        r = resolve(ns, nodes, node)
        kind = node.tag.replace(ns, "")
        if r:
            print(f"{name:28s} 0x{r[0]:08x}  len={int(r[1], 0) if r[1] else '?'}  {kind}/{r[2]}")
        else:
            print(f"{name:28s} (no direct address)  {kind}")
        if kind == "Enumeration":
            for e in node.iter(ns + "EnumEntry"):
                print(f"{'':30s}{e.get('Name')} = {text(ns, e, 'Value')}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
