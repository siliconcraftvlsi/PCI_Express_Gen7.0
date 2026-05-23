#!/usr/bin/env python3
"""Prepare SystemVerilog sources for stock Yosys (no import pkg::* support)."""

from __future__ import annotations

import argparse
import re
import shutil
import sys
from pathlib import Path


PKG_RE = re.compile(r"^\s*import\s+pcie_pkg::\*;\s*$", re.MULTILINE)
ALREADY_QUALIFIED = re.compile(r"\bpcie_pkg::")


def extract_pkg_symbols(pkg_text: str) -> list[str]:
    symbols: set[str] = set()

    for m in re.finditer(r"^\s*localparam\b[^;]*\b([A-Za-z_]\w*)\s*=", pkg_text, re.M):
        symbols.add(m.group(1))

    for m in re.finditer(r"^\s*function\s+automatic\b[^;]*\b([A-Za-z_]\w*)\s*;", pkg_text, re.M):
        symbols.add(m.group(1))

    for m in re.finditer(r"^\s*function\s+automatic\b[^(\n]*\b([A-Za-z_]\w*)\s*\(", pkg_text, re.M):
        symbols.add(m.group(1))

    for m in re.finditer(r"}\s*([A-Za-z_]\w*)\s*;", pkg_text):
        symbols.add(m.group(1))

    for block in re.finditer(r"typedef\s+enum\b[^{]*\{([^}]*)\}", pkg_text, re.S):
        for m in re.finditer(r"\b([A-Za-z_]\w*)\s*=", block.group(1)):
            symbols.add(m.group(1))

    # Drop package name itself if captured.
    symbols.discard("pcie_pkg")
    return sorted(symbols, key=len, reverse=True)


def qualify_pkg_symbols(text: str, symbols: list[str]) -> str:
    out = PKG_RE.sub("", text)
    for sym in symbols:
        out = re.sub(rf"(?<!pcie_pkg::)\b{re.escape(sym)}\b", f"pcie_pkg::{sym}", out)
    return out


def preprocess_file(src: Path, dst: Path, symbols: list[str], qualify: bool) -> None:
    text = src.read_text()
    if qualify:
        text = qualify_pkg_symbols(text, symbols)
    dst.parent.mkdir(parents=True, exist_ok=True)
    dst.write_text(text)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--pkg", type=Path, required=True, help="pcie_pkg.sv source")
    ap.add_argument("--src", type=Path, action="append", required=True, help="source file")
    ap.add_argument("--out-dir", type=Path, required=True, help="output directory")
    args = ap.parse_args()

    pkg_text = args.pkg.read_text()
    symbols = extract_pkg_symbols(pkg_text)

    for src in args.src:
        rel = src.name
        dst = args.out_dir / rel
        qualify = src.resolve() != args.pkg.resolve()
        preprocess_file(src, dst, symbols, qualify)
        if not qualify:
            continue
        print(f"yosys-prep: {src} -> {dst}", file=sys.stderr)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
