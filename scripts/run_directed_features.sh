#!/usr/bin/env bash
# PM / completion-timeout / MSI-X directed tests (iverilog)
# Tests 16–18 only: faster than full sim-strict regression.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

STRICT="${STRICT:-0}"
TARGET="sim-features"
if [[ "$STRICT" == "1" ]]; then
  TARGET="sim-features-strict"
fi

echo "=== Directed feature tests (STRICT=$STRICT) ==="
make -C tb "$TARGET"
echo "=== Done: see tb logs and build/*.vcd ==="
