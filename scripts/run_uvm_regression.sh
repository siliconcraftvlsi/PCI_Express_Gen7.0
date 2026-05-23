#!/usr/bin/env bash
# UVM regression — requires Questa or VCS (not Verilator).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

SIMULATOR="${SIMULATOR:-questa}"
SEED="${SEED:-1}"
STRICT="${STRICT_LAYERS:-0}"

echo "=== UVM compile (SIMULATOR=$SIMULATOR STRICT_LAYERS=$STRICT) ==="
make -C uvm_tb compile SIMULATOR="$SIMULATOR" STRICT_LAYERS="$STRICT"

echo "=== smoke ==="
make -C uvm_tb smoke SIMULATOR="$SIMULATOR" SEED="$SEED" STRICT_LAYERS="$STRICT"

echo "=== error_inj ==="
make -C uvm_tb error_inj SIMULATOR="$SIMULATOR" SEED="$SEED" STRICT_LAYERS="$STRICT"

echo "=== feature_regression (PM / cpl-timeout / MSI-X + event cov) ==="
make -C uvm_tb feature_regression SIMULATOR="$SIMULATOR" SEED="$SEED" STRICT_LAYERS="$STRICT"

echo "=== multi-seed regress ==="
make -C uvm_tb regress SIMULATOR="$SIMULATOR" STRICT_LAYERS="$STRICT" NUM_SEEDS="${NUM_SEEDS:-3}"

echo "=== Logs: uvm_tb/logs/  Coverage: build/uvm/*.ucdb coverage/ ==="
