#!/usr/bin/env bash
# =============================================================================
# PCIe 7.0 Controller - Coverage Database Merge Script
# =============================================================================
# Merges per-test, per-seed coverage databases into a single merged report.
# Supports Questa (ucdb) and VCS (.vdb) flow.
# Usage:
#   scripts/coverage_merge.sh questa   # merge .ucdb files
#   scripts/coverage_merge.sh vcs      # merge .vdb directories
# =============================================================================

set -euo pipefail

SIMULATOR=${1:-questa}
COV_DIR=$(dirname "$0")/../coverage
BUILD_DIR=$(dirname "$0")/../build/uvm
REPORT_DIR="${COV_DIR}/reports"

mkdir -p "${REPORT_DIR}"

case "${SIMULATOR}" in

  # --------------------------------------------------------------------------
  # Questa: vcover merge
  # --------------------------------------------------------------------------
  questa)
    echo "=== Merging Questa coverage databases ==="
    UCDB_FILES=$(find "${BUILD_DIR}" -name "*.ucdb" 2>/dev/null | tr '\n' ' ')
    if [ -z "${UCDB_FILES}" ]; then
      echo "No .ucdb files found under ${BUILD_DIR}"
      exit 1
    fi

    vcover merge "${COV_DIR}/merged.ucdb" ${UCDB_FILES}
    echo "Merged UCDB: ${COV_DIR}/merged.ucdb"

    # HTML report
    vcover report -html "${COV_DIR}/merged.ucdb" \
      -output "${REPORT_DIR}/html" \
      -details -verbose
    echo "HTML report: ${REPORT_DIR}/html/index.html"

    # Text summary
    vcover report "${COV_DIR}/merged.ucdb" \
      -details > "${REPORT_DIR}/coverage_summary.txt"
    echo "Text report: ${REPORT_DIR}/coverage_summary.txt"

    # Show top-level metrics
    vcover report "${COV_DIR}/merged.ucdb" | grep -E "Coverage|Total|Functional"
    ;;

  # --------------------------------------------------------------------------
  # VCS: urg (unified reporting)
  # --------------------------------------------------------------------------
  vcs)
    echo "=== Merging VCS coverage databases ==="
    VDB_DIRS=$(find "${BUILD_DIR}" -name "*.vdb" -type d 2>/dev/null | tr '\n' ' ')
    if [ -z "${VDB_DIRS}" ]; then
      echo "No .vdb directories found under ${BUILD_DIR}"
      exit 1
    fi

    # Merge with urg
    urg -dir ${VDB_DIRS} \
        -format both \
        -report "${REPORT_DIR}" \
        -dbname "${COV_DIR}/merged.vdb"

    echo "Merged VDB:  ${COV_DIR}/merged.vdb"
    echo "HTML report: ${REPORT_DIR}/dashboard.html"
    echo "Text report: ${REPORT_DIR}/hierarchy.txt"
    ;;

  *)
    echo "Unknown simulator: ${SIMULATOR}. Use 'questa' or 'vcs'."
    exit 1
    ;;
esac

echo ""
echo "=== Coverage Merge Complete ==="
echo "Review ${REPORT_DIR}/ and archive in coverage/reports/ before signoff."
