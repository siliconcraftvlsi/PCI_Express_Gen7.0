#!/usr/bin/env bash
# =============================================================================
# PCIe 7.0 Controller - Release Package Script
# =============================================================================
# Creates a timestamped, checksummed release tarball for IP delivery.
# Usage:
#   scripts/release_package.sh [VERSION]
#   e.g.: scripts/release_package.sh 0.2.0
#
# Output: releases/pcie_controller_v<VERSION>_<DATE>.tar.gz
#         releases/pcie_controller_v<VERSION>_<DATE>.sha256
# =============================================================================

set -euo pipefail

VERSION=${1:-"0.1.0"}
DATE=$(date +%Y%m%d)
DESIGN_ROOT=$(cd "$(dirname "$0")/.." && pwd)
RELEASE_NAME="pcie_controller_v${VERSION}_${DATE}"
RELEASE_DIR="${DESIGN_ROOT}/releases"
PACKAGE_DIR="${RELEASE_DIR}/${RELEASE_NAME}"

mkdir -p "${RELEASE_DIR}"

echo "=== PCIe 7.0 Controller Release Package Builder ==="
echo "    Version : ${VERSION}"
echo "    Date    : ${DATE}"
echo "    Output  : ${RELEASE_DIR}/${RELEASE_NAME}.tar.gz"
echo ""

# ---------------------------------------------------------------------------
# Collect files into staging directory
# ---------------------------------------------------------------------------
rm -rf "${PACKAGE_DIR}"
mkdir -p "${PACKAGE_DIR}"

copy_dir() {
  local src="${DESIGN_ROOT}/$1"
  local dst="${PACKAGE_DIR}/$1"
  if [ -d "${src}" ]; then
    mkdir -p "${dst}"
    cp -r "${src}/." "${dst}/"
    echo "  + $1/"
  fi
}

copy_file() {
  local src="${DESIGN_ROOT}/$1"
  if [ -f "${src}" ]; then
    local dstdir="${PACKAGE_DIR}/$(dirname "$1")"
    mkdir -p "${dstdir}"
    cp "${src}" "${dstdir}/"
    echo "  + $1"
  fi
}

echo "Collecting RTL..."
copy_dir rtl

echo "Collecting testbench..."
copy_dir tb
copy_dir uvm_tb

echo "Collecting SVA..."
copy_dir sva

echo "Collecting formal..."
copy_dir formal

echo "Collecting documentation..."
copy_dir docs
copy_file README.md
copy_file CHANGELOG.md

echo "Collecting constraints and lint..."
copy_dir constraints
copy_dir lint

echo "Collecting scripts..."
copy_dir scripts

echo "Collecting filelists..."
copy_dir filelists

echo "Collecting IP-XACT..."
copy_dir ip_xact

echo "Collecting coverage plan..."
copy_dir coverage

# Exclude build artifacts and git internals
rm -rf "${PACKAGE_DIR}/.git"
find "${PACKAGE_DIR}" -name "*.vvp" -delete
find "${PACKAGE_DIR}" -name "*.vcd" -delete
find "${PACKAGE_DIR}" -name "*.ucdb" -delete
find "${PACKAGE_DIR}" -name "*.vdb" -type d -exec rm -rf {} + 2>/dev/null || true

# ---------------------------------------------------------------------------
# Write release manifest
# ---------------------------------------------------------------------------
MANIFEST="${PACKAGE_DIR}/MANIFEST.txt"
echo "PCIe 7.0 Controller IP Release" > "${MANIFEST}"
echo "Version : ${VERSION}"           >> "${MANIFEST}"
echo "Date    : ${DATE}"              >> "${MANIFEST}"
echo "Builder : $(whoami)@$(hostname)" >> "${MANIFEST}"
echo ""                               >> "${MANIFEST}"
echo "Files:"                         >> "${MANIFEST}"
find "${PACKAGE_DIR}" -type f | sort | sed "s|${PACKAGE_DIR}/||" >> "${MANIFEST}"

# ---------------------------------------------------------------------------
# Create tarball
# ---------------------------------------------------------------------------
cd "${RELEASE_DIR}"
tar czf "${RELEASE_NAME}.tar.gz" "${RELEASE_NAME}/"
echo ""
echo "Tarball created: ${RELEASE_DIR}/${RELEASE_NAME}.tar.gz"

# ---------------------------------------------------------------------------
# Generate SHA-256 checksum
# ---------------------------------------------------------------------------
sha256sum "${RELEASE_NAME}.tar.gz" > "${RELEASE_NAME}.sha256"
echo "Checksum:        ${RELEASE_DIR}/${RELEASE_NAME}.sha256"
cat "${RELEASE_NAME}.sha256"

# ---------------------------------------------------------------------------
# Cleanup staging directory
# ---------------------------------------------------------------------------
rm -rf "${PACKAGE_DIR}"

echo ""
echo "=== Release Package Complete ==="
echo "Archive : ${RELEASE_NAME}.tar.gz"
echo "Checksum: ${RELEASE_NAME}.sha256"
echo ""
echo "Next steps:"
echo "  1. Verify checksum matches after transfer"
echo "  2. Update docs/signoff_checklist.md with release version and hash"
echo "  3. Tag the git commit: git tag -a v${VERSION} -m 'Release ${VERSION}'"
