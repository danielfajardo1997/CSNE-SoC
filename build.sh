#!/bin/bash
# =============================================================================
# build.sh — CSNE-SoC HPS Software Build Script
# Builds both the functional driver and benchmark suite.
# Run directly on the DE10-Nano or cross-compile from host.
# =============================================================================

set -e

DRIVER_DIR="$(cd "$(dirname "$0")/../driver" && pwd)"
OUT_DIR="$DRIVER_DIR"

echo "Building CSNE-SoC HPS software..."
echo "Source: $DRIVER_DIR"

gcc -O1 -Wall -Wextra \
    -o "$OUT_DIR/pe_hps_driver" \
    "$DRIVER_DIR/pe_hps_driver.c"
echo "  -> pe_hps_driver  [OK]"

gcc -O1 -Wall -Wextra -lm \
    -o "$OUT_DIR/pe_benchmark" \
    "$DRIVER_DIR/pe_benchmark.c" \
    -lm
echo "  -> pe_benchmark   [OK]"

echo ""
echo "Run with:"
echo "  sudo $OUT_DIR/pe_hps_driver"
echo "  sudo $OUT_DIR/pe_benchmark"
