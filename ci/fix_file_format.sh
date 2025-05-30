#!/bin/bash
set -e

# Check for required tools
if ! command -v clang-format &> /dev/null; then
    echo "Error: 'clang-format' is not installed."
    echo "Install it with: sudo apt install clang-format"
    exit 1
fi

if ! command -v verible-verilog-format &> /dev/null; then
    echo "Error: 'verible-verilog-format' is not installed."
    echo "Install it from: https://github.com/chipsalliance/verible/releases"
    exit 1
fi

echo "Running clang-format on C/C++ files..."

C_FILES=$(find . -regex '.*\.\(cpp\|hpp\|cc\|c\|h\)')
if [[ -n "$C_FILES" ]]; then
  for file in $C_FILES; do
    echo "Formatting $file"
    clang-format --style=file:ci/.clang-format -i "$file"
  done
else
  echo "No C/C++ files found."
fi

echo "Running verible-verilog-format on Verilog files..."

set -euo pipefail

# Find Verilog and SystemVerilog files
readarray -d '' VERILOG_FILES < <(find . -type f -name "*.v" -print0)
readarray -d '' SYSTEMVERILOG_FILES < <(find . -type f -name "*.sv" -print0)

# Combine both arrays
ALL_VERILOG_FILES=("${VERILOG_FILES[@]}" "${SYSTEMVERILOG_FILES[@]}")

# Path to config file
VERIBLE_CONFIG="ci/.rules.verible"

# Check formatting
if [[ ${#ALL_VERILOG_FILES[@]} -gt 0 ]]; then
  for file in "${ALL_VERILOG_FILES[@]}"; do
    echo "Checking $file"
    verible-verilog-format \
      --flagfile="$VERIBLE_CONFIG" \
      --inplace "$file"
  done
else
  echo "No Verilog or SystemVerilog files found."
fi

echo "Formatting complete."
