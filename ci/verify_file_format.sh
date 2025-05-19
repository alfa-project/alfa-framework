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

echo "Checking C/C++ formatting with clang-format..."

C_FILES=$(find . -regex '.*\.\(cpp\|hpp\|cc\|c\|h\)')
if [[ -n "$C_FILES" ]]; then
  for file in $C_FILES; do
    echo "Checking $file"
    clang-format --style=file:ci/.clang-format --dry-run --Werror "$file"
  done
else
  echo "No C/C++ files found."
fi

echo "Checking Verilog formatting with verible-verilog-format..."

VERILOG_FILES=$(find . -type f -name "*.v")
SYSTEMVERILOG_FILES=$(find . -type f -name "*.sv")

ALL_VERILOG_FILES=($VERILOG_FILES $SYSTEMVERILOG_FILES)

if [[ ${#ALL_VERILOG_FILES[@]} -gt 0 ]]; then
  for file in "${ALL_VERILOG_FILES[@]}"; do
    echo "Checking $file"
    verible-verilog-format --verify "$file"
  done
else
  echo "No Verilog or SystemVerilog files found."
fi

echo "Format verification complete. Everything looks good."
