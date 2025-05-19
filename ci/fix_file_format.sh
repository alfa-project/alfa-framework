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

VERILOG_FILES=$(find . -type f \( -name "*.v" -o -name "*.sv" \))
if [[ -n "$VERILOG_FILES" ]]; then
  for file in $VERILOG_FILES; do
    echo "Formatting $file"
    verible-verilog-format \
      --indentation_spaces=2 \
      --wrap_spaces=100 \
      --module_net_variable_alignment=align \
      --inplace "$file"
  done
else
  echo "No Verilog files found."
fi

echo "Formatting complete."
