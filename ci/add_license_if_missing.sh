#!/bin/bash
set -e

CURRENT_YEAR=$(date +%Y)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HEADER_FILE="$SCRIPT_DIR/.licenseheader"
LICENSE_TEXT=$(sed "s/XXXX/$CURRENT_YEAR/g" "$HEADER_FILE")
FILE_TYPES=("*.c" "*.cpp" "*.h" "*.v" "*.sv")

add_header_if_missing() {
    local file="$1"
    if ! grep -q "Copyright" "$file"; then
        echo "Adding header to $file"
        tmpfile=$(mktemp)
        echo "$LICENSE_TEXT" > "$tmpfile"
        echo "" >> "$tmpfile"
        cat "$file" >> "$tmpfile"
        mv "$tmpfile" "$file"
    fi
}

for pattern in "${FILE_TYPES[@]}"; do
    find . -type f -name "$pattern" | while read -r file; do
        add_header_if_missing "$file"
    done
done
