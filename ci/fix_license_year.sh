#!/bin/bash
set -e

CURRENT_YEAR=$(date +%Y)
FILE_TYPES=("*.c" "*.cpp" "*.h" "*.v" "*.sv" "*.hpp")

echo "Fixing license years to $CURRENT_YEAR..."

fix_year() {
    local file="$1"
    if grep -q "Copyright" "$file"; then
        # Replace any existing year or XXXX with the current year
        sed -i "s/Copyright [0-9]\{4\}/Copyright $CURRENT_YEAR/" "$file"
        sed -i "s/Copyright XXXX/Copyright $CURRENT_YEAR/" "$file"
        echo "Updated year in: $file"
    fi
}

for pattern in "${FILE_TYPES[@]}"; do
    find . -type f -name "$pattern" | while read -r file; do
        fix_year "$file"
    done
done

echo "License year fix complete."
