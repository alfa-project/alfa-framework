#!/bin/bash
set -e

CURRENT_YEAR=$(date +%Y)
FILE_TYPES=("*.c" "*.cpp" "*.h" "*.v" "*.sv")

echo "Verifying license headers and correct year: $CURRENT_YEAR"

failed=false
checked=0

check_header() {
    local file="$1"
    ((checked++))

    if ! grep -q "Copyright" "$file"; then
        echo "Missing license header in: $file"
        failed=true
        return
    fi

    if ! grep -q "Copyright $CURRENT_YEAR" "$file"; then
        echo "Incorrect or outdated year in: $file"
        failed=true
    fi
}

for pattern in "${FILE_TYPES[@]}"; do
    find . -type f -name "$pattern" | while read -r file; do
        check_header "$file"
    done
done

echo "Checked $checked file(s)."

if $failed; then
    echo "One or more files are missing headers or have incorrect years."
    echo "Run ./ci/fix_all.sh to correct them."
    exit 1
fi

echo "All headers are present and up-to-date."
