#!/bin/bash
set -e

echo "Adding missing license headers..."
./ci/fix_missing_headers.sh

echo "Verifying license years..."
./ci/fix_license_year.sh

echo "Running format verification tools..."
./ci/fix_file_format.sh

echo "All done. You can now commit and push your changes!!"
