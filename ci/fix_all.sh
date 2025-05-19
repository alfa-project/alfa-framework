#!/bin/bash
set -e

echo "Adding missing license headers..."
./ci/add_license_if_missing.sh

echo "Verifying license years..."
./ci/check_license_year.sh

echo "Running clang-format..."
./ci/check_file_format.sh

echo "All done. You can now commit and push your changes!!"
