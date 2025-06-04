#!/bin/bash
# Name: format.sh
# Micras Team
# Brief: Format the code using clang-format
# 06/2025

Green='\033[0;32m'      # Green
BGreen='\033[1;32m'     # Bold Green
BPurple='\033[1;35m'    # Bold Purple

# Check if clang-format is installed
if ! [ -x "$(command -v clang-format)" ]; then
  echo 'Error: clang-format is not installed.' >&2
  exit 1
fi

# Find all C, C++ and header files excluding specified directories
FILES=$(find . \( -name "*.c" -o -name "*.cpp" -o -name "*.h" -o -name "*.hpp" \) \
                -not \( -path "*/build/*" \))

# Loop through each file and format if found
for FILE in $FILES; do
    # Check if the file exists and is readable
    if [ -f "$FILE" ] && [ -r "$FILE" ]; then
        # Format the file using clang-format
        clang-format -style=file -i "$FILE"
        echo -e "${BGreen}Formatted: ${Green}$FILE"
    fi
done

echo -e "${BPurple}Formatting complete."
