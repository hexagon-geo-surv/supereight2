#!/bin/sh
# SPDX-FileCopyrightText: 2021 Smart Robotics Lab
# SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou, Imperial College London
# SPDX-License-Identifier: BSD-3-Clause
set -eu

# Ensure at least 1 input argument was provided.
if [ "$#" -lt 1 ]; then
	printf "Usage: %s FILE...\n" "$(basename "$0")"
	exit 2
fi

awk_program='
# This is the awk way of testing whether something is a number.
function is_num(x) { return x + 0 == x }
# Set the input and output column (field) separator to tab. Use 6 decimal digits
# for printing numbers.
BEGIN { FS = "\t"; OFS = "\t"; CONVFMT = "%.6f" }
# Print the header (first) line.
NR == 1
# Accumulate the numerical values of each column for all other lines.
NR > 1 { for (i = 1; i <= NF; i++) { if (is_num($i)) { s[i] += $i; n[i]++ } } }
# Create a record with the means of each column and print it.
END { for (i = 1; i <= NF; i++) $i = s[i]/n[i]; print }'

# Process all input files and align their columns at the end.
while [ "$#" -gt 0 ]; do
	# 1. Strip leading and trailing whitespace to avoid having empty columns.
	# 2. Remove the first column.
	# 3. Compute and print the column means.
	sed -E -e 's/^[[:space:]]*//' -e 's/[[:space:]]*$//' "$1" \
		| cut -f 2- \
		| awk "$awk_program"
	# Process the next input file.
	shift
done | column -t -s '	'

