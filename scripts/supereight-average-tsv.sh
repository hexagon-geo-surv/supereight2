#!/bin/sh
# SPDX-FileCopyrightText: 2021 Smart Robotics Lab
# SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou, Imperial College London
# SPDX-License-Identifier: BSD-3-Clause
set -eu

average_tsv_awk='
# This is the awk way of testing whether something is a number.
function is_num(x) { return x + 0 == x }
# Set the input and output column (field) separator to tab. Use 6 decimal digits
# for printing numbers.
BEGIN { FS = "\t"; OFS = "\t"; CONVFMT = "%.6f" }
# Print the header (first) line.
NR == 1
# Accumulate the numerical values of each column for all other lines.
NR > 1 {
	for (i = 1; i <= NF; i++) {
		if (is_num($i)) {
			s[i] += $i
			n[i]++
		}
	}
}
# Create a record with the means of each column and print it.
END {
	for (i = 1; i <= NF; i++) {
		if (n[i]) {
			$i = s[i]/n[i]
		} else {
			$i = "*"
		}
	}
	print
}'

# Usage: average_tsv FILE
average_tsv() {
	# 1. Strip leading and trailing whitespace to avoid having empty columns.
	# 2. Remove the first column.
	# 3. Compute and print the column means.
	sed -E -e 's/^[[:space:]]*//' -e 's/[[:space:]]*$//' "$1" \
		| cut -f 2- \
		| awk "$average_tsv_awk"
}

# Usage: align_tsv < FILE
# Align the columns of TSV data from stdin using spaces.
align_tsv() {
	column -t -s '	'
}

# Usage: tsv_to_markdown < FILE
# Convert TSV data from stdin to a Markdown table on stdout.
tsv_to_markdown() {
	awk '
BEGIN { FS = "\t" }
# Print all TSV lines with Markdown table separators.
{
	for (i = 1; i <= NF; i++) { printf "| %s ", $i }
	printf "|\n"
}
# Print the separator line after the header (the first line).
NR == 1 {
	for (i = 1; i <= NF; i++) { printf "| --- " }
	printf "|\n"
}
END { printf "\n" }'
}

usage() {
	printf 'Usage: %s [OPTIONS] [FILE]...\n' "$(basename "$0")"
	printf 'Show the per-frame averages from a supereight TSV log.\n'
	printf 'With no FILE read from standard input.'
	printf '\n-f FORMAT Select the output format. FORMAT can be aligned, tsv or markdown.'
	printf '\n-h        Show this help message.\n'
}

# Parse command line options.
format_converter='align_tsv'
while getopts 'f:h' opt_name; do
	case "$opt_name" in
		f)
			case "$OPTARG" in
				a*) # a[lign]
					format_converter='align_tsv'
					;;
				t*) # t[sv]
					format_converter='cat'
					;;
				m*) # m[arkdown]
					format_converter='tsv_to_markdown'
					;;
				*)
					printf 'Error: unknown FORMAT %s\n' "$OPTARG"
					usage
					exit 2
					;;
			esac
			;;
		h)
			usage
			exit 0
			;;
		*)
			usage
			exit 2
			;;
	esac
done
# Make $1 the first non-option argument.
shift "$((OPTIND - 1))"
# Read from standard input if no arguments were provided.
if [ "$#" -eq 0 ]; then
	set -- '-'
fi

# Process all input files and convert the combined output.
while [ "$#" -gt 0 ]; do
	average_tsv "$1"
	shift
done | $format_converter

