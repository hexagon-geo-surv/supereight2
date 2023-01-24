#!/bin/sh
# icl-nuim-download.sh - Download the ICL-NUIM dataset living room sequences
# SPDX-FileCopyrightText: 2019-2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-FileCopyrightText: 2019-2023 Sotiris Papatheodorou
# SPDX-License-Identifier: BSD-3-Clause
set -eu

# Show the program usage on standard output.
usage() {
	name=$(basename "$0")
	cat <<- EOF
	Usage: $name [-o] DIRECTORY [SEQUENCE_ID] ...
	       $name -h

	Download the ICL-NUIM dataset living room sequences in the TUM format
	inside DIRECTORY. SEQUENCE_ID may be 0, 1, 2 or 3. If no SEQUENCE_ID is
	supplied then all 4 living room sequences are downloaded.

	  -o Optimize the downloaded PNG images using optipng to reduce their
	     size. This will take some time but will speed up reading the
	     images when the dataset is run.
	  -h Show this help message.
	EOF
}

# Usage: sedi [OPTIONS] FILE
# Run sed in-place on FILE (like sed -i would).
sedi() {
	# Get the last argument.
	for file in "$@"; do :; done
	[ -f "$file" ] || return 1
	# The TUM format doesn't contain any .temp files so this should be
	# relatively safe.
	out_file="$file".temp
	# Store the output of sed in a different file and then move it over the
	# processed file.
	sed "$@" > "$out_file"
	mv "$out_file" "$file"
}

# Usage: download_sequence URL FILE
# Download the dataset sequence from URL into FILE.
download_sequence() {
	printf 'Downloading %s into %s\n' "$1" "$2"
	mkdir -p "$(dirname "$2")"
	wget --no-verbose --continue --output-document "$2" "$1"
}

# Usage: extract_sequence FILE DIRECTORY
# Extract the dataset sequence from FILE into DIRECTORY.
extract_sequence() {
	printf 'Extracting %s into %s\n' "$1" "$2"
	mkdir -p "$2"
	tar -xzf "$1" -C "$2"
}

# Usage: remove_frame_without_gt DIRECTORY FRAME
remove_frame_without_gt() {
	if ! grep -q '^'"$2"' ' "$1/groundtruth.txt"
	then
		# Delete the frame's images.
		rm -f "$1/depth/$2.png" "$1/rgb/$2.png"
		# Remove the frame line from the associations file.
		sed_script=$(printf '/^%s depth\\/%s\\.png %s rgb\\/%s\\.png$/d' \
			"$2" "$2" "$2" "$2")
		sedi "$sed_script" "$1/associations.txt"
	fi
}

# Usage: generate_filename_list DIRECTORY
# Generate a txt file named DIRECTORY.txt listing all the PNG images in
# DIRECTORY as required by the TUM RGB-D dataset format.
generate_filename_list() {
	input_dir="$1"
	output_file="${input_dir%%/}.txt"
	input_dir_name=$(basename "$input_dir")
	# Write the header to the output file.
	printf '# timestamp filename\n' > "$output_file"
	# Generate a line for each PNG image in the input directory, sort them
	# because they might have been processed in any order and append them to the
	# output file.
	for file in "$input_dir"/*.png
	do
		[ -f "$file" ] || continue
		file=$(basename "$file")
		timestamp=$(basename "$file" .png)
		printf '%s %s/%s\n' "$timestamp" "$input_dir_name" "$file"
	done | sort -n >> "$output_file"
}

# process_sequence DIRECTORY
# Post-process an ICL-NUIM dataset in the TUM format stored inside DIRECTORY.
process_sequence() {
	dir="$1"
	printf 'Post-processing %s\n' "$dir"
	# Rename the ground truth file to groundtruth.txt. Ignore errors to allow
	# re-processing a processed sequence.
	mv "$dir"/*.gt.freiburg "$dir/groundtruth.txt" 2>/dev/null || true
	# Remove execute permissions from the ground truth file.
	chmod -x "$dir/groundtruth.txt"
	# Remove frames without corresponding ground truth.
	remove_frame_without_gt "$dir" 0
	remove_frame_without_gt "$dir" 1
	# Generate depth.txt and rgb.txt.
	generate_filename_list "$dir/depth"
	generate_filename_list "$dir/rgb"
}

# Usage: optimize_pngs DIRECTORY
# Run optipng on all PNG images in DIRECTORY in parallel.
optimize_pngs() {
	printf 'Optimizing %s\n' "$1"
	find "$1" -type f -name '*.png' | parallel optipng > /dev/null 2>&1
}



# Parse the options.
optimize=0
while getopts 'oh' option
do
	case "$option" in
		o)
			optimize=1
			;;
		h)
			usage
			exit 0
			;;
		*)
			usage >&2
			exit 2
			;;
	esac
done
# Make $1 the first non-option argument.
shift "$((OPTIND - 1))"

# Parse the arguments.
if [ "$#" -lt 1 ]
then
	usage >&2
	exit 2
fi
# Remove trailing slashes.
output_dir="${1%%/}"
shift

# Download all 4 living room sequences by default.
if [ "$#" -eq 0 ]
then
	set -- 0 1 2 3
fi

# Download the living room sequences.
for i in "$@"
do
	url=$(printf 'https://www.doc.ic.ac.uk/~ahanda/living_room_traj%d_frei_png.tar.gz\n' "$i")
	sequence_name="$(basename "$url" '.tar.gz')"
	sequence_dir="$output_dir/$sequence_name"

	if [ -d "$sequence_dir" ]
	then
		printf 'Skipping sequence %s download, directory %s already exists\n' \
			"$sequence_name" "$sequence_dir"
	else
		sequence_archive="$output_dir/$(basename "$url")"
		download_sequence "$url" "$sequence_archive"
		extract_sequence "$sequence_archive" "$sequence_dir"
		rm -f "$sequence_archive"
	fi

	process_sequence "$sequence_dir"

	if [ "$optimize" -eq 1 ]
	then
		optimize_pngs "$sequence_dir"
	fi
done
