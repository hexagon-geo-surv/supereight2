#!/bin/sh
# SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab, Imperial College London
# SPDX-FileCopyrightText: 2019-2021 Sotiris Papatheodorou, Imperial College London
# SPDX-License-Identifier: BSD-3-Clause

set -eu
IFS="$(printf '%b_' '\t\n')"; IFS="${IFS%_}"

# Show the program usage on standard output.
usage() {
	printf "Usage: %s [OPTION]... [DIRECTORY]\n" "$(basename "$0")"
	printf "Download the ICL-NUIM dataset in the TUM format in DIRECTORY.\n"
	printf "If DIRECTORY is not provided the current directory wil be used.\n"
	# 1. Search the current file for case labels with comments.
	# 2. Remove case syntax.
	# 3. Add leading dash.
	grep -E '[[:space:]].) #' "$0" | sed -e 's/) #/ /g' -e 's/^[ \t]*/    -/g'
}

generate_filename_list() {
	input_dir="$1"
	output_file="$2"
	input_dir_name=$(basename "$input_dir")
	# Write the file header in the output file.
	printf "# timestamp filename\n" > "$output_file"
	# Loop over all PNG images in the input directory.
	for file in "$input_dir"/*.png ; do
		[ -f "$file" ] || continue
		file=$(basename "$file")
		filename_no_ext=$(basename "$file" .png)
		# Print the entry for this file.
		printf "%s %s/%s\n" "$filename_no_ext" "$input_dir_name" "$file"
	# Sort the entries and append to the output file.
	done | sort -n >> "$output_file"
}

post_process_tum() {
	dir="$1"
	# Rename the ground truth file to groundtruth.txt.
	mv "$dir"/*.gt.freiburg "$dir/groundtruth.txt"
	# Remove execute permissions from the ground truth file.
	chmod -x "$dir/groundtruth.txt"
	# Remove frame 0 because it has no corresponding ground truth pose.
	rm -f "$dir/rgb/0.png"
	rm -f "$dir/depth/0.png"
	# Remove it from the association file too.
	sed -i '/^0 depth\/0\.png 0 rgb\/0\.png$/d' "$dir/associations.txt"
	# Remove frame 1 if it has no corresponding ground truth pose. This is only
	# needed for traj0_frei_png.
	if ! grep -q '^1 ' "$dir/groundtruth.txt"; then
		rm -f "$dir/rgb/1.png"
		rm -f "$dir/depth/1.png"
		# Remove it from the association file too.
		sed -i '/^1 depth\/1\.png 1 rgb\/1\.png$/d' "$dir/associations.txt"
	fi
	# Generate rgb.txt and depth.txt.
	generate_filename_list "$dir/rgb" "$dir/rgb.txt"
	generate_filename_list "$dir/depth" "$dir/depth.txt"
}

post_process_iclnuim() {
	dir="$1"
	# Fix the file permissions.
	chmod 644 "$dir"/*
}



sequence_urls_tum='https://www.doc.ic.ac.uk/~ahanda/living_room_traj0_frei_png.tar.gz
https://www.doc.ic.ac.uk/~ahanda/living_room_traj1_frei_png.tar.gz
https://www.doc.ic.ac.uk/~ahanda/living_room_traj2_frei_png.tar.gz
https://www.doc.ic.ac.uk/~ahanda/living_room_traj3_frei_png.tar.gz'

sequence_urls_iclnuim='https://www.doc.ic.ac.uk/~ahanda/living_room_traj0_loop.tgz
https://www.doc.ic.ac.uk/~ahanda/living_room_traj1_loop.tgz
https://www.doc.ic.ac.uk/~ahanda/living_room_traj2_loop.tgz
https://www.doc.ic.ac.uk/~ahanda/living_room_traj3_loop.tgz'



# Parse the options.
sequence_urls="$sequence_urls_tum"
postprocess=post_process_tum
while getopts 'ih' opt_name ; do
	case "$opt_name" in
		i) # Download the datasets in the ICL-NUIM instead of the TUM format.
			sequence_urls="$sequence_urls_iclnuim"
			postprocess=post_process_iclnuim
			;;
		h) # Display this help message and exit.
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

# Parse the arguments.
case "$#" in
	0)
		output_dir='.'
		;;
	1)
		# Remove trailing slashes.
		output_dir="${1%%/}"
		;;
	*)
		usage
		exit 2
esac

# Create the output directory.
mkdir -p "$output_dir"
# The file where wget output is logged.
log_file="$output_dir/$(basename "$0").log"
# Clean up the log from any previous invocation.
rm -f "$log_file"

# Download each sequence.
for url in $sequence_urls; do
	filename="$output_dir/$(basename "$url")"
	sequence_name="$(basename "$url")"
	sequence_name="${sequence_name%%.*}"
	sequence_dir="$output_dir/$sequence_name"

	# Skip the sequence if its directory already exists.
	if [ -d "$sequence_dir" ] ; then
		printf "Skipping sequence %s, directory %s already exists\n" \
			"$sequence_name" "$sequence_dir"
		continue
	fi

	# Download the sequence.
	printf "Downloading %s to %s\n" "$url" "$filename"
	wget --no-verbose --append-output "$log_file" --continue \
		--output-document "$filename" "$url"

	# Extract the sequence into its own directory.
	printf "Extracting %s into %s\n" "$filename" "$sequence_dir"
	mkdir -p "$sequence_dir"
	tar -xzf "$filename" -C "$sequence_dir"

	# Post-process the sequence.
	printf "Post-processing %s\n" "$sequence_dir"
	$postprocess "$sequence_dir"

	# Remove the downloaded file.
	rm "$filename"
done

