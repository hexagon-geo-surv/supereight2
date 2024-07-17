#!/bin/bash
# SPDX-FileCopyrightText: 2023-2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-FileCopyrightText: 2023-2024 Sotiris Papatheodorou
# SPDX-License-Identifier: CC0-1.0
set -euo pipefail

# Usage: cc_to_cxx CC
# Given a C compiler name CC print the corresponding C++ compiler name.
cc_to_cxx() {
	case "$1" in
		cc)
			printf 'c++\n'
			;;
		gcc)
			printf 'g++\n'
			;;
		clang)
			printf 'clang++\n'
			;;
		*)
			printf '%s\n' "$1"
	esac
}

# Usage: ok_or_fail
# Print [OK] or [FAIL] depending on the status code of the previous command.
ok_or_fail() {
	# shellcheck disable=SC2181
	if [ "$?" -eq 0 ]
	then
		printf '[OK]\n'
	else
		printf '[FAIL]\n'
	fi
}

# Usage: enable_sanitizer CMAKELISTS ASAN|UBSAN
# Append lines enabling the address (ASAN) or undefined behavior (UBSAN)
# sanitizer on the supplied CMakeLists.txt.
enable_sanitizer() {
	case "$2" in
		UBSAN)
			sanitizer='undefined'
			;;
		*)
			sanitizer='address'
	esac
	cat <<-EOF | ed -s "$1"
	/project(/a
	add_compile_options(-fsanitize=${sanitizer} -fno-omit-frame-pointer)
	add_link_options(-fsanitize=${sanitizer} -fno-omit-frame-pointer)
	.
	w
	EOF
}

# Usage: disable_sanitizers CMAKELISTS
# Remove lines enabling the address or undefined behavior sanitizers.
disable_sanitizers() {
	cat <<-EOF | ed -s "$1"
	g/^add_.*_options(-fsanitize=/d
	w
	EOF
}

# Usage: test_license OUTPUT_DIR
# Test for license issues.
test_license() {
	printf 'reuse lint\t'
	{
		reuse --version
		time -p reuse lint
	} >"$1/reuse-lint.log" 2>&1
	ok_or_fail
}

# Usage: test_formatting OUTPUT_DIR
# Run clang-format and test for modified files.
test_formatting() {
	printf 'clang-format\t'
	{
		clang-format --version
		# shellcheck disable=SC2046
		time -p clang-format --style=file --dry-run --Werror \
			$(git ls-files '*.[ch]pp')
	} >"$1/clang-format.log" 2>&1
	ok_or_fail
}

# Usage: test_documentation OUTPUT_DIR
# Build the documentation.
test_documentation() {
	printf 'doxygen\t'
	{
		doxygen --version
		time -p doxygen
	} >"$1/doxygen.log" 2>&1
	ok_or_fail
}

# Usage: test_static_analyzer OUTPUT_DIR
# Build using the clang static analyzer and test for issues.
test_static_analyzer() {
	printf 'static analyzer\t'
	# Use a sub-shell so that changing the current working directory or
	# modifying shell options doesn't affect the parent shell.
	(
		set -e
		se2_dir=$(pwd)
		# Build in a unique temporary directory so that multiple jobs
		# can run at the same time.
		build_dir=$(mktemp -d)
		# Remove the build directory once this sub-shell exits.
		trap 'rm -rf "$build_dir"' EXIT
		cd "$build_dir" || exit
		{
			time -p cmake -DCMAKE_BUILD_TYPE=Release "$se2_dir"
			time -p scan-build --status-bugs cmake --build .
		} >"$1/static_analyzer.log" 2>&1
	)
	ok_or_fail
}

# Usage: test_sanitizer OUTPUT_DIR ASAN|UBSAN
# Build using the address (ASAN) or undefined behavior (UBSAN) sanitizer and run
# the unit tests to detect issues.
test_sanitizer() {
	printf '%s\t' "$2"
	(
		set -e
		se2_dir=$(pwd)
		enable_sanitizer 'CMakeLists.txt' "$2"
		build_dir=$(mktemp -d)
		trap 'rm -rf "$build_dir"' EXIT
		cd "$build_dir" || exit
		{
			time -p cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo "$se2_dir"
			# Limit the number of parallel builds to avoid running out of
			# memory.
			time -p cmake --build . --parallel 2
		} >"$1/$2_build.log" 2>&1
		{
			time -p ctest --output-on-failure
		} >"$1/$2_test.log" 2>&1
	)
	ok_or_fail
	# Undo any changes made to CMakeLists.txt.
	disable_sanitizers 'CMakeLists.txt'
}

# Usage: test_build OUTPUT_DIR CC BUILD RUN_TESTS
# Build the project with C compiler CC in build type BUILD and run its test
# suite if RUN_TESTS is non-zero.
test_build() {
	printf '%s build with %s\t' "$3" "$2"
	(
		set -e
		CXX=$(cc_to_cxx "$2")
		export CC="$2"
		export CXX
		build=$3
		se2_dir=$(pwd)
		build_dir=$(mktemp -d)
		trap 'rm -rf "$build_dir"' EXIT
		cd "$build_dir" || exit
		{
			time -p cmake -DCMAKE_BUILD_TYPE="$build" "$se2_dir"
			time -p cmake --build .
		} >"$1/${CC}_${build}_build.log" 2>&1
		if [ "$4" -ne 0 ]
		then
			{
				time -p ctest --output-on-failure
			} >"$1/${CC}_${build}_test.log" 2>&1
		fi
	)
	ok_or_fail
}

# Usage: test_run JOB_DIR
test_run() {
	# Don't exit on errors anymore since we want to continue if one test fails.
	set +eo pipefail
	mkdir -p "$1"
	# Disable any sanitizers left from a previous failed run.
	disable_sanitizers 'CMakeLists.txt'
	time -p {
		test_license "$1"
		test_formatting "$1"
		test_documentation "$1"
		test_static_analyzer "$1"
		test_sanitizer "$1" ASAN
		test_sanitizer "$1" UBSAN

		# Build with various compilers and run the tests.
		for compiler in gcc clang
		do
			for build in Release Debug
			do
				# Run the tests built both in Release and Debug to
				# detect different kinds of issues. No need to run them
				# for more than one compiler.
				run_tests=0
				if [ "$compiler" = 'gcc' ]
				then
					run_tests=1
				fi
				test_build "$1" "$compiler" "$build" "$run_tests"
			done
		done
	} | column -t -s '	'
}



if [ "$#" -ne 1 ]
then
	printf 'Usage: %s OUTPUT_DIR\n' "${0##*/}" >&2
	exit 2
fi

# Ensure an absolute path is used so that it's consistent after changing
# directory inside the tests.
output_dir=$(realpath "$1")

# Assuming the script is located inside the supereight2 repository.
cd "$(dirname "$0")" || exit
supereight_repo_root=$(git rev-parse --show-toplevel)
# All tests are designed to be run from the supereight2 repository root.
cd "$supereight_repo_root" || exit

# Print useful information.
git --no-pager log --date=format:'%Y-%m-%d' -n 1 \
	--pretty=format:'supereight2 commit %h %ad %s%n' HEAD
printf 'Results stored in %s\n' "$output_dir"
lsb_release -d | cut -f 2-
printf 'CMAKE_BUILD_PARALLEL_LEVEL=%s\n' "${CMAKE_BUILD_PARALLEL_LEVEL:-}"
printf 'MAKEFLAGS=%s\n' "${MAKEFLAGS:-}"
printf '\n'

# Run all tests.
test_run "$output_dir"
