#!/bin/sh
# SPDX-FileCopyrightText: 2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-FileCopyrightText: 2024 Sotiris Papatheodorou
# SPDX-License-Identifier: BSD-3-Clause

# Use ANSI escape sequences to print the supereight2 scale colours on the
# terminal by parsing the source code. Assumes a reasonable formatting of the
# code.

awk '
/^namespace colours \{$/, /^}( \/\/ namespace colours)?$/ {
	if ($0 ~ "\\{ *[0-9]+, *[0-9]+, *[0-9]+ *}") {
		sub("^.*\\{", "\033[48;2;")
		gsub(", *", ";")
		sub("}.*$", "m          \033[m scale " scale++)
		print
	}
}
' "$(dirname "$0")/../include/se/common/colour_utils.hpp"
