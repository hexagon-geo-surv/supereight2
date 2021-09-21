/*
 * SPDX-FileCopyrightText: 2020 Smart Robotics Lab, Imperial College London
 * SPDX-FileCopyrightText: 2020 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

// Detect std::filesystem support, include the appropriate header and alias the
// namespace.

#ifndef __FILESYSTEM_HPP
#define __FILESYSTEM_HPP

#if        (defined(__GNUC__)        && __GNUC__        >= 8) \
        || (defined(__clang_major__) && __clang_major__ >= 7) \
        || (defined(_MSC_VER)        && _MSC_VER        >= 1914)
// Proper std::filesystem support.
#include <filesystem>
namespace stdfs = std::filesystem;

#elif      (defined(__GNUC__)        && __GNUC__        >= 6) \
        || (defined(__clang_major__) && __clang_major__ >= 6)
// Experimental std::filesystem support.
#include <experimental/filesystem>
namespace stdfs = std::experimental::filesystem;

#else
// No std::filesystem support.
#error A compiler with support for std::filesystem is required
#endif

#endif // __FILESYSTEM_HPP

