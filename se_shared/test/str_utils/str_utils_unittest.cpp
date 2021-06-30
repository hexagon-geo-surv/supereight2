// SPDX-FileCopyrightText: 2020-2021 Smart Robotics Lab, Imperial College London
// SPDX-FileCopyrightText: 2020-2021 Sotiris Papatheodorou, Imperial College London
// SPDX-License-Identifier: BSD-3-Clause

#include <gtest/gtest.h>

#include <se/utils/str_utils.hpp>



const std::vector<std::string> s
    = {"42", "-2", "12.42", ".1", "-3.7", "-0.5f", "0-1", "1.3.", "0xD8", "7A", "abc.", ""};



TEST(StrUtils, isInt) {
  // Create the expected outputs
  const std::vector<bool> r        = {true, true,  false, false, false, false, false, false, false, false, false, false};
  const std::vector<bool> r_nonneg = {true, false, false, false, false, false, false, false, false, false, false, false};
  // Test both modes of operation
  for (size_t i = 0; i < s.size(); ++i)
  {
    EXPECT_EQ(str_utils::is_int(s[i]), r[i]);
    EXPECT_EQ(str_utils::is_int(s[i], false), r_nonneg[i]);
  }
}



TEST(StrUtils, isFloat) {
  // Create the expected outputs
  const std::vector<bool> r        = {true, true,  true, true, true,  false, false, false, false, false, false, false};
  const std::vector<bool> r_nonneg = {true, false, true, true, false, false, false, false, false, false, false, false};
  // Test both modes of operation
  for (size_t i = 0; i < s.size(); ++i)
  {
    EXPECT_EQ(str_utils::is_float(s[i]), r[i]);
    EXPECT_EQ(str_utils::is_float(s[i], false), r_nonneg[i]);
  }
}



TEST(StrUtils, removePrefix)
{
  // Create the expected outputs
  std::vector<std::string> s            = {"foo", "foo", "foofoo"};
  const std::vector<std::string> s_np   = {"foo",    "",    "foo"};
  const std::vector<std::string> prefix = {"lol", "foo",    "foo"};
  // Remove the prefixes
  for (size_t i = 0; i < s.size(); ++i)
  {
    str_utils::remove_prefix(s[i], prefix[i]);
    EXPECT_EQ(s[i], s_np[i]);
  }
}



TEST(StrUtils, removeSuffix)
{
  // Create the expected outputs
  std::vector<std::string> s            = {"foo", "foo", "foofoo"};
  const std::vector<std::string> s_np   = {"foo",    "",    "foo"};
  const std::vector<std::string> suffix = {"lol", "foo",    "foo"};
  // Remove the suffixes
  for (size_t i = 0; i < s.size(); ++i)
  {
    str_utils::remove_suffix(s[i], suffix[i]);
    EXPECT_EQ(s[i], s_np[i]);
  }
}

