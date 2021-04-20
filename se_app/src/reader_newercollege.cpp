/*
 * SPDX-FileCopyrightText: 2020 Masha Popovic, Imperial College London
 * SPDX-FileCopyrightText: 2020 Sotiris Papatheodorou, Imperial College London
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "reader_newercollege.hpp"

#include <cassert>
#include <cctype>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <regex>

#include "se/image_utils.hpp"
#include "filesystem.hpp"



Eigen::Vector3f atof3(const std::string& line) {
  Eigen::Vector3f res = Eigen::Vector3f::Zero();
  std::istringstream remaining_line(line);
  std::string s;
  if (std::getline(remaining_line, s, ' ')) {
    res.x() = atof(s.c_str());
  } else {
    // line is empty
    return res;
  }
  if (std::getline(remaining_line, s, ' ')) {
    res.y() = atof(s.c_str());
  } else {
    // line is x
    res.y() = res.x();
    res.z() = res.x();
    return res;
  }
  if (std::getline(remaining_line, s, ' ')) {
    res.z() = atof(s.c_str());
  } else {
    // line is x,y
    res.z() = res.y();
  }
  return res;
}



const int8_t se::NewerCollegeReader::pixel_offset[64] = {
  0, 6, 12, 18, 0, 6, 12, 18, 0, 6, 12, 18, 0, 6, 12, 18,
  0, 6, 12, 18, 0, 6, 12, 18, 0, 6, 12, 18, 0, 6, 12, 18,
  0, 6, 12, 18, 0, 6, 12, 18, 0, 6, 12, 18, 0, 6, 12, 18,
  0, 6, 12, 18, 0, 6, 12, 18, 0, 6, 12, 18, 0, 6, 12, 18};



se::NewerCollegeReader::NewerCollegeReader(const se::ReaderConfig& c)
    : se::Reader(c) {
  // Ensure a valid directory was provided
  if (!stdfs::is_directory(sequence_path_)) {
    status_ = se::ReaderStatus::error;
    camera_active_ = false;
    camera_open_ = false;
    return;
  }
  // Set the depth and RGBA image resolutions.
  depth_image_res_ = Eigen::Vector2i(1024, 64);
  rgba_image_res_  = Eigen::Vector2i(1024, 64);
  // Get the total number of frames.
  num_frames_ = numScans(sequence_path_);
}



void se::NewerCollegeReader::restart() {
  se::Reader::restart();
  if (stdfs::is_directory(sequence_path_)) {
    status_ = se::ReaderStatus::ok;
    camera_active_ = true;
    camera_open_ = true;
  } else {
    status_ = se::ReaderStatus::error;
    camera_active_ = false;
    camera_open_ = false;
  }
}



std::string se::NewerCollegeReader::name() const {
  return std::string("NewerCollegeReader");
}



se::ReaderStatus se::NewerCollegeReader::nextDepth(se::Image<float>& depth_image) {
  // Resize the output image if needed.
  if (   (depth_image.width()  != depth_image_res_.x())
      || (depth_image.height() != depth_image_res_.y())) {
    depth_image = se::Image<float>(depth_image_res_.x(), depth_image_res_.y());
  }
  // Initialize the image to zeros.
  std::memset(depth_image.data(), 0, depth_image_res_.prod() * sizeof(float));
  // Generate the filename and open the file for reading.
  std::ostringstream basename;
  basename << "cloud_" << std::setfill('0') << std::setw(5) << frame_ << ".pcd";
  const std::string filename (sequence_path_ + "/" + basename.str());
  std::ifstream fs (filename, std::ios::in);
  if (!fs.good()) {
    return se::ReaderStatus::error;
  }
  size_t pixel_idx = SIZE_MAX;
  std::vector<float> data (depth_image_res_.prod(), 0);
  // Read the point cloud and convert each point to a distance.
  for (std::string line; std::getline(fs, line); ) {
    // Ignore comment lines
    if (line[0] == '#') {
      continue;
    }
    // Ignore PCL lines
    if (std::isalpha(line[0])) {
      continue;
    }
    // Read the point
    const Eigen::Vector3f point = atof3(line);
    data[++pixel_idx] = point.norm();
  }

  // Reorganize the distances into a depth image.
  for (int y = 0; y < depth_image_res_.y(); ++y) {
    for (int x = 0; x < depth_image_res_.x(); ++x) {
      const int tmp_x = (x + pixel_offset[y]) % depth_image_res_.x();
      const size_t idx = tmp_x * depth_image_res_.y() + y;
      depth_image(x, y) = data[idx];
    }
  }

  return se::ReaderStatus::ok;
}



se::ReaderStatus se::NewerCollegeReader::nextRGBA(se::Image<uint32_t>& rgba_image) {
  // Resize the output image if needed.
  if (   (rgba_image.width()  != rgba_image_res_.x())
      || (rgba_image.height() != rgba_image_res_.y())) {
    rgba_image = se::Image<uint32_t>(rgba_image_res_.x(), rgba_image_res_.y());
  }
  // Create a blank image
  std::memset(rgba_image.data(), 0, rgba_image_res_.prod() * sizeof(uint32_t));
  return se::ReaderStatus::ok;
}



size_t se::NewerCollegeReader::numScans(const std::string& dir) const {
  const std::regex cloud_image_regex (".*cloud_[[:digit:]]{5}.pcd");
  size_t cloud_count = 0;
  for (const auto& p: stdfs::directory_iterator(dir)) {
    if (!stdfs::is_directory(p.path())) {
      const std::string filename = p.path().string();
      if (std::regex_match(filename, cloud_image_regex)) {
        cloud_count++;
      }
    }
  }
  if (cloud_count == 0) {
    std::cerr << "Warning: found no files matching the regular expression "
        << "'cloud_[[:digit:]]{5}.pcd'\n";
  }
  return cloud_count;
}

