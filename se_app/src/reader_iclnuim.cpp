/*
 * SPDX-FileCopyrightText: 2014 University of Edinburgh, Imperial College London, University of Manchester.
 * SPDX-FileCopyrightText: 2020 Smart Robotics Lab, Imperial College London
 * SPDX-FileCopyrightText: 2020 Sotiris Papatheodorou
 * SPDX-License-Identifier: MIT
 * Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1
 */

#include "reader_iclnuim.hpp"

#include <cassert>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <regex>

#include "lodepng.h"

#include "se/image_utils.hpp"
#include "filesystem.hpp"



constexpr float se::ICLNUIMReader::fx_;
constexpr float se::ICLNUIMReader::fy_;
constexpr float se::ICLNUIMReader::cx_;
constexpr float se::ICLNUIMReader::cy_;



se::ICLNUIMReader::ICLNUIMReader(const se::ReaderConfig& c)
    : se::Reader(c) {
  // Ensure a valid directory was provided
  if (!stdfs::is_directory(sequence_path_)) {
    status_ = se::ReaderStatus::error;
    camera_active_ = false;
    camera_open_ = false;
    return;
  }
  // Set the depth and RGBA image resolutions.
  depth_image_res_ = Eigen::Vector2i(640, 480);
  rgba_image_res_  = Eigen::Vector2i(640, 480);
  // Get the total number of frames.
  num_frames_ = numDepthImages(sequence_path_);
}



void se::ICLNUIMReader::restart() {
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



std::string se::ICLNUIMReader::name() const {
  return std::string("ICLNUIMReader");
}



float se::ICLNUIMReader::distanceToDepth(float dist, int x, int y) {
  const float u = (x - se::ICLNUIMReader::cx_) / se::ICLNUIMReader::fx_;
  const float v = (y - se::ICLNUIMReader::cy_) / se::ICLNUIMReader::fy_;
  const float proj = 1.f / std::sqrt(u*u + v*v + 1);
  return proj * dist;
}



se::ReaderStatus se::ICLNUIMReader::nextDepth(se::Image<float>& depth_image) {
  // Generate the filename and open the file for reading.
  std::ostringstream basename;
  basename << "scene_00_" << std::setfill('0') << std::setw(4) << frame_;
  const std::string filename (sequence_path_ + "/" + basename.str() + ".depth");
  std::ifstream fs (filename, std::ios::in);
  if (!fs.good()) {
    return se::ReaderStatus::error;
  }
  // Resize the output image if needed.
  if ((   depth_image.width()  != depth_image_res_.x())
      || (depth_image.height() != depth_image_res_.y())) {
    depth_image = se::Image<float>(depth_image_res_.x(), depth_image_res_.y());
  }
  // Read the image data pixel-by-pixel.
  size_t pixel = 0;
  while (fs.good() && pixel < depth_image.size()) {
    fs >> depth_image[pixel];
    pixel++;
  }
  if (pixel != depth_image.size()) {
    return se::ReaderStatus::error;
  }
  // Convert from Euclidean distance from camera centre to depth.
  for (int y = 0; y < depth_image.height(); ++y) {
    for (int x = 0; x < depth_image.width(); ++x) {
      depth_image(x, y) = distanceToDepth(depth_image(x, y), x, y);
    }
  }
  return se::ReaderStatus::ok;
}



se::ReaderStatus se::ICLNUIMReader::nextRGBA(se::Image<uint32_t>& rgba_image) {
  // Generate the filename.
  std::ostringstream basename;
  basename << "scene_00_" << std::setfill('0') << std::setw(4) << frame_;
  const std::string filename (sequence_path_ + "/" + basename.str() + ".png");
  // Read the image data.
  unsigned w = 0;
  unsigned h = 0;
  unsigned char* image_data = nullptr;
  if (lodepng_decode32_file(&image_data, &w, &h, filename.c_str())) {
    free(image_data);
    return se::ReaderStatus::error;
  }
  assert(rgba_image_res_.x() == static_cast<int>(w));
  assert(rgba_image_res_.y() == static_cast<int>(h));
  // Resize the output image if needed.
  if ((   rgba_image.width()  != rgba_image_res_.x())
      || (rgba_image.height() != rgba_image_res_.y())) {
    rgba_image = se::Image<uint32_t>(rgba_image_res_.x(), rgba_image_res_.y());
  }
  // Copy into the provided image.
  std::memcpy(rgba_image.data(), image_data, w * h * sizeof(uint32_t));
  free(image_data);
  return se::ReaderStatus::ok;
}



size_t se::ICLNUIMReader::numDepthImages(const std::string& dir) const {
  const std::regex depth_image_regex (".*scene_00_[0-9][0-9][0-9][0-9].depth");
  size_t num_depth_images = 0;
  for (const auto& p: stdfs::directory_iterator(dir)) {
    if (!stdfs::is_directory(p.path())) {
      const std::string filename = p.path().string();
      if (std::regex_match(filename, depth_image_regex)) {
        num_depth_images++;
      }
    }
  }
  return num_depth_images;
}

