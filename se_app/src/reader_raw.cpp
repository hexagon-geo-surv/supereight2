/*
 * SPDX-FileCopyrightText: 2014 University of Edinburgh, Imperial College London, University of Manchester.
 * SPDX-FileCopyrightText: 2020 Smart Robotics Lab, Imperial College London
 * SPDX-FileCopyrightText: 2020 Sotiris Papatheodorou
 * SPDX-License-Identifier: MIT
 * Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1
 */

#include "reader_raw.hpp"

#include <iostream>

#include "se/image_utils.hpp"
#include "filesystem.hpp"



constexpr size_t se::RAWReader::depth_pixel_size_;
constexpr size_t se::RAWReader::rgba_pixel_size_;
constexpr size_t se::RAWReader::res_size_;

se::RAWReader::RAWReader(const se::ReaderConfig& c)
    : se::Reader(c) {
  // Open the .raw file for reading.
  raw_fs_.open(sequence_path_, std::ios::in | std::ios::binary);
  if (!raw_fs_.good()) {
    std::cerr << "Error: Could not read raw file " << sequence_path_ << "\n";
    status_ = se::ReaderStatus::error;
    camera_active_ = false;
    camera_open_ = false;
    return;
  }
  // Read the resolution of the first depth image.
  raw_fs_.seekg(0);
  if (!readResolution(raw_fs_, depth_image_res_)) {
    std::cerr << "Error: Could not read depth image size\n";
    status_ = se::ReaderStatus::error;
    camera_active_ = false;
    camera_open_ = false;
    return;
  }
  // Pre-compute depth image sizes.
  depth_image_total_ = depth_image_res_.prod();
  depth_data_size_ = depth_pixel_size_ * depth_image_total_;
  depth_total_size_ = res_size_ + depth_data_size_;
  // Read the resolution of the first RGBA image.
  raw_fs_.seekg(depth_total_size_);
  if (!readResolution(raw_fs_, rgba_image_res_)) {
    std::cerr << "Error: Could not read RGBA image size\n";
    status_ = se::ReaderStatus::error;
    camera_active_ = false;
    camera_open_ = false;
    return;
  }
  // Pre-compute depth image sizes.
  rgba_image_total_ = rgba_image_res_.prod();
  rgba_data_size_ = rgba_pixel_size_ * rgba_image_total_;
  rgba_total_size_ = res_size_ + rgba_data_size_;
  // Start reading from the beginning of the .raw file again.
  raw_fs_.seekg(0);
  // Compute the total number of frames from the size of the .raw file.
  const uintmax_t raw_size = stdfs::file_size(sequence_path_);
  num_frames_ = raw_size / (depth_total_size_ + rgba_total_size_);
}



void se::RAWReader::restart() {
  se::Reader::restart();
  if (raw_fs_.good() || raw_fs_.eof()) {
    raw_fs_.clear();
    raw_fs_.seekg(0);
    if (raw_fs_.good()) {
      status_ = se::ReaderStatus::ok;
      camera_active_ = true;
      camera_open_ = true;
    } else {
      status_ = se::ReaderStatus::error;
      camera_active_ = false;
      camera_open_ = false;
    }
  }
}



std::string se::RAWReader::name() const {
  return std::string("RAWReader");
}



bool se::RAWReader::readResolution(std::ifstream& fs, Eigen::Vector2i& res) {
  uint32_t w;
  if (!fs.read(reinterpret_cast<char*>(&w), sizeof(uint32_t))) {
    return false;
  }
  uint32_t h;
  if (!fs.read(reinterpret_cast<char*>(&h), sizeof(uint32_t))) {
    return false;
  }
  res = Eigen::Vector2i(w, h);
  return true;
}



se::ReaderStatus se::RAWReader::nextDepth(se::Image<float>& depth_image) {
  // Seek to the appropriate place in the file.
  raw_fs_.seekg(frame_ * (depth_total_size_ + rgba_total_size_));
  // Read the image dimensions.
  Eigen::Vector2i size;
  if (!readResolution(raw_fs_, size)) {
    return se::ReaderStatus::error;
  }
  // Resize the output image if needed.
  if ((depth_image.width() != size.x()) || (depth_image.height() != size.y())) {
    depth_image = se::Image<float>(size.x(), size.y());
  }
  // Read the image data pixel-by-pixel.
  for (size_t p = 0; p < depth_image.size(); ++p) {
    uint16_t pixel;
    if (!raw_fs_.read(reinterpret_cast<char*>(&pixel), depth_pixel_size_)) {
      return se::ReaderStatus::error;
    }
    // Scale the data from millimeters to meters.
    depth_image[p] = pixel / 1000.f;
  }
  return se::ReaderStatus::ok;
}



se::ReaderStatus se::RAWReader::nextRGBA(se::Image<uint32_t>& rgba_image) {
  // Seek to the appropriate place in the file.
  raw_fs_.seekg(frame_ * (depth_total_size_ + rgba_total_size_)
      + depth_total_size_);
  // Read the image dimensions.
  Eigen::Vector2i size;
  if (!readResolution(raw_fs_, size)) {
    return se::ReaderStatus::error;
  }
  // Resize the output image if needed.
  if ((rgba_image.width() != size.x()) || (rgba_image.height() != size.y())) {
    rgba_image = se::Image<uint32_t>(size.x(), size.y());
  }
  // Read the image data pixel-by-pixel.
  for (size_t p = 0; p < rgba_image.size(); ++p) {
    uint8_t r;
    if (!raw_fs_.read(reinterpret_cast<char*>(&r), sizeof(uint8_t))) {
      return se::ReaderStatus::error;
    }
    uint8_t g;
    if (!raw_fs_.read(reinterpret_cast<char*>(&g), sizeof(uint8_t))) {
      return se::ReaderStatus::error;
    }
    uint8_t b;
    if (!raw_fs_.read(reinterpret_cast<char*>(&b), sizeof(uint8_t))) {
      return se::ReaderStatus::error;
    }
    rgba_image[p] = se::pack_rgba(r, g, b, 0xFF);
  }
  return se::ReaderStatus::ok;
}

