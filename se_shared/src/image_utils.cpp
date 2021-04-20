// SPDX-FileCopyrightText: 2019-2020 Sotiris Papatheodorou, Imperial College London
// SPDX-License-Identifier: BSD-3-Clause

#include "se/image_utils.hpp"

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>

#include "lodepng.h"



uint32_t gray_to_rgba(double h) {
  constexpr double v = 0.75;
  double r = 0, g = 0, b = 0;
  if (v > 0) {
    constexpr double m = 0.25;
    constexpr double sv = 0.6667;
    h *= 6.0;
    const int sextant = static_cast<int>(h);
    const double fract = h - sextant;
    const double vsf = v * sv * fract;
    const double mid1 = m + vsf;
    const double mid2 = v - vsf;
    switch (sextant) {
      case 0:
        r = v;
        g = mid1;
        b = m;
        break;
      case 1:
        r = mid2;
        g = v;
        b = m;
        break;
      case 2:
        r = m;
        g = v;
        b = mid1;
        break;
      case 3:
        r = m;
        g = mid2;
        b = v;
        break;
      case 4:
        r = mid1;
        g = m;
        b = v;
        break;
      case 5:
        r = v;
        g = m;
        b = mid2;
        break;
      default:
        r = 0;
        g = 0;
        b = 0;
        break;
    }
  }
  return se::pack_rgba(r * 255, g * 255, b * 255, 255);
}



void se::depth_to_rgba(uint32_t*              depth_RGBA_image_data,
                       const float*           depth_image_data,
                       const Eigen::Vector2i& depth_image_res,
                       const float            min_depth,
                       const float            max_depth) {

  const float range_scale = 1.0f / (max_depth - min_depth);
#pragma omp parallel for
  for (int y = 0; y < depth_image_res.y(); y++) {
    const int row_offset = y * depth_image_res.x();
    for (int x = 0; x < depth_image_res.x(); x++) {
      const int pixel_idx = row_offset + x;
      if (depth_image_data[pixel_idx] < min_depth) {
        depth_RGBA_image_data[pixel_idx] = 0xFFFFFFFF; // White
      } else if (depth_image_data[pixel_idx] > max_depth) {
        depth_RGBA_image_data[pixel_idx] = 0xFF000000; // Black
      } else {
        const float depth_value = (depth_image_data[pixel_idx] - min_depth) * range_scale;
        depth_RGBA_image_data[pixel_idx] = gray_to_rgba(depth_value);
      }
    }
  }
}



int se::save_depth_png(const float*           depth_image_data,
                       const Eigen::Vector2i& depth_image_res,
                       const std::string&     filename,
                       const float            scale) {
  // Scale the depth data and convert to uint16_t
  const size_t num_pixels = depth_image_res.prod();
  std::unique_ptr<uint16_t[]> depth_image_data_scaled (new uint16_t[num_pixels]);
#pragma omp parallel for
  for (size_t i = 0; i < num_pixels; ++i) {
    depth_image_data_scaled.get()[i] = std::roundf(scale * depth_image_data[i]);
  }
  // Save the uint16_t depth image
  return se::save_depth_png(depth_image_data_scaled.get(), depth_image_res, filename);
}



int se::save_depth_png(const uint16_t*        depth_image_data,
                       const Eigen::Vector2i& depth_image_res,
                       const std::string&     filename) {

  // Allocate a new image buffer to use for changing the image data from little
  // endian (used in x86 and ARM CPUs) to big endian order (used in PNG).
  const size_t num_pixels = depth_image_res.prod();
  std::unique_ptr<uint16_t[]> depth_big_endian (new uint16_t[num_pixels]);
#pragma omp parallel for
  for (size_t i = 0; i < num_pixels; ++i) {
    // Swap the byte order.
    const uint16_t depth_value = depth_image_data[i];
    const uint16_t low_byte = depth_value & 0x00FF;
    const uint16_t high_byte = (depth_value & 0xFF00) >> 8;
    depth_big_endian.get()[i] = low_byte << 8 | high_byte;
  }

  // Save the image to file.
  const unsigned ret = lodepng_encode_file(
      filename.c_str(),
      reinterpret_cast<const unsigned char*>(depth_big_endian.get()),
      depth_image_res.x(),
      depth_image_res.y(),
      LCT_GREY,
      16);

  return ret;
}



int se::load_depth_png(float**            depth_image_data,
                       Eigen::Vector2i&   depth_image_res,
                       const std::string& filename,
                       const float        inverse_scale) {
  // Load the 16-bit depth data
  uint16_t* depth_image_data_scaled = nullptr;
  const int err = se::load_depth_png(&depth_image_data_scaled, depth_image_res,
      filename);
  // Return on error
  if (err) {
    free(depth_image_data_scaled);
    return err;
  }
  // Remove the scaling from the 16-bit depth data and convert to float
  const size_t num_pixels = depth_image_res.prod();
  *depth_image_data = static_cast<float*>(malloc(num_pixels * sizeof(float)));
#pragma omp parallel for
  for (size_t i = 0; i < num_pixels; ++i) {
    (*depth_image_data)[i] = inverse_scale * depth_image_data_scaled[i];
  }
  free(depth_image_data_scaled);
  return 0;
}



int se::load_depth_png(uint16_t**         depth_image_data,
                       Eigen::Vector2i&   depth_image_res,
                       const std::string& filename) {

  // Load the image.
  const unsigned ret = lodepng_decode_file(
      reinterpret_cast<unsigned char**>(depth_image_data),
      reinterpret_cast<unsigned int*>(&(depth_image_res.x())),
      reinterpret_cast<unsigned int*>(&(depth_image_res.y())),
      filename.c_str(),
      LCT_GREY,
      16);

  // Change the image data from little endian (used in x86 and ARM CPUs) to big
  // endian order (used in PNG).
  const size_t num_pixels = depth_image_res.prod();
#pragma omp parallel for
  for (size_t i = 0; i < num_pixels; ++i) {
    // Swap the byte order.
    const uint16_t depth_value = (*depth_image_data)[i];
    const uint16_t low_byte = depth_value & 0x00FF;
    const uint16_t high_byte = (depth_value & 0xFF00) >> 8;
    (*depth_image_data)[i] = low_byte << 8 | high_byte;
  }

  return ret;
}



int se::save_depth_pgm(const float*           depth_image_data,
                       const Eigen::Vector2i& depth_image_res,
                       const std::string&     filename,
                       const float            scale) {
  // Scale the depth data and convert to uint16_t
  const size_t num_pixels = depth_image_res.prod();
  std::unique_ptr<uint16_t[]> depth_image_data_scaled (new uint16_t[num_pixels]);
#pragma omp parallel for
  for (size_t i = 0; i < num_pixels; ++i) {
    depth_image_data_scaled.get()[i] = std::roundf(scale * depth_image_data[i]);
  }
  // Save the uint16_t depth image
  return se::save_depth_pgm(depth_image_data_scaled.get(), depth_image_res, filename);
}



int se::save_depth_pgm(const uint16_t*        depth_image_data,
                       const Eigen::Vector2i& depth_image_res,
                       const std::string&     filename) {

  // Open the file for writing.
  std::ofstream file (filename.c_str());
  if (!file.is_open()) {
    std::cerr << "Unable to write file " << filename << "\n";
    return 1;
  }

  // Write the PGM header.
  file << "P2\n";
  file << depth_image_res.x() << " " << depth_image_res.y() << "\n";
  file << UINT16_MAX << "\n";

  // Write the image data.
  for (int y = 0; y < depth_image_res.y(); y++) {
    for (int x = 0; x < depth_image_res.x(); x++) {
      const int pixel_idx = x + y * depth_image_res.x();
      file << depth_image_data[pixel_idx];
      // Do not add a whitespace after the last element of a row.
      if (x < depth_image_res.x() - 1) {
        file << " ";
      }
    }
    // Add a newline at the end of each row.
    file << "\n";
  }

  file.close();

  return 0;
}



int se::load_depth_pgm(float**            depth_image_data,
                       Eigen::Vector2i&   depth_image_res,
                       const std::string& filename,
                       const float        inverse_scale) {
  // Load the 16-bit depth data
  uint16_t* depth_image_data_scaled = nullptr;
  const int err = se::load_depth_pgm(&depth_image_data_scaled, depth_image_res,
      filename);
  // Return on error
  if (err) {
    free(depth_image_data_scaled);
    return err;
  }
  // Remove the scaling from the 16-bit depth data and convert to float
  const size_t num_pixels = depth_image_res.prod();
  *depth_image_data = static_cast<float*>(malloc(num_pixels * sizeof(float)));
#pragma omp parallel for
  for (size_t i = 0; i < num_pixels; ++i) {
    (*depth_image_data)[i] = inverse_scale * depth_image_data_scaled[i];
  }
  free(depth_image_data_scaled);
  return 0;
}



int se::load_depth_pgm(uint16_t**         depth_image_data,
                       Eigen::Vector2i&   depth_image_res,
                       const std::string& filename) {

  // Open the file for reading.
  std::ifstream file (filename.c_str());
  if (!file.is_open()) {
    std::cerr << "Unable to read file " << filename << "\n";
    return 1;
  }

  // Read the file format.
  std::string pgm_format;
  std::getline(file, pgm_format);
  if (pgm_format != "P2") {
    std::cerr << "Invalid PGM format: " << pgm_format << "\n";
    return 1;
  }

  // Read the image size and allocate memory for the image.
  file >> depth_image_res.x() >> depth_image_res.y();
  const size_t num_pixels = depth_image_res.x() * depth_image_res.y();
  *depth_image_data = static_cast<uint16_t*>(malloc(num_pixels  * sizeof(uint16_t)));

  // Read the maximum pixel value.
  size_t max_value;
  file >> max_value;
  if (max_value > UINT16_MAX) {
    std::cerr << "Invalid maximum depth value " << max_value
        << " > " << UINT16_MAX << "\n";
    return 1;
  }

  // Read the image data. Do not perform any scaling since in our cases the
  // pixel values represent distances.
  for (size_t pixel_idx = 0; pixel_idx < num_pixels; ++pixel_idx) {
    file >> (*depth_image_data)[pixel_idx];
  }

  file.close();

  return 0;
}

