// SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab
// SPDX-FileCopyrightText: 2019-2021 Sotiris Papatheodorou, Imperial College London
// SPDX-License-Identifier: BSD-3-Clause

#include "se/utils/colour_utils.hpp"



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

