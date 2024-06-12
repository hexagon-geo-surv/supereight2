/*
 * SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2019-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "se/common/colour_utils.hpp"

#include <se/external/tinycolormap.hpp>

void se::depth_to_rgba(se::RGBA* depth_RGBA_image_data,
                       const float* depth_image_data,
                       const Eigen::Vector2i& depth_image_res,
                       const float min_depth,
                       const float max_depth)
{
    const float inv_depth_range = 1.0f / (max_depth - min_depth);
#pragma omp parallel for
    for (int y = 0; y < depth_image_res.y(); y++) {
        const int row_offset = y * depth_image_res.x();
        for (int x = 0; x < depth_image_res.x(); x++) {
            const int pixel_idx = x + row_offset;
            const float depth = depth_image_data[pixel_idx];
            if (depth <= 0.0f || std::isnan(depth)) {
                depth_RGBA_image_data[pixel_idx] = {0x00, 0x00, 0x00, 0xFF}; // Black
            }
            else if (depth < min_depth) {
                depth_RGBA_image_data[pixel_idx] = {0x80, 0x80, 0x80, 0xFF}; // Gray
            }
            else if (depth > max_depth) {
                depth_RGBA_image_data[pixel_idx] = {0xFF, 0xFF, 0xFF, 0xFF}; // White
            }
            else {
                const float normalized_depth = (depth - min_depth) * inv_depth_range;
                const tinycolormap::Color c = tinycolormap::GetColor(
                    1.0f - normalized_depth, tinycolormap::ColormapType::Heat);
                depth_RGBA_image_data[pixel_idx] = {c.ri(), c.gi(), c.bi()};
            }
        }
    }
}
