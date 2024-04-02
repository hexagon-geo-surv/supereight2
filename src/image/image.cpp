/*
 * SPDX-FileCopyrightText: 2019-2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2019-2024 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <se/image/image.hpp>

namespace se {
namespace image {

void rgb_to_rgba(const Image<RGB>& rgb, Image<RGBA>& rgba)
{
    assert(rgb.width() == rgba.width());
    assert(rgb.height() == rgba.height());
#pragma omp parallel for
    for (size_t i = 0; i < rgb.size(); ++i) {
        const RGB pixel = rgb[i];
        rgba[i] = RGBA{pixel.r, pixel.g, pixel.b, 0xFF};
    }
}



void rgba_to_rgb(const Image<RGBA>& rgba, Image<RGB>& rgb)
{
    assert(rgba.width() == rgb.width());
    assert(rgba.height() == rgb.height());
#pragma omp parallel for
    for (size_t i = 0; i < rgba.size(); ++i) {
        const RGBA pixel = rgba[i];
        rgb[i] = RGB{pixel.r, pixel.g, pixel.b};
    }
}

} // namespace image
} // namespace se
