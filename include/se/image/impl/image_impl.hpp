/*
 * SPDX-FileCopyrightText: 2023-2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2023-2024 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_IMAGE_IMPL_HPP
#define SE_IMAGE_IMPL_HPP

namespace se {
namespace image {

template<typename T>
void remap(const Image<T>& input, Image<T>& output, const Image<size_t>& map)
{
    assert((output.width() == map.width()) && "The output and map have the same width");
    assert((output.height() == map.height()) && "The output and map have the same height");
#pragma omp parallel for
    for (size_t i = 0; i < output.size(); i++) {
        assert((map[i] < input.size()) && "The map must contain valid indices into the input");
        output[i] = input[map[i]];
    }
}

} // namespace image
} // namespace se

#endif // SE_IMAGE_IMPL_HPP
