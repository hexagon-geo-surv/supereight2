/*
 * SPDX-FileCopyrightText: 2014 University of Edinburgh, Imperial College, University of Manchester
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2019-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: MIT
 */

#ifndef SE_PREPROCESSOR_HPP
#define SE_PREPROCESSOR_HPP

#include "se/common/colour_types.hpp"
#include "se/image/image.hpp"

namespace se {
namespace preprocessor {

/**
 * Downsample the input depth to match the resolution of the output depth. The ration between the
 * resolutions must be a power of 2. Median downsampling is used to prevent creating new depth
 * values which create artifacts behind object edges. Depth values of 0 are considered invalid and
 * are ignored when computing the median.
 */
void downsample_depth(se::Image<float>& input_depth_img, se::Image<float>& output_depth_img);

/**
 * Downsample a colour image and copy into an se::Image class.
 *
 * \param[in] input_colour_img   The image to downsample.
 * \param[out] output_colour_img The image where the downsampled data will be saved to. The
 *                               dimensions of this image determine the output resolution. The
 *                               output image dimensions must be an integer multiple of the input
 *                               image dimensions.
 */
void downsample_colour(Image<rgb_t>& input_colour_img, Image<rgb_t>& output_colour_img);

template<typename SensorT>
void depth_to_point_cloud(se::Image<Eigen::Vector3f>& point_cloud_C,
                          const se::Image<float>& depth_image,
                          const SensorT& sensor);

void point_cloud_to_depth(se::Image<float>& depth_image,
                          const se::Image<Eigen::Vector3f>& point_cloud_X,
                          const Eigen::Matrix4f& T_CX);

/**
 * NegY should only be true when reading an ICL-NUIM dataset which has a
 * left-handed coordinate system (the y focal length will be negative).
 */
template<bool NegY>
void point_cloud_to_normal(se::Image<Eigen::Vector3f>& out, const se::Image<Eigen::Vector3f>& in);

void half_sample_robust_image(se::Image<float>& out,
                              const se::Image<float>& in,
                              const float e_d,
                              const int r);

} // namespace preprocessor
} // namespace se

#include "impl/preprocessor_impl.hpp"

#endif // SE_PREPROCESSOR_HPP
