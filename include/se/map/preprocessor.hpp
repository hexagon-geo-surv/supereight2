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

#include <Eigen/Geometry>
#include <se/image/image.hpp>

namespace se {
namespace preprocessor {

/** Perform median downsampling on \p input_depth_img and save the result in \p output_depth_img.
 * Return an image with the same dimensions as \p output_depth_img containing linear indices to \p
 * input_depth_img. It can be used with se::image::remap() to downsample another image (e.g. colour)
 * and select the same pixels as the ones used for depth downsampling.
 *
 * Median downsampling avoids introducing new, non-measured depth values, which can create artifacts
 * near object edges. The median of an even number of depth values is computed as the smallest of
 * the two middle depth values instead of their mean which would introduce a new, non-measured depth
 * value. Depth values of 0 or NaN are considered invalid and are ignored when computing the median.
 *
 * \note The ratio between the input and output image resolutions must be a power of two.
 */
Image<size_t> downsample_depth(const Image<float>& input_depth_img, Image<float>& output_depth_img);

template<typename SensorT>
void depth_to_point_cloud(se::Image<Eigen::Vector3f>& point_cloud_C,
                          const se::Image<float>& depth_image,
                          const SensorT& sensor);

void point_cloud_to_depth(se::Image<float>& depth_image,
                          const se::Image<Eigen::Vector3f>& point_cloud_X,
                          const Eigen::Isometry3f& T_CX);

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
