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

/** \brief Perform median downsampling on a depth image to avoid introducing new depth values.
 * Introducing new depth values can create artifacts behind object edges. Depth values of 0 or NaN
 * are considered invalid and are ignored when computing the median.
 *
 * \note The ratio between the input and output resolutions must be a power of 2.
 *
 * \param[in]  input_depth_img  The depth image to be downsampled.
 * \param[out] output_depth_img The downsampled depth image. The downsampled image resolution is
 *                              determined by the resolution of the image passed in this parameter.
 * \return A map with the same dimensions as output_depth_img that contains indices to
 * input_depth_img. It can be used to downsample another image (e.g. colour) and select the same
 * pixels as the ones used for depth downsampling.
 */
Image<Eigen::Vector2i> downsample_depth(const Image<float>& input_depth_img,
                                        Image<float>& output_depth_img);

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
