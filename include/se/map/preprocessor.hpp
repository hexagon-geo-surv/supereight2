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

#include "se/image/image.hpp"

namespace se {
namespace preprocessor {

void downsample_depth(se::Image<float>& input_depth_img,
                      se::Image<float>& output_depth_img);

void downsample_rgba(se::Image<uint32_t>& input_RGBA_img,
                     se::Image<uint32_t>& output_RGBA_img);

void bilateral_filter(se::Image<float>&         out,
                      const se::Image<float>&   in,
                      const std::vector<float>& gaussian,
                      const float               e_d,
                      const int                 radius);


template<typename SensorT>
void depth_to_point_cloud(se::Image<Eigen::Vector3f>& point_cloud_C,
                          const se::Image<float>&     depth_image,
                          const SensorT&              sensor)
{
#pragma omp parallel for
  for (int y = 0; y < depth_image.height(); y++)
  {
    for (int x = 0; x < depth_image.width(); x++)
    {
      const Eigen::Vector2i pixel(x, y);
      if (depth_image(pixel.x(), pixel.y()) > 0)
      {
        const Eigen::Vector2f pixel_f = pixel.cast<float>();
        Eigen::Vector3f ray_dir_C;
        sensor.model.backProject(pixel_f, &ray_dir_C);
        point_cloud_C[pixel.x() + pixel.y() * depth_image.width()] = depth_image(pixel.x(), pixel.y()) * ray_dir_C;
      } else
      {
        point_cloud_C[pixel.x() + pixel.y() * depth_image.width()] = Eigen::Vector3f::Zero();
      }
    }
  }
}

void point_cloud_to_depth(se::Image<float>&                 depth_image,
                          const se::Image<Eigen::Vector3f>& point_cloud_X,
                          const Eigen::Matrix4f&            T_CX);

/**
* NegY should only be true when reading an ICL-NUIM dataset which has a
* left-handed coordinate system (the y focal length will be negative).
*/
template <bool NegY>
void point_cloud_to_normal(se::Image<Eigen::Vector3f>&       out,
                           const se::Image<Eigen::Vector3f>& in);

/**
* Downsample the input depth to match the resolution of the output depth.
* The ration between the resolutions must be a power of 2. Median downsampling
* is used to prevent creating new depth which create artifacts behind object
* edges. Depth values of 0 are considered invalid and are ignored when
* computing the median.
*/
void downsample_depth(const float*           input_depth,
                      const Eigen::Vector2i& input_res,
                      se::Image<float>&      output_depth);

void half_sample_robust_image(se::Image<float>&       out,
                              const se::Image<float>& in,
                              const float             e_d,
                              const int               r);

/**
* Downsample an RGBA image and copy into an se::Image class.
*
* \param[in] input_RGBA Pointer to the RGBA image data, 4 channels, 8 bits
* per channel.
* \param[in] input_res Size of the RGBA image in pixels (width and height).
* \param[out] output_RGB Object to store the output image to. The output image
* dimensions must be an integer multiple of the input image dimensions. The
* data for each pixel is stored in ARGB order, with the alpha channel in the
* MSB of the uint32_t and the red channel in the LSB of the uint32_t.
*/
void downsample_image(const uint32_t*        input_RGBA,
                      const Eigen::Vector2i& input_res,
                      se::Image<uint32_t>&   output_RGBA);

} // namespace preprocessor
} // namespace se

#endif // SE_PREPROCESSOR_HPP

