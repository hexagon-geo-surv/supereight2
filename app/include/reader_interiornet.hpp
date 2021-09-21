// SPDX-FileCopyrightText: 2020-2021 Smart Robotics Lab, Imperial College London
// SPDX-FileCopyrightText: 2020-2021 Sotiris Papatheodorou, Imperial College London
// SPDX-License-Identifier: BSD-3-Clause

#ifndef SE_READER_INTERIORNET_HPP
#define SE_READER_INTERIORNET_HPP



#include <cstdint>
#include <fstream>
#include <string>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

#include "se/image/image.hpp"
#include "reader_base.hpp"



namespace se {



/** Reader for InteriorNet RGBD datasets. */

class InteriorNetReader: public Reader {

public:
  /** Construct a InteriorNetReader from a ReaderConfig.
   *
   * \param[in] c The configuration struct to use.
   */
  InteriorNetReader(const ReaderConfig& c);

  /** Restart reading from the beginning. */
  void restart();


  /** The name of the reader.
   *
   * \return The string `"InteriorNetReader"`.
   */
  std::string name() const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  static constexpr float interiornet_inverse_scale_ = 1.0f / 1000.0f;
  float inverse_scale_;

  // TODO Allow setting the max_match_timestamp_dist_ and
  // max_interp_timestamp_dist_ at runtime from the YAML file. Not sure how
  // to handle this yet since they only apply to the TUM dataset reader.
  static constexpr double max_match_timestamp_dist_ = 0.02;

  static constexpr double max_interp_timestamp_dist_ = 10.0 * max_match_timestamp_dist_;

  cv::Mat projection_inv_;

  std::vector<std::string> depth_filenames_;

  std::vector<std::string> rgb_filenames_;

  ReaderStatus nextDepth(Image<float>& depth_image);

  ReaderStatus nextRGBA(Image<uint32_t>& rgba_image);
};



} // namespace se



#endif //SE_READER_INTERIORNET_HPP
