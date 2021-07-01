// SPDX-FileCopyrightText: 2020-2021 Smart Robotics Lab, Imperial College London
// SPDX-FileCopyrightText: 2020-2021 Sotiris Papatheodorou, Imperial College London
// SPDX-License-Identifier: BSD-3-Clause


#ifndef __READER_TUM_HPP
#define __READER_TUM_HPP


#include <cstdint>
#include <fstream>
#include <string>

#include <Eigen/Dense>

#include "se/image/image.hpp"
#include "reader_base.hpp"



namespace se {



/** Reader for TUM RGBD datasets. */

class TUMReader: public Reader {

public:
  /** Construct a TUMReader from a ReaderConfig.
   *
   * \param[in] c The configuration struct to use.
   */
  TUMReader(const ReaderConfig& c);


  /** Restart reading from the beginning. */
  void restart();


  /** The name of the reader.
   *
   * \return The string `"TUMReader"`.
   */
  std::string name() const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  static constexpr float tum_inverse_scale_ = 1.0f / 5000.0f;
  float inverse_scale_;

  // TODO Allow setting the max_match_timestamp_dist_ and
  // max_interp_timestamp_dist_ at runtime from the YAML file. Not sure how
  // to handle this yet since they only apply to the TUM dataset reader.
  static constexpr double max_match_timestamp_dist_ = 0.02;

  static constexpr double max_interp_timestamp_dist_ = 10.0 * max_match_timestamp_dist_;

  std::vector<std::string> depth_filenames_;

  std::vector<std::string> rgb_filenames_;

  ReaderStatus nextDepth(Image<float>& depth_image);

  ReaderStatus nextRGBA(Image<uint32_t>& rgba_image);
};



} // namespace se


#endif