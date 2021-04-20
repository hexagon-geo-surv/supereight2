/*
 * SPDX-FileCopyrightText: 2020 Masha Popovic, Imperial College London
 * SPDX-FileCopyrightText: 2020 Sotiris Papatheodorou, Imperial College London
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __READER_NEWERCOLLEGE_HPP
#define __READER_NEWERCOLLEGE_HPP

#include <cstdint>
#include <fstream>
#include <string>

#include <Eigen/Dense>

#include "se/image/image.hpp"
#include "se/projection.hpp"
#include "reader_base.hpp"



namespace se {

  /** Reader for Newer College datasets. */
  class NewerCollegeReader: public Reader {
    public:
      /** Construct an NewerCollegeReader from a ReaderConfig.
       *
       * \param[in] c The configuration struct to use.
       */
      NewerCollegeReader(const ReaderConfig& c);

      /** Restart reading from the beginning. */
      void restart();

      /** The name of the reader.
       *
       * \return The string `"NewerCollegeReader"`.
       */
      std::string name() const;

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
      static const int8_t pixel_offset[64];

      size_t ouster_pcd_idx_to_image_idx(size_t ouster_idx);

      ReaderStatus nextDepth(Image<float>& depth_image);

      ReaderStatus nextRGBA(Image<uint32_t>& rgba_image);

      /** Return the number of LIDAR scans in the supplied directory.
       * LIDAR scans are considered those whose name conforms to the pattern
       * cloud_XXXX.pcd where X is a digit 0-9.
       *
       * \param[in] dir The directory inside which to look for depth images.
       * \return The number of LIDAR scans found.
       */
      size_t numScans(const std::string& dir) const;
  };

} // namespace se

#endif

