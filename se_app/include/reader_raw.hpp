/*
 * SPDX-FileCopyrightText: 2014 University of Edinburgh, Imperial College London, University of Manchester.
 * SPDX-FileCopyrightText: 2020 Smart Robotics Lab, Imperial College London
 * SPDX-FileCopyrightText: 2020 Sotiris Papatheodorou
 * SPDX-License-Identifier: MIT
 * Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1
 */

#ifndef __READER_RAW_HPP
#define __READER_RAW_HPP

#include <cstdint>
#include <fstream>
#include <string>

#include <Eigen/Dense>

#include "se/image/image.hpp"
#include "reader_base.hpp"



namespace se {

  /** Reader for Slambench 1.0 .raw files. */
  class RAWReader: public Reader {
    public:
      /** Construct a RAWReader from a ReaderConfig.
       *
       * \param[in] c The configuration struct to use.
       */
      RAWReader(const ReaderConfig& c);

      /** Restart reading from the beginning. */
      void restart();

      /** The name of the reader.
       *
       * \return The string `"RAWReader"`.
       */
      std::string name() const;

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
      std::ifstream raw_fs_;
      size_t depth_image_total_;
      size_t rgba_image_total_;
      size_t depth_data_size_;
      size_t rgba_data_size_;
      size_t depth_total_size_;
      size_t rgba_total_size_;
      /** The size in bytes of a depth image pixel. */
      static constexpr size_t depth_pixel_size_ = sizeof(uint16_t);
      /** The size in bytes of an RGB image pixel. */
      static constexpr size_t rgba_pixel_size_ = 3 * sizeof(uint8_t);
      /** The size in bytes of the image dimensions as stored in the raw file. */
      static constexpr size_t res_size_ = 2 * sizeof(uint32_t);

      bool readResolution(std::ifstream& fs, Eigen::Vector2i& res);

      ReaderStatus nextDepth(Image<float>& depth_image);

      ReaderStatus nextRGBA(Image<uint32_t>& rgba_image);
  };

} // namespace se

#endif

