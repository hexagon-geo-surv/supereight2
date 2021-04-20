/*
 * SPDX-FileCopyrightText: 2014 University of Edinburgh, Imperial College London, University of Manchester.
 * SPDX-FileCopyrightText: 2020 Smart Robotics Lab, Imperial College London
 * SPDX-FileCopyrightText: 2020 Sotiris Papatheodorou
 * SPDX-License-Identifier: MIT
 * Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1
 */

#ifndef __READER_ICLNUIM_HPP
#define __READER_ICLNUIM_HPP

#include <cstdint>
#include <fstream>
#include <string>

#include <Eigen/Dense>

#include "se/image/image.hpp"
#include "reader_base.hpp"



namespace se {

  /** Reader for ICL-NUIM datasets. */
  class ICLNUIMReader: public Reader {
    public:
      /** Construct an ICLNUIMReader from a ReaderConfig.
       *
       * \param[in] c The configuration struct to use.
       */
      ICLNUIMReader(const ReaderConfig& c);

      /** Restart reading from the beginning. */
      void restart();

      /** The name of the reader.
       *
       * \return The string `"ICLNUIMReader"`.
       */
      std::string name() const;

      /** Convert a Euclidean distance to the camera centre to a depth value.
       * The conversion is shown in the diagram in
       * http://www.doc.ic.ac.uk/~ahanda/VaFRIC/codes.html
       *
       * \param[in] dist The distance to convert (d in the diagram).
       * \param[in] x    The horizontal pixel coordinate of dist.
       * \param[in] y    The vertical pixel coordinate of dist.
       * \return The respective depth value of dist (z in the diagram).
       */
      static float distanceToDepth(float dist, int x, int y);

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
      static constexpr float fx_ =  481.2f;
      static constexpr float fy_ = -480.0f;
      static constexpr float cx_ =  319.5f;
      static constexpr float cy_ =  239.5f;

      ReaderStatus nextDepth(Image<float>& depth_image);

      ReaderStatus nextRGBA(Image<uint32_t>& rgba_image);

      /** Return the number of depth images in the supplied directory.
       * Depth images are considered those whose name conforms to the pattern
       * scene_00_XXXX.depth where X is a digit 0-9.
       *
       * \param[in] dir The directory inside which to look for depth images.
       * \return The number of depth images found.
       */
      size_t numDepthImages(const std::string& dir) const;
  };

} // namespace se

#endif

