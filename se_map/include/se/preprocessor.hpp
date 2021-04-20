#ifndef SE_PREPROCESSOR_HPP
#define SE_PREPROCESSOR_HPP

/*

 Copyright (c) 2014 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.


 Copyright 2016 Emanuele Vespa, Imperial College London

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this
 list of conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution.

 3. Neither the name of the copyright holder nor the names of its contributors
 may be used to endorse or promote products derived from this software without
 specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <cstdint>

#include <Eigen/Dense>

#include "se/utils/math_util.hpp"
#include "se/image/image.hpp"

namespace se {
namespace preprocessor {

void downsample_depth(se::Image<float>& input_depth_img,
                      se::Image<float>& output_depth_img) {
  const float* input_depth_data = input_depth_img.data();
  
  // Check for unsupported conditions
  if ((input_depth_img.width() < output_depth_img.width()) ||
      input_depth_img.height() < output_depth_img.height()) {
    std::cerr << "Invalid ratio." << std::endl;
    exit(1);
  }
  if ((input_depth_img.width() % output_depth_img.width() != 0) ||
      (input_depth_img.height() % output_depth_img.height() != 0)) {
    std::cerr << "Invalid ratio." << std::endl;
    exit(1);
  }
  if ((input_depth_img.width() / output_depth_img.width() !=
       input_depth_img.height() / output_depth_img.height())) {
    std::cerr << "Invalid ratio." << std::endl;
    exit(1);
  }

  const int ratio = input_depth_img.width() / output_depth_img.width();
#pragma omp parallel for
  for (int y_out = 0; y_out < output_depth_img.height(); y_out++) {
    for (int x_out = 0; x_out < output_depth_img.width(); x_out++) {
      std::vector<float> box_values;
      box_values.reserve(ratio * ratio);
      for (int b = 0; b < ratio; b++) {
        for (int a = 0; a < ratio; a++) {
          const int y_in = y_out * ratio + b;
          const int x_in = x_out * ratio + a;
          const float depth_value = input_depth_data[x_in + input_depth_img.width() * y_in];
          // Only consider positive, non-NaN values for the median
          if ((depth_value < 1e-5) || std::isnan(depth_value)) {
            continue;
          } else {
            box_values.push_back(depth_value);
          }
        }
      }
      output_depth_img(x_out, y_out) = box_values.empty() ? 0.0f : se::math::almost_median(box_values);
      box_values.clear();
    }
  }
}

void downsample_rgba(se::Image<uint32_t>& input_RGBA_img,
                     se::Image<uint32_t>& output_RGBA_img) {

  const uint32_t* input_RGBA_data = input_RGBA_img.data();
  
  // Check for correct image sizes.
  assert((input_RGBA_img.width() >= output_RGBA_img.width())
         && "Error: input width must be greater than output width");
  assert((input_RGBA_img.height() >= output_RGBA_img.height())
         && "Error: input height must be greater than output height");
  assert((input_RGBA_img.width() % output_RGBA_img.width() == 0)
         && "Error: input width must be an integer multiple of output width");
  assert((input_RGBA_img.height() % output_RGBA_img.height() == 0)
         && "Error: input height must be an integer multiple of output height");
  assert((input_RGBA_img.width() / output_RGBA_img.width()
          == input_RGBA_img.height() / output_RGBA_img.height())
         && "Error: input and output width and height ratios must be the same");

  const int ratio = input_RGBA_img.width() / output_RGBA_img.width();
  // Iterate over each output pixel.
#pragma omp parallel for
  for (int y_out = 0; y_out < output_RGBA_img.height(); ++y_out) {
    for (int x_out = 0; x_out < output_RGBA_img.width(); ++x_out) {

      // Average the neighboring pixels by iterating over the nearby input
      // pixels.
      uint16_t r = 0, g = 0, b = 0;
      for (int yy = 0; yy < ratio; ++yy) {
        for (int xx = 0; xx < ratio; ++xx) {
          const int x_in = x_out * ratio + xx;
          const int y_in = y_out * ratio + yy;
          const uint32_t pixel_value
                  = input_RGBA_data[x_in + input_RGBA_img.width() * y_in];
          r += se::r_from_rgba(pixel_value);
          g += se::g_from_rgba(pixel_value);
          b += se::b_from_rgba(pixel_value);
        }
      }
      r /= ratio * ratio;
      g /= ratio * ratio;
      b /= ratio * ratio;

      // Combine into a uint32_t by adding an alpha channel with 100% opacity.
      const uint32_t rgba = se::pack_rgba(r, g, b, 255);
      output_RGBA_img(x_out, y_out) = rgba;
    }
  }
}

}
}



#endif // SE_PREPROCESSOR_HPP
