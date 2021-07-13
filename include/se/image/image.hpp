// SPDX-FileCopyrightText: 2018 Emanuele Vespa
// SPDX-FileCopyrightText: 2020-2021 Smart Robotics Lab
// SPDX-FileCopyrightText: 2020-2021 Nils Funk, Imperial College London
// SPDX-FileCopyrightText: 2020-2021 Sotiris Papatheodorou, Imperial College London
// SPDX-License-Identifier: BSD-3-Clause

#ifndef SE_IMAGE_HPP
#define SE_IMAGE_HPP

#include <Eigen/StdVector>
#include <cassert>

#include "se/common/colour_utils.hpp"

namespace se {

template <typename T>
class Image {

  public:
    Image(
            const unsigned w, const unsigned h)
        : width_(w), height_(h), data_(width_ * height_) {
      assert(width_ > 0 && height_ > 0);
    }

    Image(const unsigned w, const unsigned h, const T& val) : width_(w), height_(h) {
      assert(width_ > 0 && height_ > 0);
      data_.resize(width_ * height_, val);
    }

    T&       operator[](std::size_t idx)       { return data_[idx]; }
    const T& operator[](std::size_t idx) const { return data_[idx]; }

    T&       operator()(const int x, const int y)       { return data_[x + y * width_]; }
    const T& operator()(const int x, const int y) const { return data_[x + y * width_]; }

    std::size_t size()   const   { return width_ * height_; };
    int         width () const { return width_;  };
    int         height() const { return height_; };

    T* data()             { return data_.data(); }
    const T* data() const { return data_.data(); }

  private:
    int width_;
    int height_;
    std::vector<T, Eigen::aligned_allocator<T> > data_;
};



static void convert_to_output_rgba_img(const se::Image<uint32_t>& input_rgba_img,
                                       uint32_t*                  output_rgba_img_data)
{
  memcpy(output_rgba_img_data, input_rgba_img.data(),
         input_rgba_img.width() * input_rgba_img.height() * sizeof(uint32_t));
}



static void convert_to_output_depth_img(const se::Image<float>& input_depth_img,
                                        uint32_t*               output_depth_img_data)
{
  depth_to_rgba(output_depth_img_data, input_depth_img.data(),
      Eigen::Vector2i(input_depth_img.width(), input_depth_img.height()), 0, std::numeric_limits<float>::max());
}



static void convert_to_output_depth_img(const se::Image<float>& input_depth_img,
                                        const float             min_depth,
                                        const float             max_depth,
                                        uint32_t*               output_depth_img_data)
{
  depth_to_rgba(output_depth_img_data, input_depth_img.data(),
                Eigen::Vector2i(input_depth_img.width(), input_depth_img.height()), min_depth, max_depth);
}



} // end namespace se

#endif // SE_IMAGE_HPP
