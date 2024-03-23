/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2021-2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021-2023 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_IMAGE_HPP
#define SE_IMAGE_HPP

#include <Eigen/StdVector>
#include <cassert>
#include <memory>

#include "se/common/colour_utils.hpp"

namespace se {

template<typename T>
class Image
{
public:
    Image(const unsigned w, const unsigned h) : width_(w), height_(h), data_(new T[w * h]), data_ptr_(data_.get())
    {
        assert(width_ > 0 && height_ > 0);
    }

    Image(const unsigned w, const unsigned h, const T& val) : Image(w,h)
    {
        std::fill(data_.get(), data_.get() + w * h, val);
    }

    Image(const unsigned w, const unsigned h, T* raw_buffer) : width_(w), height_(h), data_ptr_(raw_buffer)
    {
        assert(width_ > 0 && height_ > 0);
    }

    T& operator[](std::size_t idx)
    {
        return data_ptr_[idx];
    }

    const T& operator[](std::size_t idx) const
    {
        return data_ptr_[idx];
    }

    T& operator()(const int x, const int y)
    {
        return data_ptr_[x + y * width_];
    }

    const T& operator()(const int x, const int y) const
    {
        return data_ptr_[x + y * width_];
    }

    std::size_t size() const
    {
        return width_ * height_;
    }

    int width() const
    {
        return width_;
    }

    int height() const
    {
        return height_;
    }

    const T* data() const
    {
        return this->data_ptr_;
    }

    T* data()
    {
        return this->data_ptr_;
    }

protected:
    int width_;
    int height_;
    std::unique_ptr<T[]> data_;
    T* data_ptr_;
};



static inline void convert_to_output_rgba_img(const se::Image<uint32_t>& input_rgba_img,
                                              uint32_t* output_rgba_img_data)
{
    memcpy(output_rgba_img_data,
           input_rgba_img.data(),
           input_rgba_img.width() * input_rgba_img.height() * sizeof(uint32_t));
}



static inline void convert_to_output_depth_img(const se::Image<float>& input_depth_img,
                                               uint32_t* output_depth_img_data)
{
    depth_to_rgba(output_depth_img_data,
                  input_depth_img.data(),
                  Eigen::Vector2i(input_depth_img.width(), input_depth_img.height()),
                  0,
                  std::numeric_limits<float>::max());
}



static inline void convert_to_output_depth_img(const se::Image<float>& input_depth_img,
                                               const float min_depth,
                                               const float max_depth,
                                               uint32_t* output_depth_img_data)
{
    depth_to_rgba(output_depth_img_data,
                  input_depth_img.data(),
                  Eigen::Vector2i(input_depth_img.width(), input_depth_img.height()),
                  min_depth,
                  max_depth);
}



} // end namespace se

#endif // SE_IMAGE_HPP
