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

#include "se/common/colour_utils.hpp"

namespace se {

template<typename T>
class ImageView
{
public:
    ImageView(const unsigned w, const unsigned h, T* raw_buffer) : width_(w), height_(h), buffer_(raw_buffer)
    {
        assert(width_ > 0 && height_ > 0);
    }

    T& operator[](std::size_t idx)
    {
        return buffer_[idx];
    }

    const T& operator[](std::size_t idx) const
    {
        return buffer_[idx];
    }

    T& operator()(const int x, const int y)
    {
        return buffer_[x + y * width_];
    }

    const T& operator()(const int x, const int y) const
    {
        return buffer_[x + y * width_];
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
        return this->buffer_;
    }

    T* data()
    {
        return this->buffer_;
    }

protected:
    int width_;
    int height_;
    T* buffer_;
};

template<typename T>
class Image : public ImageView<T>
{
public:
    Image(const unsigned w, const unsigned h) : ImageView<T>(w,h,nullptr)
    {
        data_.resize(this->width_ * this->height_);
        this->buffer_ = data_.data();
    }

    Image(const unsigned w, const unsigned h, const T& val) : ImageView<T>(w,h,nullptr)
    {
        data_.resize(this->width_ * this->height_, val);
        this->buffer_ = data_.data();
    }



private:
    std::vector<T, Eigen::aligned_allocator<T>> data_;

    // std::vector<bool> is specialized for space efficiency which means that element access doesn't
    // return references to the data as expected, causing compilation issues.
    static_assert(!std::is_same<T, bool>::value,
                  "Use char/uint8_t instead of bool to avoid the std::vector<bool> specialization");
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
