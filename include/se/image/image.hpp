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
#include <se/common/rgba.hpp>

#include "se/common/colour_utils.hpp"

namespace se {

template<typename T>
class Image {
    public:
    Image(const unsigned w, const unsigned h) : width_(w), height_(h), data_(width_ * height_)
    {
        assert(width_ > 0 && height_ > 0);
    }

    Image(const unsigned w, const unsigned h, const T& val) : width_(w), height_(h)
    {
        assert(width_ > 0 && height_ > 0);
        data_.resize(width_ * height_, val);
    }

    T& operator[](std::size_t idx)
    {
        return data_[idx];
    }
    const T& operator[](std::size_t idx) const
    {
        return data_[idx];
    }

    T& operator()(const int x, const int y)
    {
        return data_[x + y * width_];
    }
    const T& operator()(const int x, const int y) const
    {
        return data_[x + y * width_];
    }

    std::size_t size() const
    {
        return width_ * height_;
    };
    int width() const
    {
        return width_;
    };
    int height() const
    {
        return height_;
    };

    T* data()
    {
        return data_.data();
    }
    const T* data() const
    {
        return data_.data();
    }

    private:
    int width_;
    int height_;
    std::vector<T, Eigen::aligned_allocator<T>> data_;

    // std::vector<bool> is specialized for space efficiency which means that element access doesn't
    // return references to the data as expected, causing compilation issues.
    static_assert(!std::is_same<T, bool>::value,
                  "Use char/uint8_t instead of bool to avoid the std::vector<bool> specialization");
};



static inline void convert_to_output_rgba_img(const se::Image<RGBA>& input_rgba_img,
                                              RGBA* output_rgba_img_data)
{
    memcpy(output_rgba_img_data,
           input_rgba_img.data(),
           input_rgba_img.width() * input_rgba_img.height() * sizeof(RGBA));
}



static inline void convert_to_output_depth_img(const se::Image<float>& input_depth_img,
                                               RGBA* output_depth_img_data)
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
                                               RGBA* output_depth_img_data)
{
    depth_to_rgba(output_depth_img_data,
                  input_depth_img.data(),
                  Eigen::Vector2i(input_depth_img.width(), input_depth_img.height()),
                  min_depth,
                  max_depth);
}



namespace image {

/** Remap \p input to \p output by using a \p map which contains and index into \p input for each
 * element of \p output.
 */
template<typename T>
void remap(const Image<T>& input, Image<T>& output, const Image<size_t>& map);

} // namespace image

} // end namespace se

#include "impl/image_impl.hpp"

#endif // SE_IMAGE_HPP
