/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2021-2023 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021-2023 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_IMAGE_HPP
#define SE_IMAGE_HPP

#include <cassert>
#include <memory>
#include <se/common/rgb.hpp>
#include <se/common/rgba.hpp>

#include "se/common/colour_utils.hpp"

namespace se {

template<typename T>
class Image {
    public:
    Image(const unsigned w, const unsigned h) :
            width_(w), height_(h), owned_data_(new T[w * h]), data_ptr_(owned_data_.get())
    {
        assert(width_ > 0 && height_ > 0);
    }

    Image(const unsigned w, const unsigned h, const T& value) : Image(w, h)
    {
        std::fill(data(), data() + size(), value);
    }

    Image(const unsigned w, const unsigned h, T* raw_buffer) :
            width_(w), height_(h), data_ptr_(raw_buffer)
    {
        assert(width_ > 0 && height_ > 0);
    }

    Image(const Image& other) = delete;

    Image(Image&& other) = default;

    Image& operator=(const Image& other) = delete;

    Image& operator=(Image&& other) = default;

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
        return data_ptr_;
    }

    T* data()
    {
        return data_ptr_;
    }

    Image clone() const
    {
        if (owned_data_) {
            // Perform a deep copy of the owned data.
            Image image_copy(width(), height());
            std::copy(data(), data() + size(), image_copy.data());
            return image_copy;
        }
        else {
            // Wrap the non-owned data.
            return Image(width(), height(), data());
        }
    }

    private:
    int width_;
    int height_;
    std::unique_ptr<T[]> owned_data_;
    T* data_ptr_;
};



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

void rgb_to_rgba(const Image<RGB>& rgb, Image<RGBA>& rgba);

void rgba_to_rgb(const Image<RGBA>& rgba, Image<RGB>& rgb);

} // namespace image

} // end namespace se

#include "impl/image_impl.hpp"

#endif // SE_IMAGE_HPP
