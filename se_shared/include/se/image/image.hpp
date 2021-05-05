#ifndef SE_IMAGE_HPP
#define SE_IMAGE_HPP

#include "se/image_utils.hpp"

#include <vector>
#include <cassert>
#include <Eigen/StdVector>

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


static uint32_t gray_to_rgba(double h) {
  constexpr double v = 0.75;
  double r = 0, g = 0, b = 0;
  if (v > 0) {
    constexpr double m = 0.25;
    constexpr double sv = 0.6667;
    h *= 6.0;
    const int sextant = static_cast<int>(h);
    const double fract = h - sextant;
    const double vsf = v * sv * fract;
    const double mid1 = m + vsf;
    const double mid2 = v - vsf;
    switch (sextant) {
      case 0:
        r = v;
        g = mid1;
        b = m;
        break;
      case 1:
        r = mid2;
        g = v;
        b = m;
        break;
      case 2:
        r = m;
        g = v;
        b = mid1;
        break;
      case 3:
        r = m;
        g = mid2;
        b = v;
        break;
      case 4:
        r = mid1;
        g = m;
        b = v;
        break;
      case 5:
        r = v;
        g = m;
        b = mid2;
        break;
      default:
        r = 0;
        g = 0;
        b = 0;
        break;
    }
  }
  return se::pack_rgba(r * 255, g * 255, b * 255, 255);
}



static void convert_to_output_rgba_img(const se::Image<uint32_t>& input_rgba_img,
                                       uint32_t*                  output_rgba_img_data)
{
  memcpy(output_rgba_img_data, input_rgba_img.data(),
         input_rgba_img.width() * input_rgba_img.height() * sizeof(uint32_t));
}



static void convert_to_output_depth_img(const se::Image<float>& input_depth_img,
                                        uint32_t*               output_depth_img_data)
{
  const float min_depth = 0.4f;
  const float max_depth = 6.0f;

  const float range_scale = 1.0f / (max_depth - min_depth);
  for (int y = 0; y < input_depth_img.height(); y++) {
    const int row_offset = y * input_depth_img.width();
    for (int x = 0; x < input_depth_img.width(); x++) {
      const int pixel_idx = row_offset + x;
      if (input_depth_img[pixel_idx] < min_depth) {
        output_depth_img_data[pixel_idx] = 0xFFFFFFFF; // White
      } else if (input_depth_img[pixel_idx] > max_depth) {
        output_depth_img_data[pixel_idx] = 0xFF000000; // Black
      } else {
        const float depth_value = (input_depth_img[pixel_idx] - min_depth) * range_scale;
        output_depth_img_data[pixel_idx] = gray_to_rgba(depth_value);
      }
    }
  }
}



} // end namespace se
#endif // SE_IMAGE_HPP
