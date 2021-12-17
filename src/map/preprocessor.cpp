/*
 * SPDX-FileCopyrightText: 2014 University of Edinburgh, Imperial College, University of Manchester
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2019-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: MIT
 */

#include "se/map/preprocessor.hpp"

#include <iostream>

#include "se/common/math_util.hpp"

#define INVALID -2

namespace se {
namespace preprocessor {

void downsample_depth(se::Image<float>& input_depth_img, se::Image<float>& output_depth_img)
{
    const float* input_depth_data = input_depth_img.data();

    // Check for unsupported conditions
    if ((input_depth_img.width() < output_depth_img.width())
        || input_depth_img.height() < output_depth_img.height()) {
        std::cerr << "Invalid ratio." << std::endl;
        exit(1);
    }

    if ((input_depth_img.width() % output_depth_img.width() != 0)
        || (input_depth_img.height() % output_depth_img.height() != 0)) {
        std::cerr << "Invalid ratio." << std::endl;
        exit(1);
    }

    if ((input_depth_img.width() / output_depth_img.width()
         != input_depth_img.height() / output_depth_img.height())) {
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
                    const float depth_value =
                        input_depth_data[x_in + input_depth_img.width() * y_in];
                    // Only consider positive, non-NaN values for the median
                    if ((depth_value < 1e-5) || std::isnan(depth_value)) {
                        continue;
                    }
                    else {
                        box_values.push_back(depth_value);
                    }
                }
            }
            output_depth_img(x_out, y_out) =
                box_values.empty() ? 0.0f : se::math::almost_median(box_values);
            box_values.clear();
        }
    }
}

void downsample_rgba(se::Image<uint32_t>& input_RGBA_img, se::Image<uint32_t>& output_RGBA_img)
{
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
                    const uint32_t pixel_value =
                        input_RGBA_data[x_in + input_RGBA_img.width() * y_in];
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

void bilateral_filter(se::Image<float>& output_image,
                      const se::Image<float>& input_image,
                      const std::vector<float>& gaussian,
                      const float e_d,
                      const int radius)
{
    if ((input_image.width() != output_image.width())
        || input_image.height() != output_image.height()) {
        std::cerr << "input/output image sizes differ." << std::endl;
        exit(1);
    }

    const int width = input_image.width();
    const int height = input_image.height();
    const float e_d_squared_2 = e_d * e_d * 2.f;
#pragma omp parallel for
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            const unsigned int pixel_idx = x + y * width;
            if (input_image[pixel_idx] == 0) {
                output_image[pixel_idx] = 0;
                continue;
            }

            float factor_count = 0.0f;
            float filter_value_sum = 0.0f;

            const float centre_value = input_image[pixel_idx];

            for (int i = -radius; i <= radius; ++i) {
                for (int j = -radius; j <= radius; ++j) {
                    const Eigen::Vector2i pixel_tmp =
                        Eigen::Vector2i(se::math::clamp(x + i, 0, width - 1),
                                        se::math::clamp(y + j, 0, height - 1));
                    const float pixel_value_tmp =
                        input_image[pixel_tmp.x() + pixel_tmp.y() * width];
                    if (pixel_value_tmp > 0.f) {
                        const float mod = se::math::sq(pixel_value_tmp - centre_value);
                        const float factor = gaussian[i + radius] * gaussian[j + radius]
                            * expf(-mod / e_d_squared_2);
                        filter_value_sum += factor * pixel_value_tmp;
                        factor_count += factor;
                    }
                }
            }
            output_image[pixel_idx] = filter_value_sum / factor_count;
        }
    }
}



void point_cloud_to_depth(se::Image<float>& depth_image,
                          const se::Image<Eigen::Vector3f>& point_cloud_X,
                          const Eigen::Matrix4f& T_CX)
{
#pragma omp parallel for
    for (int y = 0; y < depth_image.height(); y++) {
        for (int x = 0; x < depth_image.width(); x++) {
            depth_image(x, y) = (T_CX * point_cloud_X(x, y).homogeneous()).z();
        }
    }
}



// Explicit template instantiation
template void point_cloud_to_normal<true>(se::Image<Eigen::Vector3f>& normals,
                                          const se::Image<Eigen::Vector3f>& point_cloud);

template void point_cloud_to_normal<false>(se::Image<Eigen::Vector3f>& normals,
                                           const se::Image<Eigen::Vector3f>& point_cloud);

template<bool NegY>
void point_cloud_to_normal(se::Image<Eigen::Vector3f>& normals,
                           const se::Image<Eigen::Vector3f>& point_cloud)
{
    const int width = point_cloud.width();
    const int height = point_cloud.height();
#pragma omp parallel for
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            const Eigen::Vector3f point = point_cloud[x + width * y];
            if (point.z() == 0.f) {
                normals[x + y * width].x() = INVALID;
                continue;
            }

            const Eigen::Vector2i p_left = Eigen::Vector2i(std::max(int(x) - 1, 0), y);
            const Eigen::Vector2i p_right = Eigen::Vector2i(std::min(x + 1, (int) width - 1), y);

            // Swapped to match the left-handed coordinate system of ICL-NUIM
            Eigen::Vector2i p_up, p_down;
            if (NegY) {
                p_up = Eigen::Vector2i(x, std::max(int(y) - 1, 0));
                p_down = Eigen::Vector2i(x, std::min(y + 1, ((int) height) - 1));
            }
            else {
                p_down = Eigen::Vector2i(x, std::max(int(y) - 1, 0));
                p_up = Eigen::Vector2i(x, std::min(y + 1, ((int) height) - 1));
            }

            const Eigen::Vector3f left = point_cloud[p_left.x() + width * p_left.y()];
            const Eigen::Vector3f right = point_cloud[p_right.x() + width * p_right.y()];
            const Eigen::Vector3f up = point_cloud[p_up.x() + width * p_up.y()];
            const Eigen::Vector3f down = point_cloud[p_down.x() + width * p_down.y()];

            if (left.z() == 0 || right.z() == 0 || up.z() == 0 || down.z() == 0) {
                normals[x + y * width].x() = INVALID;
                continue;
            }
            const Eigen::Vector3f dv_x = right - left;
            const Eigen::Vector3f dv_y = up - down;
            normals[x + y * width] = dv_x.cross(dv_y).normalized();
        }
    }
}



void half_sample_robust_image(se::Image<float>& output_image,
                              const se::Image<float>& input_image,
                              const float e_d,
                              const int radius)
{
    if ((input_image.width() / output_image.width() != 2)
        || (input_image.height() / output_image.height() != 2)) {
        std::cerr << "Invalid ratio." << std::endl;
        exit(1);
    }

#pragma omp parallel for
    for (int y = 0; y < output_image.height(); y++) {
        for (int x = 0; x < output_image.width(); x++) {
            const Eigen::Vector2i out_pixel = Eigen::Vector2i(x, y);
            const Eigen::Vector2i in_pixel = 2 * out_pixel;

            float pixel_count = 0.0f;
            float pixel_value_sum = 0.0f;
            const float in_pixel_value =
                input_image[in_pixel.x() + in_pixel.y() * input_image.width()];
            for (int i = -radius + 1; i <= radius; ++i) {
                for (int j = -radius + 1; j <= radius; ++j) {
                    Eigen::Vector2i in_pixel_tmp = in_pixel + Eigen::Vector2i(j, i);
                    se::math::clamp(in_pixel_tmp,
                                    Eigen::Vector2i::Zero(),
                                    Eigen::Vector2i(2 * output_image.width() - 1,
                                                    2 * output_image.height() - 1));
                    const float in_pixel_value_tmp =
                        input_image[in_pixel_tmp.x() + in_pixel_tmp.y() * input_image.width()];
                    if (fabsf(in_pixel_value_tmp - in_pixel_value) < e_d) {
                        pixel_count += 1.0f;
                        pixel_value_sum += in_pixel_value_tmp;
                    }
                }
            }
            output_image[out_pixel.x() + out_pixel.y() * output_image.width()] =
                pixel_value_sum / pixel_count;
        }
    }
}

} // namespace preprocessor
} // namespace se
