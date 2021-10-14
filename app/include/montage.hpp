/*
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_MONTAGE_HPP
#define SE_MONTAGE_HPP

#include <opencv2/core/mat.hpp>
#include <string>
#include <vector>

namespace se {

/**
 * \brief Create a montage of several images and overlay labels.
 * The montage image is filled with images in row-major order. If fewer than
 * `montage_width * montage_height` images are supplied then the montage image will have some
 * transparent regions. If fewer labels than images are provided then the last images won't have a
 * label.
 *
 * For `montage_width = 3`, `montage_height = 2`, 5 images and 3 labels the following montage will
 * be created
 *
 * ``` text
 * ┌───────────┬───────────┬───────────┐
 * │ images[0] │ images[1] │ images[2] │
 * │ labels[0] │ labels[1] │ labels[2] │
 * ├───────────┼───────────┼───────────┤
 * │ images[3] │ images[4] │           │
 * │           │           │           │
 * └───────────┴───────────┴───────────┘
 * ```
 *
 * \warning It is assumed that the dimensions of all images are the same and that their type is
 * `CV_8UC4`.
 *
 * \param[in] montage_width  The width of the montage image in images.
 * \param[in] montage_height The height of the montage image in images.
 * \param[in] images         The images to montage.
 * \param[in] labels         The labels corresponding to the images to montage.
 * \return A montage image from the supplied images and labels.
 */
cv::Mat montage(int montage_width,
                int montage_height,
                const std::vector<cv::Mat>& images,
                const std::vector<std::string>& labels);

} // namespace se

#endif // SE_MONTAGE_HPP
