/*
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "montage.hpp"

#include <iostream>
#include <opencv2/imgproc.hpp>

namespace se {

cv::Mat montage(int montage_width,
                int montage_height,
                const std::vector<cv::Mat>& images,
                const std::vector<std::string>& labels)
{
#ifndef NDEBUG
    // Ensure all input images have the same dimensions.
    for (size_t i = 0; i < images.size() - 1; i++) {
        assert(images[i].size == images[i + 1].size);
    }
    // Ensure all input images are RGBA.
    for (const auto& image : images) {
        assert(image.type() == CV_8UC4);
    }
#endif

    cv::Mat m;
    if (!images.empty()) {
        // The dimentions of the montage image in pixels assuming all sub-images have the same
        // dimensions.
        cv::Size m_size =
            cv::Size(images.front().cols * montage_width, images.front().rows * montage_height);
        m = cv::Mat(m_size, images.front().type(), cv::Scalar::all(0));
    }

    // Iterate over each sub-image and fill the montage in row-major order.
    for (int y = 0; y < montage_height; y++) {
        for (int x = 0; x < montage_width; x++) {
            // Compute the linear sub-image index.
            const size_t i = x + montage_width * y;
            // We might have fewer images than montage slots.
            if (i < images.size()) {
                // Compute the pixel coordinates of the top left corner of the current sub-image in
                // the montage image.
                const cv::Size image_size(images[i].cols, images[i].rows);
                int x_px = x * image_size.width;
                int y_px = y * image_size.height;
                // Create a view of the montage image containing only the region where the sub-image
                // will be copied.
                const cv::Range col_range(x_px, x_px + image_size.width);
                const cv::Range row_range(y_px, y_px + image_size.height);
                cv::Mat m_subimage(m, row_range, col_range);
                images[i].copyTo(m_subimage);

                // Add a text label to the montage at the subimage location. We might have fewer
                // labels than sub-images.
                if (i < labels.size()) {
                    constexpr auto font = cv::FONT_HERSHEY_SIMPLEX;
                    constexpr int thickness = 3;
                    // Get the dimensions of the resulting text box for a scale of 1.
                    int baseline = 0;
                    cv::Size text_size =
                        cv::getTextSize(labels[i], font, 1.0, thickness, &baseline);
                    // Compute the scale so that the text height is 10% of the image height.
                    const double scale = image_size.height / 10.0 / text_size.height;
                    // Scale the baseline.
                    baseline = scale * (baseline + thickness);
                    // Put the text on the bottom left corner.
                    cv::Point text_pos(image_size.width / 40, image_size.height - baseline);
                    // Draw the black text border.
                    cv::putText(m_subimage,
                                labels[i],
                                text_pos,
                                font,
                                scale,
                                cv::Scalar(0, 0, 0, 255),
                                thickness);
                    // Draw the white text fill.
                    cv::putText(m_subimage,
                                labels[i],
                                text_pos,
                                font,
                                scale,
                                cv::Scalar::all(255),
                                thickness - 2);
                }
            }
        }
    }
    return m;
}

} // namespace se
