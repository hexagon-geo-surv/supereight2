/*
 * SPDX-FileCopyrightText: 2020-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2022 Nils Funk
 * SPDX-FileCopyrightText: 2020-2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_READER_INTERIORNET_HPP
#define SE_READER_INTERIORNET_HPP



#include <Eigen/Core>
#include <cstdint>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <string>

#include "reader_base.hpp"
#include "se/image/image.hpp"



namespace se {



/** Reader for the InteriorNet dataset.
 * https://interiornet.org/
 */
class InteriorNetReader : public Reader {
    public:
    /** Construct an InteriorNetReader from a Config.
     *
     * \param[in] c The configuration struct to use.
     */
    InteriorNetReader(const Config& c);

    /** Restart reading from the beginning. */
    void restart();


    /** The name of the reader.
     *
     * \return The string `"InteriorNetReader"`.
     */
    std::string name() const;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
    static constexpr float interiornet_inverse_scale_ = 1.0f / 1000.0f;
    const float inverse_scale_;

    cv::Mat projection_inv_;

    std::vector<std::string> depth_filenames_;

    std::vector<std::string> rgb_filenames_;

    ReaderStatus nextDepth(Image<float>& depth_image);

    ReaderStatus nextColour(Image<RGBA>& colour_image);
};



} // namespace se



#endif //SE_READER_INTERIORNET_HPP
