/*
 * SPDX-FileCopyrightText: 2014 University of Edinburgh, Imperial College London, University of Manchester
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2020-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2022 Nils Funk
 * SPDX-FileCopyrightText: 2020-2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: MIT
 */

#ifndef __READER_RAW_HPP
#define __READER_RAW_HPP

#include <Eigen/Core>
#include <cstdint>
#include <fstream>
#include <string>

#include "reader_base.hpp"
#include "se/image/image.hpp"



namespace se {

/** Reader for SLAMBench 1.0 .raw files.
 * http://apt.cs.manchester.ac.uk/projects/PAMELA/tools/SLAMBench/
 */
class RAWReader : public Reader {
    public:
    /** Construct a RAWReader from a ReaderConfig.
     *
     * \param[in] c The configuration struct to use.
     */
    RAWReader(const ReaderConfig& c);

    /** Restart reading from the beginning. */
    void restart();

    /** The name of the reader.
     *
     * \return The string `"RAWReader"`.
     */
    std::string name() const;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
    std::ifstream raw_fs_;
    size_t depth_image_total_;
    size_t colour_image_total_;
    size_t depth_data_size_;
    size_t colour_data_size_;
    size_t depth_total_size_;
    size_t colour_total_size_;
    /** The size in bytes of a depth image pixel. */
    static constexpr size_t depth_pixel_size_ = sizeof(uint16_t);
    /** The size in bytes of an RGB image pixel. */
    static constexpr size_t colour_pixel_size_ = 3 * sizeof(uint8_t);
    /** The size in bytes of the image dimensions as stored in the raw file. */
    static constexpr size_t res_size_ = 2 * sizeof(uint32_t);

    bool readResolution(std::ifstream& fs, Eigen::Vector2i& res);

    ReaderStatus nextDepth(Image<float>& depth_image);

    ReaderStatus nextColour(Image<uint32_t>& colour_image);
};

} // namespace se

#endif
