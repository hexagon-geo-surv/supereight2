/*
 * SPDX-FileCopyrightText: 2020-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020 Marija Popovic
 * SPDX-FileCopyrightText: 2020-2022 Nils Funk
 * SPDX-FileCopyrightText: 2020-2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __READER_NEWERCOLLEGE_HPP
#define __READER_NEWERCOLLEGE_HPP

#include <Eigen/Core>
#include <cstdint>
#include <fstream>
#include <string>

#include "reader_base.hpp"
#include "se/common/projection.hpp"
#include "se/image/image.hpp"



namespace se {

/** Reader for the Newer College dataset.
 * https://ori-drs.github.io/newer-college-dataset/
 */
class NewerCollegeReader : public Reader {
    public:
    /** Construct an NewerCollegeReader from a Config.
     *
     * \param[in] c The configuration struct to use.
     */
    NewerCollegeReader(const Config& c);

    /** Restart reading from the beginning. */
    void restart();

    /** The name of the reader.
     *
     * \return The string `"NewerCollegeReader"`.
     */
    std::string name() const;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
    std::vector<std::string> scan_filenames_;

    ReaderStatus nextDepth(Image<float>& depth_image);

    ReaderStatus nextColour(Image<uint32_t>& colour_image);

    static constexpr int8_t pixel_offset[64] = {
        0,  6,  12, 18, 0,  6,  12, 18, 0,  6,  12, 18, 0,  6,  12, 18, 0,  6,  12, 18, 0,  6,
        12, 18, 0,  6,  12, 18, 0,  6,  12, 18, 0,  6,  12, 18, 0,  6,  12, 18, 0,  6,  12, 18,
        0,  6,  12, 18, 0,  6,  12, 18, 0,  6,  12, 18, 0,  6,  12, 18, 0,  6,  12, 18};

    /** Return the filenames of LIDAR scans in PCD formatin the supplied directory.
     * LIDAR scans are considered those whose name conforms to the pattern
     * cloud_XXXXXXXXXX_XXXXXXXXX.pcd where X is a digit 0-9.
     *
     * \param[in] dir The directory inside which to look for PCD files.
     * \return The filenames of the PCD files found in lexicographical order.
     */
    static std::vector<std::string> getScanFilenames(const std::string& dir);
};

} // namespace se

#endif
