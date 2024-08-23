/*
 * SPDX-FileCopyrightText: 2014 University of Edinburgh, Imperial College London, University of Manchester
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2020-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2021 Nils Funk
 * SPDX-FileCopyrightText: 2020-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: MIT
 */

#include "reader_raw.hpp"

#include <iostream>
#include <se/common/filesystem.hpp>

constexpr size_t se::RAWReader::depth_pixel_size_;
constexpr size_t se::RAWReader::colour_pixel_size_;
constexpr size_t se::RAWReader::res_size_;

se::RAWReader::RAWReader(const se::Reader::Config& c) : se::Reader(c)
{
    // Open the .raw file for reading.
    raw_fs_.open(sequence_path_, std::ios::in | std::ios::binary);
    if (!raw_fs_.good()) {
        std::cerr << "Error: Could not read raw file " << sequence_path_ << "\n";
        status_ = se::ReaderStatus::error;
        return;
    }
    // Read the resolution of the first depth image.
    raw_fs_.seekg(0);
    if (!readResolution(raw_fs_, depth_image_res_)) {
        std::cerr << "Error: Could not read depth image size\n";
        status_ = se::ReaderStatus::error;
        return;
    }
    // Pre-compute depth image sizes.
    depth_image_total_ = depth_image_res_.prod();
    depth_data_size_ = depth_pixel_size_ * depth_image_total_;
    depth_total_size_ = res_size_ + depth_data_size_;
    // Read the resolution of the first colour image.
    raw_fs_.seekg(depth_total_size_);
    if (!readResolution(raw_fs_, colour_image_res_)) {
        std::cerr << "Error: Could not read colour image size\n";
        status_ = se::ReaderStatus::error;
        return;
    }
    // Pre-compute colour image sizes.
    colour_image_total_ = colour_image_res_.prod();
    colour_data_size_ = colour_pixel_size_ * colour_image_total_;
    colour_total_size_ = res_size_ + colour_data_size_;
    // Start reading from the beginning of the .raw file again.
    raw_fs_.seekg(0);
    // Compute the total number of frames from the size of the .raw file.
    const uintmax_t raw_size = stdfs::file_size(sequence_path_);
    num_frames_ = raw_size / (depth_total_size_ + colour_total_size_);
    has_colour_ = true;
}



void se::RAWReader::restart()
{
    se::Reader::restart();
    if (raw_fs_.good() || raw_fs_.eof()) {
        raw_fs_.clear();
        raw_fs_.seekg(0);
        if (raw_fs_.good()) {
            status_ = se::ReaderStatus::ok;
        }
        else {
            status_ = se::ReaderStatus::error;
        }
    }
}



std::string se::RAWReader::name() const
{
    return std::string("RAWReader");
}



bool se::RAWReader::readResolution(std::ifstream& fs, Eigen::Vector2i& res)
{
    uint32_t w;
    if (!fs.read(reinterpret_cast<char*>(&w), sizeof(uint32_t))) {
        return false;
    }
    uint32_t h;
    if (!fs.read(reinterpret_cast<char*>(&h), sizeof(uint32_t))) {
        return false;
    }
    res = Eigen::Vector2i(w, h);
    return true;
}



se::ReaderStatus se::RAWReader::nextDepth(se::Image<float>& depth_image)
{
    // Seek to the appropriate place in the file.
    raw_fs_.seekg(frame_ * (depth_total_size_ + colour_total_size_));
    // Read the image dimensions.
    Eigen::Vector2i size;
    if (!readResolution(raw_fs_, size)) {
        return se::ReaderStatus::error;
    }
    // Resize the output image if needed.
    if ((depth_image.width() != size.x()) || (depth_image.height() != size.y())) {
        depth_image = se::Image<float>(size.x(), size.y());
    }
    // Read the whole image into a buffer.
    const size_t image_size = depth_image.size();
    std::vector<uint16_t> buffer(image_size);
    if (!raw_fs_.read(reinterpret_cast<char*>(buffer.data()), image_size * sizeof(uint16_t))) {
        return se::ReaderStatus::error;
    }
    // Scale the data from millimetres to metres.
#pragma omp parallel for
    for (size_t p = 0; p < image_size; ++p) {
        depth_image[p] = buffer.data()[p] / 1000.0f;
    }
    return se::ReaderStatus::ok;
}



se::ReaderStatus se::RAWReader::nextColour(se::Image<RGB>& colour_image)
{
    // Seek to the appropriate place in the file.
    raw_fs_.seekg(frame_ * (depth_total_size_ + colour_total_size_) + depth_total_size_);
    // Read the image dimensions.
    Eigen::Vector2i size;
    if (!readResolution(raw_fs_, size)) {
        return se::ReaderStatus::error;
    }
    // Resize the output image if needed.
    if ((colour_image.width() != size.x()) || (colour_image.height() != size.y())) {
        colour_image = se::Image<RGB>(size.x(), size.y());
    }
    // Read the whole image.
    if (!raw_fs_.read(reinterpret_cast<char*>(colour_image.data()),
                      colour_image.size() * sizeof(se::RGB))) {
        return se::ReaderStatus::error;
    }
    return se::ReaderStatus::ok;
}
