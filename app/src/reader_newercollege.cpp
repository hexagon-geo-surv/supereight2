/*
 * SPDX-FileCopyrightText: 2020-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020 Marija Popovic
 * SPDX-FileCopyrightText: 2020-2021 Nils Funk
 * SPDX-FileCopyrightText: 2020-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "reader_newercollege.hpp"

#include <cassert>
#include <cctype>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <regex>

#include "se/common/filesystem.hpp"
#include "se/common/image_utils.hpp"

#ifdef SE_PCL
#    include <pcl/io/pcd_io.h>
#    include <pcl/point_types.h>
#endif


#ifdef SE_PCL
Eigen::Vector3f atof3(const std::string& line)
{
    Eigen::Vector3f res = Eigen::Vector3f::Zero();
    std::istringstream remaining_line(line);
    std::string s;
    if (std::getline(remaining_line, s, ' ')) {
        res.x() = atof(s.c_str());
    }
    else {
        // line is empty
        return res;
    }
    if (std::getline(remaining_line, s, ' ')) {
        res.y() = atof(s.c_str());
    }
    else {
        // line is x
        res.y() = res.x();
        res.z() = res.x();
        return res;
    }
    if (std::getline(remaining_line, s, ' ')) {
        res.z() = atof(s.c_str());
    }
    else {
        // line is x,y
        res.z() = res.y();
    }
    return res;
}



se::NewerCollegeReader::NewerCollegeReader(const se::Reader::Config& c) : se::Reader(c)
{
    // Ensure a valid directory was provided
    if (!stdfs::is_directory(sequence_path_)) {
        status_ = se::ReaderStatus::error;
        std::cerr << "Error: " << sequence_path_ << " is not a directory\n";
        return;
    }
    // Set the depth and RGBA image resolutions.
    depth_image_res_ = Eigen::Vector2i(1024, 64);
    rgba_image_res_ = Eigen::Vector2i(1024, 64);
    // Get the scan filenames and total number of frames.
    scan_filenames_ = getScanFilenames(sequence_path_);
    num_frames_ = scan_filenames_.size();
    if (verbose_ >= 1) {
        std::clog << "Found " << num_frames_ << " PCD files\n";
    }
}



void se::NewerCollegeReader::restart()
{
    se::Reader::restart();
    if (stdfs::is_directory(sequence_path_)) {
        status_ = se::ReaderStatus::ok;
    }
    else {
        status_ = se::ReaderStatus::error;
        std::cerr << "Error: " << sequence_path_ << " is not a directory\n";
    }
}



std::string se::NewerCollegeReader::name() const
{
    return std::string("NewerCollegeReader");
}



se::ReaderStatus se::NewerCollegeReader::nextDepth(se::Image<float>& depth_image)
{
    // Resize the output image if needed.
    if ((depth_image.width() != depth_image_res_.x())
        || (depth_image.height() != depth_image_res_.y())) {
        depth_image = se::Image<float>(depth_image_res_.x(), depth_image_res_.y());
    }
    // Initialize the image to zeros.
    std::memset(depth_image.data(), 0, depth_image_res_.prod() * sizeof(float));
    // Read the point cloud.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    const std::string filename = scan_filenames_[frame_];
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == -1) {
        std::cerr << "Error: can't read point cloud from " << filename << "\n";
        return se::ReaderStatus::error;
    }
    else if (verbose_ >= 1) {
        std::clog << "Read point cloud from " << filename << "\n";
    }
    // Read the point cloud and convert each point to a distance.
    size_t pixel_idx = SIZE_MAX;
    std::vector<float> data(depth_image_res_.prod(), 0);
    for (const auto& point : *cloud) {
        data[++pixel_idx] = Eigen::Vector3f(point.x, point.y, point.z).norm();
    }

    // Reorganize the distances into a depth image.
    for (int y = 0; y < depth_image_res_.y(); ++y) {
        for (int x = 0; x < depth_image_res_.x(); ++x) {
            const int tmp_x = (x + pixel_offset[y]) % depth_image_res_.x();
            const size_t idx = tmp_x * depth_image_res_.y() + y;
            depth_image(x, y) = data[idx];
        }
    }

    return se::ReaderStatus::ok;
}



se::ReaderStatus se::NewerCollegeReader::nextRGBA(se::Image<uint32_t>& rgba_image)
{
    // Resize the output image if needed.
    if ((rgba_image.width() != rgba_image_res_.x())
        || (rgba_image.height() != rgba_image_res_.y())) {
        rgba_image = se::Image<uint32_t>(rgba_image_res_.x(), rgba_image_res_.y());
    }
    // Create a blank image
    std::memset(rgba_image.data(), 0, rgba_image_res_.prod() * sizeof(uint32_t));
    return se::ReaderStatus::ok;
}



std::vector<std::string> se::NewerCollegeReader::getScanFilenames(const std::string& dir)
{
    static const std::string regex_pattern = ".*cloud_[[:digit:]]{10}_[[:digit:]]{9}.pcd";
    static const std::regex cloud_image_regex(regex_pattern);
    std::vector<std::string> filenames;
    for (const auto& p : stdfs::directory_iterator(dir)) {
        if (!stdfs::is_directory(p.path())) {
            const std::string filename = p.path().string();
            if (std::regex_match(filename, cloud_image_regex)) {
                filenames.push_back(filename);
            }
        }
    }
    // Sort the filenames to make they are opened in the correct order. This assumes that any
    // numbers are zero-padded, which is true in the NewerCollege dataset.
    std::sort(filenames.begin(), filenames.end());
    if (filenames.empty()) {
        std::cerr << "Warning: no files matching the regular expression " << regex_pattern << "\n";
    }
    return filenames;
}



#else
se::NewerCollegeReader::NewerCollegeReader(const se::Reader::Config& c) : se::Reader(c)
{
    status_ = se::ReaderStatus::error;
    std::cerr << "Error: not compiled with PCL, no Newer College support\n";
}



void se::NewerCollegeReader::restart()
{
    se::Reader::restart();
}



std::string se::NewerCollegeReader::name() const
{
    return std::string("NewerCollegeReader");
}



se::ReaderStatus se::NewerCollegeReader::nextDepth(se::Image<float>&)
{
    return se::ReaderStatus::error;
}



se::ReaderStatus se::NewerCollegeReader::nextRGBA(se::Image<uint32_t>&)
{
    return se::ReaderStatus::error;
}



std::vector<std::string> se::NewerCollegeReader::getScanFilenames(const std::string&)
{
    return std::vector<std::string>();
}

#endif
