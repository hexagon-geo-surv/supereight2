/*
 * SPDX-FileCopyrightText: 2014 University of Edinburgh, Imperial College London, University of Manchester
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2020-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2020-2021 Nils Funk
 * SPDX-FileCopyrightText: 2020-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: MIT
 */

#include "reader_openni.hpp"

#include <iostream>

#include "se/common/image_utils.hpp"



#ifdef SE_OPENNI2

se::OpenNIReader::OpenNIReader(const se::ReaderConfig& c) :
        se::Reader(c), depth_image_(nullptr), colour_image_(nullptr)
{
    // Ensure this is handled as a live camera reader.
    is_live_reader_ = true;
    // Initialize OpenNI
    rc_ = openni::OpenNI::initialize();
    if (rc_ != openni::STATUS_OK) {
        std::cerr << "Error: Could not initialize OpenNI\n";
        status_ = se::ReaderStatus::error;
        return;
    }
    // Open a camera
    if (sequence_path_.empty()) {
        rc_ = device_.open(openni::ANY_DEVICE);
    }
    else {
        rc_ = device_.open(sequence_path_.c_str());
    }
    if (rc_ != openni::STATUS_OK) {
        std::cerr << "Error: No Kinect device found\n";
        status_ = se::ReaderStatus::error;
        return;
    }

    if (device_.getSensorInfo(openni::SENSOR_DEPTH) != nullptr) {
        // Create a depth stream
        rc_ = depth_stream_.create(device_, openni::SENSOR_DEPTH);
        if (rc_ != openni::STATUS_OK) {
            std::cerr << "Error: Could not create depth stream\n"
                      << openni::OpenNI::getExtendedError() << "\n";
            status_ = se::ReaderStatus::error;
            return;
        }
        // Start and stop the depth stream (to allow getting device info?)
        rc_ = depth_stream_.start();
        if (rc_ != openni::STATUS_OK) {
            std::cerr << "Error: Could not start depth stream\n"
                      << openni::OpenNI::getExtendedError() << "\n";
            depth_stream_.destroy();
            status_ = se::ReaderStatus::error;
            return;
        }
        depth_stream_.stop();

        rc_ = colour_stream_.create(device_, openni::SENSOR_COLOR);
        if (rc_ != openni::STATUS_OK) {
            std::cerr << "Error: Could not create colour stream\n"
                      << openni::OpenNI::getExtendedError() << "\n";
            status_ = se::ReaderStatus::error;
            return;
        }
    }

    device_.getSensorInfo(openni::SENSOR_DEPTH)->getSupportedVideoModes();

    if (!device_.isFile()) {
        // Set the depth camera video mode
        openni::VideoMode desired_depth_video_mode;
        desired_depth_video_mode.setFps(fps_ == 0.0f ? 30 : fps_);
        desired_depth_video_mode.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);
        desired_depth_video_mode.setResolution(640, 480);
        rc_ = depth_stream_.setVideoMode(desired_depth_video_mode);
        if (rc_ != openni::STATUS_OK) {
            std::cout << "Error: Could not set depth video mode\n";
            status_ = se::ReaderStatus::error;
            return;
        }
        // Set the colour camera video mode
        openni::VideoMode desired_colour_video_mode;
        desired_colour_video_mode.setFps(fps_ == 0.0f ? 30 : fps_);
        desired_colour_video_mode.setPixelFormat(openni::PIXEL_FORMAT_RGB888);
        desired_colour_video_mode.setResolution(640, 480);
        rc_ = colour_stream_.setVideoMode(desired_colour_video_mode);
        if (rc_ != openni::STATUS_OK) {
            std::cout << "Error: Could not set colour video mode\n";
            status_ = se::ReaderStatus::error;
            return;
        }
    }

    // Test that the video modes were set correctly
    openni::VideoMode depth_video_mode = depth_stream_.getVideoMode();
    depth_image_res_.x() = depth_video_mode.getResolutionX();
    depth_image_res_.y() = depth_video_mode.getResolutionY();
    openni::VideoMode colour_video_mode = colour_stream_.getVideoMode();
    colour_image_res_.x() = colour_video_mode.getResolutionX();
    colour_image_res_.y() = colour_video_mode.getResolutionY();
    if (depth_image_res_ != colour_image_res_) {
        std::cout << "Error: Depth and colour resolution mismatch\n";
        status_ = se::ReaderStatus::error;
        return;
    }

    device_.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);

    if (device_.isFile()) {
        device_.getPlaybackControl()->setRepeatEnabled(false);
        depth_frame_.release();
        colour_frame_.release();
        // Set the playback to manual mode i.e. read a frame whenever the
        // application requests it.
        device_.getPlaybackControl()->setSpeed(-1);
    }

    // Allocate image buffers
    depth_image_ = std::unique_ptr<uint16_t>(new uint16_t[depth_image_res_.prod()]);
    colour_image_ = std::unique_ptr<uint8_t>(new uint8_t[3 * depth_image_res_.prod()]);
    colour_stream_.setMirroringEnabled(false);
    depth_stream_.setMirroringEnabled(false);

    // Use fake allocators that just return the preallocated image buffers so
    // that OpenNI writes directly into them.
    depth_allocator_ = std::unique_ptr<se::OpenNIReader::MyFrameAllocator<uint16_t>>(
        new se::OpenNIReader::MyFrameAllocator<uint16_t>(depth_image_.get()));
    colour_allocator_ = std::unique_ptr<se::OpenNIReader::MyFrameAllocator<uint8_t>>(
        new se::OpenNIReader::MyFrameAllocator<uint8_t>(colour_image_.get()));
    depth_stream_.setFrameBuffersAllocator(depth_allocator_.get());
    colour_stream_.setFrameBuffersAllocator(colour_allocator_.get());

    // Start the streams
    depth_stream_.start();
    colour_stream_.start();
    has_colour_ = true;
}



se::OpenNIReader::~OpenNIReader()
{
    if (device_.isValid()) {
        depth_stream_.stop();
        colour_stream_.stop();
        depth_stream_.destroy();
        colour_stream_.destroy();
        device_.close();
        openni::OpenNI::shutdown();
    }
}



void se::OpenNIReader::restart()
{
    se::Reader::restart();
    // TODO How to rewind OpenNI?
}



std::string se::OpenNIReader::name() const
{
    return std::string("OpenNIReader");
}



se::ReaderStatus se::OpenNIReader::nextDepth(se::Image<float>& depth_image)
{
    rc_ = depth_stream_.readFrame(&depth_frame_);
    if (rc_ != openni::STATUS_OK) {
        std::cerr << "Error: Wait for depth image failed\n";
        return se::ReaderStatus::error;
    }
    if (depth_frame_.getVideoMode().getPixelFormat() != openni::PIXEL_FORMAT_DEPTH_1_MM) {
        std::cerr << "Error: Unexpected depth image format\n";
        return se::ReaderStatus::error;
    }

    // Read the colour image here to get the one matching the depth image.
    rc_ = colour_stream_.readFrame(&colour_frame_);
    if (rc_ != openni::STATUS_OK) {
        std::cerr << "Error: Wait for colour image failed\n";
        return se::ReaderStatus::error;
    }
    if (colour_frame_.getVideoMode().getPixelFormat() != openni::PIXEL_FORMAT_RGB888) {
        std::cerr << "Error: Unexpected colour image format\n";
        return se::ReaderStatus::error;
    }

    // Resize the output image if needed.
    if ((depth_image.width() != depth_image_res_.x())
        || (depth_image.height() != depth_image_res_.y())) {
        depth_image = se::Image<float>(depth_image_res_.x(), depth_image_res_.y());
    }

#    pragma omp parallel for
    for (int p = 0; p < depth_image_res_.prod(); ++p) {
        depth_image[p] = depth_image_.get()[p] / 1000.0f;
    }

    return se::ReaderStatus::ok;
}



se::ReaderStatus se::OpenNIReader::nextColour(se::Image<rgb_t>& colour_image)
{
    // Resize the output image if needed.
    if ((colour_image.width() != colour_image_res_.x())
        || (colour_image.height() != colour_image_res_.y())) {
        colour_image = se::Image<rgb_t>(colour_image_res_.x(), colour_image_res_.y());
    }

    std::memcpy(colour_image.data(), colour_image_.get(), colour_image_res_.prod() * sizeof(rgb_t));

    return se::ReaderStatus::ok;
}



#else



se::OpenNIReader::OpenNIReader(const se::ReaderConfig& c) : se::Reader(c)
{
    status_ = se::ReaderStatus::error;
    std::cerr << "Error: not compiled with OpenNI support\n";
}



se::OpenNIReader::~OpenNIReader()
{
}



void se::OpenNIReader::restart()
{
    se::Reader::restart();
}



std::string se::OpenNIReader::name() const
{
    return std::string("OpenNIReader");
}



se::ReaderStatus se::OpenNIReader::nextDepth(se::Image<float>&)
{
    return se::ReaderStatus::error;
}



se::ReaderStatus se::OpenNIReader::nextColour(se::Image<rgb_t>&)
{
    return se::ReaderStatus::error;
}

#endif // SE_OPENNI2
