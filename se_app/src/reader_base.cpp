/*
 * SPDX-FileCopyrightText: 2014 University of Edinburgh, Imperial College London, University of Manchester.
 * SPDX-FileCopyrightText: 2020 Smart Robotics Lab, Imperial College London
 * SPDX-FileCopyrightText: 2020 Sotiris Papatheodorou
 * SPDX-License-Identifier: MIT
 * Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1
 */

#include "reader_base.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <sstream>
#include <thread>

#include "se/str_utils.hpp"



se::Reader::Reader(const se::ReaderConfig& c)
    : camera_active_(true),
      camera_open_(true),
      sequence_path_(c.sequence_path),
      ground_truth_file_(c.ground_truth_file),
      depth_image_res_(1, 1),
      rgba_image_res_(1, 1),
      fps_(c.fps),
      spf_(1.0 / c.fps),
      drop_frames_(c.drop_frames),
      enable_print_(c.enable_print),
      is_live_reader_(false),
      status_(se::ReaderStatus::ok),
      frame_(SIZE_MAX),
      num_frames_(0),
      ground_truth_frame_(SIZE_MAX) {
  // Open the ground truth file if supplied
  if (!ground_truth_file_.empty()) {
    ground_truth_fs_.open(ground_truth_file_, std::ios::in);
    if (!ground_truth_fs_.good()) {
      std::cerr << "Error: Could not read ground truth file "
          << ground_truth_file_ << "\n";
      status_ = se::ReaderStatus::error;
      camera_active_ = false;
      camera_open_ = false;
    }
  }
  // Ensure the available clock has enough accuracy to measure the requested
  // inter-frame time intervals. Compare the clock tick interval with the
  // seconds per frame.
  using se_clock = std::chrono::steady_clock;
  constexpr double clock_tick_period = se_clock::period::num / static_cast<double>(se_clock::period::den);
  if (clock_tick_period > spf_) {
    std::cerr << "Error: The inter-frame time interval must be larger than the clock tick period. Use a lower FPS value\n";
    exit(EXIT_FAILURE);
  }
}



se::ReaderStatus se::Reader::nextData(se::Image<float>& depth_image) {
  if (!good()) {
    return status_;
  }
  nextFrame();
  status_ = nextDepth(depth_image);
  if (!good()) {
    camera_active_ = false;
    camera_open_ = false;
  }
  return status_;
}



se::ReaderStatus se::Reader::nextData(se::Image<float>&    depth_image,
                                      se::Image<uint32_t>& rgba_image) {
  if (!good()) {
    return status_;
  }
  nextFrame();
  status_ = nextDepth(depth_image);
  if (!good()) {
    camera_active_ = false;
    camera_open_ = false;
    return status_;
  }
  status_ = mergeStatus(nextRGBA(rgba_image), status_);
  if (!good()) {
    camera_active_ = false;
    camera_open_ = false;
  }
  return status_;
}



se::ReaderStatus se::Reader::nextData(se::Image<float>&    depth_image,
                                      se::Image<uint32_t>& rgba_image,
                                      Eigen::Matrix4f&     T_WB) {
  if (!good()) {
    return status_;
  }
  nextFrame();
  status_ = nextDepth(depth_image);
  if (!good()) {
    camera_active_ = false;
    camera_open_ = false;
    return status_;
  }
  status_ = mergeStatus(nextRGBA(rgba_image), status_);
  if (!good()) {
    camera_active_ = false;
    camera_open_ = false;
    return status_;
  }
  status_ = mergeStatus(nextPose(T_WB), status_);
  if (!good()) {
    camera_active_ = false;
    camera_open_ = false;
  }
  return status_;
}



void se::Reader::restart() {
  frame_ = SIZE_MAX;
  ground_truth_frame_ = SIZE_MAX;
  prev_frame_timestamp_ = std::chrono::steady_clock::time_point();
  if (ground_truth_fs_.good() || ground_truth_fs_.eof()) {
    ground_truth_fs_.clear();
    ground_truth_fs_.seekg(0);
  }
}



bool se::Reader::good() const {
  return ((status_ == se::ReaderStatus::ok)
      || (status_ == se::ReaderStatus::skip));
}



size_t se::Reader::frame() const {
  return frame_;
}



size_t se::Reader::numFrames() const {
  return num_frames_;
}



Eigen::Vector2i se::Reader::depthImageRes() const {
  return depth_image_res_;
}



Eigen::Vector2i se::Reader::RGBAImageRes() const {
  return rgba_image_res_;
}



bool se::Reader::isLiveReader() const {
  return is_live_reader_;
}



se::ReaderStatus se::Reader::mergeStatus(se::ReaderStatus status_1,
                                         se::ReaderStatus status_2) {
  const int istatus_1 = static_cast<int>(status_1);
  const int istatus_2 = static_cast<int>(status_2);
  return static_cast<se::ReaderStatus>(std::max(istatus_1, istatus_2));
}



se::ReaderStatus se::Reader::nextPose(Eigen::Matrix4f& T_WB) {
  return readPose(T_WB, frame_);
}

se::ReaderStatus se::Reader::getPose(Eigen::Matrix4f& T_WB, const size_t frame) {
  // Store and reset current ground truth frame
  size_t ground_truth_frame_curr = ground_truth_frame_;
  ground_truth_frame_ = SIZE_MAX;

  // Store and reset current ground truth file stream
  std::fpos ground_truth_fs_pos_curr = ground_truth_fs_.tellg();
  std::ios_base::iostate ground_truth_fs_state_curr = ground_truth_fs_.rdstate();
  ground_truth_fs_.clear();
  ground_truth_fs_.seekg(0);

  auto status = readPose(T_WB, frame);

  // Restore current state of ground truth variables
  ground_truth_frame_ = ground_truth_frame_curr;
  ground_truth_fs_.setstate(ground_truth_fs_state_curr);
  ground_truth_fs_.seekg(ground_truth_fs_pos_curr);
  return status;
}


se::ReaderStatus se::Reader::readPose(Eigen::Matrix4f& T_WB, const size_t frame) {
  std::string line;
  while (true) {
    std::getline(ground_truth_fs_, line);
    // EOF reached
    if (!ground_truth_fs_.good()) {
      return se::ReaderStatus::eof;
    }
    // Ignore comment lines
    if (line[0] == '#') {
      continue;
    }
    // Skip ground truth data until the ones corresponding to the current frame
    // are found. This only happens when frames are dropped.
    ground_truth_frame_++;
    if (ground_truth_frame_ < frame) {
      continue;
    }
    // Data line read, split on spaces
    const std::vector<std::string> line_data = str_utils::split_str(line, ' ');
    const size_t num_cols = line_data.size();
    if (num_cols < 7) {
      std::cerr << "Error: Invalid ground truth file format. "
                << "Expected line format: ... tx ty tz qx qy qz qw\n";
      camera_active_ = false;
      camera_open_ = false;
      return se::ReaderStatus::error;
    }
    // Convert the last 7 columns to float
    float pose_data[7];
    bool pose_data_valid = true;
    for (uint8_t i = 0; i < 7; ++i) {
      pose_data[i] = std::stof(line_data[num_cols + i - 7]);
      if (!std::isfinite(pose_data[i])) {
        pose_data_valid = false;
      }
    }
    // Ensure all the pose elements are finite
    if (!pose_data_valid) {
      if (enable_print_) {
        std::cerr << "Warning: Expected finite ground truth pose but got";
        for (uint8_t i = 0; i < 7; ++i) {
          std::cerr << " " << pose_data[i];
        }
        std::cerr << "\n";
      }
      return se::ReaderStatus::skip;
    }
    // Convert to position and orientation
    const Eigen::Vector3f position (pose_data[0], pose_data[1], pose_data[2]);
    const Eigen::Quaternionf orientation (pose_data[6], pose_data[3],
                                          pose_data[4], pose_data[5]);
    // Ensure the quaternion represents a valid orientation
    if (std::abs(orientation.norm() - 1.0f) > 1e-3) {
      if (enable_print_) {
        std::cerr << "Warning: Expected unit quaternion but got "
                  << orientation.x() << " " << orientation.y() << " "
                  << orientation.z() << " " << orientation.w()
                  << " (x,y,z,w) with norm " << orientation.norm() << "\n";
      }
      return se::ReaderStatus::skip;
    }
    // Combine into the pose
    T_WB = Eigen::Matrix4f::Identity();
    T_WB.block<3,1>(0,3) = position;
    T_WB.block<3,3>(0,0) = orientation.toRotationMatrix();
    return se::ReaderStatus::ok;
  }
}


void se::Reader::nextFrame() {
  // Just increment the frame number when no FPS was specified or a live camera
  // is being used.
  if ((fps_ == 0.0f) || is_live_reader_) {
    frame_++;
    return;
  }

  // Avoid huge names in this function.
  namespace chr = std::chrono;

  // Compute the seconds since the previous frame was read.
  const chr::steady_clock::time_point curr_frame_timestamp
      = chr::steady_clock::now();
  const double delta_t = chr::duration<double>(
      curr_frame_timestamp - prev_frame_timestamp_).count();

  if (delta_t <= spf_) {
    // The previous frame was processed quickly, wait until it's time to get
    // the next one.
    const double seconds_to_sleep = spf_ - delta_t;
    std::this_thread::sleep_for(chr::duration<double>(seconds_to_sleep));
    frame_++;
  } else {
    // The previous frame was processed too slowly. Ensure that no frames are
    // dropped if no frames have been read yet (frame_ == SIZE_MAX).
    if (drop_frames_ && (frame_ != SIZE_MAX)) {
      const size_t delta_frame = std::ceil(fps_ * delta_t);
      frame_ += delta_frame;
    } else {
      frame_++;
    }
  }

  // Record the time right after frame_ was incremented. Due to the call to
  // std::this_thread::sleep_for it might be drastically different than
  // curr_frame_timestamp.
  prev_frame_timestamp_ = chr::steady_clock::now();
}

