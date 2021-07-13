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

#include "filesystem.hpp"
#include "se/common/str_utils.hpp"
#include "se/common/yaml.hpp"



se::ReaderType se::string_to_reader_type(const std::string& s)
{
  std::string s_lowered (s);
  se::str_utils::to_lower(s_lowered);
  if (s_lowered == "openni")
  {
    return se::ReaderType::OPENNI;
  }
  else if (s_lowered == "iclnuim")
  {
    return se::ReaderType::ICLNUIM;
  }
  else if (s_lowered == "raw")
  {
    return se::ReaderType::RAW;
  }
  else if (s_lowered == "newercollege")
  {
    return se::ReaderType::NEWERCOLLEGE;
  }
  else if (s_lowered == "tum")
  {
    return se::ReaderType::TUM;
  }
  else if (s_lowered == "interiornet")
  {
    return se::ReaderType::INTERIORNET;
  }
  else
  {
    return se::ReaderType::UNKNOWN;
  }
}



std::string se::reader_type_to_string(se::ReaderType t)
{
  if (t == se::ReaderType::OPENNI)
  {
    return "OpenNI";
  }
  else if (t == se::ReaderType::ICLNUIM)
  {
    return "ICLNUIM";
  }
  else if (t == se::ReaderType::RAW)
  {
    return "raw";
  }
  else if (t == se::ReaderType::NEWERCOLLEGE)
  {
    return "NewerCollege";
  }
  else if (t == se::ReaderType::TUM)
  {
    return "TUM";
  }
  else if (t == se::ReaderType::INTERIORNET)
  {
    return "InteriorNet";
  }
  else
  {
    return "unknown";
  }
}



se::ReaderConfig::ReaderConfig()
  : reader_type(se::ReaderType::RAW), sequence_path(""), ground_truth_file(""),
    inverse_scale(0), fps(30), drop_frames(false), verbose(0)
{
}



se::ReaderConfig::ReaderConfig(const std::string& yaml_file)
  : se::ReaderConfig::ReaderConfig()
{
  // Open the file for reading.
  cv::FileStorage fs;
  try {
    if (!fs.open(yaml_file, cv::FileStorage::READ | cv::FileStorage::FORMAT_YAML)) {
      std::cerr << "Error: couldn't read configuration file " << yaml_file << "\n";
      return;
    }
  } catch (const cv::Exception& e) {
    // OpenCV throws if the file contains non-YAML data.
    std::cerr << "Error: invalid YAML in configuration file " << yaml_file << "\n";
    return;
  }

  // Get the node containing the reader configuration.
  const cv::FileNode node = fs["reader"];
  if (node.type() != cv::FileNode::MAP) {
    std::cerr << "Warning: using default reader configuration, no \"reader\" section found in "
      << yaml_file << "\n";
    return;
  }

  // Read the config parameters.
  std::string reader_type_str;
  se::yaml::subnode_as_string(node, "reader_type",       reader_type_str);
  reader_type = string_to_reader_type(reader_type_str);
  se::yaml::subnode_as_float( node, "fps",               fps);
  se::yaml::subnode_as_float( node, "inverse_scale",     inverse_scale);
  se::yaml::subnode_as_bool(  node, "drop_frames",       drop_frames);
  se::yaml::subnode_as_int(   node, "verbose",           verbose);
  se::yaml::subnode_as_string(node, "sequence_path",     sequence_path);
  se::yaml::subnode_as_string(node, "ground_truth_file", ground_truth_file);

  // Expand ~ in the paths.
  sequence_path     = se::str_utils::expand_user(sequence_path);
  ground_truth_file = se::str_utils::expand_user(ground_truth_file);

  // If the sequence_path or ground_truth_file contain relative paths, interpret them as relative
  // to the directory where filename is located.
  const stdfs::path dataset_dir = stdfs::path(yaml_file).parent_path();
  const stdfs::path sequence_path_p (sequence_path);
  if (sequence_path_p.is_relative()) {
    sequence_path = dataset_dir / sequence_path_p;
  }
  const stdfs::path ground_truth_file_p (ground_truth_file);
  if (ground_truth_file_p.is_relative()) {
    ground_truth_file = dataset_dir / ground_truth_file_p;
  }
}



std::ostream& se::operator<<(std::ostream& os, const se::ReaderConfig& c)
{
  os << "reader_type:        " << se::reader_type_to_string(c.reader_type) << "\n";
  os << "sequence_path:      " << c.sequence_path << "\n";
  os << "ground_truth_file:  " << c.ground_truth_file << "\n";
  os << "fps:                " << c.fps << "\n";
  os << "drop_frames:        " << (c.drop_frames ? "yes" : "no") << "\n";
  os << "verbose:            " << c.verbose << "\n";
  return os;
}



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
      verbose_(c.verbose),
      is_live_reader_(false),
      status_(se::ReaderStatus::ok),
      frame_(SIZE_MAX),
      num_frames_(0),
      ground_truth_frame_(SIZE_MAX) {
  // Trim trailing slashes from sequence_path_
  sequence_path_.erase(sequence_path_.find_last_not_of("/") + 1);
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
    const std::vector<std::string> line_data = se::str_utils::split_str(line, ' ');
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
      if (verbose_ == 1) {
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
      if (verbose_ == 1) {
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
