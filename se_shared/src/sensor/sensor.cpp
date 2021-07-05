// SPDX-FileCopyrightText: 2020 Sotiris Papatheodorou, Imperial College London
// SPDX-License-Identifier: BSD-3-Clause

#include <cassert>

#include "se/sensor/sensor.hpp"



se::SensorConfigBase::SensorConfigBase() :
    width(0),
    height(0),
    near_plane(0.0f),
    far_plane(INFINITY),
    left_hand_frame(false)
{
}



se::SensorConfigBase::SensorConfigBase(const std::string& yaml_file) : se::SensorConfigBase::SensorConfigBase()
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

  // Get the node containing the sensor configuration.
  const cv::FileNode node = fs["sensor"];
  if (node.type() != cv::FileNode::MAP) {
    std::cerr << "Warning: using default sensor configuration, no \"sensor\" section found in "
      << yaml_file << "\n";
    return;
  }

  // Read the config parameters.
  se::yaml::subnode_as_int(node,   "width",           width);
  se::yaml::subnode_as_int(node,   "height",          height);
  se::yaml::subnode_as_float(node, "near_plane",      near_plane);
  se::yaml::subnode_as_float(node, "far_plane",       far_plane);
  se::yaml::subnode_as_bool(node,  "left_hand_frame", left_hand_frame);
}