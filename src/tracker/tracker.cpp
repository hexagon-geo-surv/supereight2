/*
 * SPDX-FileCopyrightText: 2014 University of Edinburgh, Imperial College, University of Manchester
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: MIT
 */

#include "se/common/yaml.hpp"
#include "se/common/str_utils.hpp"
#include "se/tracker/tracker.hpp"



namespace se {



TrackerConfig::TrackerConfig() :
    iterations({10, 5, 4}),
    dist_threshold(0.1f),
    normal_threshold(0.8f),
    track_threshold(0.15f),
    icp_threshold(0.00001f)
{
}



TrackerConfig::TrackerConfig(const std::string& yaml_file)
  : TrackerConfig::TrackerConfig()
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

  // Get the node containing the tracker configuration.
  const cv::FileNode node = fs["tracker"];
  if (node.type() != cv::FileNode::MAP) {
    std::cerr << "Warning: using default tracker configuration, no \"tracker\" section found in "
      << yaml_file << "\n";
    return;
  }

  // Read the config parameters.
  se::yaml::subnode_as_vector(node, "iterations", iterations);
  se::yaml::subnode_as_float(node, "dist_threshold", dist_threshold);
  se::yaml::subnode_as_float(node, "normal_threshold", normal_threshold);
  se::yaml::subnode_as_float(node, "track_threshold", track_threshold);
  se::yaml::subnode_as_float(node, "icp_threshold", icp_threshold);
}



std::ostream& operator<<(std::ostream& os, const TrackerConfig& c)
{
  os << str_utils::vector_to_pretty_str(c.iterations,      "iterations")       << "\n";
  os << str_utils::value_to_pretty_str(c.dist_threshold,   "dist_threshold")   << "\n";
  os << str_utils::value_to_pretty_str(c.normal_threshold, "normal_threshold") << "\n";
  os << str_utils::value_to_pretty_str(c.track_threshold,  "track_threshold")  << "\n";
  os << str_utils::value_to_pretty_str(c.icp_threshold,    "icp_threshold")    << "\n";
  return os;
}
} // namespace se

