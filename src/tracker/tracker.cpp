// SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London
// SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou, Imperial College London
// SPDX-License-Identifier: BSD-3-Clause

#include "se/tracker/tracker.hpp"

#include "se/common/yaml.hpp"

namespace se {
  TrackerConfig::TrackerConfig()
    : iterations({10, 5, 4}), dist_threshold(0.1f), normal_threshold(0.8f), track_threshold(0.15f),
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
    os << "iterations:        [";
    for (size_t i = 0; i < c.iterations.size() - 1; ++i) {
      os << c.iterations[i] << ", ";
    }
    os << c.iterations.back() << "]\n";
    os << "dist_threshold:    " << c.dist_threshold << "\n";
    os << "normal_threshold:  " << c.normal_threshold << "\n";
    os << "track_threshold:   " << c.track_threshold << "\n";
    os << "icp_threshold:     " << c.icp_threshold << "\n";
    return os;
  }
} // namespace se

