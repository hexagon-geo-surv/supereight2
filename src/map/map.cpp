// SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London
// SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou, Imperial College London
// SPDX-License-Identifier: BSD-3-Clause

#include "se/map/map.hpp"

#include "se/common/yaml.hpp"

namespace se {
  MapConfig::MapConfig()
    : dim(10, 10, 3), res(0.1), origin(dim / 2.0f)
  {
  }



  MapConfig::MapConfig(const std::string& yaml_file)
    : MapConfig::MapConfig()
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

    // Get the node containing the map configuration.
    const cv::FileNode node = fs["map"];
    if (node.type() != cv::FileNode::MAP) {
      std::cerr << "Warning: using default map configuration, no \"map\" section found in "
        << yaml_file << "\n";
      return;
    }

    // Read the config parameters.
    se::yaml::subnode_as_vector3f(node, "dim", dim);
    se::yaml::subnode_as_float(node, "res", res);
    if (node["origin"].isNone()) {
      // Don't show a warning if origin is not available, set it to dim / 2.
      origin = dim / 2.0f;
    } else {
      se::yaml::subnode_as_vector3f(node, "origin", origin);
    }
  }



  std::ostream& operator<<(std::ostream& os, const MapConfig& c)
  {
    os << "dim:     " << c.dim.x() << " x " << c.dim.y() << " x " << c.dim.z() << " m\n";
    os << "res:     " << c.res << " m\n";
    os << "origin:  [" << c.origin.x() << ", " << c.origin.y() << ", " << c.origin.z() << "] m\n";
    return os;
  }
} // namespace se

