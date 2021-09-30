// SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London
// SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou, Imperial College London
// SPDX-License-Identifier: BSD-3-Clause

#include "se/map/map.hpp"

#include "se/common/yaml.hpp"

namespace se {
  MapConfig::MapConfig() :
      dim(10, 10, 3),
      res(0.1),
      T_MW(se::math::to_transformation(dim / 2))
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
      T_MW = se::math::to_transformation(dim / 2);
    } else {
      Eigen::Vector3f t_MW;
      se::yaml::subnode_as_vector3f(node, "t_MW", t_MW);
      T_MW = se::math::to_transformation(t_MW);
    }
  }



  std::ostream& operator<<(std::ostream& os, const MapConfig& c)
  {
    os << "dim:     " << c.dim.x() << " x " << c.dim.y() << " x " << c.dim.z() << " m\n";
    os << "res:     " << c.res << " m\n";
    os << "t_MW:    [" << se::math::to_translation(c.T_MW).x() << ", "
                       << se::math::to_translation(c.T_MW).y() << ", "
                       << se::math::to_translation(c.T_MW).z() << "] m\n";
    return os;
  }
} // namespace se

