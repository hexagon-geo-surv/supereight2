/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2019-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "se/map/map.hpp"

#include "se/common/yaml.hpp"
#include "se/common/str_utils.hpp"



namespace se {
  MapConfig::MapConfig() :
      dim(10, 10, 3),
      res(0.1),
      T_MW(se::math::to_transformation(Eigen::Vector3f(dim / 2)))
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
    se::yaml::subnode_as_eigen_vector3f(node, "dim", dim);
    se::yaml::subnode_as_float(node, "res", res);

    // Don't show a warning if origin is not available, set it to dim / 2.
    T_MW = se::math::to_transformation(Eigen::Vector3f(dim / 2));

    if (!node["T_MW"].isNone()) {
      se::yaml::subnode_as_eigen_matrix4f(node, "T_MW", T_MW);
    }

    if (!node["t_MW"].isNone()) {
      Eigen::Vector3f t_MW;
      se::yaml::subnode_as_eigen_vector3f(node, "t_MW", t_MW);
      T_MW.topRightCorner<3, 1>() = t_MW;
    }

    if (!node["R_MW"].isNone()) {
      Eigen::Matrix3f R_MW;
      se::yaml::subnode_as_eigen_matrix3f(node, "R_MW", R_MW);
      T_MW.topLeftCorner<3, 3>() = R_MW;
    }
  }



  std::ostream& operator<<(std::ostream& os, const MapConfig& c)
  {
    os << str_utils::volume_to_pretty_str(c.dim,        "dim")  << " m\n";
    os << str_utils::value_to_pretty_str(c.res,         "res")  << " m/voxel\n";
    os << str_utils::eigen_matrix_to_pretty_str(c.T_MW, "T_MW") << "\n";
    return os;
  }
} // namespace se

