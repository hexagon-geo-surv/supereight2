// SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London
// SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou, Imperial College London
// SPDX-License-Identifier: BSD-3-Clause

#include "se/data.hpp"

#include "se/yaml.hpp"

namespace se {
  FieldDataConfig<Field::Occupancy>::FieldDataConfig()
    : min_occupancy(-500.0f), max_occupancy(500.0f), surface_boundary(0.0f)
  {
  }



  FieldDataConfig<Field::Occupancy>::FieldDataConfig(const std::string& yaml_file)
    : FieldDataConfig<Field::Occupancy>::FieldDataConfig()
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

    // Get the node containing the data configuration.
    const cv::FileNode node = fs["data"];
    if (node.type() != cv::FileNode::MAP) {
      std::cerr << "Warning: using default data configuration, no \"data\" section found in "
        << yaml_file << "\n";
      return;
    }

    // Read the config parameters.
    se::yaml::subnode_as_float(node, "min_occupancy", min_occupancy);
    se::yaml::subnode_as_float(node, "max_occupancy", max_occupancy);
    se::yaml::subnode_as_float(node, "surface_boundary", surface_boundary);
  }



  std::ostream& operator<<(std::ostream& os, const FieldDataConfig<se::Field::Occupancy>& c)
  {
    os << "min_occupancy:     " << c.min_occupancy << " log-odds\n";
    os << "max_occupancy:     " << c.max_occupancy << " log-odds\n";
    os << "surface_boundary:  " << c.surface_boundary << " log-odds\n";
    return os;
  }



  FieldDataConfig<Field::TSDF>::FieldDataConfig()
    : truncation_boundary(0.1f), max_weight(100)
  {
  }



  FieldDataConfig<Field::TSDF>::FieldDataConfig(const std::string& yaml_file)
    : FieldDataConfig<Field::TSDF>::FieldDataConfig()
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

    // Get the node containing the data configuration.
    const cv::FileNode node = fs["data"];
    if (node.type() != cv::FileNode::MAP) {
      std::cerr << "Warning: using default data configuration, no \"data\" section found in "
        << yaml_file << "\n";
      return;
    }

    // Read the config parameters.
    // Don't show a warning if truncation_boundary is not available, just keep its default value.
    // It will be set later using the truncation_boundary_factor.
    if (!node["truncation_boundary"].isNone()) {
      se::yaml::subnode_as_float(node, "truncation_boundary", truncation_boundary);
    }
    se::yaml::subnode_as_int(node, "max_weight", max_weight);
  }



  std::ostream& operator<<(std::ostream& os, const FieldDataConfig<se::Field::TSDF>& c)
  {
    os << "truncation_boundary:  " << c.truncation_boundary << " m\n";
    os << "max_weight:           " << c.max_weight << "\n";
    return os;
  }



  ColourDataConfig<se::Colour::On>::ColourDataConfig()
  {
    // TODO Implement when colour fusion is added.
  }



  ColourDataConfig<se::Colour::On>::ColourDataConfig(const std::string& /* yaml_file */)
    : ColourDataConfig<se::Colour::On>::ColourDataConfig()
  {
    // TODO Implement when colour fusion is added.
  }



  std::ostream& operator<<(std::ostream& os, const ColourDataConfig<se::Colour::On>& /* c */)
  {
    // TODO Implement when colour fusion is added.
    return os;
  }



  SemanticDataConfig<se::Semantics::On>::SemanticDataConfig()
  {
    // TODO Implement when semantics fusion is added.
  }



  SemanticDataConfig<se::Semantics::On>::SemanticDataConfig(const std::string& /* yaml_file */)
    : SemanticDataConfig<se::Semantics::On>::SemanticDataConfig()
  {
    // TODO Implement when semantics fusion is added.
  }



  std::ostream& operator<<(std::ostream& os, const SemanticDataConfig<se::Semantics::On>& /* c */)
  {
    // TODO Implement when semantics fusion is added.
    return os;
  }
} // namespace se

