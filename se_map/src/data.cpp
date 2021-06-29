// SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London
// SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou, Imperial College London
// SPDX-License-Identifier: BSD-3-Clause

#include "se/data.hpp"

#include "se/yaml.hpp"

namespace se {
  FieldDataConfig<Field::Occupancy>::FieldDataConfig() :
      k_sigma(0.052f),
      sigma_min_factor(1.5f),
      sigma_max_factor(6.f),
      sigma_min(0.2 * 1.5f),
      sigma_max(0.2 * 6.f),
      k_tau(0.026f),
      tau_min_factor(6.f),
      tau_max_factor(16.f),
      tau_min(0.2 * 6.f),
      tau_max(0.2 * 16.f),
      min_occupancy(-100.f),
      max_occupancy(100.f),
      surface_boundary(0.f),
      log_odd_min(-5.015),
      log_odd_max(5.015),
      fs_integr_scale(1),
      uncertainty_model(UncertaintyModel::linear),
      const_surface_thickness(false)
  {
    max_weight = floor(abs(min_occupancy / (0.97 * log_odd_min)));
    factor     = (max_weight - 1) / max_weight;
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
    se::yaml::subnode_as_float(node, "k_sigma", k_sigma);
    se::yaml::subnode_as_float(node, "sigma_min", sigma_min);
    se::yaml::subnode_as_float(node, "sigma_max", sigma_max);
    se::yaml::subnode_as_float(node, "k_tau", k_tau);
    se::yaml::subnode_as_float(node, "tau_min", tau_min);
    se::yaml::subnode_as_float(node, "tau_max", tau_max);
    se::yaml::subnode_as_float(node, "min_occupancy", min_occupancy);
    se::yaml::subnode_as_float(node, "max_occupancy", max_occupancy);
    se::yaml::subnode_as_int(node, "max_weight", max_weight);               // TODO: Compute based on min_occupancy and log_odd_min
    se::yaml::subnode_as_float(node, "surface_boundary", surface_boundary);
    se::yaml::subnode_as_float(node, "log_odd_min", log_odd_min);
    se::yaml::subnode_as_float(node, "log_odd_max", log_odd_max);
    se::yaml::subnode_as_int(node, "fs_integr_scale", fs_integr_scale);
    std::string uncertainty_model_s;
    se::yaml::subnode_as_string(node, "uncertainty_model", uncertainty_model_s);

    if (uncertainty_model_s == "linear")
    {
      uncertainty_model = UncertaintyModel::linear;
    }

    if (uncertainty_model_s == "quadratic")
    {
      uncertainty_model = UncertaintyModel::quadratic;
    }

    se::yaml::subnode_as_bool(node, "const_surface_thickness", const_surface_thickness);

    max_weight = floor(abs(min_occupancy / (0.97 * log_odd_min)));
    factor     = (max_weight - 1) / max_weight;
  }



  std::ostream& operator<<(std::ostream& os, const FieldDataConfig<se::Field::Occupancy>& c)
  {
    os << "k_sigma:                 " << c.k_sigma << "\n";
    os << "sigma_min_factor:        " << c.sigma_min_factor << "\n";
    os << "sigma_max_factor:        " << c.sigma_max_factor << "\n";
    os << "sigma_min:               " << c.sigma_min << "\n";
    os << "sigma_max:               " << c.sigma_max << "\n";
    os << "k_tau:                   " << c.k_tau << "\n";
    os << "tau_min_factor:          " << c.tau_min_factor << "\n";
    os << "tau_max_factor:          " << c.tau_max_factor << "\n";
    os << "tau_min:                 " << c.tau_min << "\n";
    os << "tau_max:                 " << c.tau_max << "\n";
    os << "min_occupancy:           " << c.min_occupancy << " log-odds\n";
    os << "max_occupancy:           " << c.max_occupancy << " log-odds\n";
    os << "surface_boundary:        " << c.surface_boundary << " log-odds\n";
    os << "log_odd_min:             " << c.log_odd_min << " log-odds\n";
    os << "log_odd_max:             " << c.log_odd_max << " log-odds\n";
    os << "max_weight:              " << c.max_weight << "\n";
    os << "fs_integr_scale:         " << c.fs_integr_scale << "\n";
    os << "uncertainty_model:       " << ((c.uncertainty_model == UncertaintyModel::linear) ? "linear" : "quadratic") << "\n";
    os << "const_surface_thickness: " << c.const_surface_thickness << "\n";
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

