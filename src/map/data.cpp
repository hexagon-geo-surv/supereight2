/*
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "se/map/data.hpp"

#include <cmath>

#include "se/common/str_utils.hpp"
#include "se/common/yaml.hpp"

namespace se {
FieldDataConfig<Field::Occupancy>::FieldDataConfig() :
        k_sigma(0.052f),
        sigma_min_factor(1.5f),
        sigma_max_factor(6.f),
        k_tau(0.026f),
        tau_min_factor(6.f),
        tau_max_factor(16.f),
        log_odd_min(-5.015),
        log_odd_max(5.015),
        fs_integr_scale(1),
        uncertainty_model(UncertaintyModel::Linear),
        const_surface_thickness(false)
{
    max_weight =
        std::floor(std::fabs(FieldData<Field::Occupancy>::min_occupancy / (0.97f * log_odd_min)));
}



FieldDataConfig<Field::Occupancy>::FieldDataConfig(const std::string& yaml_file) :
        FieldDataConfig<Field::Occupancy>::FieldDataConfig()
{
    // Open the file for reading.
    cv::FileStorage fs;
    try {
        if (!fs.open(yaml_file, cv::FileStorage::READ | cv::FileStorage::FORMAT_YAML)) {
            std::cerr << "Error: couldn't read configuration file " << yaml_file << "\n";
            return;
        }
    }
    catch (const cv::Exception& e) {
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
    se::yaml::subnode_as_float(node, "sigma_min_factor", sigma_min_factor);
    se::yaml::subnode_as_float(node, "sigma_max_factor", sigma_max_factor);
    se::yaml::subnode_as_float(node, "k_tau", k_tau);
    se::yaml::subnode_as_float(node, "tau_min_factor", tau_min_factor);
    se::yaml::subnode_as_float(node, "tau_max_factor", tau_max_factor);
    se::yaml::subnode_as_float(node, "log_odd_min", log_odd_min);
    se::yaml::subnode_as_float(node, "log_odd_max", log_odd_max);
    se::yaml::subnode_as_int(node, "fs_integr_scale", fs_integr_scale);
    std::string uncertainty_model_s;
    se::yaml::subnode_as_string(node, "uncertainty_model", uncertainty_model_s);

    if (uncertainty_model_s == "linear") {
        uncertainty_model = UncertaintyModel::Linear;
    }

    if (uncertainty_model_s == "quadratic") {
        uncertainty_model = UncertaintyModel::Quadratic;
    }

    se::yaml::subnode_as_bool(node, "const_surface_thickness", const_surface_thickness);

    max_weight =
        std::floor(std::fabs(FieldData<Field::Occupancy>::min_occupancy / (0.97f * log_odd_min)));
}



std::ostream& operator<<(std::ostream& os, const FieldDataConfig<se::Field::Occupancy>& c)
{
    os << str_utils::value_to_pretty_str(c.k_sigma, "k_sigma") << "\n";
    os << str_utils::value_to_pretty_str(c.sigma_min_factor, "sigma_min_factor") << "\n";
    os << str_utils::value_to_pretty_str(c.sigma_max_factor, "sigma_max_factor") << "\n";
    os << str_utils::value_to_pretty_str(c.k_tau, "k_tau") << "\n";
    os << str_utils::value_to_pretty_str(c.tau_min_factor, "tau_min_factor") << "\n";
    os << str_utils::value_to_pretty_str(c.tau_max_factor, "tau_max_factor") << "\n";
    os << str_utils::value_to_pretty_str(c.log_odd_min, "log_odd_min") << "\n";
    os << str_utils::value_to_pretty_str(c.log_odd_max, "log_odd_max") << "\n";
    os << str_utils::value_to_pretty_str(c.max_weight, "max_weight") << "\n";
    os << str_utils::value_to_pretty_str(c.fs_integr_scale, "fs_integr_scale") << "\n";
    os << str_utils::str_to_pretty_str(
        ((c.uncertainty_model == UncertaintyModel::Linear) ? "linear" : "quadratic"),
        "uncertainty_model")
       << "\n";
    os << str_utils::bool_to_pretty_str(c.const_surface_thickness, "const_surface_thickness")
       << "\n";
    return os;
}



FieldDataConfig<Field::TSDF>::FieldDataConfig() : truncation_boundary_factor(8), max_weight(100)
{
}



FieldDataConfig<Field::TSDF>::FieldDataConfig(const std::string& yaml_file) :
        FieldDataConfig<Field::TSDF>::FieldDataConfig()
{
    // Open the file for reading.
    cv::FileStorage fs;
    try {
        if (!fs.open(yaml_file, cv::FileStorage::READ | cv::FileStorage::FORMAT_YAML)) {
            std::cerr << "Error: couldn't read configuration file " << yaml_file << "\n";
            return;
        }
    }
    catch (const cv::Exception& e) {
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

    se::yaml::subnode_as_float(node, "truncation_boundary_factor", truncation_boundary_factor);
    // Ensure an integer max_weight is provided even in a float is used as weight_t.
    int max_weight_int = max_weight;
    se::yaml::subnode_as_int(node, "max_weight", max_weight_int);
    max_weight = max_weight_int;
}



std::ostream& operator<<(std::ostream& os, const FieldDataConfig<se::Field::TSDF>& c)
{
    os << str_utils::value_to_pretty_str(c.truncation_boundary_factor, "truncation_boundary_factor")
       << "x\n";
    os << str_utils::value_to_pretty_str(c.max_weight, "max_weight") << "\n";
    return os;
}



ColourDataConfig<se::Colour::On>::ColourDataConfig()
{
    // TODO Implement when colour fusion is added.
}



ColourDataConfig<se::Colour::On>::ColourDataConfig(const std::string& /* yaml_file */) :
        ColourDataConfig<se::Colour::On>::ColourDataConfig()
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



SemanticDataConfig<se::Semantics::On>::SemanticDataConfig(const std::string& /* yaml_file */) :
        SemanticDataConfig<se::Semantics::On>::SemanticDataConfig()
{
    // TODO Implement when semantics fusion is added.
}



std::ostream& operator<<(std::ostream& os, const SemanticDataConfig<se::Semantics::On>& /* c */)
{
    // TODO Implement when semantics fusion is added.
    return os;
}
} // namespace se
