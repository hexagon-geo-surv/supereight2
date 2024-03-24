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
FieldData<Field::Occupancy>::Config::Config() :
        k_sigma(0.052f),
        sigma_min_factor(1.5f),
        sigma_max_factor(6.f),
        k_tau(0.026f),
        tau_min_factor(6.f),
        tau_max_factor(16.f),
        log_odd_min(-5.015),
        log_odd_max(5.015),
        fs_integr_scale(1),
        uncertainty_model(UncertaintyModel::Linear)
{
    max_weight =
        std::floor(std::fabs(FieldData<Field::Occupancy>::min_occupancy / (0.97f * log_odd_min)));
}



FieldData<Field::Occupancy>::Config::Config(const std::string& yaml_file) :
        FieldData<Field::Occupancy>::Config()
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

    max_weight =
        std::floor(std::fabs(FieldData<Field::Occupancy>::min_occupancy / (0.97f * log_odd_min)));
}



std::ostream& operator<<(std::ostream& os, const FieldData<Field::Occupancy>::Config& c)
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
    return os;
}



FieldData<Field::TSDF>::Config::Config() : truncation_boundary_factor(8), max_weight(100)
{
}



FieldData<Field::TSDF>::Config::Config(const std::string& yaml_file) :
        FieldData<Field::TSDF>::Config()
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
    se::yaml::subnode_as_float(node, "max_weight", max_weight);
}



std::ostream& operator<<(std::ostream& os, const FieldData<Field::TSDF>::Config& c)
{
    os << str_utils::value_to_pretty_str(c.truncation_boundary_factor, "truncation_boundary_factor")
       << "x\n";
    os << str_utils::value_to_pretty_str(c.max_weight, "max_weight") << "\n";
    return os;
}



ColourData<se::Colour::On>::Config::Config()
{
    // TODO Implement when colour fusion is added.
}



ColourData<se::Colour::On>::Config::Config(const std::string& /* yaml_file */) :
        ColourData<se::Colour::On>::Config()
{
    // TODO Implement when colour fusion is added.
}



template<>
std::ostream& operator<< <se::Colour::On>(std::ostream& os,
                                          const ColourData<se::Colour::On>::Config& /* c */)
{
    // TODO Implement when colour fusion is added.
    return os;
}



SemanticData<se::Semantics::On>::Config::Config()
{
    // TODO Implement when semantics fusion is added.
}



SemanticData<se::Semantics::On>::Config::Config(const std::string& /* yaml_file */) :
        SemanticData<se::Semantics::On>::Config()
{
    // TODO Implement when semantics fusion is added.
}



template<>
std::ostream& operator<< <se::Semantics::On>(std::ostream& os,
                                             const SemanticData<se::Semantics::On>::Config& /* c */)
{
    // TODO Implement when semantics fusion is added.
    return os;
}
} // namespace se
