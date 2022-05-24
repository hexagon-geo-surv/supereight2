/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2022 Nils Funk
 * SPDX-FileCopyrightText: 2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "config.hpp"

#include "se/common/filesystem.hpp"
#include "se/common/str_utils.hpp"
#include "se/common/yaml.hpp"


std::string process_path(const std::string& path, const std::string& dataset_dir)
{
    // Expand leading ~ in the path if possible and interpret it as relative to dataset_dir if it's
    // a relative path.
    return se::str_utils::resolve_relative_path(se::str_utils::expand_user(path), dataset_dir);
}

namespace se {

void AppConfig::readYaml(const std::string& filename)
{
    // Open the file for reading.
    cv::FileStorage fs;
    try {
        if (!fs.open(filename, cv::FileStorage::READ | cv::FileStorage::FORMAT_YAML)) {
            std::cerr << "Error: couldn't read configuration file " << filename << "\n";
            return;
        }
    }
    catch (const cv::Exception& e) {
        // OpenCV throws if the file contains non-YAML data.
        std::cerr << "Error: invalid YAML in configuration file " << filename << "\n";
        return;
    }

    // Get the node containing the app configuration.
    const cv::FileNode node = fs["app"];
    if (node.type() != cv::FileNode::MAP) {
        std::cerr << "Warning: using default app configuration, no \"app\" section found in "
                  << filename << "\n";
        return;
    }

    // Read the config parameters.
    se::yaml::subnode_as_bool(node, "enable_ground_truth", enable_ground_truth);
    se::yaml::subnode_as_bool(node, "enable_rendering", enable_rendering);
    se::yaml::subnode_as_bool(node, "enable_gui", enable_gui);
    se::yaml::subnode_as_bool(node, "enable_meshing", enable_meshing);
    se::yaml::subnode_as_bool(node, "enable_slice_meshing", enable_slice_meshing);
    se::yaml::subnode_as_bool(node, "enable_structure_meshing", enable_structure_meshing);
    se::yaml::subnode_as_string(node, "mesh_path", mesh_path);
    se::yaml::subnode_as_int(node, "sensor_downsampling_factor", sensor_downsampling_factor);
    se::yaml::subnode_as_int(node, "tracking_rate", tracking_rate);
    se::yaml::subnode_as_int(node, "integration_rate", integration_rate);
    se::yaml::subnode_as_int(node, "rendering_rate", rendering_rate);
    se::yaml::subnode_as_int(node, "meshing_rate", meshing_rate);
    se::yaml::subnode_as_int(node, "max_frames", max_frames);
    se::yaml::subnode_as_string(node, "log_file", log_file);

    const stdfs::path dataset_dir = stdfs::path(filename).parent_path();
    mesh_path = process_path(mesh_path, dataset_dir);
    log_file = process_path(log_file, dataset_dir);
}



std::ostream& operator<<(std::ostream& os, const AppConfig& c)
{
    os << str_utils::bool_to_pretty_str(c.enable_ground_truth, "enable_ground_truth") << "\n";
    os << str_utils::bool_to_pretty_str(c.enable_rendering, "enable_rendering") << "\n";
    os << str_utils::bool_to_pretty_str(c.enable_gui, "enable_gui") << "\n";
    os << str_utils::bool_to_pretty_str(c.enable_meshing, "enable_meshing") << "\n";
    os << str_utils::bool_to_pretty_str(c.enable_slice_meshing, "enable_slice_meshing") << "\n";
    os << str_utils::str_to_pretty_str(c.mesh_path, "mesh_path") << "\n";
    os << str_utils::value_to_pretty_str(c.sensor_downsampling_factor, "sensor_downsampling_factor")
       << "\n";
    os << str_utils::value_to_pretty_str(c.tracking_rate, "tracking_rate") << "\n";
    os << str_utils::value_to_pretty_str(c.integration_rate, "integration_rate") << "\n";
    os << str_utils::value_to_pretty_str(c.rendering_rate, "rendering_rate") << "\n";
    os << str_utils::value_to_pretty_str(c.meshing_rate, "meshing_rate") << "\n";
    os << str_utils::value_to_pretty_str(c.max_frames, "max_frames") << "\n";
    os << str_utils::str_to_pretty_str(c.log_file, "log_file") << "\n";
    return os;
}
} // namespace se
