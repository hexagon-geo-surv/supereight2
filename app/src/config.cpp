/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "config.hpp"

#include "se/common/filesystem.hpp"
#include "se/common/str_utils.hpp"
#include "se/common/yaml.hpp"



namespace se {
AppConfig::AppConfig() :
        enable_ground_truth(false),
        enable_rendering(true),
        enable_gui(true),
        enable_meshing(false),
        enable_slice_meshing(false),
        enable_structure_meshing(),
        mesh_output_dir(""),
        sensor_downsampling_factor(1),
        tracking_rate(1),
        integration_rate(1),
        rendering_rate(4),
        meshing_rate(100),
        max_frames(-1),
        log_file("")
{
}



AppConfig::AppConfig(const std::string& yaml_file) : AppConfig::AppConfig()
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

    // Get the node containing the app configuration.
    const cv::FileNode node = fs["app"];
    if (node.type() != cv::FileNode::MAP) {
        std::cerr << "Warning: using default app configuration, no \"app\" section found in "
                  << yaml_file << "\n";
        return;
    }

    // Read the config parameters.
    se::yaml::subnode_as_bool(node, "enable_ground_truth", enable_ground_truth);
    se::yaml::subnode_as_bool(node, "enable_rendering", enable_rendering);
    se::yaml::subnode_as_bool(node, "enable_gui", enable_gui);
    se::yaml::subnode_as_bool(node, "enable_meshing", enable_meshing);
    se::yaml::subnode_as_bool(node, "enable_slice_meshing", enable_slice_meshing);
    se::yaml::subnode_as_bool(node, "enable_structure_meshing", enable_structure_meshing);
    se::yaml::subnode_as_string(node, "mesh_output_dir", mesh_output_dir);
    se::yaml::subnode_as_int(node, "sensor_downsampling_factor", sensor_downsampling_factor);
    se::yaml::subnode_as_int(node, "tracking_rate", tracking_rate);
    se::yaml::subnode_as_int(node, "integration_rate", integration_rate);
    se::yaml::subnode_as_int(node, "rendering_rate", rendering_rate);
    se::yaml::subnode_as_int(node, "meshing_rate", meshing_rate);
    se::yaml::subnode_as_int(node, "max_frames", max_frames);
    se::yaml::subnode_as_string(node, "log_file", log_file);

    // Expand ~ in the paths.
    mesh_output_dir = se::str_utils::expand_user(mesh_output_dir);
    log_file = se::str_utils::expand_user(log_file);

    // If the mesh_output_dir or log_file contain relative paths, interpret them as relative to the
    // directory where filename is located.
    const stdfs::path dataset_dir = stdfs::path(yaml_file).parent_path();
    const stdfs::path mesh_output_dir_p(mesh_output_dir);
    if (mesh_output_dir_p.is_relative()) {
        mesh_output_dir = dataset_dir / mesh_output_dir_p;
    }
    const stdfs::path log_file_p(log_file);
    if (log_file_p.is_relative()) {
        log_file = dataset_dir / log_file_p;
    }
}



std::ostream& operator<<(std::ostream& os, const AppConfig& c)
{
    os << str_utils::bool_to_pretty_str(c.enable_ground_truth, "enable_ground_truth") << "\n";
    os << str_utils::bool_to_pretty_str(c.enable_rendering, "enable_rendering") << "\n";
    os << str_utils::bool_to_pretty_str(c.enable_gui, "enable_gui") << "\n";
    os << str_utils::bool_to_pretty_str(c.enable_meshing, "enable_meshing") << "\n";
    os << str_utils::bool_to_pretty_str(c.enable_slice_meshing, "enable_slice_meshing") << "\n";
    os << str_utils::str_to_pretty_str(c.mesh_output_dir, "mesh_output_dir") << "\n";
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
