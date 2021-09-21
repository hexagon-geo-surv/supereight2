// SPDX-FileCopyrightText: 2016 Emanuele Vespa, Imperial College London
// SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London
// SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou, Imperial College London
// SPDX-License-Identifier: BSD-3-Clause

#include "config.hpp"

#include "se/common/filesystem.hpp"
#include "se/common/yaml.hpp"

namespace se {
  AppConfig::AppConfig()
    : enable_ground_truth(false), enable_rendering(true), enable_gui(true), enable_meshing(false),
      enable_slice_meshing(false), enable_structure_meshing(), mesh_output_dir(""),
      sensor_downsampling_factor(1), tracking_rate(1), integration_rate(1), rendering_rate(4),
      meshing_rate(100), max_frames(-1), log_file("")
  {
  }



  AppConfig::AppConfig(const std::string& yaml_file)
    : AppConfig::AppConfig()
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
    const stdfs::path mesh_output_dir_p (mesh_output_dir);
    if (mesh_output_dir_p.is_relative()) {
      mesh_output_dir = dataset_dir / mesh_output_dir_p;
    }
    const stdfs::path log_file_p (log_file);
    if (log_file_p.is_relative()) {
      log_file = dataset_dir / log_file_p;
    }
  }



  std::ostream& operator<<(std::ostream& os, const AppConfig& c)
  {
    os << "enable_ground_truth:         " << (c.enable_ground_truth ? "yes" : "no") << "\n";
    os << "enable_rendering:            " << (c.enable_rendering ? "yes" : "no") << "\n";
    os << "enable_gui:                  " << (c.enable_gui ? "yes" : "no") << "\n";
    os << "enable_meshing:              " << (c.enable_meshing ? "yes" : "no") << "\n";
    os << "enable_slice_meshing:        " << (c.enable_slice_meshing ? "yes" : "no") << "\n";
    os << "enable_structure_meshing:    " << (c.enable_structure_meshing ? "yes" : "no") << "\n";
    os << "mesh_output_dir:             " << c.mesh_output_dir << "\n";
    os << "sensor_downsampling_factor:  " << c.sensor_downsampling_factor << "\n";
    os << "tracking_rate:               " << c.tracking_rate << "\n";
    os << "integration_rate:            " << c.integration_rate << "\n";
    os << "rendering_rate:              " << c.rendering_rate << "\n";
    os << "meshing_rate:                " << c.meshing_rate << "\n";
    os << "max_frames:                  " << c.max_frames << "\n";
    os << "log_file:                    " << c.log_file << "\n";
    return os;
  }
} // namespace se

