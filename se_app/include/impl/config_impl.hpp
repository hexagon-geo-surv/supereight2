// SPDX-FileCopyrightText: 2016 Emanuele Vespa, Imperial College London
// SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London
// SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou, Imperial College London
// SPDX-License-Identifier: BSD-3-Clause

#ifndef SE_CONFIG_IMPL_HPP
#define SE_CONFIG_IMPL_HPP

#include "se/yaml.hpp"

namespace se {
  template<typename DataConfigT>
  Config<DataConfigT>::Config()
  {
  }



  template<typename DataConfigT>
  Config<DataConfigT>::Config(const std::string& yaml_file)
    : data(yaml_file), map(yaml_file), sensor(yaml_file), tracker(yaml_file), reader(yaml_file),
      app(yaml_file)
  {
    // Read the truncation_boundary_factor for TSDF implementations and compute the
    // truncation_boundary based on that and the map resolution.
    if constexpr (DataConfigT::FldT == se::Field::TSDF)
    {
      // Open the file for reading.
      cv::FileStorage fs;
      try
      {
        if (!fs.open(yaml_file, cv::FileStorage::READ | cv::FileStorage::FORMAT_YAML))
        {
          std::cerr << "Error: couldn't read configuration file " << yaml_file << "\n";
          return;
        }
      } catch (const cv::Exception& e)
      {
        // OpenCV throws if the file contains non-YAML data.
        std::cerr << "Error: invalid YAML in configuration file " << yaml_file << "\n";
        return;
      }

      // Get the node containing the data configuration.
      const cv::FileNode node = fs["data"];
      if (node.type() != cv::FileNode::MAP)
      {
        std::cerr << "Warning: using default data configuration, no \"data\" section found in "
          << yaml_file << "\n";
        return;
      }

      // Read the config parameters.
      float truncation_boundary_factor;
      se::yaml::subnode_as_float(node, "truncation_boundary_factor", truncation_boundary_factor);
      data.truncation_boundary = truncation_boundary_factor * map.res;
    }
  }



  template<typename DataConfigT>
  std::ostream& operator<<(std::ostream& os, const Config<DataConfigT>& c)
  {
    os << "Data config -----------------------\n";
    os << c.data;
    os << "Map config ------------------------\n";
    os << c.map;
    os << "Sensor config ---------------------\n";
    os << c.sensor;
    os << "Tracker config --------------------\n";
    os << c.tracker;
    os << "Reader config ---------------------\n";
    os << c.reader;
    os << "App config ------------------------\n";
    os << c.app;
    return os;
  }
} // namespace se

#endif // SE_CONFIG_IMPL_HPP

