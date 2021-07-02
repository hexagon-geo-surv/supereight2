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
    : map(yaml_file), data(yaml_file), sensor(yaml_file), tracker(yaml_file), reader(yaml_file), app(yaml_file)
  {
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

