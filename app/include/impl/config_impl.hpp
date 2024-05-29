/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2021-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021-2022 Nils Funk
 * SPDX-FileCopyrightText: 2021-2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_CONFIG_IMPL_HPP
#define SE_CONFIG_IMPL_HPP

namespace se {

template<typename MapT, typename SensorT>
Config<MapT, SensorT>::Config(const std::string& yaml_file)
{
    map.readYaml(yaml_file);
    data.readYaml(yaml_file);
    sensor.readYaml(yaml_file);
    tracker.readYaml(yaml_file);
    reader.readYaml(yaml_file);
    app.readYaml(yaml_file);
}



template<typename MapT, typename SensorT>
std::ostream& operator<<(std::ostream& os, const Config<MapT, SensorT>& c)
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
