/*
 * SPDX-FileCopyrightText: 2021-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021-2022 Nils Funk
 * SPDX-FileCopyrightText: 2021-2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_DATA_COLOUR_HPP
#define SE_DATA_COLOUR_HPP

#include "utils/setup_util.hpp"
#include "utils/type_util.hpp"

namespace se {

// Defaults
static constexpr se::rgba_t dflt_rgba = 0xFFFFFFFF; // White
static constexpr se::rgba_t dflt_delta_rgba = 0;

// Colour data
template<se::Colour ColB>
struct ColourData {
};

template<>
struct ColourData<se::Colour::On> {
    ColourData() : rgba(dflt_rgba)
    {
    }
    se::rgba_t rgba;
};

///////////////////
/// DELTA DATA  ///
///////////////////

// Colour data
template<se::Colour ColB>
struct ColourDeltaData {
};

template<>
struct ColourDeltaData<se::Colour::On> {
    ColourDeltaData() : delta_rgba(dflt_delta_rgba)
    {
    }
    se::rgba_t delta_rgba;
};



///////////////////
/// DATA CONFIG ///
///////////////////

// Colour data
template<se::Colour ColB>
struct ColourDataConfig {
    ColourDataConfig()
    {
    }
    ColourDataConfig(const std::string& /* yaml_file */)
    {
    }
};

template<se::Colour ColB>
std::ostream& operator<<(std::ostream& os, const ColourDataConfig<ColB>& /* c */)
{
    return os;
}

template<>
struct ColourDataConfig<se::Colour::On> {
    /** Initializes the config to some sensible defaults.
     */
    ColourDataConfig();

    /** Initializes the config from a YAML file. Data not present in the YAML file will be
     * initialized as in ColourDataConfig<se::Colour::On>::ColourDataConfig().
     */
    ColourDataConfig(const std::string& yaml_file);
};

std::ostream& operator<<(std::ostream& os, const ColourDataConfig<se::Colour::On>& c);

} // namespace se

#endif // SE_DATA_COLOUR_HPP
