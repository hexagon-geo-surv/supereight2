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
static constexpr rgba_t dflt_rgba = 0xFFFFFFFF; // White
static constexpr rgba_t dflt_delta_rgba = 0;

// Colour data
template<Colour ColB>
struct ColourData {
};

template<>
struct ColourData<Colour::On> {
    rgba_t rgba = dflt_rgba;
};

///////////////////
/// DELTA DATA  ///
///////////////////

// Colour data
template<Colour ColB>
struct ColourDeltaData {
};

template<>
struct ColourDeltaData<Colour::On> {
    rgba_t delta_rgba = dflt_delta_rgba;
};



///////////////////
/// DATA CONFIG ///
///////////////////

// Colour data
template<Colour ColB>
struct ColourDataConfig {
    ColourDataConfig()
    {
    }
    ColourDataConfig(const std::string& /* yaml_file */)
    {
    }
};

template<Colour ColB>
std::ostream& operator<<(std::ostream& os, const ColourDataConfig<ColB>& /* c */)
{
    return os;
}

template<>
struct ColourDataConfig<Colour::On> {
    /** Initializes the config to some sensible defaults.
     */
    ColourDataConfig();

    /** Initializes the config from a YAML file. Data not present in the YAML file will be
     * initialized as in ColourDataConfig<se::Colour::On>::ColourDataConfig().
     */
    ColourDataConfig(const std::string& yaml_file);
};

std::ostream& operator<<(std::ostream& os, const ColourDataConfig<Colour::On>& c);

} // namespace se

#endif // SE_DATA_COLOUR_HPP
