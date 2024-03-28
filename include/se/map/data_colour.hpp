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

// Colour data
template<Colour ColB>
struct ColourData {
    struct Config {
        void readYaml(const std::string& /* yaml_file */)
        {
        }
    };
};

template<Colour ColB>
std::ostream& operator<<(std::ostream& os, const typename ColourData<ColB>::Config& /* c */)
{
    return os;
}

template<>
struct ColourData<Colour::On> {
    colour_t colour;
    std::uint8_t weight = 0;

    struct Config {
        /** Reads the struct members from the "data" node of a YAML file. Members not present in the
         * YAML file aren't modified.
         */
        void readYaml(const std::string& yaml_file);
    };

    /** Perform a weighted averge colour update using \p colour, while ensuring the weight doesn't
     * exceed \p max_weight. Return whether the data was updated.
     */
    bool update(const colour_t colour, const std::uint8_t max_weight);
};

template<>
std::ostream& operator<< <Colour::On>(std::ostream& os, const ColourData<Colour::On>::Config& c);

///////////////////
/// DELTA DATA  ///
///////////////////

// Colour data
template<Colour ColB>
struct ColourDeltaData {};

// Use signed 16-bit integers to allow storing all possible deltas from unsigned 8-bit integers.
template<>
struct ColourDeltaData<Colour::On> {
    struct {
        std::int16_t r = 0;
        std::int16_t g = 0;
        std::int16_t b = 0;
    } delta_colour;
    std::int16_t delta_colour_weight = 0;
};

} // namespace se

#include "impl/data_colour_impl.hpp"

#endif // SE_DATA_COLOUR_HPP
