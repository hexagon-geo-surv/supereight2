/*
 * SPDX-FileCopyrightText: 2021-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021-2022 Nils Funk
 * SPDX-FileCopyrightText: 2021-2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_DATA_COLOUR_HPP
#define SE_DATA_COLOUR_HPP

#include <se/common/bounded_vector.hpp>
#include <se/map/utils/setup_util.hpp>
#include <se/map/utils/type_util.hpp>

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

template<>
struct ColourData<Colour::On> {
    colour_t colour;
    std::uint8_t weight = 0;

    /** Perform a weighted averge colour update using \p colour, while ensuring the weight doesn't
     * exceed \p max_weight. Return whether the data was updated.
     */
    bool update(const colour_t colour, const std::uint8_t max_weight);

    /** Set to the mean of the data in \p child_data. */
    void setToMean(const BoundedVector<ColourData, 8>& child_data);

    struct Config {
        /** Reads the struct members from the "data" node of a YAML file. Members not present in the
         * YAML file aren't modified.
         */
        void readYaml(const std::string& yaml_file);
    };
};

std::ostream& operator<<(std::ostream& os, const ColourData<Colour::Off>::Config& c);
std::ostream& operator<<(std::ostream& os, const ColourData<Colour::On>::Config& c);

} // namespace se

#include "impl/data_colour_impl.hpp"

#endif // SE_DATA_COLOUR_HPP
