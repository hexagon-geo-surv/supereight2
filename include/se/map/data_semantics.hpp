/*
 * SPDX-FileCopyrightText: 2021-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021-2022 Nils Funk
 * SPDX-FileCopyrightText: 2021-2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_DATA_SEMANTICS_HPP
#define SE_DATA_SEMANTICS_HPP

#include "utils/setup_util.hpp"
#include "utils/type_util.hpp"

namespace se {

// Defaults
static constexpr semantics_t dflt_semantics = 0;

// Semantic data
template<Semantics SemB>
struct SemanticData {
    struct Config {
        Config()
        {
        }
        Config(const std::string& /* yaml_file */)
        {
        }
    };
};

template<Semantics SemB>
std::ostream& operator<<(std::ostream& os, const typename SemanticData<SemB>::Config& /* c */)
{
    return os;
}

// Semantic data
template<>
struct SemanticData<Semantics::On> {
    struct Config {
        /** Initializes the config to some sensible defaults.
         */
        Config();

        /** Initializes the config from a YAML file. Data not present in the YAML file will be
         * initialized as in SemanticData<se::Semantics::On>::Config().
         */
        Config(const std::string& yaml_file);
    };

    semantics_t sem = dflt_semantics;
};

template<>
std::ostream& operator<< <Semantics::On>(std::ostream& os,
                                         const SemanticData<Semantics::On>::Config& c);

///////////////////
/// DELTA DATA  ///
///////////////////


} // namespace se

#endif // SE_DATA_SEMANTICS_HPP
