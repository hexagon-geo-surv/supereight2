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
};

// Semantic data
template<>
struct SemanticData<Semantics::On> {
    semantics_t sem = dflt_semantics;
};



///////////////////
/// DELTA DATA  ///
///////////////////



///////////////////
/// DATA CONFIG ///
///////////////////

// Semantic data
template<Semantics SemB>
struct SemanticDataConfig {
    SemanticDataConfig()
    {
    }
    SemanticDataConfig(const std::string& /* yaml_file */)
    {
    }
};

template<Semantics SemB>
std::ostream& operator<<(std::ostream& os, const SemanticDataConfig<SemB>& /* c */)
{
    return os;
}

// Semantic data
template<>
struct SemanticDataConfig<Semantics::On> {
    /** Initializes the config to some sensible defaults.
     */
    SemanticDataConfig();

    /** Initializes the config from a YAML file. Data not present in the YAML file will be
     * initialized as in SemanticDataConfig<se::Semantics::On>SemanticDataConfig().
     */
    SemanticDataConfig(const std::string& yaml_file);
};

std::ostream& operator<<(std::ostream& os, const SemanticDataConfig<Semantics::On>& c);

} // namespace se

#endif // SE_DATA_SEMANTICS_HPP
