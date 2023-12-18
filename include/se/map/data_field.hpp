/*
 * SPDX-FileCopyrightText: 2021-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021-2022 Nils Funk
 * SPDX-FileCopyrightText: 2021-2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_DATA_FIELD_HPP
#define SE_DATA_FIELD_HPP

#include "utils/setup_util.hpp"
#include "utils/type_util.hpp"

namespace se {

template<Field FieldT>
struct FieldData {
};

template<>
struct FieldData<Field::Occupancy> {
    field_t occupancy = 0;
    weight_t weight = 0;
    bool observed = false;
    static constexpr bool invert_normals = false;
    static constexpr field_t surface_boundary = 0;
    static constexpr field_t min_occupancy = -100;
    static constexpr field_t max_occupancy = 100;
};

template<>
struct FieldData<Field::TSDF> {
    field_t tsdf = 1;
    weight_t weight = 0;
    static constexpr bool invert_normals = true;
    static constexpr field_t surface_boundary = 0;
};

///////////////////
/// DELTA DATA  ///
///////////////////

template<Field FieldT>
struct FieldDeltaData {
};

template<>
struct FieldDeltaData<Field::Occupancy> {
    field_t delta_occupancy = 0;
};

template<>
struct FieldDeltaData<Field::TSDF> {
    field_t delta_tsdf = 0;
    weight_t delta_weight = 0;
};

///////////////////
/// DATA CONFIG ///
///////////////////

enum class UncertaintyModel { Linear, Quadratic };

template<Field FieldT>
struct FieldDataConfig {
};

template<>
struct FieldDataConfig<Field::Occupancy> {
    float k_sigma;
    float sigma_min_factor;
    float sigma_max_factor;

    float k_tau;
    float tau_min_factor;
    float tau_max_factor;

    weight_t max_weight;

    field_t log_odd_min;
    field_t log_odd_max;

    int fs_integr_scale;

    UncertaintyModel uncertainty_model;

    /** Initializes the config to some sensible defaults.
     */
    FieldDataConfig();

    /** Initializes the config from a YAML file. Data not present in the YAML file will be
     * initialized as in FieldDataConfig<se::Field::Occupancy>::FieldDataConfig().
     */
    FieldDataConfig(const std::string& yaml_file);

    static constexpr Field FldT = Field::Occupancy;
};

std::ostream& operator<<(std::ostream& os, const FieldDataConfig<Field::Occupancy>& c);

template<>
struct FieldDataConfig<Field::TSDF> {
    field_t truncation_boundary_factor;
    weight_t max_weight;

    /** Initializes the config to some sensible defaults.
     */
    FieldDataConfig();

    /** Initializes the config from a YAML file. Data not present in the YAML file will be
     * initialized as in FieldDataConfig<se::Field::TSDF>::FieldDataConfig().
     */
    FieldDataConfig(const std::string& yaml_file);

    static constexpr Field FldT = Field::TSDF;
};

std::ostream& operator<<(std::ostream& os, const FieldDataConfig<Field::TSDF>& c);

} // namespace se

#endif // SE_DATA_FIELD_HPP
