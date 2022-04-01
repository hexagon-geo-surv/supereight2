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

// Defaults
static constexpr se::field_t dflt_tsdf = 1.0f;
static constexpr se::field_t dflt_delta_tsdf = 0.0f;
static constexpr se::field_t dflt_occupancy = 0.0f;
static constexpr se::field_t dflt_delta_occupancy = 0.0f;
static constexpr se::weight_t dflt_weight = 0.0f;
static constexpr se::weight_t dflt_delta_weight = 0.0f;
static constexpr se::time_stamp_t dflt_time_stamp = -1.0f;

template<se::Field FieldT>
struct FieldData {
};

template<>
struct FieldData<se::Field::Occupancy> {
    FieldData() :
            occupancy(dflt_occupancy),
            weight(dflt_weight),
            observed(false),
            time_stamp(dflt_time_stamp)
    {
    }
    se::field_t occupancy;
    se::weight_t weight;
    bool observed;
    se::time_stamp_t time_stamp;
    static constexpr bool invert_normals = false;
};

template<>
struct FieldData<se::Field::TSDF> {
    FieldData() : tsdf(dflt_tsdf), weight(dflt_weight)
    {
    }
    se::field_t tsdf;
    se::weight_t weight;
    static constexpr bool invert_normals = true;
};

///////////////////
/// DELTA DATA  ///
///////////////////

template<se::Field FieldT>
struct FieldDeltaData {
};

template<>
struct FieldDeltaData<se::Field::Occupancy> {
    FieldDeltaData() : delta_occupancy(dflt_delta_occupancy)
    {
    }
    se::field_t delta_occupancy;
};

template<>
struct FieldDeltaData<se::Field::TSDF> {
    FieldDeltaData() : delta_tsdf(dflt_delta_tsdf), delta_weight(dflt_delta_weight)
    {
    }
    se::field_t delta_tsdf;
    se::weight_t delta_weight;
};

///////////////////
/// DATA CONFIG ///
///////////////////

enum class UncertaintyModel { Linear, Quadratic };

template<se::Field FieldT>
struct FieldDataConfig {
};

template<>
struct FieldDataConfig<se::Field::Occupancy> {
    float k_sigma;
    float sigma_min_factor;
    float sigma_max_factor;

    float k_tau;
    float tau_min_factor;
    float tau_max_factor;

    se::field_t min_occupancy;
    se::field_t max_occupancy;
    se::weight_t max_weight;
    se::weight_t factor;
    se::field_t surface_boundary;

    se::field_t log_odd_min;
    se::field_t log_odd_max;

    int fs_integr_scale;

    UncertaintyModel uncertainty_model;

    bool const_surface_thickness;

    /** Initializes the config to some sensible defaults.
     */
    FieldDataConfig();

    /** Initializes the config from a YAML file. Data not present in the YAML file will be
     * initialized as in FieldDataConfig<se::Field::Occupancy>::FieldDataConfig().
     */
    FieldDataConfig(const std::string& yaml_file);

    static constexpr se::Field FldT = se::Field::Occupancy;
};

std::ostream& operator<<(std::ostream& os, const FieldDataConfig<se::Field::Occupancy>& c);

template<>
struct FieldDataConfig<se::Field::TSDF> {
    se::field_t truncation_boundary_factor;
    se::weight_t max_weight;

    /** Initializes the config to some sensible defaults.
     */
    FieldDataConfig();

    /** Initializes the config from a YAML file. Data not present in the YAML file will be
     * initialized as in FieldDataConfig<se::Field::TSDF>::FieldDataConfig().
     */
    FieldDataConfig(const std::string& yaml_file);

    static constexpr se::Field FldT = se::Field::TSDF;
};

std::ostream& operator<<(std::ostream& os, const FieldDataConfig<se::Field::TSDF>& c);

} // namespace se

#endif // SE_DATA_FIELD_HPP
