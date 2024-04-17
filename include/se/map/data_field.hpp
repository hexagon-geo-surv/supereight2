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

enum class UncertaintyModel { Linear, Quadratic };

template<Field FieldT>
struct FieldData {
    struct Config {
    };
};

template<>
struct FieldData<Field::Occupancy> {
    struct Config {
        float k_sigma = 0.052f;
        float sigma_min_factor = 1.5f;
        float sigma_max_factor = 6.0f;

        float k_tau = 0.026f;
        float tau_min_factor = 6.0f;
        float tau_max_factor = 16.0f;

        field_t log_odd_min = -5.015;
        field_t log_odd_max = 5.015;

        weight_t max_weight = std::floor(std::fabs(min_occupancy / (0.97f * log_odd_min)));

        int fs_integr_scale = 1;

        UncertaintyModel uncertainty_model = UncertaintyModel::Linear;

        /** Reads the struct members from the "data" node of a YAML file. Members not present in the
         * YAML file aren't modified.
         */
        void readYaml(const std::string& yaml_file);
    };

    field_t occupancy = 0;
    weight_t weight = 0;
    bool observed = false;
    static constexpr bool invert_normals = false;
    static constexpr field_t surface_boundary = 0;
    static constexpr field_t min_occupancy = -100;
    static constexpr field_t max_occupancy = 100;

    /** Perform a weighted average log-odds occupancy update and set the data to observed, while
     * ensuring the weight doesn't exceed \p max_weight. Return whether the data was updated.
     */
    bool update(const field_t occupancy, const weight_t max_weight);
};

std::ostream& operator<<(std::ostream& os, const FieldData<Field::Occupancy>::Config& c);

template<>
struct FieldData<Field::TSDF> {
    struct Config {
        field_t truncation_boundary_factor = 8;
        weight_t max_weight = 100;

        /** Reads the struct members from the "data" node of a YAML file. Members not present in the
         * YAML file aren't modified.
         */
        void readYaml(const std::string& yaml_file);
    };

    field_t tsdf = 1;
    weight_t weight = 0;
    static constexpr bool invert_normals = true;
    static constexpr field_t surface_boundary = 0;

    /** Perform a weighted average TSDF update by truncating the SDF value \p sdf within \p
     * truncation_boundary, while ensuring the weight doesn't exceed \p max_weight. Return whether
     * the data was updated. Data isn't updated if \p sdf is less than \p -truncation_boundary.
     */
    bool update(const field_t sdf, const field_t truncation_boundary, const weight_t max_weight);
};

std::ostream& operator<<(std::ostream& os, const FieldData<Field::TSDF>::Config& c);

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


} // namespace se

#include "impl/data_field_impl.hpp"

#endif // SE_DATA_FIELD_HPP
