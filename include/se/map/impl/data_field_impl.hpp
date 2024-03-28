/*
 * SPDX-FileCopyrightText: 2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2024 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_DATA_FIELD_IMPL_HPP
#define SE_DATA_FIELD_IMPL_HPP

namespace se {

inline bool FieldData<Field::TSDF>::update(const field_t sdf,
                                           const field_t truncation_boundary,
                                           const weight_t max_weight)
{
    if (sdf < -truncation_boundary) {
        return false;
    }
    // We only need to truncate positive SDF values due to the test above.
    const field_t tsdf_update = std::min(sdf / truncation_boundary, field_t(1));
    // Avoid overflow if max_weight is equal to the maximum value of weight_t.
    if (weight < max_weight) {
        weight++;
    }
    tsdf = (tsdf * (weight - weight_t(1)) + tsdf_update) / weight;
    return true;
}

} // namespace se

#endif // SE_DATA_FIELD_IMPL_HPP
