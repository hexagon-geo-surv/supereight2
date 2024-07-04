/*
 * SPDX-FileCopyrightText: 2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2024 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_INTEGRATOR_UNCERTAINTY_HPP
#define SE_INTEGRATOR_UNCERTAINTY_HPP

#include <se/image/image.hpp>
#include <se/map/data_field.hpp>

namespace se {
namespace uncert {

/** Return the per-pixel standard deviation of the \p depth image according to the inverse sensor
 * model in \p config and the \p map_resolution.
 */
Image<float> depth_sigma(const Image<float>& depth,
                         const float map_resolution,
                         const FieldData<Field::Occupancy>::Config& config);

} // namespace uncert
} // namespace se

#endif // SE_INTEGRATOR_UNCERTAINTY_HPP
