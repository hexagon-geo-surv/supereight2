/*
 * SPDX-FileCopyrightText: 2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2024 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "se/integrator/uncertainty.hpp"

namespace se {
namespace uncert {

Image<float> depth_sigma(const Image<float>& depth,
                         const float map_resolution,
                         const FieldData<Field::Occupancy>::Config& config)
{
    const float sigma_min = map_resolution * config.sigma_min_factor;
    const float sigma_max = map_resolution * config.sigma_max_factor;
    Image<float> sigma(depth.width(), depth.height());
#pragma omp parallel for
    for (size_t i = 0; i < depth.size(); i++) {
        const float depth_value = depth[i];
        if (depth_value <= 0.0f || std::isnan(depth_value)) {
            sigma[i] = 0.0f;
        }
        else {
            float sigma_value = config.k_sigma * depth_value;
            if (config.uncertainty_model == UncertaintyModel::Quadratic) {
                sigma_value *= depth_value;
            }
            sigma[i] = std::clamp(sigma_value, sigma_min, sigma_max);
        }
    }
    return sigma;
}

} // namespace uncert
} // namespace se
