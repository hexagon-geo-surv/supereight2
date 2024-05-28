/*
 * SPDX-FileCopyrightText: 2021-2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021-2024 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_UPDATER_HPP
#define SE_UPDATER_HPP

namespace se {

template<typename MapT, typename SensorT>
struct Updater {
    Updater(MapT& map,
            const SensorT& sensor,
            const se::Image<float>& depth_img,
            const Eigen::Isometry3f& T_WS,
            const SensorT* const colour_sensor,
            const Image<colour_t>* const colour_img,
            const Eigen::Isometry3f* const T_SSc,
            const int frame);

    Updater(MapT& map,
            std::vector<OctantBase*>& octant_ptrs,
            const SensorT& sensor,
            const se::Image<float>& depth_img,
            const Eigen::Isometry3f& T_WS,
            const SensorT* const colour_sensor,
            const Image<colour_t>* const colour_img,
            const Eigen::Isometry3f* const T_SSc,
            const int frame);
};

} // namespace se

#include "multires_ofusion_updater.hpp"
#include "multires_tsdf_updater.hpp"
#include "singleres_tsdf_updater.hpp"

#endif // SE_UPDATER_HPP
