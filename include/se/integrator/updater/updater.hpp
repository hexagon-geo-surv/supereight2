/*
 * SPDX-FileCopyrightText: 2021-2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021-2024 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_UPDATER_HPP
#define SE_UPDATER_HPP

#include <se/integrator/measurement.hpp>

namespace se {

template<typename MapT, typename SensorT>
struct Updater {
    Updater(MapT& map, const timestamp_t timestamp, const Measurements<SensorT>& measurements);

    Updater(MapT& map,
            std::vector<OctantBase*>& octant_ptrs,
            const timestamp_t timestamp,
            const Measurements<SensorT>& measurements);
};

} // namespace se

#include "multires_ofusion_updater.hpp"
#include "multires_tsdf_updater.hpp"
#include "singleres_tsdf_updater.hpp"

#endif // SE_UPDATER_HPP
