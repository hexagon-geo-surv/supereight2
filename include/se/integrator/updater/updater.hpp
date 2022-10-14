/*
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_UPDATER_HPP
#define SE_UPDATER_HPP



namespace se {



template<typename MapT, typename SensorT>
class Updater {
    public:
    Updater(MapT& map,
            const SensorT& sensor,
            const se::Image<float>& depth_img,
            const se::Image<rgb_t>* colour_img,
            const Eigen::Matrix4f& T_WS,
            const int frame);

    template<typename UpdateListT>
    void operator()(UpdateListT& updating_list);
};



// Single-res TSDF updater
template<se::Colour ColB, se::Semantics SemB, int BlockSize, typename SensorT>
class Updater<Map<Data<se::Field::TSDF, ColB, SemB>, se::Res::Single, BlockSize>, SensorT>;



// Multi-res TSDF updater
template<se::Colour ColB, se::Semantics SemB, int BlockSize, typename SensorT>
class Updater<Map<Data<se::Field::TSDF, ColB, SemB>, se::Res::Multi, BlockSize>, SensorT>;



// Multi-res Occupancy updater
template<se::Colour ColB, se::Semantics SemB, int BlockSize, typename SensorT>
class Updater<Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi, BlockSize>, SensorT>;



} // namespace se

#include "multires_ofusion_updater.hpp"
#include "multires_tsdf_updater.hpp"
#include "singleres_tsdf_updater.hpp"

#endif // SE_UPDATER_HPP
