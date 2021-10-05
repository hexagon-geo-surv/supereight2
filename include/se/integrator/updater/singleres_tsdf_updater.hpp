/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_SINGLERES_TSDF_UPDATER_HPP
#define SE_SINGLERES_TSDF_UPDATER_HPP



#include "se/map/map.hpp"
#include "se/sensor/sensor.hpp"



namespace se {



// Single-res TSDF updater
template<se::Colour ColB, se::Semantics SemB, int BlockSize, typename SensorT>
class Updater<Map<Data<se::Field::TSDF, ColB, SemB>, se::Res::Single, BlockSize>, SensorT> {
    public:
    typedef Map<Data<se::Field::TSDF, ColB, SemB>, se::Res::Single, BlockSize> MapType;
    typedef typename MapType::DataType DataType;
    typedef typename MapType::OctreeType::NodeType NodeType;
    typedef typename MapType::OctreeType::BlockType BlockType;

    struct UpdaterConfig {
        UpdaterConfig(const MapType& map) :
                truncation_boundary(map.getRes() * map.getDataConfig().truncation_boundary_factor)
        {
        }

        const float truncation_boundary;
    };

    /**
     * \param[in]  map                  The reference to the map to be updated.
     * \param[in]  sensor               The sensor model.
     * \param[in]  depth_img            The depth image to be integrated.
     * \param[in]  T_WS                 The transformation from sensor to world frame.
     * \param[in]  frame                The frame number to be integrated.
     */
    Updater(MapType& map,
            const SensorT& sensor,
            const se::Image<float>& depth_img,
            const Eigen::Matrix4f& T_WS,
            const int frame);



    void operator()(std::vector<se::OctantBase*>& block_ptrs);

    private:
    void updateVoxel(DataType& data, const field_t sdf_value);

    MapType& map_;
    const SensorT& sensor_;
    const se::Image<float>& depth_img_;
    const Eigen::Matrix4f& T_WS_;
    const int frame_;
    const UpdaterConfig config_;
};



} // namespace se

#include "impl/singleres_tsdf_updater_impl.hpp"

#endif // SE_SINGLERES_TSDF_UPDATER_HPP
