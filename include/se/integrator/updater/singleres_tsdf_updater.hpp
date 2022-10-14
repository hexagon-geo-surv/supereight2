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
template<Colour ColB, Semantics SemB, int BlockSize, typename SensorT>
class Updater<Map<Data<Field::TSDF, ColB, SemB>, Res::Single, BlockSize>, SensorT> {
    public:
    typedef Map<Data<Field::TSDF, ColB, SemB>, Res::Single, BlockSize> MapType;
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
     * \param[in]  colour_img           The colour image to be integrated or nullptr if none.
     * \param[in]  T_WS                 The transformation from sensor to world frame.
     * \param[in]  frame                The frame number to be integrated.
     */
    Updater(MapType& map,
            const SensorT& sensor,
            const Image<float>& depth_img,
            const Image<rgb_t>* colour_img,
            const Eigen::Matrix4f& T_WS,
            const int frame);



    void operator()(std::vector<OctantBase*>& block_ptrs);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
    void updateVoxel(DataType& data, field_t sdf_value);

    MapType& map_;
    const SensorT& sensor_;
    const Image<float>& depth_img_;
    const Eigen::Matrix4f& T_WS_;
    const int frame_;
    const UpdaterConfig config_;
};



} // namespace se

#include "impl/singleres_tsdf_updater_impl.hpp"

#endif // SE_SINGLERES_TSDF_UPDATER_HPP
