/*
 * SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_MULTIRES_OFUSION_UPDATER_HPP
#define SE_MULTIRES_OFUSION_UPDATER_HPP



#include "se/sensor/sensor.hpp"



namespace se {



// Multi-res Occupancy updater
template<Colour ColB, Semantics SemB, int BlockSize, typename SensorT>
class Updater<Map<Data<Field::Occupancy, ColB, SemB>, Res::Multi, BlockSize>, SensorT> {
    public:
    typedef Map<Data<Field::Occupancy, ColB, SemB>, Res::Multi, BlockSize> MapType;
    typedef typename MapType::DataType DataType;
    typedef typename MapType::OctreeType OctreeType;
    typedef typename MapType::OctreeType::NodeType NodeType;
    typedef typename MapType::OctreeType::BlockType BlockType;


    struct UpdaterConfig {
        UpdaterConfig(const MapType& map) :
                sigma_min(map.getRes() * map.getDataConfig().sigma_min_factor),
                sigma_max(map.getRes() * map.getDataConfig().sigma_max_factor),
                tau_min(map.getRes() * map.getDataConfig().tau_min_factor),
                tau_max(map.getRes() * map.getDataConfig().tau_max_factor)
        {
        }

        const float sigma_min;
        const float sigma_max;
        const float tau_min;
        const float tau_max;
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
            const Image<float>& depth_img,
            const Eigen::Matrix4f& T_WS,
            const int frame);

    void operator()(VolumeCarverAllocation& allocation_list);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
    /** \brief Propage all newly integrated values from the voxel block level up to the root of the
     * octree.
     */
    void propagateToRoot(std::vector<OctantBase*>& block_list);

    /** \brief Update all voxels of a block that project into the image plane.
     *
     * \param[in,out] octant_ptr The block to be updated.
     */
    void updateBlock(OctantBase* octant_ptr, bool low_variance, bool project_inside);

    /** \brief Update a node and all its children by the minimum occupancy log-odd.
     *
     * \param[in,out] octant_ptr The node to be updated.
     * \param[in]     depth      The depth of the node.
     */
    void updateNodeFree(OctantBase* octant_ptr, int depth);

    /** \brief Update all voxels of a block by the minimum occupancy log-odd.
     *
     * \param[in,out] octant_ptr The block to be updated.
     */
    void updateBlockFree(OctantBase* octant_ptr);

    private:
    MapType& map_;
    OctreeType& octree_;
    const SensorT& sensor_;
    const Image<float>& depth_img_;
    const Eigen::Matrix4f T_SW_;
    const int frame_;
    const float map_res_;
    const UpdaterConfig config_;
    std::vector<std::set<OctantBase*>> node_set_;
    std::vector<OctantBase*> freed_block_list_;
};



} // namespace se

#include "impl/multires_ofusion_updater_impl.hpp"

#endif // SE_MULTIRES_OFUSION_UPDATER_HPP
