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
template<se::Colour    ColB,
         se::Semantics SemB,
         int           BlockSize,
         typename      SensorT
>
class Updater<Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi, BlockSize>, SensorT>
{
public:
  typedef Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi, BlockSize> MapType;
  typedef typename MapType::DataType                                             DataType;
  typedef typename MapType::OctreeType                                           OctreeType;
  typedef typename MapType::OctreeType::NodeType                                 NodeType;
  typedef typename MapType::OctreeType::BlockType                                BlockType;


  struct UpdaterConfig
  {
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
  Updater(MapType&                               map,
          const SensorT&                         sensor,
          const se::Image<float>&                depth_img,
          const Eigen::Matrix4f&                 T_WS,
          const int                              frame);

  void operator()(se::VolumeCarverAllocation& allocation_list)
  {
#pragma omp parallel for
    for (unsigned int i = 0; i < allocation_list.node_list.size(); ++i)
    {
      auto node_ptr = static_cast<NodeType*>(allocation_list.node_list[i]);
      const int depth = map_.getOctree()->getMaxScale() - se::math::log2_const(node_ptr->getSize());
      freeNodeRecurse(allocation_list.node_list[i], depth);
    }

#pragma omp parallel for
    for (unsigned int i = 0; i < allocation_list.block_list.size(); ++i)
    {
      updateBlock(allocation_list.block_list[i],
                  allocation_list.variance_state_list[i] == se::VarianceState::Constant,
                  allocation_list.projects_inside_list[i]);
    }

    /// Propagation
#pragma omp parallel for
    for (unsigned int i = 0; i < allocation_list.block_list.size(); ++i)
    {
      updater::propagateBlockToCoarsestScale<BlockType>(allocation_list.block_list[i]);
    }
#pragma omp parallel for
    for (unsigned int i = 0; i < freed_block_list_.size(); ++i)
    {
      updater::propagateBlockToCoarsestScale<BlockType>(freed_block_list_[i]);
    }

    propagateToRoot(allocation_list.block_list);
  }


private:
  /**
   * \brief Propage all newly integrated values from the voxel block depth up to the root of the octree.
   */
  void propagateToRoot(std::vector<se::OctantBase*>& block_list);

  void freeBlock(se::OctantBase* octant_ptr);

  /**
   * \brief Compute integration scale for a given voxel block and update all voxels that project into the image plane.
   *
   * \note The minimum integration scale has only an effect if no data has been integrated into the block yet, i.e.
   *       the integration scale of the block has not been initialised yet.
   *
   * \param[out] block                 The block to be updated.
   * \param[out] min_integration_scale The minimum integration scale.
   */
  void updateBlock(se::OctantBase* octant_ptr,
                   bool            low_variance,
                   bool            project_inside);


  /**
   * \brief Recursively reduce all children by the minimum occupancy log-odd for a single integration.
   */
  void freeNodeRecurse(se::OctantBase* octant_ptr,
                       int             depth);

private:
  MapType&                               map_;
  OctreeType&                            octree_;
  const SensorT&                         sensor_;
  const se::Image<float>&                depth_img_;
  const Eigen::Matrix4f                  T_SW_;
  const int                              frame_;
  const float                            map_res_;
  const UpdaterConfig                    config_;
  std::vector<std::set<se::OctantBase*>> node_set_;
  std::vector<se::OctantBase*>           freed_block_list_;
};



} // namespace se

#include "impl/multires_ofusion_updater_impl.hpp"

#endif // SE_MULTIRES_OFUSION_UPDATER_HPP

