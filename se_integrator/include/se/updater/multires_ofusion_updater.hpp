#ifndef SE_MULTIRES_OFUSION_UPDATER_HPP
#define SE_MULTIRES_OFUSION_UPDATER_HPP



#include "se/sensor.hpp"



namespace se {



template<typename MapT, typename SensorT>
class MultiresOFusionUpdater
{
public:
  typedef typename MapT::DataType              DataType;
  typedef typename MapT::OctreeType::NodeType  NodeType;
  typedef typename MapT::OctreeType::BlockType BlockType;


  /**
   * \param[in]  map                  The reference to the map to be updated.
   * \param[out] block_list           The list of blocks to be updated (used for up-propagation in later stage).
   * \param[out] node_list            The list of nodes that have been updated (used for up-propagation in later stage.
   * \param[out] free_list            The list verifying if the updated block should be freed. <bool>
   * \param[out] low_variance_list    The list verifying if the updated block has low variance. <bool>
   * \param[out] projects_inside_list The list verifying if the updated block projects completely into the image. <bool>
   * \param[in]  depth_image          The depth image to be integrated.
   * \param[in]  pooling_depth_image  The pointer to the pooling image created from the depth image.
   * \param[in]  sensor               The sensor model.
   * \param[in]  T_CM                 The transformation from map to camera frame.
   * \param[in]  voxel_dim            The dimension in meters of the finest voxel / map resolution.
   * \param[in]  voxel_depth          The tree depth of the finest voxel.
   * \param[in]  max_depth_value      The maximum depth value in the image.
   * \param[in]  frame                The frame number to be integrated.
   */
  MultiresOFusionUpdater(const se::Image<float>& depth_image,
                         MapT&                    map,
                         const PinholeCamera&     sensor,
                         const Eigen::Matrix4f&   T_SM,
                         const int                frame,
                         std::vector<std::set<se::OctantBase*>>& node_set,
                         std::vector<se::OctantBase*>& freed_block_list);

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
  const se::Image<float>&    depth_image_;
  MapT&                      map_;
  typename MapT::OctreeType& octree_;
  const PinholeCamera&       sensor_;
  const Eigen::Matrix4f&     T_CM_;
  const int                  frame_;
  const float                map_res_;
  const float                sigma_min_;
  const float                sigma_max_;
  const float                tau_min_;
  const float                tau_max_;
  std::vector<std::set<se::OctantBase*>>& node_set_;
  std::vector<se::OctantBase*>& freed_block_list_;
};



} // namespace se



#include "impl/multires_ofusion_updater_impl.hpp"



#endif //SE_MULTIRES_OFUSION_UPDATER_HPP
