//
// Created by nils on 27/05/2021.
//

#ifndef SE_VOLUME_CARVER_HPP
#define SE_VOLUME_CARVER_HPP



#include "se/updater/multires_ofusion_updater_models.hpp"
#include "se/utils/image_utils.hpp"
#include "se/utils/math_util.hpp"
#include "se/utils/str_utils.hpp"

#include <set>
#include <Eigen/Core>



namespace se {



template<typename MapT, typename SensorT>
class VolumeCarver {
public:
  VolumeCarver(const se::Image<float>&                     depth_image,
               const se::DensePoolingImage<SensorT>&       depth_pooling_img,
               MapT&                                       map,
               const SensorT&                              sensor,
               const Eigen::Matrix4f&                      T_SM,
               const int                                   frame,
               std::vector<se::OctantBase*>&               node_list,
               std::vector<se::OctantBase*>&               block_list,
               std::vector<bool>&                          low_variance_list,
               std::vector<bool>&                          projects_inside_list)
   {
   };
};



template<se::Colour    ColB,
         se::Semantics SemB
>
class VolumeCarver<Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi>, PinholeCamera>
{
public:
  typedef Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi> MapType;
  typedef typename MapType::OctreeType                                OctreeType;
  typedef typename OctreeType::NodeType                               NodeType;
  typedef typename OctreeType::BlockType                              BlockType;

  /**
   * \param[in]  map                  The reference to the map to be updated.
   * \param[out] block_list           The list of blocks to be updated (used for up-propagation in later stage).
   * \param[out] node_list            The list of nodes that have been updated (used for up-propagation in later stage.
   * \param[out] free_list            The list verifying if the updated block should be freed. <bool>
   * \param[out] low_variance_list    The list verifying if the updated block has low variance. <bool>
   * \param[out] projects_inside_list The list verifying if the updated block projects completely into the image. <bool>
   * \param[in]  depth_image          The depth image to be integrated.
   * \param[in]  depth_pooling_img  The pointer to the pooling image created from the depth image.
   * \param[in]  sensor               The sensor model.
   * \param[in]  T_CM                 The transformation from map to camera frame.
   * \param[in]  voxel_dim            The dimension in meters of the finest voxel / map resolution.
   * \param[in]  voxel_depth          The tree depth of the finest voxel.
   * \param[in]  max_depth_value      The maximum depth value in the image.
   * \param[in]  frame                The frame number to be integrated.
   */
  VolumeCarver(const se::Image<float>&                     depth_image,
               const se::DensePoolingImage<PinholeCamera>& depth_pooling_img,
               MapType&                                    map,
               const PinholeCamera&                        sensor,
               const Eigen::Matrix4f&                      T_SM,
               const int                                   frame,
               std::vector<se::OctantBase*>&               node_list,
               std::vector<se::OctantBase*>&               block_list,
               std::vector<bool>&                          low_variance_list,
               std::vector<bool>&                          projects_inside_list);

  /**
   * \brief Verify if the node crosses the camera frustum excluding the case of the camera in the node.
   *
   * \param[in] proj_corner_stati The stati of the projection of the eight octant corners into the image plane
   *
   * \return True/false statement if node crosses the camera frustum.
   */
  bool crossesFrustum(std::vector<srl::projection::ProjectionStatus>&  proj_corner_stati);

  /**
   * \brief Verify if the camera is inside a given node.
   *
   * \param[in] node_coord  The coordinates of the node in voxel coordinates.
   * \param[in] node_size   The size of the node in voxel units.
   * \param[in] T_MC        The transformation from camera to map frame.
   *
   * \return True/false statement if the camera is inside the node.
   */
  bool cameraInNode(const Eigen::Vector3i& node_coord,
                    const int              node_size,
                    const Eigen::Matrix4f& T_MC);

  void operator()(const Eigen::Vector3i& node_coord,
                  const int              node_size,
                  const int              depth,
                  const Eigen::Vector3i& rel_step,
                  se::OctantBase*        parent_ptr);

private:
  const se::Image<float>&                     depth_image_;
  const se::DensePoolingImage<PinholeCamera>& depth_pooling_img_;
  MapType&                                    map_;
  OctreeType&                                 octree_;
  const PinholeCamera&                        sensor_;
  const Eigen::Matrix4f&                      T_CM_;
  const int                                   frame_;
  std::vector<se::OctantBase*>&               node_list_;
  std::vector<se::OctantBase*>&               block_list_;
  std::vector<bool>&                          low_variance_list_;
  std::vector<bool>&                          projects_inside_list_;
  const float                                 voxel_dim_;
  const float                                 max_depth_value_;
  const float                                 zero_depth_band_;
  const float                                 size_to_radius_;
};

}

#include "impl/volume_carver_impl.hpp"

#endif //SE_VOLUME_CARVER_HPP
