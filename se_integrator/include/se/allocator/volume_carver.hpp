#ifndef SE_VOLUME_CARVER_HPP
#define SE_VOLUME_CARVER_HPP



#include "se/updater/multires_ofusion_core.hpp"
#include "se/utils/image_utils.hpp"
#include "se/utils/math_util.hpp"
#include "se/utils/str_utils.hpp"

#include <set>
#include <Eigen/Core>



namespace se {



enum class VarianceState { Constant, Gradient, Undefined};

struct VolumeCarverAllocation
{
  std::vector<se::OctantBase*>   node_list;
  std::vector<se::OctantBase*>   block_list;
  std::vector<se::VarianceState> variance_state_list;
  std::vector<bool>              projects_inside_list;
};



template<typename MapT,
         typename SensorT
>
class VolumeCarver {
public:
  VolumeCarver(MapT&                                       map,
               const PinholeCamera&                        sensor,
               const se::Image<float>&                     depth_img,
               const se::DensePoolingImage<PinholeCamera>& depth_pooling_img,
               const Eigen::Matrix4f&                      T_SM,
               const int                                   frame)
  {
  };
};




template<se::Colour    ColB,
         se::Semantics SemB,
         int           BlockSize
>
class VolumeCarver<Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi, BlockSize>, PinholeCamera>
{
public:
  typedef Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi> MapType;
  typedef typename MapType::OctreeType                                OctreeType;
  typedef typename OctreeType::NodeType                               NodeType;
  typedef typename OctreeType::BlockType                              BlockType;

  /**
   * \param[in]  map                  The reference to the map to be updated.
   * \param[in]  sensor               The sensor model.
   * \param[in]  depth_img            The depth image to be integrated.
   * \param[in]  T_SM                 The transformation from map to camera frame.
   * \param[in]  frame                The frame number to be integrated.
   */
  VolumeCarver(MapType&                                    map,
               const PinholeCamera&                        sensor,
               const se::Image<float>&                     depth_img,
               const Eigen::Matrix4f&                      T_SM,
               const int                                   frame);

  /**
   * \brief Allocate the frustum using a map-to-camera volume carving approach
   */
  VolumeCarverAllocation allocateFrustum();



private:
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

  /**
   * \brief Return a conservative meassure of the expected variance of a sensor model inside a voxel
   *        given its position and depth variance.
   *
   * \param[in] depth_value_min Depth measurement max value inside voxel.
   * \param[in] depth_value_max Depth measurement min value inside voxel.
   * \param[in] node_dist_min_m Minimum node distance along z-axis in meter.
   * \param[in] node_dist_max_m Maximum node distance along z-axis in meter.
   *
   * \return Estimate of the variance
   */
  se::VarianceState computeVariance(const float       depth_value_min,
                                    const float       depth_value_max,
                                    const float       node_dist_min_m,
                                    const float       node_dist_max_m);

  void operator()(const Eigen::Vector3i& node_coord,
                  const int              node_size,
                  const int              octant_depth,
                  const Eigen::Vector3i& rel_step,
                  se::OctantBase*        parent_ptr);

  MapType&                                    map_;
  OctreeType&                                 octree_;
  const PinholeCamera&                        sensor_;
  const se::DensePoolingImage<PinholeCamera>  depth_pooling_img_;
  const Eigen::Matrix4f&                      T_CM_;
  const int                                   frame_;
  const float                                 map_res_;
  const float                                 sigma_min_;
  const float                                 sigma_max_;
  const float                                 tau_min_;
  const float                                 tau_max_;
  const float                                 max_depth_value_;
  const float                                 zero_depth_band_;
  const float                                 size_to_radius_;
  VolumeCarverAllocation                      allocation_list_;
};



} // namespace se

#include "impl/volume_carver_impl.hpp"

#endif //SE_VOLUME_CARVER_HPP
