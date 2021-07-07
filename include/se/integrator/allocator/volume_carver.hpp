#ifndef SE_VOLUME_CARVER_HPP
#define SE_VOLUME_CARVER_HPP



#include "se/integrator/updater/multires_ofusion_core.hpp"
#include "se/common/image_utils.hpp"
#include "se/common/math_util.hpp"
#include "se/common/str_utils.hpp"

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
  VolumeCarver(MapT&                                 map,
               const SensorT&                        sensor,
               const se::Image<float>&               depth_img,
               const se::DensePoolingImage<SensorT>& depth_pooling_img,
               const Eigen::Matrix4f&                T_SM,
               const int                             frame)
  {
  };
};



/**
 * \brief Allocate the frustum using a map-to-camera volume carving approach
 *
 * \tparam ColB
 * \tparam SemB
 * \tparam BlockSize
 * \tparam SensorT
 */
template<se::Colour    ColB,
         se::Semantics SemB,
         int           BlockSize,
         typename      SensorT
>
class VolumeCarver<Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi, BlockSize>, SensorT>
{
public:
  typedef Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi> MapType;
  typedef typename MapType::OctreeType                                OctreeType;
  typedef typename OctreeType::NodeType                               NodeType;
  typedef typename OctreeType::BlockType                              BlockType;

  /**
   * \brief The config file of the volume carver
   *
   * \param[in] map   The map allocate the frustum in
   */
  struct VolumeCarverConfig
  {
    VolumeCarverConfig(const MapType& map) :
        sigma_min(map.getDataConfig().sigma_min_factor * map.getRes()),
        sigma_max(map.getDataConfig().sigma_max_factor * map.getRes()),
        tau_min(map.getDataConfig().tau_min_factor * map.getRes()),
        tau_max(map.getDataConfig().tau_max_factor * map.getRes())
    {
    }

    const float sigma_min;
    const float sigma_max;
    const float tau_min;
    const float tau_max;
  };


  /**
   * \brief Setup the volume carver.
   *
   * \param[in]  map                  The reference to the map to be updated.
   * \param[in]  sensor               The sensor model.
   * \param[in]  depth_img            The depth image to be integrated.
   * \param[in]  T_MS                 The transformation from map to camera frame.
   * \param[in]  frame                The frame number to be integrated.
   */
  VolumeCarver(MapType&                map,
               const SensorT&          sensor,
               const se::Image<float>& depth_img,
               const Eigen::Matrix4f&  T_MS,
               const int               frame);

  /**
   * \brief Allocate the frustum using a map-to-camera volume carving approach
   */
  VolumeCarverAllocation operator()();



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

  /**
   * \brief Recursively decide if to allocate or terminate a node.
   *
   * \note se::PinholeCamera implementation
   *
   * \tparam SensorTDummy
   * \param[in] node_coord      The coordinates of the nodes corner (front, left, bottom) to be evaluated
   * \param[in] node_size       The size of the node in [voxel]
   * \param[in] octant_depth    The tree depth of the node
   * \param[in] rel_step        The relative child step of the node
   * \param[in] parent_ptr      The pointer to the nodes parent
   */
  template<class SensorTDummy = SensorT>
  typename std::enable_if_t<std::is_same<SensorTDummy, se::PinholeCamera>::value, void>
  operator()(const Eigen::Vector3i& node_coord,
             const int              node_size,
             const int              octant_depth,
             const Eigen::Vector3i& rel_step,
             se::OctantBase*        parent_ptr);

  /**
   * \brief Recursively decide if to allocate or terminate a node.
   *
   * \note se::PinholeCamera implementation
   *
   * \tparam SensorTDummy
   * \param[in] node_coord      The coordinates of the nodes corner (front, left, bottom) to be evaluated
   * \param[in] node_size       The size of the node in [voxel]
   * \param[in] octant_depth    The tree depth of the node
   * \param[in] rel_step        The relative child step of the node
   * \param[in] parent_ptr      The pointer to the nodes parent
   */
  template<class SensorTDummy = SensorT>
  typename std::enable_if_t<std::is_same<SensorTDummy, se::OusterLidar>::value, void>
  operator()(const Eigen::Vector3i& node_coord,
             const int              node_size,
             const int              octant_depth,
             const Eigen::Vector3i& rel_step,
             se::OctantBase*        parent_ptr);

  MapType&                             map_;
  OctreeType&                          octree_;
  const SensorT&                       sensor_;
  const se::DensePoolingImage<SensorT> depth_pooling_img_;
  const Eigen::Matrix4f                T_SM_;
  const int                            frame_;
  const float                          map_res_;
  VolumeCarverConfig                   config_;
  const float                          max_depth_value_;
  const float                          zero_depth_band_;
  const float                          size_to_radius_;
  VolumeCarverAllocation               allocation_list_;
};



} // namespace se

#include "impl/volume_carver_impl.hpp"

#endif //SE_VOLUME_CARVER_HPP
