#ifndef SE_MAP_INTEGRATOR_IMPL_HPP
#define SE_MAP_INTEGRATOR_IMPL_HPP

#include "se/octree/propagator.hpp"
#include "se/allocator/dense_pooling_image.hpp"
#include "se/updater/multires_ofusion_updater_models.hpp"
#include "se/utils/math_util.hpp"
#include "se/allocator/volume_carver.hpp"
#include "se/updater/multires_ofusion_updater.hpp"



namespace se {
namespace allocator {



template<typename SensorT,
         typename MapT
>
std::vector<se::OctantBase*> frustum(const se::Image<depth_t>& depth_img,
                                    SensorT&                   sensor,
                                    const Eigen::Matrix4f&     T_MS,
                                    MapT&                      map,
                                    const float                band)
{
  // Fetch the currently allocated Blocks in the sensor frustum.
  std::vector<se::OctantBase*> blocks_in_frustum = fetcher::frustum(map, sensor, T_MS);

  auto octree_ptr = map.getOctree();

  const int num_steps = ceil(band / map.getRes());

  const Eigen::Vector3f t_MS = T_MS.topRightCorner<3, 1>();

#pragma omp declare reduction (merge : std::set<se::key_t> : omp_out.insert(omp_in.begin(), omp_in.end()))
  std::set<se::key_t> voxel_key_set;

#pragma omp parallel for reduction(merge: voxel_key_set)
  for (int x = 0; x < depth_img.width(); ++x) {
    for (int y = 0; y < depth_img.height(); ++y) {
      const Eigen::Vector2i pixel(x, y);
      const float depth_value = depth_img(pixel.x(), pixel.y());
      if (depth_value < sensor.near_plane) {
        continue;
      }

      Eigen::Vector3f ray_dir_C;
      const Eigen::Vector2f pixel_f = pixel.cast<float>();
      sensor.model.backProject(pixel_f, &ray_dir_C);
      const Eigen::Vector3f point_M = (T_MS * (depth_value * ray_dir_C).homogeneous()).head<3>();

      const Eigen::Vector3f reverse_ray_dir_M = (t_MS - point_M).normalized();

      const Eigen::Vector3f ray_origin_M = point_M - (band * 0.5f) * reverse_ray_dir_M;
      const Eigen::Vector3f step = (reverse_ray_dir_M * band) / num_steps;

      Eigen::Vector3f ray_pos_M = ray_origin_M;
      for (int i = 0; i < num_steps; i++) {
        Eigen::Vector3i voxel_coord;

        if (map.template pointToVoxel<se::Safe::On>(ray_pos_M, voxel_coord)) {
          se::key_t voxel_key;
          se::keyops::encode_key(voxel_coord, octree_ptr->max_block_scale, voxel_key);
          voxel_key_set.insert(voxel_key);
        }
        ray_pos_M += step;
      }
    }
  }

  // Allocate the Blocks and get pointers only to the newly-allocated Blocks.
  std::vector<key_t> voxel_keys(voxel_key_set.begin(), voxel_key_set.end());
  std::vector<se::OctantBase*> fetched_block_ptrs = se::allocator::blocks(voxel_keys, *octree_ptr, octree_ptr->getRoot(), true);

  // Merge the previously-allocated and newly-allocated Block pointers.
  fetched_block_ptrs.reserve(fetched_block_ptrs.size() + blocks_in_frustum.size());
  fetched_block_ptrs.insert(fetched_block_ptrs.end(), blocks_in_frustum.begin(), blocks_in_frustum.end());
  return fetched_block_ptrs;
}



template<typename MapT,
         typename SensorT
>
void frustum(const se::Image<depth_t>&               depth_img,
             const se::DensePoolingImage<SensorT>    depth_pooling_img,
             MapT&                                   map,
             SensorT&                                sensor,
             const Eigen::Matrix4f&                  T_MS,
             std::vector<se::OctantBase*>&           block_list,           ///< List of blocks to be updated.
             std::vector<std::set<se::OctantBase*>>& node_list,            ///< Node list of size map.getOctree()->getBlockDepth()
             std::vector<bool>&                      low_variance_list,    ///< Has updated block low variance? <bool>
             std::vector<bool>&                      projects_inside_list) ///< Does the updated block reproject completely into the image? <bool>
{
  VolumeCarver volume_carver(depth_img, depth_pooling_img, map, sensor, se::math::to_inverse_transformation(T_MS), block_list, node_list, low_variance_list, projects_inside_list);

  // Launch on the 8 voxels of the first depth
#pragma omp parallel for
  for (int child_idx = 0; child_idx < 8; ++child_idx)
  {
    int child_size = map.size() / 2;
    Eigen::Vector3i child_rel_step = Eigen::Vector3i((child_idx & 1) > 0, (child_idx & 2) > 0, (child_idx & 4) > 0);
    Eigen::Vector3i child_coord = child_rel_step * child_size; // Because, + corner is (0, 0, 0) at root depth
    volume_carver(child_coord, child_size, 1, child_rel_step, map.root());
  }
}



} // namespace allocator



namespace fetcher {



template<typename MapT,
         typename SensorT
>
inline std::vector<se::OctantBase*> frustum(MapT&                  map,
                                            const SensorT&         sensor,
                                            const Eigen::Matrix4f& T_MS)
{
  const Eigen::Matrix4f T_SM = se::math::to_inverse_transformation(T_MS);
  // The edge lengh of Blocks in voxels.
  constexpr int block_size = MapT::OctreeType::BlockType::getSize();
  // The radius of Blocks in metres.
  const float block_radius = std::sqrt(3.0f) / 2.0f * map.getRes() * block_size ;
  // Loop over all allocated Blocks.
  std::vector<se::OctantBase*> block_ptrs;
  for (auto block_ptr_itr = BlocksIterator<typename MapT::OctreeType>(map.getOctree().get());
      block_ptr_itr != BlocksIterator<typename MapT::OctreeType>(); ++block_ptr_itr) {
    auto& block = **block_ptr_itr;
    // Get the centre of the Block in the map frame.
    Eigen::Vector3f block_centre_point_M;
    map.voxelToPoint(block.getCoord(), block_size, block_centre_point_M);
    // Convert it to the sensor frame.
    const Eigen::Vector3f block_centre_point_S
        = (T_SM * block_centre_point_M.homogeneous()).head<3>();
    if (sensor.sphereInFrustum(block_centre_point_S, block_radius)) {
      block_ptrs.push_back(&block);
    }
  }
  return block_ptrs;
}



} // namespace fetcher



static inline Eigen::Vector3f get_sample_coord(const Eigen::Vector3i& octant_coord,
                                               const int              octant_size)
{
  return octant_coord.cast<float>() + se::sample_offset_frac * octant_size;
}



namespace {



/**
 * Integration helper struct for partial function specialisation
 */
template<se::Field FldT,
         se::Res ResT
>
struct IntegrateDepthImplD
{

    template<typename SensorT,
            typename MapT,
            typename ConfigT
    >
    static void integrate(const se::Image<se::depth_t>& depth_img,
                          const SensorT&                sensor,
                          const Eigen::Matrix4f&        T_MS,
                          const unsigned int            frame,
                          MapT&                         map,
                          ConfigT&                      /* config */); // TODO:
};



/**
 * Single-res TSDF integration helper struct for partial function specialisation
 */
template <>
struct IntegrateDepthImplD<se::Field::TSDF, se::Res::Single>
{
    template<typename SensorT,
            typename MapT,
            typename ConfigT
    >
    static void integrate(const se::Image<se::depth_t>& depth_img,
                          const SensorT&                sensor,
                          const Eigen::Matrix4f&        T_MS,
                          const unsigned int            frame,
                          MapT&                         map,
                          ConfigT&                      /* config */);
};



/**
 * Multi-res TSDF integration helper struct for partial function specialisation
 */
template <>
struct IntegrateDepthImplD<se::Field::TSDF, se::Res::Multi>
{
    template<typename SensorT,
            typename MapT,
            typename ConfigT
    >
    static void integrate(const se::Image<se::depth_t>& depth_img,
                          const SensorT&                sensor,
                          const Eigen::Matrix4f&        T_MS,
                          const unsigned int            frame,
                          MapT&                         map,
                          ConfigT&                      /* config */);
};



/**
 * Multi-res OFusion integration helper struct for partial function specialisation
 */
template <>
struct IntegrateDepthImplD<se::Field::Occupancy, se::Res::Multi>
{
    template<typename SensorT,
            typename MapT,
            typename ConfigT
    >
    static void integrate(const se::Image<se::depth_t>& depth_img,
                          const SensorT&                sensor,
                          const Eigen::Matrix4f&        T_MS,
                          const unsigned int            frame,
                          MapT&                         map,
                          ConfigT&                      /* config */);
};



template <typename MapT>
using IntegrateDepthImpl = IntegrateDepthImplD<MapT::fld_, MapT::res_>;



template<se::Field FldT,
         se::Res ResT
>
template<typename SensorT,
         typename MapT,
         typename ConfigT
>
void IntegrateDepthImplD<FldT, ResT>::integrate(const se::Image<se::depth_t>& depth_img,
                                                const SensorT&                sensor,
                                                const Eigen::Matrix4f&        T_MS,
                                                const unsigned int            frame,
                                                MapT&                         map,
                                                ConfigT&                      config)
{
}



template<typename SensorT, typename MapT, typename ConfigT>
void IntegrateDepthImplD<se::Field::TSDF, se::Res::Single>::integrate(const se::Image<se::depth_t>& depth_img,
                                                                      const SensorT&                sensor,
                                                                      const Eigen::Matrix4f&        T_MS,
                                                                      const unsigned int            frame,
                                                                      MapT&                         map,
                                                                      ConfigT&                      /* config */)
{
  const float truncation_boundary = map.getDataConfig().truncation_boundary;
  const se::weight_t max_weight   = map.getDataConfig().max_weight;

  // Allocation
  std::vector<OctantBase*> block_ptrs = se::allocator::frustum(depth_img, sensor, T_MS, map, 2 * truncation_boundary);

  // Update
  unsigned int block_size = MapT::OctreeType::block_size;
  const Eigen::Matrix4f T_SM = se::math::to_inverse_transformation(T_MS);

  auto valid_predicate = [&](float depth_value) { return depth_value >= sensor.near_plane; };

#pragma omp parallel for
  for (unsigned int i = 0; i < block_ptrs.size(); i++)
  {
    typename MapT::OctreeType::BlockType* block_ptr = static_cast<typename MapT::OctreeType::BlockType*>(block_ptrs[i]);
    block_ptr->setTimeStamp(frame);
    Eigen::Vector3i block_coord = block_ptr->getCoord();
    Eigen::Vector3f point_base_M;
    map.voxelToPoint(block_coord, point_base_M);
    const Eigen::Vector3f point_base_S = (T_SM * point_base_M.homogeneous()).head(3);
    const Eigen::Matrix3f point_delta_matrix_S = (se::math::to_rotation(T_SM) * map.getRes() *
                                                  Eigen::Matrix3f::Identity());

    for (unsigned int i = 0; i < block_size; ++i)
    {
      for (unsigned int j = 0; j < block_size; ++j)
      {
        for (unsigned int k = 0; k < block_size; ++k)
        {
          // Set voxel coordinates
          Eigen::Vector3i voxel_coord = block_coord + Eigen::Vector3i(i, j, k);

          // Set sample point in camera frame
          Eigen::Vector3f point_S = point_base_S + point_delta_matrix_S * Eigen::Vector3f(i, j, k);

          if (point_S.norm() > sensor.farDist(point_S))
          {
            continue;
          }

          // Fetch image value
          float depth_value(0);
          if (!sensor.projectToPixelValue(point_S, depth_img, depth_value, valid_predicate))
          {
            continue;
          }

          // Update the TSDF
          const float m = sensor.measurementFromPoint(point_S);
          const float sdf_value = (depth_value - m) / m * point_S.norm();
          if (sdf_value > -truncation_boundary)
          {
            const float tsdf_value = std::min(1.f, sdf_value / truncation_boundary);
            typename MapT::DataType data = block_ptr->getData(voxel_coord);
            data.tsdf = (data.tsdf * data.weight + tsdf_value) / (data.weight + 1.f);
            data.tsdf = se::math::clamp(data.tsdf, -1.f, 1.f);
            data.weight = std::min(data.weight + 1, max_weight);
            block_ptr->setData(voxel_coord, data);
          }

        } // k
      } // j
    } // i

  }

  se::propagator::propagateTimeStampToRoot(block_ptrs);
}




template<typename SensorT,
         typename MapT,
         typename ConfigT
>
void IntegrateDepthImplD<se::Field::TSDF, se::Res::Multi>::integrate(const se::Image<se::depth_t>& depth_img,
                                                                     const SensorT&                sensor,
                                                                     const Eigen::Matrix4f&        T_MS,
                                                                     const unsigned int            frame,
                                                                     MapT&                         map,
                                                                     ConfigT&                      /* config */)
{
  const float truncation_boundary = map.getDataConfig().truncation_boundary;
  const se::weight_t max_weight   = map.getDataConfig().max_weight;

  // Allocation
  std::vector<OctantBase*> block_ptrs = se::allocator::frustum(depth_img, sensor, T_MS, map, 2 * truncation_boundary);

  // Update
  unsigned int block_size = MapT::OctreeType::block_size;
  const Eigen::Matrix4f T_SM = se::math::to_inverse_transformation(T_MS);

  auto valid_predicate = [&](float depth_value) { return depth_value >= sensor.near_plane; };

  typedef typename MapT::OctreeType::BlockType BlockType;

#pragma omp parallel for
  for (unsigned int i = 0; i < block_ptrs.size(); i++)
  {
    typename MapT::OctreeType::BlockType* block_ptr = static_cast<BlockType*>(block_ptrs[i]);
    block_ptr->setTimeStamp(frame);
    Eigen::Vector3i block_coord = block_ptr->getCoord();
    Eigen::Vector3f block_centre_point_M;
    map.voxelToPoint(block_coord, block_ptr->getSize(), block_centre_point_M);
    const Eigen::Vector3f block_centre_point_S = (T_SM * block_centre_point_M.homogeneous()).head(3);
    const int last_curr_scale = block_ptr->getCurrentScale();
    const int lower_curr_scale_limit = last_curr_scale - 1;

    const int curr_scale = std::max(
            sensor.computeIntegrationScale(block_centre_point_S, map.getRes(), last_curr_scale, block_ptr->getMinScale(), block_ptr->getMaxScale()),
            lower_curr_scale_limit);

    block_ptr->setMinScale(block_ptr->getMinScale() < 0 ? curr_scale : std::min(block_ptr->getMinScale(), curr_scale));

    if (curr_scale < last_curr_scale)
    {
      auto parent_down_funct = [](const typename MapT::OctreeType& /* octree */,
                                  se::OctantBase*                  /* octant_ptr */,
                                  typename BlockType::DataUnion&   data_union)
      {
        data_union.prop_data.delta_tsdf   = data_union.data.tsdf;
        data_union.prop_data.delta_weight = 0;
      };

      auto child_down_funct = [&](const typename MapT::OctreeType&  octree,
                                  se::OctantBase*                   /* octant_ptr */,
                                  typename BlockType::DataUnion&    child_data_union,
                                  typename BlockType::DataUnion&    parent_data_union)
      {
        se::field_t delta_tsdf = parent_data_union.data.tsdf - parent_data_union.prop_data.delta_tsdf;

        if (child_data_union.data.weight != 0)
        {
          child_data_union.data.tsdf              = std::max(child_data_union.data.tsdf + delta_tsdf, -1.f);
          child_data_union.data.weight            = fminf(child_data_union.data.weight + parent_data_union.prop_data.delta_weight, max_weight);
          ;
          child_data_union.prop_data.delta_weight = parent_data_union.prop_data.delta_weight;
        }
        else
        {
          const Eigen::Vector3f child_sample_coord_f = se::get_sample_coord(child_data_union.coord, 1 << child_data_union.scale);
          int child_scale_returned;
          auto interp_field_value = se::visitor::getFieldInterp(octree, child_sample_coord_f, child_data_union.scale, child_scale_returned);

          if (interp_field_value)
          {
            child_data_union.data.tsdf              = *interp_field_value;
            child_data_union.data.weight            = parent_data_union.data.weight;

            child_data_union.prop_data.delta_tsdf   = child_data_union.data.tsdf;
            child_data_union.prop_data.delta_weight = 0;
          }
        }
      };

      se::propagator::propagateBlockDown(*map.getOctree(), static_cast<se::OctantBase*>(block_ptr), curr_scale, child_down_funct, parent_down_funct);
    }

    block_ptr->setCurrentScale(curr_scale);
    const int stride = 1 << curr_scale;

    Eigen::Vector3f point_base_M;
    map.voxelToPoint(block_coord, stride, point_base_M);
    const Eigen::Vector3f point_base_S = (T_SM * point_base_M.homogeneous()).head(3);
    const Eigen::Matrix3f point_delta_matrix_S = (se::math::to_rotation(T_SM) * map.getRes() *
                                                  Eigen::Matrix3f::Identity());

    for (unsigned int i = 0; i < block_size; i += stride)
    {
      for (unsigned int j = 0; j < block_size; j += stride)
      {
        for (unsigned int k = 0; k < block_size; k += stride)
        {
          // Set voxel coordinates
          Eigen::Vector3i voxel_coord = block_coord + Eigen::Vector3i(i, j, k);

          // Set sample point in camera frame
          Eigen::Vector3f point_S = point_base_S + point_delta_matrix_S * Eigen::Vector3f(i, j, k);

          if (point_S.norm() > sensor.farDist(point_S))
          {
            continue;
          }

          // Fetch image value
          float depth_value(0);
          if (!sensor.projectToPixelValue(point_S, depth_img, depth_value, valid_predicate))
          {
            continue;
          }

          // Update the TSDF
          const float m = sensor.measurementFromPoint(point_S);
          const float sdf_value = (depth_value - m) / m * point_S.norm();

          if (sdf_value > -truncation_boundary)
          {
            const float tsdf_value = std::min(1.f, sdf_value / truncation_boundary);
            typename MapT::OctreeType::BlockType::DataUnion data_union = block_ptr->getDataUnion(voxel_coord, block_ptr->getCurrentScale());
            data_union.data.tsdf   = (data_union.data.tsdf * data_union.data.weight + tsdf_value) / (data_union.data.weight + 1.f);
            data_union.data.tsdf   = se::math::clamp(data_union.data.tsdf, -1.f, 1.f);
            data_union.data.weight = std::min(data_union.data.weight + 1, max_weight);
            data_union.prop_data.delta_weight++;
            block_ptr->setDataUnion(data_union);
          }

        } // k
      } // j
    } // i


    auto parent_up_funct = [](typename BlockType::DataUnion& parent_data_union,
                              typename BlockType::DataType&  data_tmp,
                              const int                      sample_count)
    {
      if (sample_count != 0)
      {
        data_tmp.tsdf /= sample_count;
        data_tmp.weight /= sample_count;
        parent_data_union.data.tsdf              = data_tmp.tsdf;
        parent_data_union.prop_data.delta_tsdf   = data_tmp.tsdf;
        parent_data_union.data.weight            = ceil(data_tmp.weight);
        parent_data_union.prop_data.delta_weight = 0;
      } else
      {
        parent_data_union.data      = typename BlockType::DataType();
        parent_data_union.prop_data = typename BlockType::PropDataType();
      }
    };

    auto child_up_funct = [](typename BlockType::DataUnion& child_data_union, typename BlockType::DataType& data_tmp)
    {
      if (child_data_union.data.weight != 0)
      {
       data_tmp.tsdf   += child_data_union.data.tsdf;
       data_tmp.weight += child_data_union.data.weight;
       return 1;
      }
      return 0;
    };

    se::propagator::propagateBlockUp(*map.getOctree(), static_cast<se::OctantBase*>(block_ptr), curr_scale, child_up_funct, parent_up_funct);
  }

  se::propagator::propagateTimeStampToRoot(block_ptrs);
}



template<typename SensorT,
         typename MapT,
         typename ConfigT
>
void IntegrateDepthImplD<se::Field::Occupancy, se::Res::Multi>::integrate(const se::Image<se::depth_t>& depth_img,
                                                                          const SensorT&                sensor,
                                                                          const Eigen::Matrix4f&        T_MS,
                                                                          const unsigned int            frame,
                                                                          MapT&                         map,
                                                                          ConfigT&                      /* config */)
{
  // Create min/map depth pooling image for different bounding box sizes
  se::DensePoolingImage<SensorT> pooling_depth_image(depth_img);

  std::vector<se::OctantBase*> node_list;             ///< List of nodes to be updated
  std::vector<se::OctantBase*> block_list;            ///< List of blocks to be updated
  std::vector<bool>            low_variance_list;     ///< Has updated block low variance? <bool>
  std::vector<bool>            projects_inside_list;  ///< Does the updated block reproject completely into the image? <bool>

  const Eigen::Matrix4f T_SM = se::math::to_inverse_transformation(T_MS);

  // Allocate the frustum
  VolumeCarver<MapT, SensorT> volume_carver(depth_img, pooling_depth_image,
                                            map, sensor, T_SM, frame,
                                            node_list,                                            //< all to be freed
                                            block_list, low_variance_list, projects_inside_list); //< process based on low_variance and project inside

  // Launch on the 8 voxels of the first depth
  const int child_size     = map.getOctree()->getSize() / 2;
  se::OctantBase* root_ptr = map.getOctree()->getRoot();
#pragma omp parallel for
  for (int child_idx = 0; child_idx < 8; ++child_idx)
  {
    Eigen::Vector3i child_rel_step = Eigen::Vector3i((child_idx & 1) > 0, (child_idx & 2) > 0, (child_idx & 4) > 0);
    Eigen::Vector3i child_coord    = child_rel_step * child_size; // Because, + corner is (0, 0, 0) at root depth
    volume_carver(child_coord, child_size, 1, child_rel_step, root_ptr);
  }

  // ----------------------

  std::vector<std::set<se::OctantBase*>> node_set(map.getOctree()->getBlockDepth());
  std::vector<se::OctantBase*>           freed_block_list;
  MultiresOFusionUpdater<MapT, SensorT> updater(depth_img, map, sensor, T_SM, frame, node_set, freed_block_list);

#pragma omp parallel for
  for (unsigned int i = 0; i < node_list.size(); ++i)
  {
    auto node_ptr = static_cast<typename MapT::OctreeType::NodeType*>(node_list[i]);
    const int depth = map.getOctree()->getMaxScale() - se::math::log2_const(node_ptr->getSize());
    updater.freeNodeRecurse(node_list[i], depth);
  }

#pragma omp parallel for
  for (unsigned int i = 0; i < block_list.size(); ++i)
  {
    updater.updateBlock(block_list[i], low_variance_list[i], projects_inside_list[i]);
  }

  /// Propagation
#pragma omp parallel for
  for (unsigned int i = 0; i < block_list.size(); ++i)
  {
    updater::propagateBlockToCoarsestScale<typename MapT::OctreeType::BlockType>(block_list[i]);
  }
#pragma omp parallel for
  for (unsigned int i = 0; i < freed_block_list.size(); ++i)
  {
    updater::propagateBlockToCoarsestScale<typename MapT::OctreeType::BlockType>(freed_block_list[i]);
  }

  updater.propagateToRoot(block_list);
}



} // namespace anonymous



template<typename MapT>
MapIntegrator<MapT>::MapIntegrator(MapT&                   map,
                                   const IntegratorConfig& config)
    : map_(map), config_(config)
{
}



template<typename MapT>
template<typename SensorT>
void MapIntegrator<MapT>::integrateDepth(const se::Image<se::depth_t>& depth_img,
                                         const SensorT&                sensor,
                                         const Eigen::Matrix4f&        T_MS,
                                         const unsigned int            frame)
{
  IntegrateDepthImpl<MapT>::integrate(depth_img, sensor, T_MS, frame, map_, config_);
}



} // namespace se

#endif //SE_MAP_INTEGRATOR_IMPL_HPP
