#ifndef SE_MAP_INTEGRATOR_HPP
#define SE_MAP_INTEGRATOR_HPP

#include <cstddef>
#include <iterator>

#include "se/utils/setup_util.hpp"
#include "se/octree/integrator.hpp"
#include "se/octree/fetcher.hpp"
#include "se/timings.hpp"



#define MU 0.16



namespace se {

template <typename BlockT, typename MapT>
class BlockIterator {
public:
  BlockIterator(BlockT*               block_ptr,
                MapT&                 map,
                const Eigen::Matrix4f T_SM)
          : block_ptr_(block_ptr), idx_(0)
  {
    Eigen::Vector3i block_coord = block_ptr->getCoord();
    Eigen::Vector3f point_base_M;
    map.voxelToPoint(block_coord , point_base_M);
    point_base_S_         = (T_SM * point_base_M.homogeneous()).head(3);
    point_delta_matrix_S_ = (se::math::to_rotation(T_SM) * map.getRes() * Eigen::Matrix3f::Identity());
  };

  bool next(typename BlockT::DataType& data,
            Eigen::Vector3f&           point_S) {
    block_ptr_->getData(idx_, data);
    point_S = point_base_S_ + point_delta_matrix_S_ * Eigen::Vector3f(idx_ & 7, (idx_ >> 3) & 7, (idx_ >> 6) & 7);
    return idx_++ < block_ptr_->size_cu;
  }

  bool set(typename BlockT::DataType& data) {
    block_ptr_->setData(idx_ - 1, data);
    return true;
  }

private:
  BlockT* block_ptr_;
  unsigned int idx_;
  Eigen::Vector3f point_base_S_;
  Eigen::Matrix3f point_delta_matrix_S_;
};

namespace allocator {

template<typename SensorT, typename MapT>
std::vector<typename MapT::OctreeType::BlockType*> frustum(const se::Image<depth_t>&  depth_img,
                                                           SensorT&                   sensor,
                                                           const Eigen::Matrix4f&     T_MS,
                                                           MapT&                      map)
{
  TICK("build-allocation-list")
  typedef typename MapT::OctreeType::BlockType BlockType;
  auto octree_ptr = map.getOctree();

  const float band = 2.f * MU;
  const int num_steps = ceil(band / map.getRes());

  const Eigen::Vector3f t_MS = T_MS.topRightCorner<3, 1>();

#pragma omp declare reduction (merge : std::set<se::key_t> : omp_out.insert(omp_in.begin(), omp_in.end()))
  se::set<se::key_t> voxel_key_set;

#ifdef _OPENMP
  omp_set_num_threads(10);
#endif
#pragma omp parallel for reduction(merge: voxel_key_set)
  for (int x = 0; x < depth_img.width(); ++x)
  {
    for (int y = 0; y < depth_img.height(); ++y)
    {
      const Eigen::Vector2i pixel(x, y);
      const float depth_value = depth_img(pixel.x(), pixel.y());
      if (depth_value < sensor.near_plane)
      {
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
      for (int i = 0; i < num_steps; i++)
      {
        Eigen::Vector3i voxel_coord;

        if (map. template pointToVoxel<se::Safe::On>(ray_pos_M, voxel_coord))
        {
          se::key_t voxel_key;
          se::keyops::encode_key(voxel_coord, octree_ptr->max_block_scale, voxel_key);
          voxel_key_set.insert(voxel_key);
        }
        ray_pos_M += step;
      }
    }
  }
  TOCK("build-allocation-list")

  se::vector<key_t> voxel_keys(voxel_key_set.begin(), voxel_key_set.end());

  TICK("allocate-frustum")
  se::vector<BlockType*> fetched_block_ptrs = se::allocator::blocks(voxel_keys, octree_ptr, octree_ptr->getRoot());
  TOCK("allocate-frustum")

  return fetched_block_ptrs;
}

} // namespace allocator


namespace {


template<se::Field FldT,
        se::Res ResT
>
struct IntegrateDepthImplD {
    template<typename SensorT, typename MapT, typename ConfigT>
    static bool integrate(const se::Image<se::depth_t>& depth_img,
                          const SensorT&                sensor,
                          const Eigen::Matrix4f&        T_MS,
                          MapT&                         map,
                          ConfigT&                      config)
    {
      return false;
    }
};



static inline Eigen::Vector3f get_sample_coord(const Eigen::Vector3i& octant_coord,
                                               const int              octant_size,
                                               const Eigen::Vector3f& sample_offset_frac) {
  return octant_coord.cast<float>() + sample_offset_frac * octant_size;
}

template <>
struct IntegrateDepthImplD<se::Field::TSDF, se::Res::Single>
{
  template <typename SensorT, typename MapT, typename ConfigT>
  static bool integrate(const se::Image<se::depth_t>& depth_img,
                        const SensorT&                sensor,
                        const Eigen::Matrix4f&        T_MS,
                        MapT&                         map,
                        ConfigT&                      /* config */)
  {
    const float        truncation_boundary = map.getDataConfig().truncation_boundary;
    const se::weight_t max_weight          = map.getDataConfig().max_weight;

    // Allocation
    std::vector<typename MapT::OctreeType::BlockType*> block_ptrs = se::allocator::frustum(depth_img, sensor, T_MS, map);

    // Update
    TICK("update")
    unsigned int block_size = MapT::OctreeType::block_size;
    const Eigen::Matrix4f T_SM = se::math::to_inverse_transformation(T_MS);

    auto valid_predicate = [&](float depth_value){ return depth_value >= sensor.near_plane; };

#ifdef _OPENMP
    omp_set_num_threads(10);
#endif
#pragma omp parallel for
    for (unsigned int i = 0; i < block_ptrs.size(); i++)
    {
      auto block_ptr = block_ptrs[i];
      Eigen::Vector3i block_coord = block_ptr->getCoord();
      Eigen::Vector3f point_base_M;
      map.voxelToPoint(block_coord , point_base_M);
      const Eigen::Vector3f point_base_S         = (T_SM * point_base_M.homogeneous()).head(3);
      const Eigen::Matrix3f point_delta_matrix_S = (se::math::to_rotation(T_SM) * map.getRes() * Eigen::Matrix3f::Identity());

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
              const float tsdf_value = fminf(1.f, sdf_value / truncation_boundary);
              typename MapT::DataType data;
              block_ptr->getData(voxel_coord, data);
              data.tsdf = (data.tsdf * data.weight + tsdf_value) / (data.weight + 1.f);
              data.tsdf = se::math::clamp(data.tsdf, -1.f, 1.f);
              data.weight = fminf(data.weight + 1, max_weight);
              block_ptr->setData(voxel_coord, data);
            }
          }
        }
      }
    }
    TOCK("update")
    return true;
  }
};



template <typename MapT>
using IntegrateDepthImpl = IntegrateDepthImplD<MapT::fld_, MapT::ress_>;

}


template <typename MapT, typename ConfigT>
class MapIntegrator
{
public:
  MapIntegrator(MapT&   map,
                ConfigT config)
      : map_(map), config_(config) { };

  template <typename SensorT>
  bool integrateDepth(const se::Image<se::depth_t>&      depth_img,
                      const SensorT&                     sensor,
                      const Eigen::Matrix4f&             T_MS)
  {
    return IntegrateDepthImpl<MapT>::integrate(depth_img, sensor, T_MS, map_, config_);
  }



private:
  MapT&    map_;
  ConfigT  config_;
};



class SimpleIntegrator
{
public:
  template <typename MapT>
  bool setData(MapT&                         map,
               const typename MapT::DataType data,
               const Eigen::Vector3f&        point_M) {
    Eigen::Vector3i voxel_coord;
    map.pointToVoxel(point_M, voxel_coord);
    se::integrator::setData(map.octree_, voxel_coord, data);
    return true;
  }
};

} // namespace se

#endif //SE_TRYOUT_MAP_INTEGRATOR_HPP
