#ifndef SE_SINGLERES_TSDF_UPDATER_HPP
#define SE_SINGLERES_TSDF_UPDATER_HPP



#include "se/map.hpp"
#include "se/sensor.hpp"



namespace se {



// Single-res TSDF updater
template<se::Colour    ColB,
         se::Semantics SemB,
         int           BlockSize,
         typename      SensorT
>
class Updater<Map<Data<se::Field::TSDF, ColB, SemB>, se::Res::Single, BlockSize>, SensorT>
{
public:
  typedef Map<Data<se::Field::TSDF, ColB, SemB>, se::Res::Single>  MapType;
  typedef typename MapType::DataType                               DataType;
  typedef typename MapType::OctreeType::NodeType                   NodeType;
  typedef typename MapType::OctreeType::BlockType                  BlockType;

  /**
   * \param[in]  map                  The reference to the map to be updated.
   * \param[in]  sensor               The sensor model.
   * \param[in]  depth_img            The depth image to be integrated.
   * \param[in]  T_MS                 The transformation from camera to map frame.
   * \param[in]  frame                The frame number to be integrated.
   */
  Updater(MapType&                map,
          const SensorT&          sensor,
          const se::Image<float>& depth_img,
          const Eigen::Matrix4f&  T_MS,
          const int               frame) :
      map_(map),
      sensor_(sensor),
      depth_img_(depth_img),
      T_MS_(T_MS),
      frame_(frame) {}


  template <typename UpdateListT,
            typename UpdateConfigT
  >
  void operator()(UpdateListT&         block_ptrs,
                  const UpdateConfigT& truncation_boundary)
  {
    unsigned int block_size = BlockType::getSize();
    const Eigen::Matrix4f T_SM = se::math::to_inverse_transformation(T_MS_);

    auto valid_predicate = [&](float depth_value) { return depth_value >= sensor_.near_plane; };

#pragma omp parallel for
    for (unsigned int i = 0; i < block_ptrs.size(); i++)
    {
      BlockType* block_ptr = static_cast<BlockType*>(block_ptrs[i]);
      block_ptr->setTimeStamp(frame_);
      Eigen::Vector3i block_coord = block_ptr->getCoord();
      Eigen::Vector3f point_base_M;
      map_.voxelToPoint(block_coord, point_base_M);
      const Eigen::Vector3f point_base_S = (T_SM * point_base_M.homogeneous()).head(3);
      const Eigen::Matrix3f point_delta_matrix_S = (se::math::to_rotation(T_SM) * map_.getRes() *
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

            if (point_S.norm() > sensor_.farDist(point_S))
            {
              continue;
            }

            // Fetch image value
            float depth_value(0);
            if (!sensor_.projectToPixelValue(point_S, depth_img_, depth_value, valid_predicate))
            {
              continue;
            }

            // Update the TSDF
            const float m = sensor_.measurementFromPoint(point_S);
            const float sdf_value = (depth_value - m) / m * point_S.norm();

            DataType& data = block_ptr->getData(voxel_coord);
            updateVoxel(data, sdf_value, truncation_boundary);
          } // k
        } // j
      } // i

    }

    se::propagator::propagateTimeStampToRoot(block_ptrs);
  }

private:
  void updateVoxel(DataType&      data,
                   const field_t  sdf_value,
                   const field_t  truncation_boundary)
  {
    if (sdf_value > -truncation_boundary)
    {
      const float tsdf_value = std::min(1.f, sdf_value / truncation_boundary);

      data.tsdf = (data.tsdf * data.weight + tsdf_value) / (data.weight + 1.f);
      data.tsdf = se::math::clamp(data.tsdf, -1.f, 1.f);
      data.weight = std::min(data.weight + 1, map_.getDataConfig().max_weight);
    }
  }

  MapType&                map_;
  const SensorT&          sensor_;
  const se::Image<float>& depth_img_;
  const Eigen::Matrix4f&  T_MS_;
  const int               frame_;
};



} // namespace se

#endif //SE_SINGLERES_TSDF_UPDATER_HPP
