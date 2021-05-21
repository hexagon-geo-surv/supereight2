#ifndef SE_MAP_IMPL_HPP
#define SE_MAP_IMPL_HPP

#include "se/timings.hpp"



namespace se {

template <Field     FldT,
          Colour    ColB,
          Semantics SemB,
          Res       ResT,
          int       BlockSize
>
Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::Map(const Eigen::Vector3f& dim,
                                                  const float            res,
                                                  const se::DataConfig<FldT, ColB, SemB> data_config)
    : dimension_(dim), resolution_(res), origin_M_(dim / 2),
     lb_(- origin_M_), ub_(dim - origin_M_),
     data_config_(data_config)
{
  initialiseOctree();
}



template <Field     FldT,
          Colour    ColB,
          Semantics SemB,
          Res       ResT,
          int       BlockSize
>
Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::Map(const MapConfig&                       map_config,
                                                  const se::DataConfig<FldT, ColB, SemB> data_config)
    : dimension_(map_config.dim), resolution_(map_config.res), origin_M_(map_config.origin),
      lb_(- origin_M_), ub_(map_config.dim - origin_M_),
      data_config_(data_config)
{
  if (!contains(origin_M_))
  {
    std::cerr << "Map origin is outside the map" << std::endl;
  }
  initialiseOctree();
}


template <Field     FldT,
          Colour    ColB,
          Semantics SemB,
          Res       ResT,
          int       BlockSize
>
inline bool Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::contains(const Eigen::Vector3f& point_M) const
{
  return (point_M.x() >= lb_.x() && point_M.x() < ub_.x() &&
          point_M.y() >= lb_.y() && point_M.y() < ub_.y() &&
          point_M.z() >= lb_.z() && point_M.z() < ub_.z());
}



template <Field     FldT,
          Colour    ColB,
          Semantics SemB,
          Res       ResT,
          int       BlockSize
>
template<Safe SafeB>
inline const typename Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::DataType
Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::getData(const Eigen::Vector3f& point_M) const
{
  Eigen::Vector3i voxel_coord;

  if constexpr(SafeB == Safe::Off) // Evaluate at compile time
  {
    pointToVoxel<Safe::Off>(point_M, voxel_coord);
  } else
  {
    if (!pointToVoxel<Safe::On>(point_M, voxel_coord))
    {
      return DataType();
    }
  }

  return se::visitor::getData(*octree_ptr_, voxel_coord);
}



template <Field     FldT,
          Colour    ColB,
          Semantics SemB,
          Res       ResT,
          int       BlockSize
>
template<Safe SafeB>
inline std::optional<se::field_t> Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::getFieldInterp(const Eigen::Vector3f& point_M) const
{
  Eigen::Vector3f voxel_coord_f;

  if constexpr(SafeB == Safe::Off) // Evaluate at compile time
  {
    pointToVoxel<Safe::Off>(point_M, voxel_coord_f);
  } else
  {
    if (!pointToVoxel<Safe::On>(point_M, voxel_coord_f))
    {
      return {};
    }
  }

  return se::visitor::getFieldInterp(*octree_ptr_, voxel_coord_f);
}



template <Field     FldT,
          Colour    ColB,
          Semantics SemB,
          Res       ResT,
          int       BlockSize
>
template<Safe SafeB,
         Res ResTDummy = ResT
>
inline typename std::enable_if_t<ResTDummy == Res::Multi, std::optional<se::field_t>>
Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::getFieldInterp(const Eigen::Vector3f& point_M,
                                                             int&                   returned_scale) const
{
  Eigen::Vector3f voxel_coord_f;

  if constexpr(SafeB == Safe::Off) // Evaluate at compile time
  {
    pointToVoxel<Safe::Off>(point_M, voxel_coord_f);
  } else
  {
    if (!pointToVoxel<Safe::On>(point_M, voxel_coord_f))
    {
      return {};
    }
  }

  return se::visitor::getFieldInterp(*octree_ptr_, voxel_coord_f, returned_scale);
}



template <Field     FldT,
          Colour    ColB,
          Semantics SemB,
          Res       ResT,
          int       BlockSize
>
template<Safe SafeB>
inline std::optional<se::field_vec_t> Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::getFieldGrad(const Eigen::Vector3f& point_M) const
{
  Eigen::Vector3f voxel_coord_f;

  if constexpr(SafeB == Safe::Off) // Evaluate at compile time
  {
    pointToVoxel<Safe::Off>(point_M, voxel_coord_f);
  } else
  {
    if (!pointToVoxel<Safe::On>(point_M, voxel_coord_f))
    {
    return {};
    }
  }

  auto field_grad = se::visitor::getFieldGrad(*octree_ptr_, voxel_coord_f);
  if (field_grad)
  {
    *field_grad *= resolution_;
  }

  return field_grad;
}



template <Field     FldT,
          Colour    ColB,
          Semantics SemB,
          Res       ResT,
          int       BlockSize
>
bool Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::initialiseOctree()
{
  if (octree_ptr_ != nullptr)
  {
    std::cerr << "Octree has already been initialised" << std::endl;
    return false;
  }

  float    max_dim  = dimension_.maxCoeff();
  unsigned max_size = ceil(max_dim / resolution_);
  unsigned oct_size = math::power_two_up(max_size);
  octree_ptr_ =
          std::shared_ptr<se::Octree<DataType, ResT, BlockSize> >(new se::Octree<DataType, ResT, BlockSize>(oct_size));
  return true;
}



template <Field     FldT,
          Colour    ColB,
          Semantics SemB,
          Res       ResT,
          int       BlockSize
>
void Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::saveSlice(const std::string&     file_path,
                                                             const Eigen::Vector3f& point_M,
                                                             const std::string&     num) const
{
  Eigen::Vector3i voxel_coord;
  pointToVoxel(point_M, voxel_coord);

  const std::string file_name_x = (num == std::string("")) ? (file_path + "_x.vtk") : (file_path + "_x_" + num + ".vtk");
  const std::string file_name_y = (num == std::string("")) ? (file_path + "_y.vtk") : (file_path + "_y_" + num + ".vtk");
  const std::string file_name_z = (num == std::string("")) ? (file_path + "_z.vtk") : (file_path + "_z_" + num + ".vtk");
  se::io::save_3d_slice_vtk(*octree_ptr_, file_name_x, Eigen::Vector3i(voxel_coord.x(), 0, 0), Eigen::Vector3i(voxel_coord.x() + 1, octree_ptr_->getSize(), octree_ptr_->getSize()));
  se::io::save_3d_slice_vtk(*octree_ptr_, file_name_y, Eigen::Vector3i(0, voxel_coord.y(), 0), Eigen::Vector3i(octree_ptr_->getSize(), voxel_coord.y() + 1, octree_ptr_->getSize()));
  se::io::save_3d_slice_vtk(*octree_ptr_, file_name_z, Eigen::Vector3i(0,0, voxel_coord.z()), Eigen::Vector3i(octree_ptr_->getSize(), octree_ptr_->getSize(), voxel_coord.z() + 1));
}



template <Field     FldT,
          Colour    ColB,
          Semantics SemB,
          Res       ResT,
          int       BlockSize
>
void Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::saveStrucutre(const std::string& file_path,
                                                                 const std::string& num) const
{
  const std::string file_name = (num == std::string("")) ? (file_path + ".ply") : (file_path + "_" + num + ".ply");
  se::io::save_octree_structure_ply(*octree_ptr_, file_name);
}




template <Field     FldT,
          Colour    ColB,
          Semantics SemB,
          Res       ResT,
          int       BlockSize
>
void Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::saveMesh(const std::string& file_path,
                                                            const std::string& num) const
{
  std::vector<se::Triangle> mesh;
  se::algorithms::marching_cube(*octree_ptr_, mesh);

  const std::string file_name_mesh_primal = (num == std::string("")) ? (file_path + "_primal.vtk") : (file_path + "_primal_" + num + ".vtk");
  se::io::save_mesh_vtk(mesh, file_name_mesh_primal, Eigen::Matrix4f::Identity());

  std::vector<se::Triangle> dual_mesh;
  se::algorithms::dual_marching_cube(*octree_ptr_, dual_mesh);

  const std::string file_name_mesh_dual = (num == std::string("")) ? (file_path + "_dual.vtk") : (file_path + "_dual_" + num + ".vtk");
  se::io::save_mesh_vtk(dual_mesh, file_name_mesh_dual, Eigen::Matrix4f::Identity());
}



template <Field     FldT,
          Colour    ColB,
          Semantics SemB,
          Res       ResT,
          int       BlockSize
>
template<se::Safe SafeB>
inline typename std::enable_if_t<SafeB == se::Safe::On, bool>
Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::pointToVoxel(const Eigen::Vector3f& point_M,
                                                           Eigen::Vector3i&       voxel_coord) const
{
  if (!contains(point_M))
  {
    voxel_coord = Eigen::Vector3i::Constant(-1);
    return false;
  }
  voxel_coord = ((point_M + origin_M_) / resolution_).cast<int>();
  return true;
}



template <Field     FldT,
          Colour    ColB,
          Semantics SemB,
          Res       ResT,
          int       BlockSize
>
template<se::Safe SafeB>
inline typename std::enable_if_t<SafeB == se::Safe::Off, bool>
Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::pointToVoxel(const Eigen::Vector3f& point_M,
                                                           Eigen::Vector3i&       voxel_coord) const
{
  voxel_coord = ((point_M + origin_M_) / resolution_).cast<int>();
  return true;
}



template <Field     FldT,
          Colour    ColB,
          Semantics SemB,
          Res       ResT,
          int       BlockSize
>
template<se::Safe SafeB>
inline typename std::enable_if_t<SafeB == se::Safe::On, bool>
Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::pointToVoxel(const Eigen::Vector3f& point_M,
                                                           Eigen::Vector3f&       voxel_coord_f) const
{
  if (!contains(point_M))
  {
    voxel_coord_f = Eigen::Vector3f::Constant(-1);
    return false;
  }
  voxel_coord_f = ((point_M + origin_M_) / resolution_);
  return true;
}



template <Field     FldT,
          Colour    ColB,
          Semantics SemB,
          Res       ResT,
          int       BlockSize
>
template<se::Safe SafeB>
inline typename std::enable_if_t<SafeB == se::Safe::Off, bool>
Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::pointToVoxel(const Eigen::Vector3f& point_M,
                                                           Eigen::Vector3f&       voxel_coord_f) const
{
  voxel_coord_f = ((point_M + origin_M_) / resolution_);
  return true;
}



template <Field     FldT,
          Colour    ColB,
          Semantics SemB,
          Res       ResT,
          int       BlockSize
>
template<se::Safe SafeB>
inline typename std::enable_if_t<SafeB == se::Safe::On, bool>
Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::pointsToVoxels(const std::vector<Eigen::Vector3f>& points_M,
                                                             std::vector<Eigen::Vector3i>&       voxel_coords) const
{
  bool all_valid = true;

  for (auto point_M : points_M)
  {
    Eigen::Vector3i voxel_coord;
    if (pointToVoxel<SafeB>(point_M, voxel_coord))
    {
      voxel_coords.push_back(voxel_coord);
    } else
    {
      all_valid = false;
    }
  }
  return all_valid;
}



template <Field     FldT,
          Colour    ColB,
          Semantics SemB,
          Res       ResT,
          int       BlockSize
>
template<se::Safe SafeB>
inline typename std::enable_if_t<SafeB == se::Safe::Off, bool>
Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::pointsToVoxels(const std::vector<Eigen::Vector3f>& points_M,
                                                             std::vector<Eigen::Vector3i>&       voxel_coords) const
{
  for (auto point_M : points_M)
  {
    Eigen::Vector3i voxel_coord;
    pointToVoxel<SafeB>(point_M, voxel_coord);
    voxel_coords.push_back(voxel_coord);
  }
  return true;
}



template <Field     FldT,
          Colour    ColB,
          Semantics SemB,
          Res       ResT,
          int       BlockSize
>
inline void Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::voxelToPoint(const Eigen::Vector3i& voxel_coord,
                                                                       Eigen::Vector3f&       point_M) const
{
  point_M = ((voxel_coord.cast<float>() + sample_offset_frac) * resolution_) - origin_M_;
}



template <Field     FldT,
          Colour    ColB,
          Semantics SemB,
          Res       ResT,
          int  BlockSize
>
inline void Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::voxelToPoint(const Eigen::Vector3i& voxel_coord,
                                                                       const int              stride,
                                                                       Eigen::Vector3f&       point_M) const
{
  point_M = ((voxel_coord.cast<float>() + stride * sample_offset_frac) * resolution_) - origin_M_;
}



} // namespace se



#endif //SE_MAP_IMPL_HPP
