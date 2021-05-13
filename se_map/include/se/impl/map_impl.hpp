#ifndef SE_MAP_IMPL_HPP
#define SE_MAP_IMPL_HPP

#include "se/timings.hpp"



namespace se {

template <Field     FldT,
          Colour    ColB,
          Semantics SemB,
          Res       ResT,
          unsigned  BlockSize
>
Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::Map(const Eigen::Vector3f& dim,
                                                  const float            res,
                                                  const se::DataConfig<FldT, ColB, SemB> data_config)
    : dim_(dim), res_(res), origin_M_(dim / 2),
     lb_(- origin_M_), ub_(dim - origin_M_),
     data_config_(data_config)
{
  initialiseOctree();
}



template <Field     FldT,
          Colour    ColB,
          Semantics SemB,
          Res       ResT,
          unsigned  BlockSize
>
Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::Map(const MapConfig&                       map_config,
                                                  const se::DataConfig<FldT, ColB, SemB> data_config)
    : dim_(map_config.dim), res_(map_config.res), origin_M_(map_config.origin),
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
          unsigned  BlockSize
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
          unsigned  BlockSize
>
template<Safe SafeB>
inline bool Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::getData(const Eigen::Vector3f& point_M,
                                                                  DataType&              data) const
{
  Eigen::Vector3i voxel_coord;

  if constexpr(SafeB == Safe::Off) // Evaluate at compile time
  {
    pointToVoxel<Safe::Off>(point_M, voxel_coord);
  } else
  {
    if (!pointToVoxel<Safe::On>(point_M, voxel_coord))
    {
      return false;
    }
  }

  return se::visitor::getData(*octree_ptr_, voxel_coord, data);
}



template <Field     FldT,
          Colour    ColB,
          Semantics SemB,
          Res       ResT,
          unsigned  BlockSize
>
template<Safe SafeB>
inline bool Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::interpField(const Eigen::Vector3f& point_M,
                                                                      float&                 field_value) const
{
  Eigen::Vector3f voxel_coord_f;

  if constexpr(SafeB == Safe::Off) // Evaluate at compile time
  {
    pointToVoxel<Safe::Off>(point_M, voxel_coord_f);
  } else
  {
    if (!pointToVoxel<Safe::On>(point_M, voxel_coord_f))
    {
      return false;
    }
  }
  return se::visitor::interpField(*octree_ptr_, voxel_coord_f, field_value);
}



template <Field     FldT,
          Colour    ColB,
          Semantics SemB,
          Res       ResT,
          unsigned  BlockSize
>
template<Safe SafeB>
inline bool Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::gradField(const Eigen::Vector3f& point_M,
                                                                    Eigen::Vector3f&       field_grad) const
{
  Eigen::Vector3f voxel_coord_f;

  if constexpr(SafeB == Safe::Off) // Evaluate at compile time
  {
    pointToVoxel<Safe::Off>(point_M, voxel_coord_f);
  } else
  {
    if (!pointToVoxel<Safe::On>(point_M, voxel_coord_f))
    {
    return false;
    }
  }

  const bool is_valid = se::visitor::gradField(*octree_ptr_, voxel_coord_f, field_grad);
  field_grad *= res_;

  return is_valid;
}



template <Field     FldT,
          Colour    ColB,
          Semantics SemB,
          Res       ResT,
          unsigned  BlockSize
>
template<Safe SafeB>
inline Eigen::Vector3f Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::gradField(const Eigen::Vector3f& point_M) const
{
  Eigen::Vector3f voxel_coord_f;

  if constexpr(SafeB == Safe::Off) // Evaluate at compile time
  {
    pointToVoxel<Safe::Off>(point_M, voxel_coord_f);
  } else
  {
    if (!pointToVoxel<Safe::On>(point_M, voxel_coord_f))
    {
    return Eigen::Vector3f::Constant(0);
    }
  }

  return res_ * se::visitor::gradField(*octree_ptr_, voxel_coord_f);
}



template <Field     FldT,
          Colour    ColB,
          Semantics SemB,
          Res       ResT,
          unsigned  BlockSize
>
bool Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::initialiseOctree()
{
  if (octree_ptr_ != nullptr)
  {
    std::cerr << "Octree has already been initialised" << std::endl;
    return false;
  }

  float    max_dim  = dim_.maxCoeff();
  unsigned max_size = ceil(max_dim / res_);
  unsigned oct_size = math::power_two_up(max_size);
  octree_ptr_ =
          std::shared_ptr<se::Octree<DataType, ResT, BlockSize> >(new se::Octree<DataType, ResT, BlockSize>(oct_size));
  return true;
}



template <Field     FldT,
          Colour    ColB,
          Semantics SemB,
          Res       ResT,
          unsigned  BlockSize
>
void Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::saveSlice(const std::string      file_path,
                                                             const Eigen::Vector3f& point_M,
                                                             const std::string      num)
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
          unsigned  BlockSize
>
void Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::saveStrucutre(const std::string file_path,
                                                                 const std::string num)
{
  const std::string file_name = (num == std::string("")) ? (file_path + ".ply") : (file_path + "_" + num + ".ply");
  se::io::save_octree_structure_ply(*octree_ptr_, file_name);
}




template <Field     FldT,
          Colour    ColB,
          Semantics SemB,
          Res       ResT,
          unsigned  BlockSize
>
void Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::saveMesh(const std::string file_path,
                                                            const std::string num)
{
  se::vector<se::Triangle> mesh;
  TICK("meshing")
  se::algorithms::marching_cube(*octree_ptr_, mesh);
  TOCK("meshing")

  const std::string file_name_mesh_primal = (num == std::string("")) ? (file_path + "_primal.vtk") : (file_path + "_primal_" + num + ".vtk");
  se::io::save_mesh_vtk(mesh, file_name_mesh_primal, Eigen::Matrix4f::Identity());

  se::vector<se::Triangle> dual_mesh;
  TICK("dual_meshing")
  se::algorithms::dual_marching_cube(*octree_ptr_, dual_mesh);
  TOCK("dual_meshing")

  const std::string file_name_mesh_dual = (num == std::string("")) ? (file_path + "_dual.vtk") : (file_path + "_dual_" + num + ".vtk");
  se::io::save_mesh_vtk(dual_mesh, file_name_mesh_dual, Eigen::Matrix4f::Identity());
}



template <Field     FldT,
          Colour    ColB,
          Semantics SemB,
          Res       ResT,
          unsigned  BlockSize
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
  voxel_coord = ((point_M + origin_M_) / res_).cast<int>();
  return true;
}



template <Field     FldT,
          Colour    ColB,
          Semantics SemB,
          Res       ResT,
          unsigned  BlockSize
>
template<se::Safe SafeB>
inline typename std::enable_if_t<SafeB == se::Safe::Off, bool>
Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::pointToVoxel(const Eigen::Vector3f& point_M,
                                                           Eigen::Vector3i&       voxel_coord) const
{
  voxel_coord = ((point_M + origin_M_) / res_).cast<int>();
  return true;
}



template <Field     FldT,
          Colour    ColB,
          Semantics SemB,
          Res       ResT,
          unsigned  BlockSize
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
  voxel_coord_f = ((point_M + origin_M_) / res_);
  return true;
}



template <Field     FldT,
          Colour    ColB,
          Semantics SemB,
          Res       ResT,
          unsigned  BlockSize
>
template<se::Safe SafeB>
inline typename std::enable_if_t<SafeB == se::Safe::Off, bool>
Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::pointToVoxel(const Eigen::Vector3f& point_M,
                                                           Eigen::Vector3f&       voxel_coord_f) const
{
  voxel_coord_f = ((point_M + origin_M_) / res_);
  return true;
}



template <Field     FldT,
          Colour    ColB,
          Semantics SemB,
          Res       ResT,
          unsigned  BlockSize
>
template<se::Safe SafeB>
inline typename std::enable_if_t<SafeB == se::Safe::On, bool>
Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::pointsToVoxels(const se::vector<Eigen::Vector3f>& points_M,
                                                             se::vector<Eigen::Vector3i>&       voxel_coords) const
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
        unsigned  BlockSize
>
template<se::Safe SafeB>
inline typename std::enable_if_t<SafeB == se::Safe::Off, bool>
Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::pointsToVoxels(const se::vector<Eigen::Vector3f>& points_M,
                                                             se::vector<Eigen::Vector3i>&       voxel_coords) const
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
          unsigned  BlockSize
>
inline void Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::voxelToPoint(const Eigen::Vector3i& voxel_coord,
                                                                       Eigen::Vector3f&       point_M) const
{
  point_M = ((voxel_coord.cast<float>() + sample_offset_frac) * res_) - origin_M_;
}

} // namespace se



#endif //SE_MAP_IMPL_HPP
