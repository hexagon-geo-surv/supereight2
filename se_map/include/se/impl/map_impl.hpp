#ifndef SE_MAP_IMPL_HPP
#define SE_MAP_IMPL_HPP

#include "se/timings.hpp"



namespace se {

template <se::Field FldT, se::Colour ColB, se::Semantics SemB, se::Res ResT, unsigned BlockSizeT>
Map<Data<FldT, ColB, SemB>, ResT, BlockSizeT>::Map(const Eigen::Vector3f& dim,
                                                   const float            res)
         : dim_(dim), res_(res), origin_M_(dim / 2),
           lb_(- origin_M_), ub_(dim - origin_M_)
{
  initialiseOctree();
}



template <se::Field FldT, se::Colour ColB, se::Semantics SemB, se::Res ResT, unsigned BlockSizeT>
Map<Data<FldT, ColB, SemB>, ResT, BlockSizeT>::Map(const MapConfig& mc)
        : dim_(mc.dim), res_(mc.res), origin_M_(mc.origin),
          lb_(- origin_M_), ub_(mc.dim - origin_M_)
{
  if (!contains(origin_M_))
  {
    std::cerr << "Map origin is outside the map" << std::endl;
  }
  initialiseOctree();
}


template <se::Field FldT, se::Colour ColB, se::Semantics SemB, se::Res ResT, unsigned BlockSizeT>
bool Map<Data<FldT, ColB, SemB>, ResT, BlockSizeT>::contains(const Eigen::Vector3f& point_M)
{
  return (point_M.x() >= lb_.x() && point_M.x() < ub_.x() &&
          point_M.y() >= lb_.y() && point_M.y() < ub_.y() &&
          point_M.z() >= lb_.z() && point_M.z() < ub_.z());
}



template <se::Field FldT, se::Colour ColB, se::Semantics SemB, se::Res ResT, unsigned BlockSizeT>
template<Safe SafeB>
bool Map<Data<FldT, ColB, SemB>, ResT, BlockSizeT>::getData(const Eigen::Vector3f& point_M, DataType& data) {
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

  return se::visitor::getData(octree_, voxel_coord, data);
}



template <se::Field FldT, se::Colour ColB, se::Semantics SemB, se::Res ResT, unsigned BlockSizeT>
template<Safe SafeB>
bool Map<Data<FldT, ColB, SemB>, ResT, BlockSizeT>::interpField(const Eigen::Vector3f& point_M,
                                                                float&                 field_value)
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
  return se::visitor::interpField(octree_, voxel_coord_f, field_value);
}



template <se::Field FldT, se::Colour ColB, se::Semantics SemB, se::Res ResT, unsigned BlockSizeT>
bool Map<Data<FldT, ColB, SemB>, ResT, BlockSizeT>::initialiseOctree()
{
  if (octree_ != nullptr)
  {
    std::cerr << "Octree has already been initialised" << std::endl;
    return false;
  }

  float    max_dim  = dim_.maxCoeff();
  unsigned max_size = ceil(max_dim / res_);
  unsigned oct_size = math::power_two_up(max_size);
  octree_ =
          std::shared_ptr<se::Octree<DataType, ResT, BlockSizeT> >(new se::Octree<DataType, ResT, BlockSizeT>(oct_size));
  return true;
}



template <se::Field FldT, se::Colour ColB, se::Semantics SemB, se::Res ResT, unsigned BlockSizeT>
void Map<Data<FldT, ColB, SemB>, ResT, BlockSizeT>::saveSlice(const Eigen::Vector3f& point_M, const std::string num)
{
  Eigen::Vector3i voxel_coord;
  pointToVoxel(point_M, voxel_coord);
  se::io::save_3d_slice_vtk(octree_, "./slice_x_" + num + ".vtk", Eigen::Vector3i(voxel_coord.x(), 0, 0), Eigen::Vector3i(voxel_coord.x() + 1, octree_->getSize(), octree_->getSize()));
  se::io::save_3d_slice_vtk(octree_, "./slice_y_" + num + ".vtk", Eigen::Vector3i(0, voxel_coord.y(), 0), Eigen::Vector3i(octree_->getSize(), voxel_coord.y() + 1, octree_->getSize()));
  se::io::save_3d_slice_vtk(octree_, "./slice_z_" + num + ".vtk", Eigen::Vector3i(0,0, voxel_coord.z()), Eigen::Vector3i(octree_->getSize(), octree_->getSize(), voxel_coord.z() + 1));
}



template <se::Field FldT, se::Colour ColB, se::Semantics SemB, se::Res ResT, unsigned BlockSizeT>
void Map<Data<FldT, ColB, SemB>, ResT, BlockSizeT>::saveStrucutre(const std::string num)
{
  se::io::save_octree_structure_ply(octree_, "./structure_" + num + ".ply");
}




template <se::Field FldT, se::Colour ColB, se::Semantics SemB, se::Res ResT, unsigned BlockSizeT>
void Map<Data<FldT, ColB, SemB>, ResT, BlockSizeT>::saveMesh(const std::string num)
{
  se::vector<se::Triangle> mesh;
  TICK("meshing")
  se::algorithms::marching_cube(octree_, mesh);
  TOCK("meshing")

  se::io::save_mesh_vtk(mesh, "./mesh_" + num + ".vtk", Eigen::Matrix4f::Identity());

  se::vector<se::Triangle> dual_mesh;
  TICK("dual_meshing")
  se::algorithms::dual_marching_cube(octree_, dual_mesh);
  TOCK("dual_meshing")

  se::io::save_mesh_vtk(dual_mesh, "./dual_mesh_" + num + ".vtk", Eigen::Matrix4f::Identity());
}



template <Field FldT, Colour ColB, Semantics SemB, Res ResT, unsigned BlockSizeT>
template<se::Safe SafeB>
inline typename std::enable_if_t<SafeB == se::Safe::On, bool>
        Map<Data<FldT, ColB, SemB>, ResT, BlockSizeT>::pointToVoxel(const Eigen::Vector3f& point_M,
                                                                    Eigen::Vector3i&       voxel_coord)
{
  if (!contains(point_M))
  {
  voxel_coord = Eigen::Vector3i::Constant(-1);
  return false;
  }
  voxel_coord = ((point_M + origin_M_) / res_).cast<int>();
  return true;
}



template <se::Field FldT, se::Colour ColB, se::Semantics SemB, se::Res ResT, unsigned BlockSizeT>
template<se::Safe SafeB>
inline typename std::enable_if_t<SafeB == se::Safe::Off, bool>
Map<Data<FldT, ColB, SemB>, ResT, BlockSizeT>::pointToVoxel(const Eigen::Vector3f& point_M,
                                                            Eigen::Vector3i&       voxel_coord)
{
  voxel_coord = ((point_M + origin_M_) / res_).cast<int>();
  return true;
}



template <Field FldT, Colour ColB, Semantics SemB, Res ResT, unsigned BlockSizeT>
template<se::Safe SafeB>
inline typename std::enable_if_t<SafeB == se::Safe::On, bool>
        Map<Data<FldT, ColB, SemB>, ResT, BlockSizeT>::pointToVoxel(const Eigen::Vector3f& point_M,
                                                                    Eigen::Vector3f&       voxel_coord_f)
{
  if (!contains(point_M))
  {
    voxel_coord_f = Eigen::Vector3f::Constant(-1);
    return false;
  }
  voxel_coord_f = ((point_M + origin_M_) / res_);
  return true;
}



template <se::Field FldT, se::Colour ColB, se::Semantics SemB, se::Res ResT, unsigned BlockSizeT>
template<se::Safe SafeB>
inline typename std::enable_if_t<SafeB == se::Safe::Off, bool>
        Map<Data<FldT, ColB, SemB>, ResT, BlockSizeT>::pointToVoxel(const Eigen::Vector3f& point_M,
                                                                    Eigen::Vector3f&       voxel_coord_f)
{
  voxel_coord_f = ((point_M + origin_M_) / res_);
  return true;
}



template <Field FldT, Colour ColB, Semantics SemB, Res ResT, unsigned BlockSizeT>
template<se::Safe SafeB>
typename std::enable_if_t<SafeB == se::Safe::On, bool>
Map<Data<FldT, ColB, SemB>, ResT, BlockSizeT>::pointsToVoxels(const se::vector<Eigen::Vector3f>& points_M,
                                                              se::vector<Eigen::Vector3i>&       voxel_coords)
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



template <Field FldT, Colour ColB, Semantics SemB, Res ResT, unsigned BlockSizeT>
template<se::Safe SafeB>
typename std::enable_if_t<SafeB == se::Safe::Off, bool>
Map<Data<FldT, ColB, SemB>, ResT, BlockSizeT>::pointsToVoxels(const se::vector<Eigen::Vector3f>& points_M,
                                                              se::vector<Eigen::Vector3i>&       voxel_coords)
{
  for (auto point_M : points_M)
  {
    Eigen::Vector3i voxel_coord;
    pointToVoxel<SafeB>(point_M, voxel_coord);
    voxel_coords.push_back(voxel_coord);
  }
  return true;
}



template <Field FldT, Colour ColB, Semantics SemB, Res ResT, unsigned BlockSizeT>
void Map<Data<FldT, ColB, SemB>, ResT, BlockSizeT>::voxelToPoint(const Eigen::Vector3i& voxel_coord, Eigen::Vector3f& point_M)
{
  point_M = (voxel_coord.cast<float>() * res_) - origin_M_;
}

} // namespace se



#endif //SE_MAP_IMPL_HPP
