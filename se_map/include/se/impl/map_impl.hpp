#ifndef SE_MAP_IMPL_HPP
#define SE_MAP_IMPL_HPP

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
                                                            Eigen::Vector3i& voxel_coord)
{
  std::cout << "Risk it all!" << std::endl;
  voxel_coord = ((point_M + origin_M_) / res_).cast<int>();
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
