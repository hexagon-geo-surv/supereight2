#ifndef SE_MAP_HPP
#define SE_MAP_HPP

#include <memory>
#include <Eigen/Dense>

#include "se/utils/math_util.hpp"
#include "se/utils/setup_util.hpp"
#include "se/data.hpp"
#include "se/octree.hpp"
#include "se/io/octree_io.hpp"
#include "se/io/meshing_io.hpp"
#include "se/visitor.hpp"
#include "se/fetcher.hpp"

#include "se/raycaster.hpp"

namespace se {

// Default values
Eigen::Vector3f default_map_dim    = Eigen::Vector3f(10, 10, 3); ///< 10m x 10m x 3m
float           default_map_res    = 0.1;                                 ///< 10cm
Eigen::Vector3f default_map_origin = default_map_dim / 2;

struct MapConfig {
public:
  Eigen::Vector3f dim    = default_map_dim;
  float           res    = default_map_res;
  Eigen::Vector3f origin = default_map_origin;
};

// Forward Declaration
template <typename DataT      = se::Data<Field::TSDF, Colour::Off, Semantics::Off>,
          Res      ResT       = Res::Single,
          unsigned BlockSizeT = 8
> class Map;



template <Field     FldT,
          Colour    ColB,
          Semantics SemB,
          Res       ResT,
          unsigned BlockSizeT
>
class Map<se::Data<FldT, ColB, SemB>, ResT, BlockSizeT> {
public:
  typedef Data<FldT, ColB, SemB> DataType;
  typedef se::Octree<DataType, ResT, BlockSizeT> OctreeType;

  Map(const Eigen::Vector3f& dim,
      const float            res);

  Map(const MapConfig& mc);

  bool contains(const Eigen::Vector3f& point_M);

  /**
   * Get the dimensions of the map in [meter] (length x width x height)
   *
   * \return Dimensions of the map
   */
  const Eigen::Vector3f& getDim() { return dim_; }
  float getRes() { return res_; }

  /**
   * \brief
   *
   * \param[in]  point The coordinates of the point in map frame [meter] to evaluate
   * \param[out] data
   *
   * \return
   */
  template<Safe SafeB = Safe::Off>
  bool getData(const Eigen::Vector3f& point_M, DataType& data);

  void saveSlice(const Eigen::Vector3f& point_M)
  {
    Eigen::Vector3i voxel_coord;
    pointToVoxel(point_M, voxel_coord);
    se::io::save_3d_slice_vtk(octree_, "./slice_x.vtk", Eigen::Vector3i(voxel_coord.x(), 0, 0), Eigen::Vector3i(voxel_coord.x() + 1, octree_->getSize(), octree_->getSize()));
    se::io::save_3d_slice_vtk(octree_, "./slice_y.vtk", Eigen::Vector3i(0, voxel_coord.y(), 0), Eigen::Vector3i(octree_->getSize(), voxel_coord.y() + 1, octree_->getSize()));
    se::io::save_3d_slice_vtk(octree_, "./slice_z.vtk", Eigen::Vector3i(0,0, voxel_coord.z()), Eigen::Vector3i(octree_->getSize(), octree_->getSize(), voxel_coord.z() + 1));
  };

  bool saveStrucutre()
  {
    se::io::save_octree_structure_ply(octree_, "./structure.ply");
    return true;
  }

  void voxelToPoint(const Eigen::Vector3i& voxel_coord, Eigen::Vector3f& point_M);

  static constexpr Field     fld_ = FldT;
  static constexpr Colour    col_ = ColB;
  static constexpr Semantics sem_ = SemB;

  static constexpr Res       ress_ = ResT;



//protected:

  bool initialiseOctree();

  template<se::Safe SafeB = se::Safe::On>
  inline typename std::enable_if_t<SafeB == se::Safe::On, bool>
  pointToVoxel(const Eigen::Vector3f& point_M, Eigen::Vector3i& voxel_coord);

  /**
   *
   * \note Use as `map. template pointToVoxel<se::Safe::Off>(...)
   *
   * \tparam SafeB
   * \param point_M
   * \param voxel_coord
   * \return
   */
  template<se::Safe SafeB>
  typename std::enable_if_t<SafeB == se::Safe::Off, bool>
  inline pointToVoxel(const Eigen::Vector3f& point_M, Eigen::Vector3i& voxel_coord);

  template<se::Safe SafeB = se::Safe::On>
  typename std::enable_if_t<SafeB == se::Safe::On, bool>
  pointsToVoxels(const se::vector<Eigen::Vector3f>& points_M,
                 se::vector<Eigen::Vector3i>&       voxel_coords);

  /**
   *
   * \note Use as `map. template pointsToVoxels<se::Safe::Off>(...)
   *
   * \tparam SafeB
   * \param points_M
   * \param voxel_coords
   * \return
   */
  template<se::Safe SafeB>
  typename std::enable_if_t<SafeB == se::Safe::Off, bool>
  pointsToVoxels(const se::vector<Eigen::Vector3f>& points_M,
                 se::vector<Eigen::Vector3i>&       voxel_coords);

  const Eigen::Vector3f dim_;      ///< The dimensions of the map
  const float res_;                ///< The resolution of the map
  const Eigen::Vector3f origin_M_; ///< The origin of the map frame

  const Eigen::Vector3f lb_;       ///< The lower map bound
  const Eigen::Vector3f ub_;       ///< The upper map bound

  std::shared_ptr< OctreeType >octree_ = nullptr;

  friend class SimpleIntegrator;
  template<typename SensorT, typename MapT>
  friend std::vector<typename MapT::OctreeType::BlockType::Ptr> se::fetcher::frustum(const se::Image<depth_t>& depth_img,
                                                                                     SensorT&                  sensor,
                                                                                     const Eigen::Matrix4f&    T_MS,
                                                                                     MapT&                     map);
};

//// Full alias template for alternative setup
template <se::Field     FldT      = se::Field::TSDF,
          se::Colour    ColB      = se::Colour::Off,
          se::Semantics SemB      = se::Semantics::Off,
          se::Res       ResT      = se::Res::Single,
          unsigned BlockSizeT     = 8
>
using MapD = Map<Data<FldT, ColB, SemB>, ResT, BlockSizeT> ;


// Occupancy map setups
template <se::Res ResT = se::Res::Single, unsigned BlockSizeT = 8>
using OccMap = Map<OccData, ResT, BlockSizeT> ;

template <se::Res ResT = se::Res::Single, unsigned BlockSizeT = 8>
using OccColMap = Map<OccColData, ResT, BlockSizeT> ;

template <se::Res ResT = se::Res::Single, unsigned BlockSizeT = 8>
using OccSemMap = Map<OccSemData, ResT, BlockSizeT> ;

template <se::Res ResT = se::Res::Single, unsigned BlockSizeT = 8>
using OccColSemMap = Map<OccColSemData, ResT, BlockSizeT> ;


// TSDF map setups
template <se::Res ResT = se::Res::Single, unsigned BlockSizeT = 8>
using TSDFMap = Map<TSDFData, ResT, BlockSizeT> ;

template <se::Res ResT = se::Res::Single, unsigned BlockSizeT = 8>
using TSDFColMap = Map<TSDFColData, ResT, BlockSizeT> ;

template <se::Res ResT = se::Res::Single, unsigned BlockSizeT = 8>
using TSDFSemMap = Map<TSDFSemData, ResT, BlockSizeT> ;

template <se::Res ResT = se::Res::Single, unsigned BlockSizeT = 8>
using TSDFColSemMap = Map<TSDFColSemData, ResT, BlockSizeT> ;

} // namespace se

#include "impl/map_impl.hpp"

#endif //SE_MAP_HPP
