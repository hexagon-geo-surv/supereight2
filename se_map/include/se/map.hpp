#ifndef SE_MAP_HPP
#define SE_MAP_HPP

#include <memory>
#include <ostream>

#include <Eigen/Dense>

#include "se/utils/math_util.hpp"
#include "se/utils/setup_util.hpp"
#include "se/data.hpp"
#include "se/octree/octree.hpp"
#include "se/octree/visitor.hpp"
#include "se/octree/fetcher.hpp"
#include "se/io/octree_io.hpp"
#include "se/io/meshing_io.hpp"

#include "se/raycaster.hpp"



namespace se {

struct MapConfig
{
  Eigen::Vector3f dim;
  float           res;
  Eigen::Vector3f origin;

  /** Initializes the config to a 10m x 10m x 3m map with a 10cm resolution and the origin at the
   * centre of the volume.
   */
  MapConfig();

  /** Initializes the config from a YAML file. Data not present in the YAML file will be initialized
   * as in MapConfig::MapConfig().
   */
  MapConfig(const std::string& yaml_file);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

std::ostream& operator<<(std::ostream& os, const MapConfig& c);



// Forward Declaration
template <typename DataT     = se::Data<Field::TSDF, Colour::Off, Semantics::Off>,
          Res      ResT      = Res::Single,
          int      BlockSize = 8
> class Map;



template <Field     FldT,
          Colour    ColB,
          Semantics SemB,
          Res       ResT,
          int       BlockSize
>
class Map<se::Data<FldT, ColB, SemB>, ResT, BlockSize>
{
public:
  typedef Data<FldT, ColB, SemB> DataType;
  typedef DataConfig<FldT, ColB, SemB> DataConfigType;
  typedef se::Octree<DataType, ResT, BlockSize> OctreeType;

  Map(const Eigen::Vector3f&                 dim,
      const float                            res,
      const se::DataConfig<FldT, ColB, SemB> data_config = se::DataConfig<FldT, ColB, SemB>());

  Map(const MapConfig&                       map_config,
      const se::DataConfig<FldT, ColB, SemB> data_config = se::DataConfig<FldT, ColB, SemB>());

  inline bool contains(const Eigen::Vector3f& point_M) const;

  /**
   * Get the dimensions of the map in [meter] (length x width x height)
   *
   * \return Dimensions of the map
   */
  inline Eigen::Vector3f getDim() const { return dimension_; }

  inline float getRes() const { return resolution_; }

  inline DataConfigType getDataConfig() const { return data_config_; }

  /**
   * \brief
   *
   * \param[in]  point The coordinates of the point in map frame [meter] to evaluate
   * \param[out] data
   *
   * \return
   */
  template<Safe SafeB = Safe::Off>
  inline const DataType getData (const Eigen::Vector3f& point_M) const;

  template<Safe SafeB = Safe::Off>
  inline std::optional<se::field_t> getFieldInterp(const Eigen::Vector3f& point_M) const;

  template<Safe SafeB    = Safe::Off,
           Res ResTDummy = ResT
  >
  inline typename std::enable_if_t<ResTDummy == Res::Multi, std::optional<se::field_t>>
  getFieldInterp(const Eigen::Vector3f& point_M,
                 int&                   returned_scale) const;

  template<Safe SafeB = Safe::Off>
  inline std::optional<se::field_vec_t> getFieldGrad(const Eigen::Vector3f& point_M) const;

  void saveSlice(const std::string&     file_path,
                 const Eigen::Vector3f& point_M,
                 const std::string&     num = "") const;

  void saveStrucutre(const std::string& file_path,
                     const std::string& num = "") const;

  void saveMesh(const std::string& file_path,
                const std::string& num = "") const;

  inline void voxelToPoint(const Eigen::Vector3i& voxel_coord,
                           Eigen::Vector3f&       point_M) const;

  inline void voxelToPoint(const Eigen::Vector3i& voxel_coord,
                           const int              stride,
                           Eigen::Vector3f&       point_M) const;

  /**
   *
   * \note Use as `map. template pointToVoxel<se::Safe::Off>(...)
   *
   * \tparam SafeB
   * \param point_M
   * \param voxel_coord
   * \return
   */
  template<se::Safe SafeB = se::Safe::On>
  inline typename std::enable_if_t<SafeB == se::Safe::On, bool>
  pointToVoxel(const Eigen::Vector3f& point_M, Eigen::Vector3i& voxel_coord) const;

  template<se::Safe SafeB>
  inline typename std::enable_if_t<SafeB == se::Safe::Off, bool>
  pointToVoxel(const Eigen::Vector3f& point_M, Eigen::Vector3i& voxel_coord) const;

  template<se::Safe SafeB = se::Safe::On>
  inline typename std::enable_if_t<SafeB == se::Safe::On, bool>
  pointToVoxel(const Eigen::Vector3f& point_M, Eigen::Vector3f& voxel_coord_f) const;

  template<se::Safe SafeB>
  inline typename std::enable_if_t<SafeB == se::Safe::Off, bool>
  pointToVoxel(const Eigen::Vector3f& point_M, Eigen::Vector3f& voxel_coord_f) const;

  /**
   *
   * \note Use as `map. template pointsToVoxels<se::Safe::Off>(...)
   *
   * \tparam SafeB
   * \param points_M
   * \param voxel_coords
   * \return
   */
  template<se::Safe SafeB = se::Safe::On>
  inline typename std::enable_if_t<SafeB == se::Safe::On, bool>
  pointsToVoxels(const std::vector<Eigen::Vector3f>& points_M,
                 std::vector<Eigen::Vector3i>&       voxel_coords) const;

  template<se::Safe SafeB>
  inline typename std::enable_if_t<SafeB == se::Safe::Off, bool>
  pointsToVoxels(const std::vector<Eigen::Vector3f>& points_M,
                 std::vector<Eigen::Vector3i>&       voxel_coords) const;

  inline std::shared_ptr< OctreeType > getOctree() { return octree_ptr_; };

  inline std::shared_ptr< OctreeType > getOctree() const { return octree_ptr_; };

  void setOctree(std::shared_ptr< OctreeType > octree_ptr)
  {
//    delete octree_ptr_; // TODO: Delete old octree when setting a new one.
    octree_ptr_ = octree_ptr;
  };

  static constexpr Field     fld_ = FldT;
  static constexpr Colour    col_ = ColB;
  static constexpr Semantics sem_ = SemB;

  static constexpr Res       res_ = ResT;

protected:

  bool initialiseOctree();

  const Eigen::Vector3f dimension_;      ///< The dimensions of the map
  const float           resolution_;      ///< The resolution of the map
  const Eigen::Vector3f origin_M_; ///< The origin of the map frame

  const Eigen::Vector3f lb_;       ///< The lower map bound
  const Eigen::Vector3f ub_;       ///< The upper map bound

  std::shared_ptr< OctreeType >octree_ptr_ = nullptr;

  DataConfigType data_config_;
};

//// Full alias template for alternative setup
template <se::Field     FldT      = se::Field::TSDF,
          se::Colour    ColB      = se::Colour::Off,
          se::Semantics SemB      = se::Semantics::Off,
          se::Res       ResT      = se::Res::Single,
          int           BlockSize = 8
>
using MapD = Map<Data<FldT, ColB, SemB>, ResT, BlockSize> ;


// Occupancy map setups
template <se::Res ResT = se::Res::Single, int BlockSize = 8>
using OccMap = Map<OccData, ResT, BlockSize> ;

template <se::Res ResT = se::Res::Single, int BlockSize = 8>
using OccColMap = Map<OccColData, ResT, BlockSize> ;

template <se::Res ResT = se::Res::Single, int BlockSize = 8>
using OccSemMap = Map<OccSemData, ResT, BlockSize> ;

template <se::Res ResT = se::Res::Single, int BlockSize = 8>
using OccColSemMap = Map<OccColSemData, ResT, BlockSize> ;


// TSDF map setups
template <se::Res ResT = se::Res::Single, int BlockSize = 8>
using TSDFMap = Map<TSDFData, ResT, BlockSize> ;

template <se::Res ResT = se::Res::Single, int BlockSize = 8>
using TSDFColMap = Map<TSDFColData, ResT, BlockSize> ;

template <se::Res ResT = se::Res::Single, int BlockSize = 8>
using TSDFSemMap = Map<TSDFSemData, ResT, BlockSize> ;

template <se::Res ResT = se::Res::Single, int BlockSize = 8>
using TSDFColSemMap = Map<TSDFColSemData, ResT, BlockSize> ;

} // namespace se

#include "impl/map_impl.hpp"

#endif //SE_MAP_HPP
