/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2019-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_MAP_HPP
#define SE_MAP_HPP

#include "se/common/math_util.hpp"
#include "se/common/str_utils.hpp"
#include "se/map/algorithms/marching_cube.hpp"
#include "se/map/algorithms/structure_meshing.hpp"
#include "se/map/data.hpp"
#include "se/map/io/mesh_io.hpp"
#include "se/map/io/octree_io.hpp"
#include "se/map/octree/visitor.hpp"
#include "se/map/raycaster.hpp"
#include "se/map/utils/octant_util.hpp"



namespace se {

struct MapConfig {
    /** The dimensions of the map in metres.
     */
    Eigen::Vector3f dim;

    /** The resolution of map voxels in metres.
     */
    float res;

    /** The transformation from the world frame W to the map frame M.
     */
    Eigen::Matrix4f T_MW;

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
template<typename DataT = se::Data<Field::TSDF, Colour::Off, Semantics::Off>,
         Res ResT = Res::Single,
         int BlockSize = 8>
class Map;



template<Field FldT, Colour ColB, Semantics SemB, Res ResT, int BlockSize>
class Map<se::Data<FldT, ColB, SemB>, ResT, BlockSize> {
    public:
    typedef Data<FldT, ColB, SemB> DataType;
    typedef DataConfig<FldT, ColB, SemB> DataConfigType;
    typedef se::Octree<DataType, ResT, BlockSize> OctreeType;

    /**
     * \brief The map constructor.
     *
     * \note Default map origin is set at the centre of the map.
     *
     * \param[in] dim             The dimension of the map in [meter]
     * \param[in] res             The resolution of the map in [meter/voxel]
     * \param[in] data_config     The configuration file for the data
     */
    Map(const Eigen::Vector3f& dim,
        const float res,
        const se::DataConfig<FldT, ColB, SemB> data_config = se::DataConfig<FldT, ColB, SemB>());

    /**
     * \brief The map constructor.
     *
     * \param map_config          The configuration file for the map (e.g. map dimension, resolution and origin)
     * \param data_config         The configuration file for the data
     */
    Map(const MapConfig& map_config,
        const se::DataConfig<FldT, ColB, SemB> data_config = se::DataConfig<FldT, ColB, SemB>());

    /**
     * \brief Verify if a point is inside the map.
     *
     * \param[in] point_W         The point to be verified
     * \return True if the point is inside the map, false otherwise
     */
    inline bool contains(const Eigen::Vector3f& point_W) const;

    /**
     * \brief Get the transformation from world to map frame
     *
     * \return T_MW
     */
    inline Eigen::Matrix4f getTMW() const
    {
        return T_MW_;
    };

    /**
     * \brief Get the transformation from map to world frame
     *
     * \return T_WM
     */
    inline Eigen::Matrix4f getTWM() const
    {
        return T_WM_;
    };

    /**
     * \brief Get the translation from world to map frame
     *
     * \return t_MW
     */
    inline Eigen::Vector3f gettMW() const
    {
        return se::math::to_translation(T_MW_);
    }

    /**
     * \brief Get the translation from map to world frame
     *
     * \return t_WM
     */
    inline Eigen::Vector3f gettWM() const
    {
        return se::math::to_translation(T_WM_);
    }

    /**
     * \brief Get the rotation from world to map frame
     *
     * \return R_MW
     */
    inline Eigen::Matrix3f getRMW() const
    {
        return se::math::to_rotation(T_MW_);
    }

    /**
     * \brief Get the rotation from map to world frame
     *
     * \return R_WM
     */
    inline Eigen::Matrix3f getRWM() const
    {
        return se::math::to_rotation(T_WM_);
    }

    /**
     * \brief Get the dimensions of the map in [meter] (length x width x height)
     *
     * \return The dimensions of the map
     */
    inline Eigen::Vector3f getDim() const
    {
        return dimension_;
    }

    /**
     * \brief Get the resolution of the map in [meter/voxel]
     *
     * \return The resolution of the map
     */
    inline float getRes() const
    {
        return resolution_;
    }

    /**
     * \brief Get the data configuration of the map.
     *
     * \return The data configuration of the map
     */
    inline DataConfigType getDataConfig() const
    {
        return data_config_;
    }

    /**
     * \brief Get the stored data at the provided coordinates in [meter].
     *
     * \tparam SafeB          The parameter turning "contains point" verification on and off (Off by default)
     * \param[in]  point_W    The coordinates of the point in world frame [meter] to evaluate
     * \return The data at the provided coordinates
     */
    template<Safe SafeB = Safe::Off>
    inline const DataType getData(const Eigen::Vector3f& point_W) const;

    /**
     * \brief Get the stored max data at the provided coordinates in [meter] for a given scale.
     *
     * \tparam SafeB          The parameter turning "contains point" verification on and off (Off by default)
     * \tparam ResTDummy      The dummy parameter disabling the function off for single res and TSDF maps // TODO: Clean up with C++20 using required
     * \param point_W         The coordinates of the point in world frame [meter] to accessed
     * \param scale_desired   The scale to be accessed
     * \return The max data at the provided coordinates and scale
     */
    template<Safe SafeB = Safe::Off, Res ResTDummy = ResT>
    inline typename std::enable_if_t<ResTDummy == Res::Multi, DataType>
    getMaxData(const Eigen::Vector3f& point_W, const int scale_desired) const
    {
        Eigen::Vector3i voxel_coord;

        if constexpr (SafeB == Safe::Off) // Evaluate at compile time
        {
            pointToVoxel<Safe::Off>(point_W, voxel_coord);
        }
        else {
            if (!pointToVoxel<Safe::On>(point_W, voxel_coord)) {
                return DataType();
            }
        }

        return se::visitor::getMaxData(*octree_ptr_, voxel_coord, scale_desired);
    }


    /**
     * \brief Get the interpolated field value at the provided coordinates.
     *
     * \tparam SafeB          The parameter turning "contains point" verification on and off (Off by default)
     * \param[in] point_W     The coordinates of the point in world frame [meter] to accessed
     * \return                The interpolated field value at the coordinates
     */
    template<Safe SafeB = Safe::Off>
    inline std::optional<se::field_t> getFieldInterp(const Eigen::Vector3f& point_W) const;

    /**
     * \brief Get the interpolated field value at the provided coordinates and the scale it is stored at.
     *
     * \tparam SafeB              The parameter turning "contains point" verification on and off (Off by default)
     * \tparam ResTDummy          The dummy parameter disabling the function off for single res maps // TODO: Clean up with C++20 using required
     * \param[in] point_W         The coordinates of the point in world frame [meter] to accessed
     * \param[out] returned_scale The scale the data is stored at
     * \return                    The interpolated field value at the coordinates
     */
    template<Safe SafeB = Safe::Off, Res ResTDummy = ResT>
    inline typename std::enable_if_t<ResTDummy == Res::Multi, std::optional<se::field_t>>
    getFieldInterp(const Eigen::Vector3f& point_W, int& returned_scale) const;

    /**
     * \brief Get the field gradient at the provided coordinates.
     *
     * \tparam SafeB          The parameter turning "contains point" verification on and off (Off by default)
     * \param[in] point_W     The coordinates of the point in world frame [meter] to accessed
     * \return                The filed gradient at the coordinates
     */
    template<Safe SafeB = Safe::Off>
    inline std::optional<se::field_vec_t> getFieldGrad(const Eigen::Vector3f& point_W) const;

    /**
     * \brief Save three slices of the field value, each perpendicular to one of the axes (x, y and
     * z) at the provided coordinates. Setting any of the filenames to the empty string will skip
     * saving the respective slice.
     *
     * \note Only VTK (`.vtk`) files are currently supported.
     *
     * \param[in] filename_x The file where the slice perpendicular to the x axis will be written.
     * \param[in] filename_y The file where the slice perpendicular to the y axis will be written.
     * \param[in] filename_z The file where the slice perpendicular to the z axis will be written.
     * \param[in] point_W    The point in the world frame in units of meters where the slices
     *                       intersect. The x coordinate denotes the position along the x axis that
     *                       the slice perpendicular to the x axis will be computed etc.
     * \return Zero on success and non-zero on error.
     */
    int saveFieldSlices(const std::string& filename_x,
                        const std::string& filename_y,
                        const std::string& filename_z,
                        const Eigen::Vector3f& point_W) const;

    /**
     * \brief Save three slices of the maximum field value, each perpendicular to one of the axes
     * (x, y and z) at the provided coordinates. Setting any of the filenames to the empty string
     * will skip saving the respective slice.
     *
     * \note Only VTK (`.vtk`) files are currently supported.
     *
     * \param[in] filename_x The file where the slice perpendicular to the x axis will be written.
     * \param[in] filename_y The file where the slice perpendicular to the y axis will be written.
     * \param[in] filename_z The file where the slice perpendicular to the z axis will be written.
     * \param[in] point_W    The point in the world frame in units of meters where the slices
     *                       intersect. The x coordinate denotes the position along the x axis that
     *                       the slice perpendicular to the x axis will be computed etc.
     * \param[in] scale      The minimum scale the maximum field values will be extracted from.
     * \return Zero on success and non-zero on error.
     */
    template<se::Field FldTDummy = FldT>
    typename std::enable_if_t<FldTDummy == se::Field::Occupancy, int>
    saveMaxFieldSlices(const std::string& filename_x,
                       const std::string& filename_y,
                       const std::string& filename_z,
                       const Eigen::Vector3f& point_W,
                       const int scale) const;

    /**
     * \brief Save three slices of the integration scale, each perpendicular to one of the axes (x,
     * y and z) at the provided coordinates. Setting any of the filenames to the empty string will
     * skip saving the respective slice.
     *
     * \note Only VTK (`.vtk`) files are currently supported.
     *
     * \param[in] filename_x The file where the slice perpendicular to the x axis will be written.
     * \param[in] filename_y The file where the slice perpendicular to the y axis will be written.
     * \param[in] filename_z The file where the slice perpendicular to the z axis will be written.
     * \param[in] point_W    The point in the world frame in units of meters where the slices
     *                       intersect. The x coordinate denotes the position along the x axis that
     *                       the slice perpendicular to the x axis will be computed etc.
     * \return Zero on success and non-zero on error.
     */
    template<Res ResTDummy = ResT>
    typename std::enable_if_t<ResTDummy == Res::Multi, int>
    saveScaleSlices(const std::string& filename_x,
                    const std::string& filename_y,
                    const std::string& filename_z,
                    const Eigen::Vector3f& point_W) const;

    /**
     * \brief Save the octree structure to a file.
     *
     * \param[in] filename The file where the mesh will be saved. The file format will be selected
     *                     based on the file extension. Allowed file extensions are `.ply`, `.vtk` and
     *                     `.obj`.
     * \param[in] T_WM     Transformation from the world frame where the mesh is generated to the world
     *                     frame. Defaults to identity.
     * \return Zero on success and non-zero on error.
     */
    int saveStructure(const std::string& filename,
                      const Eigen::Matrix4f& T_WM = Eigen::Matrix4f::Identity()) const;

    /**
     * \brief Create a mesh in the world frame in units of metres and save it to a file.
     *
     * \param[in] filename The file where the mesh will be saved. The file format will be selected
     *                     based on the file extension. Allowed file extensions are `.ply`, `.vtk` and
     *                     `.obj`.
     * \param[in] T_OW     Transformation from the world frame in units of metres to the output frame. Defaults to identity.
     * \return Zero on success and non-zero on error.
     */
    int saveMesh(const std::string& filename,
                 const Eigen::Matrix4f& T_OW = Eigen::Matrix4f::Identity()) const;

    /**
     * \brief Create a mesh in the map frame in units of voxel and save it to a file.
     *
     * \param[in] filename The file where the mesh will be saved. The file format will be selected
     *                     based on the file extension. Allowed file extensions are `.ply`, `.vtk` and
     *                     `.obj`.
     * \return Zero on success and non-zero on error.
     */
    int saveMeshVoxel(const std::string& filename) const;

    /**
     * \brief Convert voxel coordinates in [voxel] to its centre point coordinates in [meter].
     *
     * \warning The function assumes the voxel has size 1 (i.e. scale 0).
     *
     * \param[in]  voxel_coord    The voxel coordinates in [voxel] to be converted
     * \param[out] point_W        The converted centre point coordinates in [meter]
     */
    inline void voxelToPoint(const Eigen::Vector3i& voxel_coord, Eigen::Vector3f& point_W) const;

    /**
     * \brief Convert voxel coordinates in [voxel] for a given voxel size to its centre point coordinates in [meter].
     *
     * \param[in]  voxel_coord    The voxel coordinates in [voxel] to be converted
     * \param[in]  voxel_size     The size of the voxel in [voxel]
     * \param[out] point_W        The converted centre point coordinates in [meter]
     */
    inline void voxelToPoint(const Eigen::Vector3i& voxel_coord,
                             const int voxel_size,
                             Eigen::Vector3f& point_W) const;

    /**
     * \brief Convert voxel coordinates in [voxel] for a given voxel size to its eight corner point coordinates in [meter].
     *
     * \warning The function assumes the voxel has size 1 (i.e. scale 0).
     *
     * \param[in]  voxel_coord        The voxel coordinates in [voxel] to be converted
     * \param[out] corner_points_W    The converted centre point coordinates in [meter]
     */
    inline void voxelToCornerPoints(const Eigen::Vector3i& voxel_coord,
                                    Eigen::Matrix<float, 3, 8>& corner_points_W) const;

    /**
     * \brief Convert voxel coordinates in [voxel] for a given voxel size to its eight corner point coordinates in [meter].
     *
     * \param[in]  voxel_coord        The voxel coordinates in [voxel] to be converted
     * \param[in]  voxel_size         The size of the voxel in [voxel]
     * \param[out] corner_points_W    The converted centre point coordinates in [meter]
     */
    inline void voxelToCornerPoints(const Eigen::Vector3i& voxel_coord,
                                    const int voxel_size,
                                    Eigen::Matrix<float, 3, 8>& corner_points_W) const;

    /**
     * \brief Convert point coordinates in [meter] to its voxel coordinates (bottom, front, left corner) in [voxel]
     *
     * \note Use as `map. template pointToVoxel<se::Safe::Off>(...)
     *
     * \tparam SafeB              The parameter turning "contains point" verification on and off (On by default)
     * \param[in]  point_W        The point coordinates in [meter] to be converted
     * \param[out] voxel_coord    The converted voxel coordinates (bottom, front, left corner) in [voxel]
     * \return True if the point inside the map, false otherwise
     */
    template<se::Safe SafeB = se::Safe::On>
    inline typename std::enable_if_t<SafeB == se::Safe::On, bool>
    pointToVoxel(const Eigen::Vector3f& point_W, Eigen::Vector3i& voxel_coord) const;

    /**
     * \brief Convert point coordinates in [meter] to its voxel coordinates (bottom, front, left corner) in [voxel]
     *
     * \tparam SafeB              The parameter turning "contains point" verification on and off (On by default)
     * \param[in]  point_W        The point coordinates in [meter] to be converted
     * \param[out] voxel_coord    The converted voxel coordinates (bottom, front, left corner) in [voxel]
     * \return True
     */
    template<se::Safe SafeB>
    inline typename std::enable_if_t<SafeB == se::Safe::Off, bool>
    pointToVoxel(const Eigen::Vector3f& point_W, Eigen::Vector3i& voxel_coord) const;

    /**
     * \brief Convert point coordinates in [meter] to its voxel coordinates in [voxel]
     *
     * \tparam SafeB              The parameter turning "contains point" verification on and off (On by default)
     * \param[in]  point_W        The point coordinates in [meter] to be converted
     * \param[out] voxel_coord_f  The converted voxel coordinates in [voxel]
     * \return True if the point inside the map, false otherwise
     */
    template<se::Safe SafeB = se::Safe::On>
    inline typename std::enable_if_t<SafeB == se::Safe::On, bool>
    pointToVoxel(const Eigen::Vector3f& point_W, Eigen::Vector3f& voxel_coord_f) const;

    /**
     * \brief Convert point coordinates in [meter] to its voxel coordinates in [voxel]
     *
     * \tparam SafeB              The parameter turning "contains point" verification on and off (On by default)
     * \param[in]  point_W        The point coordinates in [meter] to be converted
     * \param[out] voxel_coord_f  The converted voxel coordinates in [voxel]
     * \return True
     */
    template<se::Safe SafeB>
    inline typename std::enable_if_t<SafeB == se::Safe::Off, bool>
    pointToVoxel(const Eigen::Vector3f& point_W, Eigen::Vector3f& voxel_coord_f) const;

    /**
     * \brief Convert a vector of point coordinates in [meter] to its voxel coordinates in [voxel]
     *
     * \tparam SafeB              The parameter turning "contains point" verification on and off (On by default)
     * \param[in]  points_W       The vector of point coordinates in [meter] to be converted
     * \param[out] voxel_coords   The vector of converted voxel coordinates in [voxel]
     * \return True if all points are inside the map, false otherwise
     */
    template<se::Safe SafeB = se::Safe::On>
    inline typename std::enable_if_t<SafeB == se::Safe::On, bool>
    pointsToVoxels(const std::vector<Eigen::Vector3f>& points_W,
                   std::vector<Eigen::Vector3i>& voxel_coords) const;

    /**
     * \brief Convert a vector of point coordinates in [meter] to its voxel coordinates in [voxel]
     *
     * \tparam SafeB              The parameter turning "contains point" verification on and off (On by default)
     * \param[in]  points_W       The vector of point coordinates in [meter] to be converted
     * \param[out] voxel_coords   The vector of converted voxel coordinates in [voxel]
     * \return True
     */
    template<se::Safe SafeB>
    inline typename std::enable_if_t<SafeB == se::Safe::Off, bool>
    pointsToVoxels(const std::vector<Eigen::Vector3f>& points_W,
                   std::vector<Eigen::Vector3i>& voxel_coords) const;

    /**
     * \brief Get the shared pointer to the octree.
     *
     * \return The shared pointer to the octree
     */
    inline std::shared_ptr<OctreeType> getOctree()
    {
        return octree_ptr_;
    };

    /**
     * \brief Get the const shared pointer to the octree.
     *
     * \return The const shared pointer to the octree
     */
    inline std::shared_ptr<OctreeType> getOctree() const
    {
        return octree_ptr_;
    };

    /**
     * \brief Update the octree pointer to a new one.
     *
     * TODO: Requires to delete the old octree
     *
     * \param[in] octree_ptr  The shared pointer to the new octree to be assigned
     */
    void setOctree(std::shared_ptr<OctreeType> octree_ptr)
    {
        //    delete octree_ptr_; // TODO: Delete old octree when setting a new one.
        octree_ptr_ = octree_ptr;
    };

    static constexpr Field fld_ = FldT;
    static constexpr Colour col_ = ColB;
    static constexpr Semantics sem_ = SemB;

    static constexpr Res res_ = ResT;

    protected:
    /**
     * \brief Setup octree fitting the map dimension and resolution.
     *
     * \return True if no octree has been allocated yet, false otherwise
     */
    bool initialiseOctree();

    const Eigen::Vector3f dimension_; ///< The dimensions of the map
    const float resolution_;          ///< The resolution of the map
    const Eigen::Matrix4f T_MW_;      ///< The transformation from world to map frame
    const Eigen::Matrix4f T_WM_;      ///< The transformation from map to world frame

    const Eigen::Vector3f lb_M_; ///< The lower map bound
    const Eigen::Vector3f ub_M_; ///< The upper map bound

    std::shared_ptr<OctreeType> octree_ptr_ = nullptr;

    DataConfigType data_config_; ///< The configuration of the data

    Eigen::Matrix<float, 3, 8> corner_rel_steps_; ///< The eight relative unit corner offsets
};

//// Full alias template for alternative setup
template<se::Field FldT = se::Field::TSDF,
         se::Colour ColB = se::Colour::Off,
         se::Semantics SemB = se::Semantics::Off,
         se::Res ResT = se::Res::Single,
         int BlockSize = 8>
using MapD = Map<Data<FldT, ColB, SemB>, ResT, BlockSize>;


// Occupancy map setups
template<se::Res ResT = se::Res::Single, int BlockSize = 8>
using OccupancyMap = Map<OccupancyData, ResT, BlockSize>;

template<se::Res ResT = se::Res::Single, int BlockSize = 8>
using OccupancyColMap = Map<OccupancyColData, ResT, BlockSize>;

template<se::Res ResT = se::Res::Single, int BlockSize = 8>
using OccupancySemMap = Map<OccupancySemData, ResT, BlockSize>;

template<se::Res ResT = se::Res::Single, int BlockSize = 8>
using OccupancyColSemMap = Map<OccupancyColSemData, ResT, BlockSize>;


// TSDF map setups
template<se::Res ResT = se::Res::Single, int BlockSize = 8>
using TSDFMap = Map<TSDFData, ResT, BlockSize>;

template<se::Res ResT = se::Res::Single, int BlockSize = 8>
using TSDFColMap = Map<TSDFColData, ResT, BlockSize>;

template<se::Res ResT = se::Res::Single, int BlockSize = 8>
using TSDFSemMap = Map<TSDFSemData, ResT, BlockSize>;

template<se::Res ResT = se::Res::Single, int BlockSize = 8>
using TSDFColSemMap = Map<TSDFColSemData, ResT, BlockSize>;

} // namespace se

#include "impl/map_impl.hpp"

#endif // SE_MAP_HPP
