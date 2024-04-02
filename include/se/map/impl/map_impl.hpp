/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2019-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_MAP_IMPL_HPP
#define SE_MAP_IMPL_HPP

namespace se {

// clang-format off
template<Field FldT, Colour ColB, Semantics SemB, Res ResT, int BlockSize>
const Eigen::Matrix<float, 3, 8> Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::corner_rel_steps_ =
    (Eigen::Matrix<float, 3, 8>() << 0, 1, 0, 1, 0, 1, 0, 1,
                                     0, 0, 1, 1, 0, 0, 1, 1,
                                     0, 0, 0, 0, 1, 1, 1, 1).finished();
// clang-format on



template<Field FldT, Colour ColB, Semantics SemB, Res ResT, int BlockSize>
Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::Map(
    const Eigen::Vector3f& dim,
    const float res,
    const typename Data<FldT, ColB, SemB>::Config& data_config) :
        Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::Map(
            {dim, res /* T_MW uses default member initializer */},
            data_config)
{
}



template<Field FldT, Colour ColB, Semantics SemB, Res ResT, int BlockSize>
Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::Map(
    const Config& map_config,
    const typename Data<FldT, ColB, SemB>::Config& data_config) :
        octree_(std::ceil(map_config.dim.maxCoeff() / map_config.res)),
        resolution_(map_config.res),
        dimension_(Eigen::Vector3f::Constant(octree_.getSize() * resolution_)),
        T_MW_(map_config.T_MW),
        T_WM_(T_MW_.inverse()),
        lb_M_(Eigen::Vector3f::Zero()),
        ub_M_(dimension_),
        data_config_(data_config)
{
    const Eigen::Vector3f t_MW = T_MW_.translation();
    if (t_MW.x() < 0 || t_MW.x() >= dimension_.x() || t_MW.y() < 0 || t_MW.y() >= dimension_.y()
        || t_MW.z() < 0 || t_MW.z() >= dimension_.z()) {
        std::cout << "World origin is outside the map" << std::endl;
    }
}


template<Field FldT, Colour ColB, Semantics SemB, Res ResT, int BlockSize>
bool Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::contains(const Eigen::Vector3f& point_W) const
{
    const Eigen::Vector3f point_M = T_MW_ * point_W;
    return (point_M.x() >= lb_M_.x() && point_M.x() < ub_M_.x() && point_M.y() >= lb_M_.y()
            && point_M.y() < ub_M_.y() && point_M.z() >= lb_M_.z() && point_M.z() < ub_M_.z());
}



template<Field FldT, Colour ColB, Semantics SemB, Res ResT, int BlockSize>
template<Safe SafeB>
typename Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::DataType
Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::getData(const Eigen::Vector3f& point_W) const
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

    return se::visitor::getData(octree_, voxel_coord);
}



template<Field FldT, Colour ColB, Semantics SemB, Res ResT, int BlockSize>
template<Safe SafeB, Res ResTDummy>
std::enable_if_t<ResTDummy == Res::Multi,
                 typename Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::DataType>
Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::getMinData(const Eigen::Vector3f& point_W,
                                                         const int scale_desired) const
{
    Eigen::Vector3i voxel_coord;
    if constexpr (SafeB == Safe::Off) {
        pointToVoxel<Safe::Off>(point_W, voxel_coord);
    }
    else {
        if (!pointToVoxel<Safe::On>(point_W, voxel_coord)) {
            return DataType();
        }
    }
    return se::visitor::getMinData(octree_, voxel_coord, scale_desired);
}



template<Field FldT, Colour ColB, Semantics SemB, Res ResT, int BlockSize>
template<Safe SafeB>
std::optional<se::field_t>
Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::getFieldInterp(const Eigen::Vector3f& point_W) const
{
    Eigen::Vector3f voxel_coord_f;

    if constexpr (SafeB == Safe::Off) // Evaluate at compile time
    {
        pointToVoxel<Safe::Off>(point_W, voxel_coord_f);
    }
    else {
        if (!pointToVoxel<Safe::On>(point_W, voxel_coord_f)) {
            return std::nullopt;
        }
    }

    return se::visitor::getFieldInterp(octree_, voxel_coord_f);
}



template<Field FldT, Colour ColB, Semantics SemB, Res ResT, int BlockSize>
template<Safe SafeB, Res ResTDummy>
typename std::enable_if_t<ResTDummy == Res::Multi, std::optional<se::field_t>>
Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::getFieldInterp(const Eigen::Vector3f& point_W,
                                                             int& returned_scale) const
{
    Eigen::Vector3f voxel_coord_f;

    if constexpr (SafeB == Safe::Off) // Evaluate at compile time
    {
        pointToVoxel<Safe::Off>(point_W, voxel_coord_f);
    }
    else {
        if (!pointToVoxel<Safe::On>(point_W, voxel_coord_f)) {
            return std::nullopt;
        }
    }

    return se::visitor::getFieldInterp(octree_, voxel_coord_f, returned_scale);
}



template<Field FldT, Colour ColB, Semantics SemB, Res ResT, int BlockSize>
template<Safe SafeB>
std::optional<se::field_vec_t>
Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::getFieldGrad(const Eigen::Vector3f& point_W) const
{
    Eigen::Vector3f voxel_coord_f;

    if constexpr (SafeB == Safe::Off) // Evaluate at compile time
    {
        pointToVoxel<Safe::Off>(point_W, voxel_coord_f);
    }
    else {
        if (!pointToVoxel<Safe::On>(point_W, voxel_coord_f)) {
            return std::nullopt;
        }
    }

    if constexpr (ResT == Res::Multi) {
        int scale_returned;
        auto field_grad = se::visitor::getFieldGrad(octree_, voxel_coord_f, scale_returned);
        if (field_grad) {
            *field_grad /= resolution_ * (1 << scale_returned);
        }
        return field_grad;
    }
    else {
        auto field_grad = se::visitor::getFieldGrad(octree_, voxel_coord_f);
        if (field_grad) {
            *field_grad /= resolution_;
        }
        return field_grad;
    }
}



template<Field FldT, Colour ColB, Semantics SemB, Res ResT, int BlockSize>
int Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::saveFieldSlices(
    const std::string& filename_x,
    const std::string& filename_y,
    const std::string& filename_z,
    const Eigen::Vector3f& point_W) const
{
    Eigen::Vector3i voxel_coord;
    pointToVoxel(point_W, voxel_coord);

    auto get_field_value = [&](const Eigen::Vector3i& coord) {
        return se::get_field(se::visitor::getData(octree_, coord));
    };

    if (!filename_x.empty()) {
        se::io::save_3d_slice_vtk(
            filename_x,
            Eigen::Vector3i(voxel_coord.x(), 0, 0),
            Eigen::Vector3i(voxel_coord.x() + 1, octree_.getSize(), octree_.getSize()),
            get_field_value);
    }
    if (!filename_y.empty()) {
        se::io::save_3d_slice_vtk(
            filename_y,
            Eigen::Vector3i(0, voxel_coord.y(), 0),
            Eigen::Vector3i(octree_.getSize(), voxel_coord.y() + 1, octree_.getSize()),
            get_field_value);
    }
    if (!filename_z.empty()) {
        se::io::save_3d_slice_vtk(
            filename_z,
            Eigen::Vector3i(0, 0, voxel_coord.z()),
            Eigen::Vector3i(octree_.getSize(), octree_.getSize(), voxel_coord.z() + 1),
            get_field_value);
    }
    return 0;
}



template<Field FldT, Colour ColB, Semantics SemB, Res ResT, int BlockSize>
template<se::Field FldTDummy>
typename std::enable_if_t<FldTDummy == se::Field::Occupancy, int>
Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::saveMinFieldSlices(const std::string& filename_x,
                                                                 const std::string& filename_y,
                                                                 const std::string& filename_z,
                                                                 const Eigen::Vector3f& point_W,
                                                                 const int scale) const
{
    Eigen::Vector3i voxel_coord;
    pointToVoxel(point_W, voxel_coord);

    auto get_min_field_value = [&](const Eigen::Vector3i& coord) {
        return get_field(se::visitor::getMinData(octree_, coord, scale));
    };

    if (!filename_x.empty()) {
        se::io::save_3d_slice_vtk(
            filename_x,
            Eigen::Vector3i(voxel_coord.x(), 0, 0),
            Eigen::Vector3i(voxel_coord.x() + 1, octree_.getSize(), octree_.getSize()),
            get_min_field_value);
    }
    if (!filename_y.empty()) {
        se::io::save_3d_slice_vtk(
            filename_y,
            Eigen::Vector3i(0, voxel_coord.y(), 0),
            Eigen::Vector3i(octree_.getSize(), voxel_coord.y() + 1, octree_.getSize()),
            get_min_field_value);
    }
    if (!filename_z.empty()) {
        se::io::save_3d_slice_vtk(
            filename_z,
            Eigen::Vector3i(0, 0, voxel_coord.z()),
            Eigen::Vector3i(octree_.getSize(), octree_.getSize(), voxel_coord.z() + 1),
            get_min_field_value);
    }
    return 0;
}



template<Field FldT, Colour ColB, Semantics SemB, Res ResT, int BlockSize>
template<se::Field FldTDummy>
typename std::enable_if_t<FldTDummy == se::Field::Occupancy, int>
Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::saveMaxFieldSlices(const std::string& filename_x,
                                                                 const std::string& filename_y,
                                                                 const std::string& filename_z,
                                                                 const Eigen::Vector3f& point_W,
                                                                 const int scale) const
{
    Eigen::Vector3i voxel_coord;
    pointToVoxel(point_W, voxel_coord);

    auto get_max_field_value = [&](const Eigen::Vector3i& coord) {
        return get_field(se::visitor::getMaxData(octree_, coord, scale));
    };

    if (!filename_x.empty()) {
        se::io::save_3d_slice_vtk(
            filename_x,
            Eigen::Vector3i(voxel_coord.x(), 0, 0),
            Eigen::Vector3i(voxel_coord.x() + 1, octree_.getSize(), octree_.getSize()),
            get_max_field_value);
    }
    if (!filename_y.empty()) {
        se::io::save_3d_slice_vtk(
            filename_y,
            Eigen::Vector3i(0, voxel_coord.y(), 0),
            Eigen::Vector3i(octree_.getSize(), voxel_coord.y() + 1, octree_.getSize()),
            get_max_field_value);
    }
    if (!filename_z.empty()) {
        se::io::save_3d_slice_vtk(
            filename_z,
            Eigen::Vector3i(0, 0, voxel_coord.z()),
            Eigen::Vector3i(octree_.getSize(), octree_.getSize(), voxel_coord.z() + 1),
            get_max_field_value);
    }
    return 0;
}



template<Field FldT, Colour ColB, Semantics SemB, Res ResT, int BlockSize>
template<Res ResTDummy>
typename std::enable_if_t<ResTDummy == Res::Multi, int>
Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::saveScaleSlices(const std::string& filename_x,
                                                              const std::string& filename_y,
                                                              const std::string& filename_z,
                                                              const Eigen::Vector3f& point_W) const
{
    Eigen::Vector3i voxel_coord;
    pointToVoxel(point_W, voxel_coord);

    se::OctantBase* root_ptr = octree_.getRoot();
    const int max_scale = octree_.getMaxScale();

    auto get_scale = [&](const Eigen::Vector3i& coord) {
        const se::OctantBase* leaf_ptr = se::fetcher::template leaf<OctreeType>(coord, root_ptr);

        if (!leaf_ptr) {
            return max_scale;
        }
        else if (leaf_ptr->isBlock()) {
            const typename OctreeType::BlockType* block_ptr =
                static_cast<const typename OctreeType::BlockType*>(leaf_ptr);
            return block_ptr->getCurrentScale();
        }
        else {
            const typename OctreeType::NodeType* node_ptr =
                static_cast<const typename OctreeType::NodeType*>(leaf_ptr);
            return (node_ptr->isLeaf()) ? se::octantops::size_to_scale(node_ptr->getSize()) : -1;
        }
    };

    if (!filename_x.empty()) {
        se::io::save_3d_slice_vtk(
            filename_x,
            Eigen::Vector3i(voxel_coord.x(), 0, 0),
            Eigen::Vector3i(voxel_coord.x() + 1, octree_.getSize(), octree_.getSize()),
            get_scale);
    }
    if (!filename_y.empty()) {
        se::io::save_3d_slice_vtk(
            filename_y,
            Eigen::Vector3i(0, voxel_coord.y(), 0),
            Eigen::Vector3i(octree_.getSize(), voxel_coord.y() + 1, octree_.getSize()),
            get_scale);
    }
    if (!filename_z.empty()) {
        se::io::save_3d_slice_vtk(
            filename_z,
            Eigen::Vector3i(0, 0, voxel_coord.z()),
            Eigen::Vector3i(octree_.getSize(), octree_.getSize(), voxel_coord.z() + 1),
            get_scale);
    }
    return 0;
}

template<Field FldT, Colour ColB, Semantics SemB, Res ResT, int BlockSize>
int Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::saveStructure(const std::string& filename,
                                                                const Eigen::Isometry3f& T_WM) const
{
    const QuadMesh mesh = octree_structure_mesh(octree_);
    return io::save_mesh(mesh, filename, T_WM);
}



template<Field FldT, Colour ColB, Semantics SemB, Res ResT, int BlockSize>
int Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::saveMesh(const std::string& filename,
                                                           const Eigen::Isometry3f& T_OW) const
{
    se::TriangleMesh mesh;
    if constexpr (ResT == se::Res::Single) {
        se::algorithms::marching_cube(octree_, mesh);
    }
    else {
        se::algorithms::dual_marching_cube(octree_, mesh);
    }
    const Eigen::Affine3f T_OM = T_OW * T_WM_ * Eigen::Scaling(resolution_);
    return io::save_mesh(mesh, filename, T_OM);
}



template<Field FldT, Colour ColB, Semantics SemB, Res ResT, int BlockSize>
int Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::saveMeshVoxel(const std::string& filename) const
{
    se::TriangleMesh mesh;
    if constexpr (ResT == se::Res::Single) {
        se::algorithms::marching_cube(octree_, mesh);
    }
    else {
        se::algorithms::dual_marching_cube(octree_, mesh);
    }
    return io::save_mesh(mesh, filename);
}



template<Field FldT, Colour ColB, Semantics SemB, Res ResT, int BlockSize>
void Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::voxelToPoint(const Eigen::Vector3i& voxel_coord,
                                                                Eigen::Vector3f& point_W) const
{
    point_W = T_WM_ * ((voxel_coord.cast<float>() + sample_offset_frac) * resolution_);
}



template<Field FldT, Colour ColB, Semantics SemB, Res ResT, int BlockSize>
void Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::voxelToPoint(const Eigen::Vector3i& voxel_coord,
                                                                const int stride,
                                                                Eigen::Vector3f& point_W) const
{
    point_W = T_WM_ * ((voxel_coord.cast<float>() + stride * sample_offset_frac) * resolution_);
}



template<Field FldT, Colour ColB, Semantics SemB, Res ResT, int BlockSize>
void Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::voxelToCornerPoints(
    const Eigen::Vector3i& voxel_coord,
    Eigen::Matrix<float, 3, 8>& corner_points_W) const
{
    Eigen::Matrix<float, 3, 8> corner_points_M =
        (corner_rel_steps_.colwise() + voxel_coord.cast<float>()) * resolution_;
    corner_points_W = T_WM_ * corner_points_M;
}



template<Field FldT, Colour ColB, Semantics SemB, Res ResT, int BlockSize>
void Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::voxelToCornerPoints(
    const Eigen::Vector3i& voxel_coord,
    const int stride,
    Eigen::Matrix<float, 3, 8>& corner_points_W) const
{
    Eigen::Matrix<float, 3, 8> corner_points_M =
        ((stride * corner_rel_steps_).colwise() + voxel_coord.cast<float>()) * resolution_;
    corner_points_W = T_WM_ * corner_points_M;
}



template<Field FldT, Colour ColB, Semantics SemB, Res ResT, int BlockSize>
template<se::Safe SafeB>
typename std::enable_if_t<SafeB == se::Safe::On, bool>
Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::pointToVoxel(const Eigen::Vector3f& point_W,
                                                           Eigen::Vector3i& voxel_coord) const
{
    if (!contains(point_W)) {
        voxel_coord = Eigen::Vector3i::Constant(-1);
        return false;
    }
    voxel_coord = ((T_MW_ * point_W) / resolution_).template cast<int>();
    return true;
}



template<Field FldT, Colour ColB, Semantics SemB, Res ResT, int BlockSize>
template<se::Safe SafeB>
typename std::enable_if_t<SafeB == se::Safe::Off, bool>
Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::pointToVoxel(const Eigen::Vector3f& point_W,
                                                           Eigen::Vector3i& voxel_coord) const
{
    voxel_coord = ((T_MW_ * point_W) / resolution_).template cast<int>();
    return true;
}



template<Field FldT, Colour ColB, Semantics SemB, Res ResT, int BlockSize>
template<se::Safe SafeB>
typename std::enable_if_t<SafeB == se::Safe::On, bool>
Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::pointToVoxel(const Eigen::Vector3f& point_W,
                                                           Eigen::Vector3f& voxel_coord_f) const
{
    if (!contains(point_W)) {
        voxel_coord_f = Eigen::Vector3f::Constant(-1);
        return false;
    }
    voxel_coord_f = (T_MW_ * point_W) / resolution_;
    return true;
}



template<Field FldT, Colour ColB, Semantics SemB, Res ResT, int BlockSize>
template<se::Safe SafeB>
typename std::enable_if_t<SafeB == se::Safe::Off, bool>
Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::pointToVoxel(const Eigen::Vector3f& point_W,
                                                           Eigen::Vector3f& voxel_coord_f) const
{
    voxel_coord_f = (T_MW_ * point_W) / resolution_;
    return true;
}



template<Field FldT, Colour ColB, Semantics SemB, Res ResT, int BlockSize>
template<se::Safe SafeB>
typename std::enable_if_t<SafeB == se::Safe::On, bool>
Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::pointsToVoxels(
    const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& points_W,
    std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>>& voxel_coords) const
{
    bool all_valid = true;

    for (auto point_W : points_W) {
        Eigen::Vector3i voxel_coord;
        if (pointToVoxel<SafeB>(point_W, voxel_coord)) {
            voxel_coords.push_back(voxel_coord);
        }
        else {
            all_valid = false;
        }
    }
    return all_valid;
}



template<Field FldT, Colour ColB, Semantics SemB, Res ResT, int BlockSize>
template<se::Safe SafeB>
typename std::enable_if_t<SafeB == se::Safe::Off, bool>
Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::pointsToVoxels(
    const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& points_W,
    std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>>& voxel_coords) const
{
    for (auto point_W : points_W) {
        Eigen::Vector3i voxel_coord;
        pointToVoxel<SafeB>(point_W, voxel_coord);
        voxel_coords.push_back(voxel_coord);
    }
    return true;
}



template<Field FldT, Colour ColB, Semantics SemB, Res ResT, int BlockSize>
const Eigen::AlignedBox3f& Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::aabb() const
{
    const Eigen::AlignedBox3i& octree_aabb = octree_.aabb();
    // Ensure the AABBs are compared only if both aren't empty to avoid undefined behavior.
    const bool octree_aabbs_not_empty = !cached_octree_aabb_.isEmpty() && !octree_aabb.isEmpty();
    const bool octree_aabb_changed = (cached_octree_aabb_.isEmpty() != octree_aabb.isEmpty())
        || (octree_aabbs_not_empty && !cached_octree_aabb_.isApprox(octree_aabb));
    if (octree_aabb_changed) {
        cached_octree_aabb_ = octree_aabb;
        if (cached_octree_aabb_.isEmpty()) {
            cached_aabb_.setEmpty();
        }
        else {
            // Extend the the octree AABB so that it contains the whole volume and not just the
            // voxel coordinates. Then get the immediately smaller floats for the upper limit so
            // that no equality tests are performed with the vertices/edges/faces of the AABB that
            // are away from the origin. This is done because valid voxel coordinates must be
            // strictly smaller than the octree size.
            Eigen::AlignedBox3f a(cached_octree_aabb_.min().template cast<float>(),
                                  (cached_octree_aabb_.max().array() + 1).template cast<float>());
            for (int i = 0; i < a.dim(); i++) {
                a.max()[i] = std::nextafter(a.max()[i], a.min()[i]);
            }
            cached_aabb_ =
                eigen::transform(Eigen::Isometry3f(T_WM_) * Eigen::Scaling(resolution_), a);
        }
    }
    return cached_aabb_;
}



template<Field FldT, Colour ColB, Semantics SemB, Res ResT, int BlockSize>
void Map<Data<FldT, ColB, SemB>, ResT, BlockSize>::Config::readYaml(const std::string& yaml_file)
{
    // Open the file for reading.
    cv::FileStorage fs;
    try {
        if (!fs.open(yaml_file, cv::FileStorage::READ | cv::FileStorage::FORMAT_YAML)) {
            std::cerr << "Error: couldn't read configuration file " << yaml_file << "\n";
            return;
        }
    }
    catch (const cv::Exception& e) {
        // OpenCV throws if the file contains non-YAML data.
        std::cerr << "Error: invalid YAML in configuration file " << yaml_file << "\n";
        return;
    }

    // Get the node containing the map configuration.
    const cv::FileNode node = fs["map"];
    if (node.type() != cv::FileNode::MAP) {
        std::cerr << "Warning: using default map configuration, no \"map\" section found in "
                  << yaml_file << "\n";
        return;
    }

    // Read the config parameters.
    se::yaml::subnode_as_eigen_vector3f(node, "dim", dim);
    se::yaml::subnode_as_float(node, "res", res);

    // Don't show a warning if origin is not available, set it to dim / 2.
    T_MW = Eigen::Isometry3f(Eigen::Translation3f(dim / 2));

    if (!node["T_MW"].isNone()) {
        se::yaml::subnode_as_eigen_matrix4f(node, "T_MW", T_MW.matrix());
    }

    if (!node["t_MW"].isNone()) {
        Eigen::Vector3f t_MW;
        se::yaml::subnode_as_eigen_vector3f(node, "t_MW", t_MW);
        T_MW.translation() = t_MW;
    }

    if (!node["R_MW"].isNone()) {
        Eigen::Matrix3f R_MW;
        se::yaml::subnode_as_eigen_matrix3f(node, "R_MW", R_MW);
        T_MW.linear() = R_MW;
    }
}



template<typename MapT>
std::ostream& operator<<(std::ostream& os, const typename MapT::Config& c)
{
    os << str_utils::volume_to_pretty_str(c.dim, "dim") << " m\n";
    os << str_utils::value_to_pretty_str(c.res, "res") << " m/voxel\n";
    os << str_utils::eigen_matrix_to_pretty_str(c.T_MW.matrix(), "T_MW") << "\n";
    return os;
}

} // namespace se

#endif // SE_MAP_IMPL_HPP
