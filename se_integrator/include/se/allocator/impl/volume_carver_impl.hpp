#ifndef SE_VOXEL_CARVER_IMPL_HPP
#define SE_VOXEL_CARVER_IMPL_HPP



namespace se {



template<se::Colour    ColB,
         se::Semantics SemB
>
VolumeCarver<Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi>, PinholeCamera>::VolumeCarver(
           const se::Image<float>&                     depth_image,
           const se::DensePoolingImage<PinholeCamera>& depth_pooling_img,
           MapType&                                    map,
           const PinholeCamera&                        sensor,
           const Eigen::Matrix4f&                      T_SM,
           const int                                   frame,
           std::vector<se::OctantBase*>&               node_list,
           std::vector<se::OctantBase*>&               block_list,
           std::vector<bool>&                          low_variance_list,
           std::vector<bool>&                          projects_inside_list) :
  depth_image_(depth_image),
  depth_pooling_img_(depth_pooling_img),
  map_(map),
  octree_(*(map.getOctree())),
  sensor_(sensor),
  T_CM_(T_SM),
  frame_(frame),
  node_list_(node_list),
  block_list_(block_list),
  low_variance_list_(low_variance_list),
  projects_inside_list_(projects_inside_list),
  voxel_dim_(map.getRes()),
  max_depth_value_(std::min(sensor.far_plane, depth_pooling_img.maxValue() + map.getDataConfig().tau_max)),
  zero_depth_band_(1.0e-6f),
  size_to_radius_(std::sqrt(3.0f) / 2.0f)
{
}



template<se::Colour    ColB,
          se::Semantics SemB
  >
bool VolumeCarver<Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi>, PinholeCamera>::crossesFrustum(
        std::vector<srl::projection::ProjectionStatus>&  proj_corner_stati)
{
  for (int corner_idx = 0; corner_idx < 8; corner_idx++)
  {
    if (proj_corner_stati[corner_idx] == srl::projection::ProjectionStatus::Successful)
    {
      return true;
    }
  }
  return false;
}



template<se::Colour    ColB,
        se::Semantics SemB
>
bool VolumeCarver<Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi>, PinholeCamera>::cameraInNode(
        const Eigen::Vector3i& node_coord,
        const int              node_size,
        const Eigen::Matrix4f& T_MC)
{
  Eigen::Vector3f voxel_coord_f;
  map_.pointToVoxel(se::math::to_translation(T_MC), voxel_coord_f);
  if (   voxel_coord_f.x() >= node_coord.x() && voxel_coord_f.x() <= node_coord.x() + node_size
         && voxel_coord_f.y() >= node_coord.y() && voxel_coord_f.y() <= node_coord.y() + node_size
         && voxel_coord_f.z() >= node_coord.z() && voxel_coord_f.z() <= node_coord.z() + node_size)
  {
    return true;
  }
  return false;
}



template<se::Colour    ColB,
        se::Semantics SemB
>
void VolumeCarver<Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi>, PinholeCamera>::operator()(
        const Eigen::Vector3i& node_coord,
        const int              node_size,
        const int              depth,
        const Eigen::Vector3i& rel_step,
        se::OctantBase*        parent_ptr)
{
  /// Approximate max and min depth to quickly check if the node is behind the camera or maximum depth.
  // Compute the node centre's depth in the camera frame

  /// CHANGE: USE voxelToPoint(...)
  Eigen::Vector3f node_centre_point_M;
  map_.voxelToPoint(node_coord, node_size, node_centre_point_M);

  const Eigen::Vector3f node_centre_point_C = (T_CM_ * node_centre_point_M.homogeneous()).head(3);

  // Extend and reduce the depth by the sphere radius covering the entire cube
  const float approx_depth_value_max = node_centre_point_C.z() + node_size * size_to_radius_ * voxel_dim_;
  const float approx_depth_value_min = node_centre_point_C.z() - node_size * size_to_radius_ * voxel_dim_;

  /// CASE 0.1 (OUT OF BOUNDS): Block is behind the camera or behind the maximum depth value
  if (approx_depth_value_min > max_depth_value_ ||
      approx_depth_value_max < zero_depth_band_)
  { // TODO: Alternative sensor_.near_plane.
//    std::cout << "CASE 0.1 - Return 0" << std::endl;
    return;
  }


  // Compute the 8 corners of the node to be evaluated
  /// CHANGE: USE voxelToCornerPoints(...)
  Eigen::Matrix<float, 3, 8> node_corner_points_M;
  map_.voxelToCornerPoints(node_coord, node_size, node_corner_points_M);
  Eigen::Matrix<float, 3, 8> node_corner_points_C =
          (T_CM_  * node_corner_points_M.colwise().homogeneous()).topRows(3);

  Eigen::VectorXi node_corners_infront(8);
  node_corners_infront << 1, 1, 1, 1, 1, 1, 1, 1;
  for (int corner_idx = 0; corner_idx < 8; corner_idx++)
  {
    if (node_corner_points_C(2, corner_idx) < zero_depth_band_)
    {
      node_corners_infront(corner_idx) = 0;
    }
  }

  int num_node_corners_infront = node_corners_infront.sum();

  /// CASE 0.2 (OUT OF BOUNDS): Node is behind the camera.
  if (num_node_corners_infront == 0)
  {
//    std::cout << "CASE 0.2 - Return 1" << std::endl;
    return;
  }

  // Project the 8 corners into the image plane
  Eigen::Matrix2Xf proj_node_corner_pixels_f(2, 8);
  const Eigen::VectorXf node_corners_diff = node_corner_points_C.row(2);
  std::vector<srl::projection::ProjectionStatus> proj_node_corner_stati;
  sensor_.model.projectBatch(node_corner_points_C, &proj_node_corner_pixels_f, &proj_node_corner_stati);

  /// Approximate a 2D bounding box covering the projected node in the image plane.
  bool should_split = false;
  bool projects_inside = false;
  int low_variance = 0; ///<< -1 := low variance infront of the surface, 0 := high variance, 1 = low_variance behind the surface.
  se::Pixel pooling_pixel = se::Pixel::crossingUnknownPixel(); ///<< min, max pixel batch depth + crossing frustum state + contains unknown values state.

  if (depth < octree_.getBlockDepth() + 1)
  {
    if (num_node_corners_infront < 8)
    {
      /// CASE 1 (CAMERA IN NODE):
      if (cameraInNode(node_coord, node_size, se::math::to_inverse_transformation(T_CM_)))
      {
//        std::cout << "CASE 1 - Camera in Node - Split 0" << std::endl;
        should_split = true;
        /// CASE 2 (FRUSTUM BOUNDARY): Node partly behind the camera and crosses the the frustum boundary
      } else if (crossesFrustum(proj_node_corner_stati))
      {
//        std::cout << "CASE 2 - Frustum Boundary - Split 1" << std::endl;
        should_split = true;
        /// CASE 2 (FRUSTUM BOUNDARY): Node partly behind the camera and crosses the the frustum boundary without a corner reprojecting
      } else if (sensor_.sphereInFrustumInf(node_centre_point_C, node_size * size_to_radius_ * voxel_dim_))
      {
//        std::cout << "CASE 2 - Frustum Boundary - Split 2" << std::endl;
        should_split = true;
      } else
      {
//        std::cout << "Return 2" << std::endl;
        return;
      }

    } else
    {
      // Compute the minimum and maximum pixel values to generate the bounding box
      const Eigen::Vector2i image_bb_min = proj_node_corner_pixels_f.rowwise().minCoeff().cast<int>();
      const Eigen::Vector2i image_bb_max = proj_node_corner_pixels_f.rowwise().maxCoeff().cast<int>();
      const float node_dist_min_m = node_corners_diff.minCoeff();
      const float node_dist_max_m = node_corners_diff.maxCoeff();

      pooling_pixel = depth_pooling_img_.conservativeQuery(image_bb_min, image_bb_max);

      /// CASE 0.3 (OUT OF BOUNDS): The node is outside frustum (i.e left, right, below, above) or
      ///                           all pixel values are unknown -> return intermediately
      if(pooling_pixel.status_known == se::Pixel::statusKnown::unknown)
      {
//        std::cout << "Case 0.3 - Out of Bounds - Return 3" << std::endl;
        return;
      }

      /// CASE 0.4 (OUT OF BOUNDS): The node is behind surface
      if (node_dist_min_m > pooling_pixel.max + map_.getDataConfig().tau_max)
      { // TODO: Can be changed to node_dist_max_m?
//        std::cout << "Case 0.4 - Out of Bounds - Return 4" << std::endl;
        return;
      }

      low_variance = se::updater::lowVariance(
              pooling_pixel.min, pooling_pixel.max, node_dist_min_m, node_dist_max_m, map_.getDataConfig());

      const int node_scale = octree_.getMaxScale() - depth;
      const se::code_t node_code = se::keyops::encode_code(node_coord); // TODO: Might be correct now
      const unsigned int child_idx = se::keyops::code_to_child_idx(node_code, node_scale); // TODO: Might be correct now

      /// CASE 1 (REDUNDANT DATA): Depth values in the bounding box are far away from the node or unknown (1).
      ///                          The node to be evaluated is free (2) and fully observed (3),
      se::OctantBase* child_ptr = static_cast<NodeType*>(parent_ptr)->getChild(child_idx);
      if (low_variance != 0 && child_ptr)
      {
        typename OctreeType::DataType child_data = (child_ptr->isBlock()) ? static_cast<BlockType*>(child_ptr)->getMaxData() : static_cast<NodeType*>(child_ptr)->getData();

        if (   child_data.observed  // Check if the child is fully observed (i.e. all children are observed) // TODO: incooperate MAX occupancy
               && child_data.occupancy * child_data.weight <= 0.95 * map_.getDataConfig().min_occupancy)
        {
          return;
        }
      }

      // TODO: ^SWITCH 1 - Alternative approach (conservative)
      // Don't free node even more under given conditions.
//        if (   low_variance != 0
//            && parent->childData(child_idx).observed
//            && parent->childData(child_idx).x <= 0.95 * map_.getDataConfig().log_odd_min
//            && parent->childData(child_idx).y > map_.getDataConfig().max_weight / 2) {
//          return;
//        }

      /// CASE 2 (FRUSTUM BOUNDARY): The node is crossing the frustum boundary
      if(pooling_pixel.status_crossing == se::Pixel::statusCrossing::crossing)
      {
//        std::cout << "Case 2 - Frustum Bondary - Split 4" << std::endl;
        should_split = true;
      }

        /// CASE 3: The node is inside the frustum, but projects into partially known pixel
      else if (pooling_pixel.status_known == se::Pixel::statusKnown::part_known)
      {
//        std::cout << "Case 3 - Split 5" << std::endl;
        // TODO: SWITCH 1 - Alternative approach (conservative)
        // If the entire node is already free don't bother splitting it to free only parts of it even more because
        // of missing data and just move on.
        // Approach only saves time.
//          if (   low_variance != 0
//              && parent->childData(child_idx).observed
//              && parent->childData(child_idx).x <= 0.95 * map_.getDataConfig().log_odd_min
//              && parent->childData(child_idx).y > map_.getDataConfig().max_weight / 2) {
//            return;
//          }

        should_split = true;
      }

        /// CASE 4: The node is inside the frustum with only known data + node has a potential high variance
      else if (low_variance == 0)
      {
//        std::cout << "Case 4 - Split 5" << std::endl;
        should_split = true;
      }

      else
      {
//        std::cout << "DON'T SPLIT" << std::endl;
      }

      projects_inside = (pooling_pixel.status_known == se::Pixel::known);
    }
  } else
  {
//    std::cout << "TOO DEEP" << std::endl;
  }

  // Once we reach this point the node is either updated or split.
  // In either case the node needs to be allocated.
  const int octant_idx = rel_step.x() + rel_step.y() * 2 + rel_step.z() * 4;
  se::OctantBase* octant_ptr = octree_.allocate(static_cast<NodeType*>(parent_ptr), octant_idx);

  if (should_split)
  {
    // Returns a pointer to the according node if it has previously been allocated.
    if(octant_ptr->isBlock())
    { // Evaluate the node directly if it is a voxel block
      octant_ptr->setActive(true);
      // Cast from node to voxel block
#pragma omp critical (block_lock)
      { // Add voxel block to voxel block list for later update and up-propagation
        block_list_.push_back(octant_ptr);
        low_variance_list_.push_back((low_variance == -1));
        projects_inside_list_.push_back(projects_inside);
      }
    } else
    {
      // Split! Start recursive process
#pragma omp parallel for
      for (int child_idx = 0; child_idx < 8; ++child_idx)
      {
        int child_size = node_size / 2;
        const Eigen::Vector3i child_rel_step =
                Eigen::Vector3i((child_idx & 1) > 0, (child_idx & 2) > 0, (child_idx & 4) > 0);
        const Eigen::Vector3i child_coord = node_coord + child_rel_step * child_size;
        (*this)(child_coord, child_size, depth + 1, child_rel_step, octant_ptr);
      }
    }
  } else
  {
    assert(depth);

    if (octant_ptr->isBlock())
    {
#pragma omp critical (block_lock)
      { // Add node to node list for later up propagation (finest node for this branch)
        block_list_.push_back(octant_ptr);
        low_variance_list_.push_back((low_variance == -1));
        projects_inside_list_.push_back(projects_inside);
      }
    } else if (low_variance == -1)
    {
#pragma omp critical (node_lock)
      { // Add node to node list for later up propagation (finest node for this branch)
        node_list_.push_back(octant_ptr);
      }
    } // else node has low variance behind surface (ignore)

  }
}



} // namespace se



#endif //SE_VOXEL_CARVER_IMPL_HPP
