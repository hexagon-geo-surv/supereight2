#ifndef SE_VISITOR_HPP
#define SE_VISITOR_HPP

/**
 * Helper wrapper to traverse the octree. All functions take a const octree and as no manipulation of the octree is done.
 */
namespace visitor {

/**
 * \brief Get the voxel data for a given coordinate.
 *        The function returns false and invalid data if the data is not allocated.
 *
 * \tparam OctreeT          The type of the octree used
 * \param[in]  octree_ptr   The pointer to the octree
 * \param[in]  voxel_coord  The voxel coordinates to be accessed
 * \param[out] data         The data in the voxel to be accessed
 * \return True if the data is available, False otherwise
 */
template <typename OctreeT>
bool getData(std::shared_ptr<OctreeT>     octree_ptr,
             const Eigen::Vector3i&       voxel_coord,
             typename OctreeT::DataType&  data);

template <typename OctreeT>
float getField(std::shared_ptr<OctreeT> octree_ptr,
               const int                x,
               const int                y,
               const int                z);

template <typename OctreeT, typename FieldT>
bool interpField(const std::shared_ptr<OctreeT> octree_ptr,
                 const Eigen::Vector3f&         voxel_coord_f,
                 FieldT&                        interp_field_value);

template <typename OctreeT>
bool gradField(const std::shared_ptr<OctreeT> octree_ptr,
               const Eigen::Vector3f&         voxel_coord_f,
               Eigen::Vector3f&               grad_field_value);

} // namespace visitor

#include "impl/visitor_impl.hpp"

#endif //SE_TRYOUT_VISITOR_HPP
