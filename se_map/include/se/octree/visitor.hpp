#ifndef SE_VISITOR_HPP
#define SE_VISITOR_HPP

/**
 * Helper wrapper to traverse the octree. All functions take a const octree and as no manipulation of the octree is done.
 */
namespace se {
namespace visitor {

/**
 * \brief Get the voxel data for a given coordinate.
 *        The function returns false and invalid data if the data is not allocated.
 *
 * \tparam OctreeT          The type of the octree used
 * \param[in]  octree_ptr   The pointer to the octree
 * \param[in]  voxel_coord  The voxel coordinates to be accessed
 * \param[out] data         The data in the voxel to be accessed
 *
 * \return True if the data is available, False otherwise
 */
template <typename OctreeT>
inline bool getData(const OctreeT&              octree,
                    const Eigen::Vector3i&      voxel_coord,
                    typename OctreeT::DataType& data);

template <typename OctreeT, typename BlockT>
inline bool getData(const OctreeT&              octree,
                    BlockT*                     block_ptr,
                    const Eigen::Vector3i&      voxel_coord,
                    typename OctreeT::DataType& data);

/**
 * \brief Get the voxel data for a given coordinate.
 *
 * \warning The data might be invalid.
 *
 * \tparam OctreeT         The type of the octree used
 * \param[in] octree_ptr   The pointer to the octree
 * \param[in] voxel_coord  The voxel coordinates to be accessed
 *
 * \return The data in the voxel to be accessed
 */
template <typename OctreeT>
inline typename OctreeT::DataType getData(const OctreeT&         octree,
                                          const Eigen::Vector3i& voxel_coord);

template <typename OctreeT, typename BlockT>
inline typename OctreeT::DataType getData(const OctreeT&         octree,
                                          BlockT*                block_ptr,
                                          const Eigen::Vector3i& voxel_coord);

/**
 * \brief Get the field value for a given coordinate.
 *        The function returns false and invalid data if the data is not allocated.
 *
 * \tparam OctreeT         The type of the octree used
 * \param[in] octree_ptr   The pointer to the octree
 * \param[in] voxel_coord  The voxel coordinates to be accessed
 * \param[in] field_value  The field value to be accessed
 *
 * \return True if the field value is available, False otherwise
 */
template <typename OctreeT>
inline bool getField(const OctreeT&         octree,
                     const Eigen::Vector3i& voxel_coord,
                     se::field_t&           field_value);

template <typename OctreeT, typename BlockT>
inline bool getField(const OctreeT&         octree,
                     BlockT*                block_ptr,
                     const Eigen::Vector3i& voxel_coord,
                     se::field_t&           field_value);

/**
 * \brief Get the field value for a given coordinate.
 *        The function returns false and invalid data if the data is not allocated.
 *
 * \tparam OctreeT         The type of the octree used
 * \param[in] octree_ptr   The pointer to the octree
 * \param[in] voxel_coord  The voxel coordinates to be accessed
 *
 * \return The field value to be accessed
 */
template <typename OctreeT>
inline se::field_t getField(const OctreeT&         octree,
                            const Eigen::Vector3i& voxel_coord);

template <typename OctreeT, typename BlockT>
inline se::field_t getField(const OctreeT&         octree,
                            BlockT*                block_ptr,
                            const Eigen::Vector3i& voxel_coord);

/**
 * \brief Get the interplated field value for a given coordinate [float voxel coordinates].
 *        The function returns false and invalid data if the data is not allocated.
 *
 * \tparam OctreeT                The type of the octree used
 * \param[in] octree_ptr          The pointer to the octree
 * \param[in] voxel_coord_f       The voxel coordinates to be accessed [float voxel coordiantes]
 * \param[in] interp_field_value  The interplated field value to be accessed
 *
 * \return True if the field value is available, False otherwise
 */
template <typename OctreeT, typename FieldT>
inline bool interpField(const OctreeT&         octree,
                        const Eigen::Vector3f& voxel_coord_f,
                        FieldT&                interp_field_value);

/**
 * \brief Get the field gradient for a given coordinate [float voxel coordinates].
 *
 * \warning The function only returns false if the base block is not allocated and might
 *          compute the gradient from invalid data. TODO
 *
 * \tparam OctreeT              The type of the octree used
 * \param[in] octree_ptr        The pointer to the octree
 * \param[in] voxel_coord_f     The voxel coordinates to be accessed [float voxel coordiantes]
 * \param[in] grad_field_value  The field gradient to be accessed
 *
 * \return True if base block pointer is allocated, False otherwise
 */
template <typename OctreeT>
inline bool gradField(const OctreeT&         octree_ptr,
                      const Eigen::Vector3f& voxel_coord_f,
                      Eigen::Vector3f&       grad_field_value);

/**
 * \brief Get the field gradient for a given coordinate [float voxel coordinates].
 *
 * \warning The function only returns false if the base block is not allocated and might
 *          compute the gradient from invalid data. TODO
 *
 * \tparam OctreeT              The type of the octree used
 * \param[in] octree_ptr        The pointer to the octree
 * \param[in] voxel_coord_f     The voxel coordinates to be accessed [float voxel coordiantes]
 * \param[in] grad_field_value  The field gradient to be accessed
 *
 * \return True if base block pointer is allocated, False otherwise
 */
template <typename OctreeT>
inline Eigen::Vector3f gradField(const OctreeT&         octree_ptr,
                                 const Eigen::Vector3f& voxel_coord_f);

} // namespace visitor
} // namespace se

#include "impl/visitor_impl.hpp"

#endif //SE_TRYOUT_VISITOR_HPP
