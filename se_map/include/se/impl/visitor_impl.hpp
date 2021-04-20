#ifndef SE_VISITOR_IMPL_HPP
#define SE_VISITOR_IMPL_HPP

namespace se {
namespace visitor {

template <typename OctreeT>
bool getData(std::shared_ptr<OctreeT>     octree_ptr,
             const Eigen::Vector3i&       voxel_coord,
             typename OctreeT::DataType&  data)
{

  se::OctantBase* octant_ptr = octree_ptr->getRoot();
  se::OctantBase* octant_tmp_ptr = nullptr;
  if(!octant_ptr) // Return false if the octree root isn't allocated, this should never be the case.
  {
    std::cerr << "Re-initalise octree. Root missing" << std::endl;
    return false;
  }

  unsigned int child_size = octree_ptr->getSize() >> 1;
  for (; child_size >= OctreeT::block_size; child_size = child_size >> 1)
  {
    typename OctreeT::NodeType* node_ptr = static_cast<typename OctreeT::NodeType*>(octant_ptr);

    unsigned int child_idx;
    get_child_idx(voxel_coord, node_ptr, child_idx);
    if (!node_ptr->getChild(child_idx, octant_tmp_ptr)) // Return false if the voxel is not allocated
    {
      return false;
    }
    octant_ptr = octant_tmp_ptr;
  }

  static_cast<typename OctreeT::BlockType*>(octant_ptr)->getData(voxel_coord, data);

  return true;
}

} // namespace visitor
} // namespace se

#endif // SE_VISITOR_IMPL_HPP
