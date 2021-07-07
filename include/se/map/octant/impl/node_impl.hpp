#ifndef SE_NODE_IMPL_HPP
#define SE_NODE_IMPL_HPP

namespace se {



template <typename NodeT>
inline void get_child_idx(const Eigen::Vector3i& voxel_coord,
                          NodeT*                 node_ptr,
                          unsigned int&          child_idx)
{
  const Eigen::Vector3i node_coord = node_ptr->getCoord();
  const int             node_size  = node_ptr->getSize;

  assert(se::keyops::is_child(se::keyops::encode_key(node_coord, se::math::log2_const(node_size)),
                              se::keyops::encode_key(voxel_coord, 0)));

  Eigen::Vector3i offset = voxel_coord - node_coord;
  const unsigned int child_size = node_size >> 1;

  child_idx = ((      offset.x() & child_size) > 0) +
                2 * ((offset.y() & child_size) > 0) +
                4 * ((offset.z() & child_size) > 0);
}



template <typename DataT,
          Res      ResT
>
Node<DataT, ResT>::Node(const Eigen::Vector3i& coord,
                        const int              size) :
    OctantBase(false, coord, nullptr),
    std::conditional<ResT == Res::Single,
        NodeSingleRes<DataT>,
        NodeMultiRes<DataT,Node<DataT, ResT>>>::type(DataT()),
    size_(size)
{
  children_ptr_.fill(nullptr);
}



template <typename DataT,
          Res      ResT
>
Node<DataT, ResT>::Node(Node* parent_ptr,
                        const int child_idx) :
    OctantBase(false,
               parent_ptr->coord_ + (parent_ptr->size_ >> 1) * Eigen::Vector3i((1 & child_idx) > 0, (2 & child_idx) > 0, (4 & child_idx) > 0),
               parent_ptr),
    std::conditional<ResT == Res::Single,
        NodeSingleRes<DataT>,
        NodeMultiRes<DataT,Node<DataT, ResT>>>::type(parent_ptr->getData()),
    size_(parent_ptr->size_ >> 1)
{
  children_ptr_.fill(nullptr);
}



template <typename DataT,
          Res      ResT
>
inline int Node<DataT, ResT>::getSize() const
{
  return size_;
}



template <typename DataT,
        Res      ResT
>
inline se::OctantBase* Node<DataT, ResT>::getChild(const unsigned child_idx)
{
  return children_ptr_[child_idx];
}



template <typename DataT,
        Res      ResT
>
inline const se::OctantBase* Node<DataT, ResT>::getChild(const unsigned child_idx) const
{
  return children_ptr_[child_idx];
}



template <typename DataT,
        Res      ResT
>
inline se::OctantBase* Node<DataT, ResT>::setChild(const unsigned child_idx, se::OctantBase* child_ptr)
{
  children_mask_ |= 1 << child_idx;
  std::swap(child_ptr, children_ptr_[child_idx]);
  return child_ptr;
}



} // namespace se

#endif // SE_NODE_IMPL_HPP

