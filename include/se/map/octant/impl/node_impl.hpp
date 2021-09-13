#ifndef SE_NODE_IMPL_HPP
#define SE_NODE_IMPL_HPP

namespace se {



template <typename NodeT>
inline int get_child_idx(const Eigen::Vector3i& child_coord,
                         NodeT*                 parent_ptr)
{
  const Eigen::Vector3i parent_coord = parent_ptr->getCoord();
  const int             parent_size  = parent_ptr->getSize;

  assert(se::keyops::is_child(se::keyops::encode_key(parent_coord, se::math::log2_const(parent_size)),
                              se::keyops::encode_key(child_coord, 0)));

  Eigen::Vector3i offset = child_coord - parent_coord;
  const unsigned int child_size = parent_size >> 1;

  return ((      offset.x() & child_size) > 0) +
           2 * ((offset.y() & child_size) > 0) +
           4 * ((offset.z() & child_size) > 0);
}



template <typename DataT,
          Res      ResT
>
Node<DataT, ResT>::Node(const Eigen::Vector3i& coord,
                        const int              size,
                        const DataT            init_data) :
    OctantBase(false, coord, nullptr),
    std::conditional<ResT == Res::Single,
        NodeSingleRes<DataT>,
        NodeMultiRes<DataT,Node<DataT, ResT>>>::type(init_data),
    size_(size)
{
  children_ptr_.fill(nullptr);
}



template <typename DataT,
          Res      ResT
>
Node<DataT, ResT>::Node(Node*       parent_ptr,
                        const int   child_idx,
                        const DataT init_data) :
    OctantBase(false,
               parent_ptr->coord_ + (parent_ptr->size_ >> 1) * Eigen::Vector3i((1 & child_idx) > 0, (2 & child_idx) > 0, (4 & child_idx) > 0),
               parent_ptr),
    std::conditional<ResT == Res::Single,
        NodeSingleRes<DataT>,
        NodeMultiRes<DataT,Node<DataT, ResT>>>::type(init_data),
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
inline se::OctantBase* Node<DataT, ResT>::getChild(const int child_idx)
{
  return children_ptr_[child_idx];
}



template <typename DataT,
        Res      ResT
>
inline const se::OctantBase* Node<DataT, ResT>::getChild(const int child_idx) const
{
  return children_ptr_[child_idx];
}



template <typename DataT,
        Res      ResT
>
inline se::OctantBase* Node<DataT, ResT>::setChild(const int       child_idx,
                                                   se::OctantBase* child_ptr)
{
  children_mask_ |= 1 << child_idx;
  std::swap(child_ptr, children_ptr_[child_idx]);
  return child_ptr;
}



} // namespace se

#endif // SE_NODE_IMPL_HPP

