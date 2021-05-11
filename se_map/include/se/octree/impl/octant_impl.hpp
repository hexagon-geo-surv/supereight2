#ifndef SE_OCTANT_IMPL_HPP
#define SE_OCTANT_IMPL_HPP

#include <iostream>

#include "se/utils/key_util.hpp"



namespace se {



template <typename NodeT>
void get_child_idx(const Eigen::Vector3i& voxel_coord,
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



template <typename DerivedT>
NodeBase<DerivedT>::NodeBase(const Eigen::Vector3i& coord,
                             const unsigned         size,
                             se::OctantBase*        parent_ptr)
        : OctantBase(coord, parent_ptr), size_(size)
{
  children_ptr_.fill(nullptr);
}




template <typename DerivedT>
unsigned int NodeBase<DerivedT>::getSize()
{
  return size_;
}



template <typename DerivedT>
inline se::OctantBase* NodeBase<DerivedT>::getChild(const unsigned child_idx)
{
  return children_ptr_[child_idx];
}



template <typename DerivedT>
inline se::OctantBase* NodeBase<DerivedT>::setChild(const unsigned child_idx, se::OctantBase* child_ptr)
{
  children_mask_ |= 1 << child_idx;
  std::swap(child_ptr, children_ptr_[child_idx]);
  return child_ptr;
}



template <typename DataT, Res ResT>
Node<DataT, ResT>::Node(const Eigen::Vector3i&                coord,
                        const unsigned                        size)
        : NodeBase<Node<DataT, ResT>>(coord, size, nullptr)
{
}



template <typename DataT, Res ResT>
Node<DataT, ResT>::Node(Node*              parent_ptr,
                        const unsigned int child_idx)
        : NodeBase<Node<DataT, ResT>>(parent_ptr->coord_ + (parent_ptr->size_ >> 1) *
                                      Eigen::Vector3i((1 & child_idx) > 0, (2 & child_idx) > 0, (4 & child_idx) > 0), (parent_ptr->size_ >> 1),
                                      parent_ptr)
{
}



template <typename DataT, Res ResT>
void Node<DataT, ResT>::setData(const DataT& data)
{
  data_ = data;
}



template <typename DataT, Res ResT>
void Node<DataT, ResT>::getData(const DataT& data)
{
  data = data_;
}



template <typename DerivedT, unsigned SizeT>
BlockBase<DerivedT, SizeT>::BlockBase(const Eigen::Vector3i& coord,
                                      se::OctantBase*        parent_ptr)
        : OctantBase(coord, parent_ptr)
{
}



template <typename DerivedT, typename DataT, unsigned SizeT>
void BlockSingleRes<DerivedT, DataT, SizeT>::getData(const Eigen::Vector3i& voxel_coord,
                                                     DataType&              data)
{
  Eigen::Vector3i voxel_offset = voxel_coord - this->underlying().coord_;
  data = block_data_[voxel_offset.x() +
                     voxel_offset.y() * this->underlying().size +
                     voxel_offset.z() * this->underlying().size_qu];
}



template <typename DerivedT, typename DataT, unsigned SizeT>
void BlockSingleRes<DerivedT, DataT, SizeT>::getData(const Eigen::Vector3i& voxel_coord,
                                                     DataType&              data) const
{
  Eigen::Vector3i voxel_offset = voxel_coord - this->underlying().coord_;
  data = block_data_[voxel_offset.x() +
                     voxel_offset.y() * this->underlying().size +
                     voxel_offset.z() * this->underlying().size_qu];
}



template <typename DerivedT, typename DataT, unsigned SizeT>
void BlockSingleRes<DerivedT, DataT, SizeT>::getData(const unsigned voxel_idx,
                                                     DataType&      data)
{
  data = block_data_[voxel_idx];
}



template <typename DerivedT, typename DataT, unsigned SizeT>
void BlockSingleRes<DerivedT, DataT, SizeT>::getData(const unsigned voxel_idx,
                                                     DataType&      data) const
{
  data = block_data_[voxel_idx];
}



template <typename DerivedT, typename DataT, unsigned SizeT>
void BlockSingleRes<DerivedT, DataT, SizeT>::setData(const Eigen::Vector3i& voxel_coord,
                                                     const DataType&        data)
{
  Eigen::Vector3i voxel_offset = voxel_coord - this->underlying().coord_;
  block_data_[voxel_offset.x() +
              voxel_offset.y() * this->underlying().size +
              voxel_offset.z() * this->underlying().size_qu] = data;
}



template <typename DerivedT, typename DataT, unsigned SizeT>
void BlockSingleRes<DerivedT, DataT, SizeT>::setData(const unsigned  voxel_idx,
                                                     const DataType& data)
{
  block_data_[voxel_idx] = data;
}



template <typename DerivedT, typename DataT, unsigned SizeT>
BlockMultiRes<DerivedT, DataT, SizeT>::BlockMultiRes() {
  curr_scale_ = max_scale_;
  min_scale_  = max_scale_;
  block_data_.push_back(std::unique_ptr<DataT[]>(new DataType[1]));
}



template <typename DerivedT, typename DataT, unsigned SizeT>
void BlockMultiRes<DerivedT, DataT, SizeT>::getData(const Eigen::Vector3i& voxel_coord,
                                                    DataType&              data)
{
  Eigen::Vector3i voxel_offset = voxel_coord - this->underlying().coord_;
  voxel_offset = voxel_offset / (1 << curr_scale_);
  const int size_at_scale = this->underlying().size >> curr_scale_;
  data = block_data_[max_scale_ - curr_scale_][voxel_offset.x() +
                                               voxel_offset.y() * size_at_scale +
                                               voxel_offset.z() * size_at_scale * size_at_scale];
}



template <typename DerivedT, typename DataT, unsigned SizeT>
void BlockMultiRes<DerivedT, DataT, SizeT>::getData(const Eigen::Vector3i& voxel_coord,
                                                    const unsigned         scale,
                                                    DataType&              data)
{
  std::cout << "MultiRes::getData(vc, s, d)" << std::endl;
}



template <typename DerivedT, typename DataT, unsigned SizeT>
void BlockMultiRes<DerivedT, DataT, SizeT>::setData(const Eigen::Vector3i& voxel_coord,
                                                    const DataType&        data)
{
  std::cout << "MultiRes::setData(vc, d)" << std::endl;
  Eigen::Vector3i voxel_offset = voxel_coord - this->underlying().getCoord();
  voxel_offset = voxel_offset / (1 << curr_scale_);
  const int size_at_scale = this->underlying().size >> curr_scale_;
  block_data_[max_scale_ - curr_scale_][voxel_offset.x() +
                                        voxel_offset.y() * size_at_scale +
                                        voxel_offset.z() * size_at_scale * size_at_scale] = data;
}



template <typename DataT, Res ResT, unsigned SizeT, typename PolicyT>
Block<DataT, ResT, SizeT, PolicyT>::Block(se::Node<DataT, ResT>* parent_ptr,
                                          const unsigned         child_id)
        : BlockBase<Block<DataT, ResT, SizeT>, SizeT>(
                parent_ptr->getCoord() + SizeT * Eigen::Vector3i((1 & child_id) > 0, (2 & child_id) > 0, (4 & child_id) > 0),
                parent_ptr)
{
  assert(SizeT == (parent_ptr->getSize() >> 1));
}

} // namespace se

#endif // SE_OCTANT_IMPL_HPP
