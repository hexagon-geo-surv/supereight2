#ifndef SE_OCTANT_IMPL_HPP
#define SE_OCTANT_IMPL_HPP

#include <iostream>

namespace se {

template <typename NodeT>
bool get_child_idx(const Eigen::Vector3i& voxel_coord,
                   NodeT*                 node_ptr,
                   unsigned int&          child_idx)
{
  const Eigen::Vector3i node_coord = node_ptr->getCoord();
  const int node_size     = node_ptr->getSize();
  if (voxel_coord.x() < node_coord.x() ||
      voxel_coord.y() < node_coord.y() ||
      voxel_coord.z() < node_coord.z() ||
      voxel_coord.x() >= node_coord.x() + node_size ||
      voxel_coord.y() >= node_coord.y() + node_size ||
      voxel_coord.z() >= node_coord.z() + node_size)
  {
    child_idx = 0;
    return false;
  }

  Eigen::Vector3i offset = voxel_coord - node_coord;
  const unsigned int child_size = node_size >> 1;

  child_idx = ((      offset.x() & child_size) > 0) +
                2 * ((offset.y() & child_size) > 0) +
                4 * ((offset.z() & child_size) > 0);

  return true;
}

/**
 * \brief Sets the Base for every node and block.
 */
OctantBase::OctantBase(const Eigen::Vector3i& coord,
                       const OctantBase*      parent_ptr)
        : parent_ptr_(parent_ptr), coord_(coord), time_stamp_(0), children_mask_(0)
    {
}



const Eigen::Vector3i& OctantBase::getCoord()
{
  return coord_;
}



bool OctantBase::getParent(const se::OctantBase*& parent_ptr)
{
  parent_ptr = parent_ptr_;
  return parent_ptr_ != nullptr;
}



const unsigned int OctantBase::getTimeStamp() { return time_stamp_; }



void OctantBase::setTimeStamp(const unsigned int time_stamp) { time_stamp_ = time_stamp; }



const unsigned char OctantBase::getChildrenMask() { return children_mask_; }



template <typename DerivedT>
NodeBase<DerivedT>::NodeBase(const Eigen::Vector3i&                coord,
                             const unsigned                        size,
                             const se::OctantBase* parent_ptr)
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
bool NodeBase<DerivedT>::getChild(const unsigned child_idx, se::OctantBase*& child_ptr)
{
  child_ptr = children_ptr_[child_idx];
  return child_ptr != nullptr;
}

template <typename DerivedT>
bool NodeBase<DerivedT>::setChild(const unsigned child_idx, se::OctantBase* child_ptr)
{
  if(children_ptr_[child_idx])
  {
    return false;
  }

  children_ptr_[child_idx] = child_ptr;

  children_mask_ |= 1 << child_idx;

  return true;
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
        : NodeBase<Node<DataT, ResT>>(parent_ptr->getCoord() + (parent_ptr->getSize() >> 1) *
                                      Eigen::Vector3i((1 & child_idx) > 0, (2 & child_idx) > 0, (4 & child_idx) > 0), (parent_ptr->getSize() >> 1),
                                      parent_ptr)
{
}


template <typename DataT, Res ResT>
bool Node<DataT, ResT>::setData(const DataT& data)
{
  data_ = data;
  return true;
}



template <typename DataT, Res ResT>
bool Node<DataT, ResT>::getData(const DataT& data)
{
  data = data_;
  return true;
}



template <typename DerivedT, unsigned SizeT>
BlockBase<DerivedT, SizeT>::BlockBase(const Eigen::Vector3i& coord,
                                      const se::OctantBase*  parent_ptr)
        : OctantBase(coord, parent_ptr)
{
}



template <typename DerivedT, typename DataT, unsigned SizeT>
bool BlockSingleRes<DerivedT, DataT, SizeT>::getData(const Eigen::Vector3i& voxel_coord,
                                                    DataType&              data)
{
  Eigen::Vector3i voxel_offset = voxel_coord - this->underlying().getCoord();
  data = block_data_[voxel_offset.x() +
                     voxel_offset.y() * this->underlying().size +
                     voxel_offset.z() * this->underlying().size_qu];
  return true;
}



template <typename DerivedT, typename DataT, unsigned SizeT>
bool BlockSingleRes<DerivedT, DataT, SizeT>::getData(const unsigned voxel_idx,
                                                     DataType&      data)
{
  data = block_data_[voxel_idx];
  return true;
}



template <typename DerivedT, typename DataT, unsigned SizeT>
bool BlockSingleRes<DerivedT, DataT, SizeT>::setData(const Eigen::Vector3i& voxel_coord,
                                                     const DataType&        data)
{
  Eigen::Vector3i voxel_offset = voxel_coord - this->underlying().getCoord();
  block_data_[voxel_offset.x() +
                     voxel_offset.y() * this->underlying().size +
                     voxel_offset.z() * this->underlying().size_qu] = data;
  return true;
}



template <typename DerivedT, typename DataT, unsigned SizeT>
bool BlockSingleRes<DerivedT, DataT, SizeT>::setData(const unsigned  voxel_idx,
                                                     const DataType& data)
{
  block_data_[voxel_idx] = data;
  return true;
}



template <typename DerivedT, typename DataT, unsigned SizeT>
BlockMultiRes<DerivedT, DataT, SizeT>::BlockMultiRes() {
  curr_scale_ = max_scale_;
  min_scale_  = max_scale_;
  block_data_.push_back(std::unique_ptr<DataT[]>(new DataType[1]));
}

template <typename DerivedT, typename DataT, unsigned SizeT>
bool BlockMultiRes<DerivedT, DataT, SizeT>::getData(const Eigen::Vector3i& voxel_coord,
                                                    DataType&              data)
{
  Eigen::Vector3i voxel_offset = voxel_coord - this->underlying().getCoord();
  voxel_offset = voxel_offset / (1 << curr_scale_);
  const int size_at_scale = this->underlying().size >> curr_scale_;
  data = block_data_[max_scale_ - curr_scale_][voxel_offset.x() +
                                               voxel_offset.y() * size_at_scale +
                                               voxel_offset.z() * size_at_scale * size_at_scale];
  return true;
}



template <typename DerivedT, typename DataT, unsigned SizeT>
bool BlockMultiRes<DerivedT, DataT, SizeT>::getData(const Eigen::Vector3i& voxel_coord,
                                                    const unsigned         scale,
                                                    DataType&              data)
{
  std::cout << "MultiRes::getData(vc, s, d)" << std::endl;
  return true;
}



template <typename DerivedT, typename DataT, unsigned SizeT>
bool BlockMultiRes<DerivedT, DataT, SizeT>::setData(const Eigen::Vector3i& voxel_coord,
                                                    const DataType&        data)
{
  std::cout << "MultiRes::setData(vc, d)" << std::endl;
  Eigen::Vector3i voxel_offset = voxel_coord - this->underlying().getCoord();
  voxel_offset = voxel_offset / (1 << curr_scale_);
  const int size_at_scale = this->underlying().size >> curr_scale_;
  block_data_[max_scale_ - curr_scale_][voxel_offset.x() +
                                        voxel_offset.y() * size_at_scale +
                                        voxel_offset.z() * size_at_scale * size_at_scale] = data;
  return true;
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
