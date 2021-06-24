#ifndef SE_OCTANT_IMPL_HPP
#define SE_OCTANT_IMPL_HPP

#include <iostream>

#include "se/utils/key_util.hpp"



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
Node<DataT, ResT>::Node(const Eigen::Vector3i&                coord,
                        const unsigned                        size)
        : OctantBase(false, coord, nullptr), size_(size)
{
  children_ptr_.fill(nullptr);
}



template <typename DataT,
          Res      ResT
>
Node<DataT, ResT>::Node(Node*              parent_ptr,
                        const unsigned int child_idx)
    : OctantBase(false,
                 parent_ptr->coord_ + (parent_ptr->size_ >> 1) * Eigen::Vector3i((1 & child_idx) > 0, (2 & child_idx) > 0, (4 & child_idx) > 0),
                 parent_ptr), size_(parent_ptr->size_ >> 1)
{
  children_ptr_.fill(nullptr);
}



template <typename DataT,
          Res      ResT
>
inline unsigned int Node<DataT, ResT>::getSize() const
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



/// Single-res Block ///

template <typename DerivedT,
          typename DataT,
          int      BlockSize
>
BlockSingleRes<DerivedT, DataT, BlockSize>::BlockSingleRes(const DataType init_data)
{
  block_data_.fill(init_data); // TODO: Verify that initialisation doesn't cause regression
}


template <typename DerivedT,
          typename DataT,
          int      BlockSize
>
inline const typename BlockSingleRes<DerivedT, DataT, BlockSize>::DataType& BlockSingleRes<DerivedT, DataT, BlockSize>::getData(const int voxel_idx) const
{
  return block_data_[voxel_idx];
}



template <typename DerivedT,
          typename DataT,
          int      BlockSize
>
inline typename BlockSingleRes<DerivedT, DataT, BlockSize>::DataType& BlockSingleRes<DerivedT, DataT, BlockSize>::getData(const int voxel_idx)
{
  return block_data_[voxel_idx];
}



template <typename DerivedT,
          typename DataT,
          int      BlockSize
>
inline const typename BlockSingleRes<DerivedT, DataT, BlockSize>::DataType& BlockSingleRes<DerivedT, DataT, BlockSize>::getData(const Eigen::Vector3i& voxel_coord) const
{
  Eigen::Vector3i voxel_offset = voxel_coord - this->underlying().coord_;
  return block_data_[voxel_offset.x() +
                     voxel_offset.y() * this->underlying().size +
                     voxel_offset.z() * this->underlying().size_qu];
}



template <typename DerivedT,
          typename DataT,
          int      BlockSize
>
inline typename BlockSingleRes<DerivedT, DataT, BlockSize>::DataType& BlockSingleRes<DerivedT, DataT, BlockSize>::getData(const Eigen::Vector3i& voxel_coord)
{
  Eigen::Vector3i voxel_offset = voxel_coord - this->underlying().coord_;
  return block_data_[voxel_offset.x() +
                     voxel_offset.y() * this->underlying().size +
                     voxel_offset.z() * this->underlying().size_qu];
}



template <typename DerivedT,
          typename DataT,
          int      BlockSize
>
inline void BlockSingleRes<DerivedT, DataT, BlockSize>::setData(const Eigen::Vector3i& voxel_coord,
                                                                const DataType&        data)
{
  Eigen::Vector3i voxel_offset = voxel_coord - this->underlying().coord_;
  block_data_[voxel_offset.x() +
              voxel_offset.y() * this->underlying().size +
              voxel_offset.z() * this->underlying().size_qu] = data;
}



template <typename DerivedT,
          typename DataT,
          int      BlockSize
>
inline void BlockSingleRes<DerivedT, DataT, BlockSize>::setData(const unsigned  voxel_idx,
                                                                const DataType& data)
{
  block_data_[voxel_idx] = data;
}



/// Multi-res Block ///

template <Colour    ColB,
          Semantics SemB,
          int       BlockSize,
          typename  DerivedT
>
BlockMultiRes<se::Data<se::Field::TSDF, ColB, SemB>, BlockSize, DerivedT>::BlockMultiRes(const DataType init_data) : min_scale_(-1), curr_scale_(-1)
{
  block_data_.fill(init_data); // TODO: Verify that initialisation doesn't cause regression
}



template <Colour    ColB,
          Semantics SemB,
          int       BlockSize,
          typename  DerivedT
>
inline int BlockMultiRes<se::Data<se::Field::TSDF, ColB, SemB>, BlockSize, DerivedT>::getVoxelIdx(const Eigen::Vector3i& voxel_coord) const
{
  const Eigen::Vector3i voxel_offset = (voxel_coord - this->underlying().coord_) / (1 << curr_scale_);
  const int size_at_scale = size_at_scales_[curr_scale_];
  return scale_offsets_[curr_scale_] + voxel_offset.x() +
                                       voxel_offset.y() * size_at_scale +
                                       voxel_offset.z() * se::math::sq(size_at_scale);
}



template <Colour    ColB,
          Semantics SemB,
          int       BlockSize,
          typename  DerivedT
>
inline int BlockMultiRes<se::Data<se::Field::TSDF, ColB, SemB>, BlockSize, DerivedT>::getVoxelIdx(const Eigen::Vector3i& voxel_coord,
                                                                                                  const int              scale) const
{
  const Eigen::Vector3i voxel_offset = (voxel_coord - this->underlying().coord_) / (1 << scale);
  const int size_at_scale = size_at_scales_[scale];
  return scale_offsets_[scale] + voxel_offset.x() +
                                 voxel_offset.y() * size_at_scale +
                                 voxel_offset.z() * se::math::sq(size_at_scale);
}



template <Colour    ColB,
          Semantics SemB,
          int       BlockSize,
          typename  DerivedT
>
inline const typename BlockMultiRes<se::Data<se::Field::TSDF, ColB, SemB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<se::Data<se::Field::TSDF, ColB, SemB>, BlockSize, DerivedT>::getData(const int voxel_idx) const
{
  return block_data_[voxel_idx];
}



template <Colour    ColB,
        Semantics SemB,
        int       BlockSize,
        typename  DerivedT
>
inline typename BlockMultiRes<se::Data<se::Field::TSDF, ColB, SemB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<se::Data<se::Field::TSDF, ColB, SemB>, BlockSize, DerivedT>::getData(const int voxel_idx)
{
  return block_data_[voxel_idx];
}



template <Colour    ColB,
          Semantics SemB,
          int       BlockSize,
          typename  DerivedT
>
inline const typename BlockMultiRes<se::Data<se::Field::TSDF, ColB, SemB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<se::Data<se::Field::TSDF, ColB, SemB>, BlockSize, DerivedT>::getData(const Eigen::Vector3i& voxel_coord) const
{
  const Eigen::Vector3i voxel_offset = (voxel_coord - this->underlying().coord_) / (1 << curr_scale_);
  const int size_at_scale = size_at_scales_[curr_scale_];
  const int voxel_idx = scale_offsets_[curr_scale_] + voxel_offset.x() +
                                                      voxel_offset.y() * size_at_scale +
                                                      voxel_offset.z() * se::math::sq(size_at_scale);
  return block_data_[voxel_idx];
}



template <Colour    ColB,
        Semantics SemB,
        int       BlockSize,
        typename  DerivedT
>
inline typename BlockMultiRes<se::Data<se::Field::TSDF, ColB, SemB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<se::Data<se::Field::TSDF, ColB, SemB>, BlockSize, DerivedT>::getData(const Eigen::Vector3i& voxel_coord)
{
  const Eigen::Vector3i voxel_offset = (voxel_coord - this->underlying().coord_) / (1 << curr_scale_);
  const int size_at_scale = size_at_scales_[curr_scale_];
  const int voxel_idx = scale_offsets_[curr_scale_] + voxel_offset.x() +
                                                      voxel_offset.y() * size_at_scale +
                                                      voxel_offset.z() * se::math::sq(size_at_scale);
  return block_data_[voxel_idx];
}



template <Colour    ColB,
          Semantics SemB,
          int       BlockSize,
          typename  DerivedT
>
inline const typename BlockMultiRes<se::Data<se::Field::TSDF, ColB, SemB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<se::Data<se::Field::TSDF, ColB, SemB>, BlockSize, DerivedT>::getData(const Eigen::Vector3i& voxel_coord,
                                                                                   const int              scale_desired,
                                                                                   int&                   scale_returned) const
{
  scale_returned = std::max(scale_desired, curr_scale_);

  const Eigen::Vector3i voxel_offset = (voxel_coord - this->underlying().coord_) / (1 << scale_returned);
  const int size_at_scale = size_at_scales_[scale_returned];
  const int voxel_idx = scale_offsets_[scale_returned] + voxel_offset.x() +
                                                         voxel_offset.y() * size_at_scale +
                                                         voxel_offset.z() * se::math::sq(size_at_scale);
  return block_data_[voxel_idx];
}



template <Colour    ColB,
          Semantics SemB,
          int       BlockSize,
          typename  DerivedT
>
inline typename BlockMultiRes<se::Data<se::Field::TSDF, ColB, SemB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<se::Data<se::Field::TSDF, ColB, SemB>, BlockSize, DerivedT>::getData(const Eigen::Vector3i& voxel_coord,
                                                                                   const int              scale_desired,
                                                                                   int&                   scale_returned)
{
  scale_returned = std::max(scale_desired, curr_scale_);
  const Eigen::Vector3i voxel_offset = (voxel_coord - this->underlying().coord_) / (1 << scale_returned);
  const int size_at_scale = size_at_scales_[scale_returned];
  const int voxel_idx = scale_offsets_[scale_returned] + voxel_offset.x() +
                                                         voxel_offset.y() * size_at_scale +
                                                         voxel_offset.z() * se::math::sq(size_at_scale);

  return block_data_[voxel_idx];
}



template <Colour    ColB,
          Semantics SemB,
          int       BlockSize,
          typename  DerivedT
>
inline const typename BlockMultiRes<se::Data<se::Field::TSDF, ColB, SemB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<se::Data<se::Field::TSDF, ColB, SemB>, BlockSize, DerivedT>::getData(const Eigen::Vector3i& voxel_coord,
                                                                                   const int              scale) const
{
  const Eigen::Vector3i voxel_offset = (voxel_coord - this->underlying().coord_) / (1 << scale);
  const int size_at_scale = size_at_scales_[scale];
  const int voxel_idx = scale_offsets_[scale] + voxel_offset.x() +
                                                voxel_offset.y() * size_at_scale +
                                                voxel_offset.z() * se::math::sq(size_at_scale);
  return block_data_[voxel_idx];
}



template <Colour    ColB,
        Semantics SemB,
        int       BlockSize,
        typename  DerivedT
>
inline typename BlockMultiRes<se::Data<se::Field::TSDF, ColB, SemB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<se::Data<se::Field::TSDF, ColB, SemB>, BlockSize, DerivedT>::getData(const Eigen::Vector3i& voxel_coord,
                                                                                   const int              scale)
{
  const Eigen::Vector3i voxel_offset = (voxel_coord - this->underlying().coord_) / (1 << scale);
  const int size_at_scale = size_at_scales_[scale];
  const int voxel_idx = scale_offsets_[scale] + voxel_offset.x() +
                                                voxel_offset.y() * size_at_scale +
                                                voxel_offset.z() * se::math::sq(size_at_scale);
  return block_data_[voxel_idx];
}



template <Colour    ColB,
          Semantics SemB,
          int       BlockSize,
          typename  DerivedT
>
inline const typename BlockMultiRes<se::Data<se::Field::TSDF, ColB, SemB>, BlockSize, DerivedT>::DataUnion
BlockMultiRes<se::Data<se::Field::TSDF, ColB, SemB>, BlockSize, DerivedT>::getDataUnion(const Eigen::Vector3i& voxel_coord,
                                                                                        const int              scale) const
{
  const Eigen::Vector3i voxel_offset = (voxel_coord - this->underlying().coord_) / (1 << scale);
  const int size_at_scale = size_at_scales_[scale];
  const int voxel_idx = scale_offsets_[scale] + voxel_offset.x() +
                                                voxel_offset.y() * size_at_scale +
                                                voxel_offset.z() * se::math::sq(size_at_scale);
  DataUnion data_union;
  data_union.coord         = voxel_coord;
  data_union.scale         = scale;
  data_union.data          = block_data_[voxel_idx];
  data_union.prop_data     = block_prop_data_[voxel_idx];
  data_union.data_idx      = voxel_idx;
  data_union.prop_data_idx = voxel_idx;

  return data_union;
}



template <Colour    ColB,
        Semantics SemB,
        int       BlockSize,
        typename  DerivedT
>
inline typename BlockMultiRes<se::Data<se::Field::TSDF, ColB, SemB>, BlockSize, DerivedT>::DataUnion
BlockMultiRes<se::Data<se::Field::TSDF, ColB, SemB>, BlockSize, DerivedT>::getDataUnion(const Eigen::Vector3i& voxel_coord,
                                                                                        const int              scale)
{
  const Eigen::Vector3i voxel_offset = (voxel_coord - this->underlying().coord_) / (1 << scale);
  const int size_at_scale = size_at_scales_[scale];
  const int voxel_idx = scale_offsets_[scale] + voxel_offset.x() +
                        voxel_offset.y() * size_at_scale +
                        voxel_offset.z() * se::math::sq(size_at_scale);
  DataUnion data_union;
  data_union.coord         = voxel_coord;
  data_union.scale         = scale;
  data_union.data          = block_data_[voxel_idx];
  data_union.prop_data     = block_prop_data_[voxel_idx];
  data_union.data_idx      = voxel_idx;
  data_union.prop_data_idx = voxel_idx;

  return data_union;
}



template <Colour    ColB,
          Semantics SemB,
          int       BlockSize,
          typename  DerivedT
>
inline void BlockMultiRes<se::Data<se::Field::TSDF, ColB, SemB>, BlockSize, DerivedT>::setData(const int       voxel_idx,
                                                                                               const DataType& data)
{
  block_data_[voxel_idx] = data;
}



template <Colour    ColB,
          Semantics SemB,
          int       BlockSize,
          typename  DerivedT
>
inline void BlockMultiRes<se::Data<se::Field::TSDF, ColB, SemB>, BlockSize, DerivedT>::setData(const Eigen::Vector3i& voxel_coord,
                                                                                               const DataType&        data)
{
  const Eigen::Vector3i voxel_offset = (voxel_coord - this->underlying().coord_) / (1 << curr_scale_);
  const int size_at_scale = size_at_scales_[curr_scale_];
  const int voxel_idx = scale_offsets_[curr_scale_] + voxel_offset.x() +
                                                      voxel_offset.y() * size_at_scale +
                                                      voxel_offset.z() * se::math::sq(size_at_scale);
  block_data_[voxel_idx] = data;
}



template <Colour    ColB,
          Semantics SemB,
          int       BlockSize,
          typename  DerivedT
>
inline void BlockMultiRes<se::Data<se::Field::TSDF, ColB, SemB>, BlockSize, DerivedT>::setData(const Eigen::Vector3i& voxel_coord,
                                                                                               const int              scale,
                                                                                               const DataType&        data)
{
  const Eigen::Vector3i voxel_offset = (voxel_coord - this->underlying().coord_) / (1 << scale);
  const int size_at_scale = size_at_scales_[scale];
  const int voxel_idx = scale_offsets_[scale] + voxel_offset.x() +
                                                voxel_offset.y() * size_at_scale +
                                                voxel_offset.z() * se::math::sq(size_at_scale);
  block_data_[voxel_idx] = data;
}



template <Colour    ColB,
          Semantics SemB,
          int       BlockSize,
          typename  DerivedT
>
inline void BlockMultiRes<se::Data<se::Field::TSDF, ColB, SemB>, BlockSize, DerivedT>::setDataUnion(const DataUnion& data_union)
{
  block_data_[data_union.data_idx]           = data_union.data;
  block_prop_data_[data_union.prop_data_idx] = data_union.prop_data;
}


/// Multi-res occupancy

template <Colour    ColB,
          Semantics SemB,
          int       BlockSize,
          typename  DerivedT
>
BlockMultiRes<se::Data<se::Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::BlockMultiRes(const DataType init_data) :
  curr_scale_(0),
  min_scale_(-1),
  buffer_scale_(-1),
  init_data_(init_data)
{
}



template <Colour    ColB,
          Semantics SemB,
          int       BlockSize,
          typename  DerivedT
>
BlockMultiRes<se::Data<se::Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::BlockMultiRes(const Block<se::Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi, BlockSize>& block)
{
  initFromBlock(block);
}



template <Colour    ColB,
          Semantics SemB,
          int       BlockSize,
          typename  DerivedT
>
inline void BlockMultiRes<se::Data<se::Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::operator=(const Block<se::Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi, BlockSize>& block)
{
  initFromBlock(block);
}

template <Colour    ColB,
          Semantics SemB,
          int       BlockSize,
          typename  DerivedT
>
BlockMultiRes<se::Data<se::Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::~BlockMultiRes()
{
  for (auto& data_at_scale : block_data_)
  {
    delete[] data_at_scale;
  }
  block_data_.clear();

  block_max_data_.pop_back(); ///<< Avoid double free as the min scale data points to the same data.
  for (auto& max_data_at_scale : block_max_data_)
  {
    delete[] max_data_at_scale;
  }
  block_max_data_.clear();

  if (buffer_data_ && buffer_scale_ < min_scale_)
  {
    delete[] buffer_data_;
  }
}



/// Get data at current scale

template <Colour    ColB,
          Semantics SemB,
          int       BlockSize,
          typename  DerivedT
>
inline const typename BlockMultiRes<se::Data<se::Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<se::Data<se::Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::getData(const Eigen::Vector3i& voxel_coord) const
{
//  std::cout << "GET DATA - 1" << std::endl;
  Eigen::Vector3i voxel_offset = voxel_coord - this->underlying().coord_;
  voxel_offset = voxel_offset / (1 << curr_scale_);
  const int size_at_scale = BlockSize >> curr_scale_;
  return block_data_[max_scale_ - curr_scale_][voxel_offset.x() +
                                               voxel_offset.y() * size_at_scale +
                                               voxel_offset.z() * se::math::sq(size_at_scale)];
}



template <Colour    ColB,
          Semantics SemB,
          int       BlockSize,
          typename  DerivedT
>
inline typename BlockMultiRes<se::Data<se::Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<se::Data<se::Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::getData(const Eigen::Vector3i& voxel_coord)
{
//  std::cout << "GET DATA - 2" << std::endl;
  Eigen::Vector3i voxel_offset = voxel_coord - this->underlying().coord_;
  voxel_offset = voxel_offset / (1 << curr_scale_);
  const int size_at_scale = BlockSize >> curr_scale_;
  return block_data_[max_scale_ - curr_scale_][voxel_offset.x() +
                                               voxel_offset.y() * size_at_scale +
                                               voxel_offset.z() * se::math::sq(size_at_scale)];
}



/// Get data at current scale or coarser

template <Colour    ColB,
          Semantics SemB,
          int       BlockSize,
          typename  DerivedT
>
inline const typename BlockMultiRes<se::Data<se::Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<se::Data<se::Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::getData(const Eigen::Vector3i& voxel_coord,
                                                                                        const int              scale_in,
                                                                                        int&                   scale_out) const
{
//  std::cout << "GET DATA - 3" << std::endl;
  scale_out = std::max(scale_in, curr_scale_);
  Eigen::Vector3i voxel_offset = voxel_coord - this->underlying().coord_;
  voxel_offset = voxel_offset / (1 << scale_out);
  const int size_at_scale = BlockSize >> scale_out;
  return block_data_[max_scale_ - scale_out][voxel_offset.x() +
                                             voxel_offset.y() * size_at_scale +
                                             voxel_offset.z() * se::math::sq(size_at_scale)];
}



template <Colour    ColB,
          Semantics SemB,
          int       BlockSize,
          typename  DerivedT
>
inline typename BlockMultiRes<se::Data<se::Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<se::Data<se::Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::getData(const Eigen::Vector3i& voxel_coord,
                                                                                        const int              scale_in,
                                                                                        int&                   scale_out)
{
//  std::cout << "GET DATA - 4" << std::endl;
  scale_out = std::max(scale_in, curr_scale_);
  Eigen::Vector3i voxel_offset = voxel_coord - this->underlying().coord_;
  voxel_offset = voxel_offset / (1 << scale_out);
  const int size_at_scale = BlockSize >> scale_out;
  return block_data_[max_scale_ - scale_out][voxel_offset.x() +
                                             voxel_offset.y() * size_at_scale +
                                             voxel_offset.z() * se::math::sq(size_at_scale)];
}



/// Get data at scale

template <Colour    ColB,
          Semantics SemB,
          int       BlockSize,
          typename  DerivedT
>
inline const typename BlockMultiRes<se::Data<se::Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<se::Data<se::Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::getData(const Eigen::Vector3i& voxel_coord,
                                                                                        const int              scale) const
{
//  std::cout << "GET DATA - 5" << std::endl;
  if (max_scale_ - (block_data_.size() - 1) > static_cast<size_t>(scale)) {
    return init_data_;
  } else
  {
    Eigen::Vector3i voxel_offset = voxel_coord - this->underlying().coord_;
    voxel_offset = voxel_offset / (1 << scale);
    const int size_at_scale = BlockSize >> scale;
    return block_data_[max_scale_ - scale][voxel_offset.x() +
                                           voxel_offset.y() * size_at_scale +
                                           voxel_offset.z() * se::math::sq(size_at_scale)];
  }
}



template <Colour    ColB,
          Semantics SemB,
          int       BlockSize,
          typename  DerivedT
>
inline typename BlockMultiRes<se::Data<se::Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<se::Data<se::Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::getData(const Eigen::Vector3i& voxel_coord,
                                                                                        const int              scale)
{
//  std::cout << "GET DATA - 6" << std::endl;
  if (max_scale_ - (block_data_.size() - 1) > static_cast<size_t>(scale)) {
    return init_data_;
  } else
  {
//    std::cout << this->underlying().coord_ << std::endl;
    Eigen::Vector3i voxel_offset = voxel_coord - this->underlying().coord_;
    voxel_offset = voxel_offset / (1 << scale);
    const int size_at_scale = BlockSize >> scale;
    return block_data_[max_scale_ - scale][voxel_offset.x() +
                                           voxel_offset.y() * size_at_scale +
                                           voxel_offset.z() * se::math::sq(size_at_scale)];
  }
}



template <Colour    ColB,
          Semantics SemB,
          int       BlockSize,
          typename  DerivedT
>
inline const typename BlockMultiRes<se::Data<se::Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<se::Data<se::Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::getMaxData(const Eigen::Vector3i& voxel_coord) const
{
//  std::cout << "GET DATA - 7" << std::endl;
  Eigen::Vector3i voxel_offset = voxel_coord - this->underlying().coord_;
  voxel_offset = voxel_offset / (1 << curr_scale_);
  const int size_at_scale = BlockSize >> curr_scale_;
  return block_max_data_[max_scale_ - curr_scale_][voxel_offset.x() +
                                                   voxel_offset.y() * size_at_scale +
                                                   voxel_offset.z() * se::math::sq(size_at_scale)];
}



template <Colour    ColB,
          Semantics SemB,
          int       BlockSize,
          typename  DerivedT
>
inline typename BlockMultiRes<se::Data<se::Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<se::Data<se::Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::getMaxData(const Eigen::Vector3i& voxel_coord)
{
//  std::cout << "GET MAX DATA - 1" << std::endl;
  Eigen::Vector3i voxel_offset = voxel_coord - this->underlying().coord_;
  voxel_offset = voxel_offset / (1 << curr_scale_);
  const int size_at_scale = BlockSize >> curr_scale_;
  return block_max_data_[max_scale_ - curr_scale_][voxel_offset.x() +
                                                   voxel_offset.y() * size_at_scale +
                                                   voxel_offset.z() * se::math::sq(size_at_scale)];
}



template <Colour    ColB,
          Semantics SemB,
          int       BlockSize,
          typename  DerivedT
>
inline const typename BlockMultiRes<se::Data<se::Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<se::Data<se::Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::getMaxData(const Eigen::Vector3i& voxel_coord,
                                                                                           const int              scale_in,
                                                                                           int&                   scale_out) const
{
//  std::cout << "GET MAX DATA - 2" << std::endl;
  scale_out = std::max(scale_in, curr_scale_);
  Eigen::Vector3i voxel_offset = voxel_coord - this->underlying().coord_;
  voxel_offset = voxel_offset / (1 << scale_out);
  const int size_at_scale = BlockSize >> scale_out;
  return block_max_data_[max_scale_ - scale_out][voxel_offset.x() +
                                                 voxel_offset.y() * size_at_scale +
                                                 voxel_offset.z() * se::math::sq(size_at_scale)];
}



template <Colour    ColB,
          Semantics SemB,
          int       BlockSize,
          typename  DerivedT
>
inline typename BlockMultiRes<se::Data<se::Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<se::Data<se::Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::getMaxData(const Eigen::Vector3i& voxel_coord,
                                                                                           const int              scale_in,
                                                                                           int&                   scale_out)
{
//  std::cout << "GET MAX DATA - 3" << std::endl;
  scale_out = std::max(scale_in, curr_scale_);
  Eigen::Vector3i voxel_offset = voxel_coord - this->underlying().coord_;
  voxel_offset = voxel_offset / (1 << scale_out);
  const int size_at_scale = BlockSize >> scale_out;
  return block_max_data_[max_scale_ - scale_out][voxel_offset.x() +
                                                 voxel_offset.y() * size_at_scale +
                                                 voxel_offset.z() * se::math::sq(size_at_scale)];
}



template <Colour    ColB,
          Semantics SemB,
          int       BlockSize,
          typename  DerivedT
>
inline const typename BlockMultiRes<se::Data<se::Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<se::Data<se::Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::getMaxData(const Eigen::Vector3i& voxel_coord,
                                                                                           const int              scale) const
{
//  std::cout << "GET MAX DATA - 4" << std::endl;
  if (max_scale_ - (block_data_.size() - 1) > static_cast<size_t>(scale)) {
    return init_data_;
  } else {
    Eigen::Vector3i voxel_offset = voxel_coord - this->underlying().coord_;
    voxel_offset = voxel_offset / (1 << scale);
    const int size_at_scale = BlockSize >> scale;
    return block_max_data_[max_scale_ - scale][voxel_offset.x() +
                                               voxel_offset.y() * size_at_scale +
                                               voxel_offset.z() * se::math::sq(size_at_scale)];
  }
}



template <Colour    ColB,
          Semantics SemB,
          int       BlockSize,
          typename  DerivedT
>
inline typename BlockMultiRes<se::Data<se::Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<se::Data<se::Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::getMaxData(const Eigen::Vector3i& voxel_coord,
                                                                                           const int              scale)
{
//  std::cout << "GET MAX DATA - 5" << std::endl;
  if (max_scale_ - (block_data_.size() - 1) > static_cast<size_t>(scale)) {
    return init_data_;
  } else {
    Eigen::Vector3i voxel_offset = voxel_coord - this->underlying().coord_;
    voxel_offset = voxel_offset / (1 << scale);
    const int size_at_scale = BlockSize >> scale;
    return block_max_data_[max_scale_ - scale][voxel_offset.x() +
                                               voxel_offset.y() * size_at_scale +
                                               voxel_offset.z() * se::math::sq(size_at_scale)];
  }
}



template <Colour    ColB,
          Semantics SemB,
          int       BlockSize,
          typename  DerivedT
>
inline void BlockMultiRes<se::Data<se::Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::setData(const Eigen::Vector3i& voxel_coord,
                                                                                                    const DataType&        data)
{
//  std::cout << "SET DATA - 1" << std::endl;
  Eigen::Vector3i voxel_offset = voxel_coord - this->underlying().coord_;
  voxel_offset = voxel_offset / (1 << curr_scale_);
  const int size_at_scale = BlockSize >> curr_scale_;
  block_data_[max_scale_ - curr_scale_][voxel_offset.x() +
                                        voxel_offset.y() * size_at_scale +
                                        voxel_offset.z() * se::math::sq(size_at_scale)] = data;
}



template <Colour    ColB,
          Semantics SemB,
          int       BlockSize,
          typename  DerivedT
>
inline void BlockMultiRes<se::Data<se::Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::setData(const Eigen::Vector3i& voxel_coord,
                                                                                                    const int              scale,
                                                                                                    const DataType&        data)
{
//  std::cout << "SET DATA - 2" << std::endl;
  Eigen::Vector3i voxel_offset = voxel_coord - this->underlying().coord_;
  voxel_offset = voxel_offset / (1 << scale);
  const int size_at_scale = BlockSize >> scale;
  block_data_[max_scale_ - scale][voxel_offset.x() +
                                  voxel_offset.y() * size_at_scale +
                                  voxel_offset.z() * se::math::sq(size_at_scale)] = data;
}



template <Colour    ColB,
          Semantics SemB,
          int       BlockSize,
          typename  DerivedT
>
inline void BlockMultiRes<se::Data<se::Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::setMaxData(const Eigen::Vector3i& voxel_coord,
                                                                                                       const DataType&        data)
{
//  std::cout << "SET MAX DATA - 1" << std::endl;
  Eigen::Vector3i voxel_offset = voxel_coord - this->underlying().coord_;
  voxel_offset = voxel_offset / (1 << curr_scale_);
  const int size_at_scale = BlockSize >> curr_scale_;
  block_max_data_[max_scale_ - curr_scale_][voxel_offset.x() +
                                            voxel_offset.y() * size_at_scale +
                                            voxel_offset.z() * se::math::sq(size_at_scale)] = data;
}



template <Colour    ColB,
          Semantics SemB,
          int       BlockSize,
          typename  DerivedT
>
inline void BlockMultiRes<se::Data<se::Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::setMaxData(const Eigen::Vector3i& voxel_coord,
                                                                                                       const int              scale,
                                                                                                       const DataType&        data)
{
//  std::cout << "SET MAX DATA - 2" << std::endl;
  Eigen::Vector3i voxel_offset = voxel_coord - this->underlying().coord_;
  voxel_offset = voxel_offset / (1 << scale);
  const int size_at_scale = BlockSize >> scale;
  block_max_data_[max_scale_ - scale][voxel_offset.x() +
                                      voxel_offset.y() * size_at_scale +
                                      voxel_offset.z() * se::math::sq(size_at_scale)] = data;
}



template <Colour    ColB,
          Semantics SemB,
          int       BlockSize,
          typename  DerivedT
>
inline void BlockMultiRes<se::Data<se::Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::allocateDownTo()
{
  if (max_scale_ - (block_data_.size() - 1) != 0)
  {
    for (int scale = max_scale_ - block_data_.size(); scale >= 0; scale --)
    {
      int size_at_scale = BlockSize >> scale;
      int num_voxels_at_scale = se::math::cu(size_at_scale);

      if (scale == 0)
      {
        DataType* data_at_scale = new DataType[num_voxels_at_scale];
        initialiseData(data_at_scale, num_voxels_at_scale);
        block_data_.push_back(data_at_scale);
        block_max_data_.push_back(data_at_scale); ///<< Mean and max data are the same at the min scale.
      } else
      {
        DataType* data_at_scale     = new DataType[num_voxels_at_scale];
        DataType* max_data_at_scale = new DataType[num_voxels_at_scale];
        initialiseData(data_at_scale, num_voxels_at_scale);
        block_data_.push_back(data_at_scale);
        std::copy(data_at_scale, data_at_scale + num_voxels_at_scale, max_data_at_scale); ///<< Copy init content.
        block_max_data_.push_back(max_data_at_scale);
      }
    }

    curr_scale_ = 0;
    min_scale_ = 0;
    curr_data_ = block_data_[max_scale_];
  }
}



template <Colour    ColB,
          Semantics SemB,
          int       BlockSize,
          typename  DerivedT
>
inline void BlockMultiRes<se::Data<se::Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::allocateDownTo(const int new_min_scale)
{
  if (max_scale_ - (block_data_.size() - 1) > static_cast<size_t>(new_min_scale))
  {

    for (int scale = max_scale_ - block_data_.size(); scale >= new_min_scale; scale--)
    {
      int size_at_scale = BlockSize >> scale;
      int num_voxels_at_scale = se::math::cu(size_at_scale);

      if (scale == new_min_scale)
      {
        DataType* data_at_scale = new DataType[num_voxels_at_scale];
        initialiseData(data_at_scale, num_voxels_at_scale);
        block_data_.push_back(data_at_scale);
        block_max_data_.push_back(data_at_scale); ///<< Mean and max data are the same at the min scale.
      } else
      {
        DataType* data_at_scale     = new DataType[num_voxels_at_scale];
        DataType* max_data_at_scale = new DataType[num_voxels_at_scale];
        initialiseData(data_at_scale, num_voxels_at_scale);
        block_data_.push_back(data_at_scale);
        std::copy(data_at_scale, data_at_scale + num_voxels_at_scale, max_data_at_scale); ///<< Copy init content.
        block_max_data_.push_back(max_data_at_scale);
      }
    }

    curr_scale_ = new_min_scale;
    min_scale_  = new_min_scale;
    curr_data_  = block_data_[max_scale_ - new_min_scale];
  }
}



template <Colour    ColB,
          Semantics SemB,
          int       BlockSize,
          typename  DerivedT
>
inline void BlockMultiRes<se::Data<se::Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::deleteUpTo(const int new_min_scale)
{
  if (min_scale_ == -1 || min_scale_ >= new_min_scale) return;

  auto& data_at_scale = block_data_[max_scale_ - min_scale_];
  delete[] data_at_scale;
  block_data_.pop_back();
  block_max_data_.pop_back(); ///<< Avoid double free as the min scale data points to the same data.

  for (int scale = min_scale_ + 1; scale < new_min_scale; scale++)
  {
    // Delete mean data
    data_at_scale = block_data_[max_scale_ - scale];
    delete[] data_at_scale;
    block_data_.pop_back();

    // Delete max data
    auto& max_data_at_scale = block_max_data_[max_scale_ - scale];
    delete[] max_data_at_scale;
    block_max_data_.pop_back();
  }

  // Replace max data at min scale with same as mean data.
  auto& max_data_at_scale = block_max_data_[max_scale_ - new_min_scale];
  delete[] max_data_at_scale;
  block_max_data_.pop_back();
  block_max_data_.push_back(block_data_[max_scale_ - new_min_scale]);

  min_scale_ = new_min_scale;
}


template <Colour    ColB,
          Semantics SemB,
          int       BlockSize,
          typename  DerivedT
>
inline void BlockMultiRes<se::Data<se::Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::incrCurrObservedCount(bool do_increment)
{
  if (do_increment)
  {
    curr_observed_count_++;
  }
}



template <Colour    ColB,
          Semantics SemB,
          int       BlockSize,
          typename  DerivedT
>
inline void BlockMultiRes<se::Data<se::Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::resetCurrCount()
{
  curr_integr_count_   = 0;
  curr_observed_count_ = 0;
}



template <Colour    ColB,
          Semantics SemB,
          int       BlockSize,
          typename  DerivedT
>
inline void BlockMultiRes<se::Data<se::Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::initCurrCout()
{
  if (init_data_.observed)
  {
    int size_at_scale = BlockSize >> curr_scale_;
    int num_voxels_at_scale = se::math::cu(size_at_scale);
    curr_integr_count_   = init_data_.weight;
    curr_observed_count_ = num_voxels_at_scale;
  } else
  {
    resetCurrCount();
  }
}



template <Colour    ColB,
          Semantics SemB,
          int       BlockSize,
          typename  DerivedT
>
inline void BlockMultiRes<se::Data<se::Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::incrBufferIntegrCount(const bool do_increment)
{
  if (do_increment ||
      buffer_observed_count_ * se::math::cu(1 << buffer_scale_) >= 0.90 * curr_observed_count_ * se::math::cu(1 << curr_scale_))
  {
    buffer_integr_count_++;
  }
}



template <Colour    ColB,
          Semantics SemB,
          int       BlockSize,
          typename  DerivedT
>
inline void BlockMultiRes<se::Data<se::Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::incrBufferObservedCount(const bool do_increment)
{
  if (do_increment)
  {
    buffer_observed_count_++;
  }
}



template <Colour    ColB,
          Semantics SemB,
          int       BlockSize,
          typename  DerivedT
>
inline void BlockMultiRes<se::Data<se::Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::resetBufferCount()
{
  buffer_integr_count_   = 0;
  buffer_observed_count_ = 0;
}



template <Colour    ColB,
          Semantics SemB,
          int       BlockSize,
          typename  DerivedT
>
inline void BlockMultiRes<se::Data<se::Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::resetBuffer()
{
  if (buffer_scale_ < curr_scale_)
  {
    delete[] buffer_data_;
  }
  buffer_data_ = nullptr;
  buffer_scale_ = -1;
  resetBufferCount();
}

template <Colour    ColB,
          Semantics SemB,
          int       BlockSize,
          typename  DerivedT
>
inline void BlockMultiRes<se::Data<se::Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::initBuffer(const int buffer_scale)
{
  resetBuffer();

  buffer_scale_ = buffer_scale;

  if (buffer_scale < curr_scale_)
  {
    // Initialise all data to init data.
    const int size_at_scale = BlockSize >> buffer_scale;
    const int num_voxels_at_scale = se::math::cu(size_at_scale);
    buffer_data_ = new DataType[num_voxels_at_scale]; ///<< Data must still be initialised.
  } else
  {
    buffer_data_ = block_data_[max_scale_ - buffer_scale_];
  }
}



template <Colour    ColB,
          Semantics SemB,
          int       BlockSize,
          typename  DerivedT
>
inline bool BlockMultiRes<se::Data<se::Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::switchData()
{
  if (buffer_integr_count_ >= 20 &&
      buffer_observed_count_ * se::math::cu(1 << buffer_scale_) >= 0.9 * curr_observed_count_ * se::math::cu(1 << curr_scale_)) { // TODO: Find threshold

    /// !!! We'll switch !!!
    if (buffer_scale_ < curr_scale_)
    { ///<< Switch to finer scale.
      block_data_.push_back(buffer_data_);
      block_max_data_.push_back(buffer_data_); ///< Share data at finest scale.

      /// Add allocate data for the scale that mean and max data shared before.
      const int size_at_scale = BlockSize >> (buffer_scale_ + 1);
      const int num_voxels_at_scale = se::math::cu(size_at_scale);
      block_max_data_[max_scale_ - (buffer_scale_ + 1)] = new DataType[num_voxels_at_scale]; ///<< Data must still be initialised.

    } else
    { ///<< Switch to coarser scale.
      deleteUpTo(buffer_scale_);
    }

    /// Update observed state
    const int size_at_buffer_scale = BlockSize >> buffer_scale_;
    const int num_voxels_at_buffer_scale = se::math::cu(size_at_buffer_scale);

    int missed_observed_count = 0;
    for (int voxel_idx = 0; voxel_idx < num_voxels_at_buffer_scale; voxel_idx++)
    {
      DataType& data = buffer_data_[voxel_idx];
      if (data.weight > 0 && !data.observed)
      {
        data.observed = true;
        buffer_observed_count_++;
        missed_observed_count++;
      }
    }

    curr_scale_ = buffer_scale_;
    min_scale_  = buffer_scale_;

    curr_data_           = buffer_data_;
    curr_integr_count_   = buffer_integr_count_;
    curr_observed_count_ = buffer_observed_count_;
    buffer_data_ = nullptr;
    buffer_scale_ = -1;
    resetBufferCount();
    return true;
  }
  return false;
}



template <Colour    ColB,
          Semantics SemB,
          int       BlockSize,
          typename  DerivedT
>
inline typename BlockMultiRes<se::Data<se::Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<se::Data<se::Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::bufferData(const Eigen::Vector3i& voxel_coord) const
{
  Eigen::Vector3i voxel_offset = voxel_coord - this->underlying()->coord_;
  voxel_offset = voxel_offset / (1 << buffer_scale_);
  const int size_at_scale = BlockSize >> buffer_scale_;
  return buffer_data_[voxel_offset.x() + voxel_offset.y() * size_at_scale + voxel_offset.z() * se::math::sq(size_at_scale)];
}



template <Colour    ColB,
          Semantics SemB,
          int       BlockSize,
          typename  DerivedT
>
inline typename BlockMultiRes<se::Data<se::Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::DataType&
BlockMultiRes<se::Data<se::Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::bufferData(const Eigen::Vector3i& voxel_coord)
{
  Eigen::Vector3i voxel_offset = voxel_coord - this->underlying()->coord_;
  voxel_offset = voxel_offset / (1 << buffer_scale_);
  const int size_at_scale = BlockSize >> buffer_scale_;
  return buffer_data_[voxel_offset.x() + voxel_offset.y() * size_at_scale + voxel_offset.z() * se::math::sq(size_at_scale)];
}



template <Colour    ColB,
          Semantics SemB,
          int       BlockSize,
          typename  DerivedT
>
inline typename BlockMultiRes<se::Data<se::Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::DataType*
BlockMultiRes<se::Data<se::Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::blockDataAtScale(const int scale)
{
  if (scale < min_scale_)
  {
    return nullptr;
  } else
  {
    return block_data_[max_scale_ - scale];
  }
}



template <Colour    ColB,
          Semantics SemB,
          int       BlockSize,
          typename  DerivedT
>
inline typename BlockMultiRes<se::Data<se::Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::DataType*
BlockMultiRes<se::Data<se::Field::Occupancy, ColB, SemB>, BlockSize, DerivedT>::blockMaxDataAtScale(const int scale)
{
  if (scale < min_scale_)
  {
    return nullptr;
  } else
  {
    return block_max_data_[max_scale_ - scale];
  }
}



template <typename DataT,
          Res      ResT,
          int      BlockSize,
          typename PolicyT
>
Block<DataT, ResT, BlockSize, PolicyT>::Block(se::Node<DataT, ResT>* parent_ptr,
                                              const unsigned         child_id) :
    std::conditional<ResT == Res::Single,
      BlockSingleRes<Block<DataT, ResT, BlockSize>, DataT, BlockSize>,
      BlockMultiRes<DataT, BlockSize, Block<DataT, ResT, BlockSize>>>::type(),
    OctantBase(true,
               parent_ptr->getCoord() + BlockSize * Eigen::Vector3i((1 & child_id) > 0, (2 & child_id) > 0, (4 & child_id) > 0),
               parent_ptr)

{
  assert(BlockSize == (parent_ptr->getSize() >> 1));
}



} // namespace se

#endif // SE_OCTANT_IMPL_HPP
