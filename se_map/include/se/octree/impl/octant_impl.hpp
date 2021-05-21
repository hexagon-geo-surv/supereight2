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



template <typename DataT,
          Res      ResT
>
inline void Node<DataT, ResT>::setData(const DataT& data)
{
  data_ = data;
}



template <typename DataT,
          Res ResT
>
inline void Node<DataT, ResT>::getData(const DataT& data)
{
  data = data_;
}



/// Single-res Block ///

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
BlockMultiRes<se::Data<se::Field::TSDF, ColB, SemB>, BlockSize, DerivedT>::BlockMultiRes() : min_scale_(-1), curr_scale_(-1)
{
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
                                                                                   const int              scale_in,
                                                                                   int&                   scale_out) const
{
  scale_out = std::max(scale_in, curr_scale_);

  const Eigen::Vector3i voxel_offset = (voxel_coord - this->underlying().coord_) / (1 << scale_out);
  const int size_at_scale = size_at_scales_[scale_out];
  const int voxel_idx = scale_offsets_[scale_out] + voxel_offset.x() +
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
                                                                                   const int              scale_in,
                                                                                   int&                   scale_out)
{
  scale_out = std::max(scale_in, curr_scale_);
  const Eigen::Vector3i voxel_offset = (voxel_coord - this->underlying().coord_) / (1 << scale_out);
  const int size_at_scale = size_at_scales_[scale_out];
  const int voxel_idx = scale_offsets_[scale_out] + voxel_offset.x() +
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



template <typename DataT,
          Res      ResT,
          int      BlockSize,
          typename PolicyT
>
Block<DataT, ResT, BlockSize, PolicyT>::Block(se::Node<DataT, ResT>* parent_ptr,
                                              const unsigned         child_id)
    : OctantBase(true,
                 parent_ptr->getCoord() + BlockSize * Eigen::Vector3i((1 & child_id) > 0, (2 & child_id) > 0, (4 & child_id) > 0),
                 parent_ptr)
{
  assert(BlockSize == (parent_ptr->getSize() >> 1));
}



} // namespace se

#endif // SE_OCTANT_IMPL_HPP
