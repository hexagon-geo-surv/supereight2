#ifndef SE_OCTANT_HPP
#define SE_OCTANT_HPP

#include <memory>
#include <Eigen/Dense>

#include "se/utils/setup_util.hpp"
#include "se/utils/math_util.hpp"
#include "se/data.hpp"

#include <iostream>

namespace se {

/**
 * \brief This class only helps to dynamic cast the octant to the right type and builds the base of nodes and blocks.
 */
class OctantBase
{
public:
  OctantBase(const bool             is_block,
             const Eigen::Vector3i& coord,
             OctantBase*            parent_ptr = nullptr);

  inline bool isBlock() const { return is_block_; };

  inline Eigen::Vector3i getCoord() const { return coord_; }

  inline se::OctantBase* getParent() { return parent_ptr_; }

  inline se::OctantBase* getParent() const { return parent_ptr_; }

  inline int getTimeStamp() const { return time_stamp_; }

  inline void setTimeStamp(const unsigned int time_stamp) { time_stamp_ = time_stamp; }

  inline unsigned int getChildrenMask() const { return children_mask_; }

  inline void setChildrenMask(const unsigned int child_idx) { children_mask_ |= 1 << child_idx; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
  const bool            is_block_;
  OctantBase*           parent_ptr_;    ///< Every node/block (other than root) needs a parent
  const Eigen::Vector3i coord_;         ///< The coordinates of the block (left, front , bottom corner)
  int                   time_stamp_;    ///< The frame of the last update
  unsigned int          children_mask_; ///< The allocated children

  template<typename DerT, typename DatT, int BS>
  friend class BlockSingleRes;

  template<typename DatT, int BS, typename DerT>
  friend class BlockMultiRes;
};



///////////////////////////
/// NODE IMPLEMENTATION ///
///////////////////////////

template <typename NodeT>
inline void get_child_idx(const Eigen::Vector3i& voxel_coord,
                          NodeT*                 node_ptr,
                          unsigned int&          child_idx);



template <typename DataT,
          se::Res  ResT = se::Res::Single
>
class Node : public OctantBase
{
public:
  typedef DataT DataType;

  Node(const Eigen::Vector3i& coord,
       const unsigned int     size);

  Node(Node*              parent_ptr,
       const unsigned int child_idx);

  inline unsigned int getSize() const;

  inline se::OctantBase* getChild(const unsigned child_idx);

  inline const se::OctantBase* getChild(const unsigned child_idx) const;

  inline se::OctantBase* setChild(const unsigned child_idx, se::OctantBase* child_ptr);

  inline void getData(const DataT& data);

  inline void setData(const DataT& data);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  std::array<se::OctantBase*, 8> children_ptr_; ///< Pointers to the eight children (should be all nullptr at initialisation due to smart pointers)
  const unsigned int             size_;         ///< The size in [voxel] of the node in comparision to the finest voxel
  DataType                       data_; ///< The data of the node
};



////////////////////////////
/// BLOCK IMPLEMENTATION ///
////////////////////////////



/**
 * \brief The base used for single-resolution blocks
 */
template <typename DerivedT,
          typename DataT,
          int      BlockSize
>
class BlockSingleRes
{
public:
  typedef DataT DataType;

  inline const DataType& getData(const int voxel_idx) const;

  inline       DataType& getData(const int voxel_idx);

  inline const DataType& getData(const Eigen::Vector3i& voxel_coord) const;

  inline       DataType& getData(const Eigen::Vector3i& voxel_coord);

  inline void setData(const unsigned voxel_idx,
                      const DataT&   data);

  inline void setData(const Eigen::Vector3i& voxel_coord,
                      const DataT&           data);


  static inline int getMinScale() { return min_scale_; }

  static inline int getCurrentScale() { return curr_scale_; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  std::array<DataType, BlockSize * BlockSize * BlockSize> block_data_;
  static constexpr int min_scale_ = 0;
  static constexpr int curr_scale_ = 0;

  DerivedT& underlying() { return static_cast<DerivedT&>(*this); }
  const DerivedT& underlying() const { return static_cast<const DerivedT&>(*this); }
};



// Forward Declaration
template <typename DataT,
          int      BlockSize,
          typename DerivedT
> class BlockMultiRes
{
};



template <Field     FldT,
          Colour    ColB,
          Semantics SemB,
          int       BlockSize,
          typename  DerivedT
>
class BlockMultiRes<se::Data<FldT, ColB, SemB>, BlockSize, DerivedT>
{
};



template <Colour    ColB,
          Semantics SemB,
          int       BlockSize,
          typename  DerivedT
>
class BlockMultiRes<se::Data<se::Field::TSDF, ColB, SemB>, BlockSize, DerivedT>
{
public:
  typedef se::Data<se::Field::TSDF, ColB, SemB>      DataType;
  typedef se::DeltaData<se::Field::TSDF, ColB, SemB> PropDataType;

  BlockMultiRes();

  struct DataUnion
  {
    DataUnion() {};
    Eigen::Vector3i coord;
    int             scale;

    DataType        data;
    PropDataType    prop_data;

    int             data_idx;
    int             prop_data_idx;
  };

  /// Get voxel index

  inline int getVoxelIdx(const Eigen::Vector3i& voxel_coord) const;

  inline int getVoxelIdx(const Eigen::Vector3i& voxel_coord,
                         const int              scale) const;

  /// Get data at current scale

  inline const DataType& getData(const int voxel_idx) const;

  inline       DataType& getData(const int voxel_idx);

  inline const DataType& getData(const Eigen::Vector3i& voxel_coord) const;

  inline       DataType& getData(const Eigen::Vector3i& voxel_coord);

  /// Get data at current scale or coarser

  inline const DataType& getData(const Eigen::Vector3i& voxel_coord,
                                 const int              scale_in,
                                 int&                   scale_out) const;

  inline       DataType& getData(const Eigen::Vector3i& voxel_coord,
                                 const int              scale_in,
                                 int&                   scale_out);

  /// Get data at scale

  inline const DataType& getData(const Eigen::Vector3i& voxel_coord,
                                 const int              scale) const;

  inline       DataType& getData(const Eigen::Vector3i& voxel_coord,
                                 const int              scale);

  /// Get data

  inline const DataUnion getDataUnion(const Eigen::Vector3i& voxel_coord,
                                      const int              scale) const;

  inline       DataUnion getDataUnion(const Eigen::Vector3i& voxel_coord,
                                      const int              scale);

  /// Set data at current scale

  inline void setData(const int       voxel_idx,
                      const DataType& data);

  inline void setData(const Eigen::Vector3i& voxel_coord,
                      const DataType&        data);

  inline void setData(const Eigen::Vector3i& voxel_coord,
                      const int              scale,
                      const DataType&        data);

  inline void setDataUnion(const DataUnion& data_union);


  /// Get scales

  inline int getMinScale() const { return min_scale_; }

  inline void setMinScale(const int min_scale) { min_scale_ = min_scale; }

  static constexpr inline int getMaxScale() { return max_scale_; }

  inline int getCurrentScale() const { return curr_scale_; }

  inline void setCurrentScale(const int curr_scale) { curr_scale_ = curr_scale; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:

  static constexpr int max_scale_ = math::log2_const(BlockSize);

  static constexpr int compute_num_voxels()
  {
    size_t voxel_count = 0;
    unsigned int size_at_scale = BlockSize;
    while(size_at_scale >= 1)
    {
      voxel_count += size_at_scale * size_at_scale * size_at_scale;

      size_at_scale = size_at_scale >> 1;
    }
    return voxel_count;
  }

  static constexpr std::array<int, se::math::log2_const(BlockSize) + 1> compute_size_at_scales()
  {
    std::array<int, se::math::log2_const(BlockSize) + 1> size_at_scales{};

    int size_at_scale = BlockSize;
    int scale = 0;
    while(size_at_scale >= 1)
    {
      size_at_scales[scale]= size_at_scale;
      size_at_scale = size_at_scale >> 1;
      scale++;
    }
    return size_at_scales;
  }

  static constexpr std::array<int, se::math::log2_const(BlockSize) + 1> compute_scale_offsets()
  {
    std::array<int, se::math::log2_const(BlockSize) + 1> scale_offsets{};

    unsigned int size_at_scale = BlockSize;
    scale_offsets[0] = 0;
    int scale = 1;
    while(size_at_scale > 1)
    {
      scale_offsets[scale] = scale_offsets[scale - 1] + size_at_scale * size_at_scale * size_at_scale;
      size_at_scale = size_at_scale >> 1;
      scale++;
    }
    return scale_offsets;
  }

  static constexpr int num_voxels_ = compute_num_voxels();

  static constexpr std::array<int, se::math::log2_const(BlockSize) + 1> size_at_scales_ = compute_size_at_scales();

  static constexpr std::array<int, se::math::log2_const(BlockSize) + 1> scale_offsets_  = compute_scale_offsets();


  std::array<DataType, num_voxels_>     block_data_;
  std::array<PropDataType, num_voxels_> block_prop_data_;

  int min_scale_;
  int curr_scale_;

  DerivedT& underlying() { return static_cast<DerivedT&>(*this); }
  const DerivedT& underlying() const { return static_cast<const DerivedT&>(*this); }
};



/**
 * \brief The actual block used in the tree.
 *
 */
template <typename DataT,
          Res      ResT      = Res::Single,
          int      BlockSize = 8,
          typename PolicyT   = std::enable_if_t<math::is_power_of_two(BlockSize)> ///< Verify that the block size is sensible
>
class Block : public std::conditional<ResT == Res::Single,
        BlockSingleRes<Block<DataT, ResT, BlockSize>, DataT, BlockSize>,
        BlockMultiRes<DataT, BlockSize, Block<DataT, ResT, BlockSize>>>::type, ///< Conditional CRTP
        public OctantBase
{
public:
  typedef DataT DataType;
  static constexpr int    size    = BlockSize;
  static constexpr int    size_qu = BlockSize * BlockSize;
  static constexpr int    size_cu = BlockSize * BlockSize * BlockSize;

  /**
   * \brief Initialise block via parent node
   *
   * \param parent        The shared pointer to the parent node
   * \param child_id      The child id {0,...,7} in relation to the parent
   * \param init_data     The initial data of the block
   */
  Block(se::Node<DataT, ResT>* parent_ptr = nullptr,
        const unsigned         child_id   = 0);

  static constexpr unsigned int getSize() { return BlockSize; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace se

#include "impl/octant_impl.hpp"

#endif // SE_OCTANT_HPP

//  template <typename OctreeT>
//  inline void setCurrentScale(const OctreeT octree,
//                              int           curr_scale)
//  {
//    if (curr_scale < curr_scale_)
//    {
//
//      auto getDataUnion = [&this](const OctreeT&              octree,
//                                  const Eigen::Vector3i&      voxel_coord,
//                                  const int                   scale)
//      {
//        Eigen::Vector3i voxel_offset = voxel_coord - this->underlying().coord_;
//        int scale_offset = 0;
//        int scale_tmp    = 0;
//        int num_voxels   = BlockSize * BlockSize * BlockSize;
//
//        while(scale_tmp < scale)
//        {
//          scale_offset += num_voxels;
//          num_voxels /= 8;
//          ++scale_tmp;
//        }
//
//        const int size_at_scale = BlockSize / (1 << scale);
//        voxel_offset = voxel_offset / (1 << scale);
//        const int voxel_idx = scale_offset + voxel_offset.x() +
//                              voxel_offset.y() * size_at_scale +
//                              voxel_offset.z() * se::math::sq(size_at_scale)
//
//        DataUnion data_union;
//        data_union.data           = block_data_[voxel_idx];
//        data_union.delta_data     = block_delta_data_[voxel_idx];
//        data_union.data_idx       = voxel_idx;
//        data_union.delta_data_idx = voxel_idx;
//
//        return data_union;
//      };
//

//
//      se::propagator::propagateBlockDown(octree,
//                                         getDataUnion, setChildDataUnion,
//                                         getDataUnion, setParentDataUnion)
//    }
//    curr_scale_ = curr_scale;
//  }

//  std::union DataUnion
//  {
//    DataType      data;
//    DeltaDataType delta_data;
//    int           data_idx;
//    int           delta_data_idx;
//  }