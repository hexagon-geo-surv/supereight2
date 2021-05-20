#ifndef SE_OCTANT_HPP
#define SE_OCTANT_HPP

#include <memory>
#include <Eigen/Dense>

#include "se/utils/setup_util.hpp"
#include "se/utils/math_util.hpp"

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

  template<typename DerT, typename DatT, int BS>
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


/**
 * \brief The base used for multi-resolution blocks
 */
template <typename DerivedT,
          typename DataT,
          int      BlockSize>
class BlockMultiRes
{
public:
  typedef DataT DataType;

  BlockMultiRes();

  inline void getData(const Eigen::Vector3i& voxel_coord,
                      DataType&              data);

  inline void getData(const Eigen::Vector3i& voxel_coord,
                      const int              scale,
                      DataType&              data);

  inline void setData(const Eigen::Vector3i& voxel_coord,
                      const DataType&        data);

  inline int getMinScale() const { return min_scale_; }

  inline int getCurrentScale() const { return curr_scale_; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  static constexpr int max_scale_ = math::log2_const(BlockSize);
  std::vector<DataType*> block_data_;
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
        BlockMultiRes<Block<DataT, ResT, BlockSize>, DataT, BlockSize>>::type, ///< Conditional CRTP
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
