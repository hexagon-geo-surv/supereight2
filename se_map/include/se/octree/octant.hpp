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

    OctantBase(const Eigen::Vector3i& coord,
               const OctantBase*      parent_ptr = nullptr);

    virtual bool isBlock() = 0;

    const Eigen::Vector3i& getCoord();

    const Eigen::Vector3i& getCoord() const;

    bool getParent(const OctantBase*& parent_ptr);

    bool getParent(const OctantBase*& parent_ptr) const;

    unsigned int getTimeStamp();

    unsigned int getTimeStamp() const;

    void setTimeStamp(const unsigned int time_stamp);

    unsigned char getChildrenMask();

    unsigned char getChildrenMask() const;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
    const OctantBase*     parent_ptr_;    ///< Every node/block (other than root) needs a parent
    const Eigen::Vector3i coord_;         ///< The coordinates of the block (left, front , bottom corner)
    unsigned int          time_stamp_;    ///< The frame of the last update
    unsigned char         children_mask_; ///< The allocated children
};



///////////////////////////
/// NODE IMPLEMENTATION ///
///////////////////////////

template <typename NodeT>
bool get_child_idx(const Eigen::Vector3i& voxel_coord,
                   NodeT*                 node_ptr,
                   unsigned int&          child_idx);

template <typename DerivedT>
class NodeBase : public OctantBase
{
public:
    NodeBase(const Eigen::Vector3i& coord,
             const unsigned         size,
             const OctantBase*      parent_ptr = nullptr);

    bool isBlock() { return false; }

    bool isBlock() const { return false; }

    unsigned int getSize();

    bool getChild(const unsigned child_idx, se::OctantBase*& child_ptr);

    bool setChild(const unsigned child_idx, se::OctantBase* child_ptr);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    std::array<se::OctantBase*, 8> children_ptr_; ///< Pointers to the eight children (should be all nullptr at initialisation due to smart pointers)
    const unsigned int             size_;         ///< The size in [voxel] of the node in comparision to the finest voxel
};

template <typename DataT, se::Res ResT = se::Res::Single>
class Node : public NodeBase< Node<DataT, ResT> >
{
public:
    typedef DataT DataType;

    Node(const Eigen::Vector3i& coord,
         const unsigned int     size);

    Node(Node*              parent_ptr,
         const unsigned int child_idx);

    bool getData(const DataT& data);

    bool setData(const DataT& data);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    DataType data_; ///< The data of the node
    template<typename DT, Res RT, unsigned ST, typename PT>
    friend class Block;
};



////////////////////////////
/// BLOCK IMPLEMENTATION ///
////////////////////////////

template <typename DerivedT, unsigned SizeT>
class BlockBase : public OctantBase {
public:
    BlockBase(const Eigen::Vector3i& coord,
              const se::OctantBase*  parent_ptr = nullptr);

    bool isBlock() { return true; }

    bool isBlock() const { return true; }

    static unsigned int getSize() { return SizeT; }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};



/**
 * \brief The base used for single-resolution blocks
 */
template <typename DerivedT, typename DataT, unsigned SizeT>
class BlockSingleRes
{
public:
    typedef DataT DataType;

    bool getData(const Eigen::Vector3i& voxel_coord,
                 DataType&              data);

    bool getData(const Eigen::Vector3i& voxel_coord,
                 DataType&              data) const;

    bool getData(const unsigned voxel_idx,
                 DataT&         data);

    bool getData(const unsigned voxel_idx,
                 DataT&         data) const;

    bool setData(const Eigen::Vector3i& voxel_coord,
                 const DataT&           data);

    bool setData(const unsigned voxel_idx,
                 const DataT&   data);

    static inline int getMinScale() { return min_scale_; }

    static inline int getCurrentScale() { return curr_scale_; }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    std::array<DataType, SizeT * SizeT * SizeT> block_data_;
    static constexpr int min_scale_ = 0;
    static constexpr int curr_scale_ = 0;

    DerivedT& underlying() { return static_cast<DerivedT&>(*this); }
    DerivedT const& underlying() const { return static_cast<DerivedT const&>(*this); }
};


/**
 * \brief The base used for multi-resolution blocks
 */
template <typename DerivedT, typename DataT, unsigned SizeT>
class BlockMultiRes
{
public:
    typedef DataT DataType;

    BlockMultiRes();

    bool getData(const Eigen::Vector3i& voxel_coord,
                 DataType&              data);

    bool getData(const Eigen::Vector3i& voxel_coord,
                 const unsigned         scale,
                 DataType&              data);

    bool setData(const Eigen::Vector3i& voxel_coord,
                 const DataType&        data);

    int getMinScale() { return min_scale_; }

    int getMinScale() const { return min_scale_; }

    int getCurrentScale() { return curr_scale_; }

    int getCurrentScale() const { return curr_scale_; }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    static constexpr int max_scale_ = math::log2_const(SizeT);
    std::vector<std::unique_ptr<DataType[]>> block_data_;
    int min_scale_;
    int curr_scale_;

    DerivedT& underlying() { return static_cast<DerivedT&>(*this); }
    DerivedT const& underlying() const { return static_cast<DerivedT const&>(*this); }
};

/**
 * \brief The actual block used in the tree.
 *
 */
template <typename DataT,
          Res      ResT    = Res::Single,
          unsigned SizeT   = 8,
          typename PolicyT = std::enable_if_t<math::is_power_of_two(SizeT)> ///< Verify that the block size is sensible
>
class Block : public std::conditional<ResT == Res::Single,
        BlockSingleRes<Block<DataT, ResT, SizeT>, DataT, SizeT>,
        BlockMultiRes<Block<DataT, ResT, SizeT>, DataT, SizeT>>::type, ///< Conditional CRTP
        public BlockBase< Block<DataT, ResT, SizeT>, SizeT >           ///< Base CRTP
{
public:
    typedef DataT DataType;
    static constexpr size_t size    = SizeT;
    static constexpr size_t size_qu = SizeT * SizeT;
    static constexpr size_t size_cu = SizeT * SizeT * SizeT;

    /**
     * \brief Initialise block via parent node
     *
     * \param parent        The shared pointer to the parent node
     * \param child_id      The child id {0,...,7} in relation to the parent
     * \param init_data     The initial data of the block
     */
    Block(se::Node<DataT, ResT>* parent_ptr = nullptr,
          const unsigned         child_id   = 0);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

} // namespace se

#include "impl/octant_impl.hpp"

#endif // SE_OCTANT_HPP