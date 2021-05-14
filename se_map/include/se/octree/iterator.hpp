#ifndef SE_ITERATOR_HPP
#define SE_ITERATOR_HPP

#include <stack>

#include "se/octree/octree.hpp"



namespace se {

template <typename DerivedT>
struct BaseTraits;

/** \brief Iterates over all valid data in the octree at the last scale it
 * was updated at.
 * The iterator performs a depth-first traversal of the octree. To use it
 * just use the se::Octree::begin() and se::Octree::end() functions or a
 * range-based for loop:
 *
 * ``` cpp
 * for (auto& volume : octree) {
 *     // Do stuff with volume
 * }
 * ```
 *
 * \note Changes to the se::Octree while iterating will result in strange
 * behavior.
 */
template <typename DerivedT>
class BaseIterator {
public:
    typedef typename BaseTraits<DerivedT>::OctreeType OctreeType;
    typedef typename BaseTraits<DerivedT>::NodeType   NodeType;
    typedef typename BaseTraits<DerivedT>::BlockType  BlockType;

    BaseIterator();

    BaseIterator(OctreeType* octree_ptr);

    BaseIterator(const BaseIterator& other);

    BaseIterator& operator++();

    BaseIterator operator++(int);

    bool operator==(const BaseIterator& other) const;

    bool operator!=(const BaseIterator& other) const;

    se::OctantBase* operator*() const;

    // Iterator traits
    using difference_type   = long;
    using value_type        = se::OctantBase;
    using pointer           = se::OctantBase*;
    using reference         = se::OctantBase&;
    using iterator_category = std::forward_iterator_tag;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:

    DerivedT& underlying() { return static_cast<DerivedT&>(*this); }
    const DerivedT& underlying() const { return static_cast<const DerivedT&>(*this); }

    // Find the next Volume with valid data.
    void nextData();

    // Reset the iterator state to invalid. Used when finished iterating.
    void clear();

    // The 3 stacks should always be kept in sync.
    // Pointers to the Nodes that haven't been checked yet.
    std::stack<se::OctantBase*> octant_stack_;

    se::OctantBase* octant_;

    OctreeType* octree_ptr_ = nullptr;
};



template <typename OctreeT>
class OctreeIterator : public BaseIterator< OctreeIterator<OctreeT> > {
public:
    OctreeIterator() : BaseIterator< OctreeIterator<OctreeT> > () {};

    OctreeIterator(OctreeT* octree_ptr) : BaseIterator< OctreeIterator<OctreeT> > (octree_ptr) {};

    static constexpr bool has_ignore_condition = false;

    bool isNext(se::OctantBase* /* octant_ptr */) { return true; }

protected:
    friend class BaseIterator< OctreeIterator<OctreeT> >;
};



template <typename OctreeT>
class NodesIterator : public BaseIterator<NodesIterator<OctreeT> > {
public:
    NodesIterator() : BaseIterator< NodesIterator<OctreeT> > () {};

    NodesIterator(OctreeT* octree_ptr) : BaseIterator< NodesIterator<OctreeT> > (octree_ptr) {};

    static constexpr bool has_ignore_condition = false;

    bool isNext(se::OctantBase* octant_ptr) { return !octant_ptr->isBlock(); }

protected:
    friend class BaseIterator< NodesIterator<OctreeT> >;
};



template <typename OctreeT>
class BlocksIterator : public BaseIterator<BlocksIterator<OctreeT> > {
public:
    BlocksIterator() : BaseIterator< BlocksIterator<OctreeT> >() {};

    BlocksIterator(OctreeT* octree_ptr) : BaseIterator< BlocksIterator<OctreeT> > (octree_ptr) {};

    static constexpr bool has_ignore_condition = false;

    bool isNext(se::OctantBase* octant_ptr) { return octant_ptr->isBlock(); }

protected:
    friend class BaseIterator< BlocksIterator<OctreeT> >;
};



template <typename OctreeT>
class LeavesIterator : public BaseIterator<LeavesIterator<OctreeT> > {
public:
    LeavesIterator() : BaseIterator< LeavesIterator<OctreeT> > () {};

    LeavesIterator(OctreeT* octree_ptr) : BaseIterator< LeavesIterator<OctreeT> > (octree_ptr) {};

    static constexpr bool has_ignore_condition = false;

    bool isNext(se::OctantBase* octant_ptr) { return octant_ptr->getChildrenMask() == 0; }

protected:
    friend class BaseIterator< LeavesIterator<OctreeT> >;
};



template <typename OctreeT>
class UpdateIterator : public BaseIterator< UpdateIterator<OctreeT> > {
public:
    UpdateIterator() : BaseIterator< UpdateIterator<OctreeT> > (), time_stamp_(0) {};

    UpdateIterator(OctreeT*           octree_ptr,
                   const unsigned int time_stamp) : BaseIterator< UpdateIterator<OctreeT> > (octree_ptr), time_stamp_(time_stamp) {};


    static constexpr bool has_ignore_condition = true;

    bool isNext(se::OctantBase* octant_ptr) { return    octant_ptr->isBlock()
                                                     && octant_ptr->getTimeStamp() >= time_stamp_; }

    bool doIgnore(se::OctantBase* octant_ptr) { return octant_ptr->getTimeStamp() < time_stamp_; }

    const unsigned int time_stamp_;

protected:
    friend class BaseIterator< UpdateIterator<OctreeT> >;
};



// Declare and define a base_traits specialization for the octree iterator:
template <typename OctreeT>
struct BaseTraits<OctreeIterator<OctreeT>> {
  typedef          OctreeT            OctreeType;
  typedef typename OctreeT::NodeType  NodeType;
  typedef typename OctreeT::BlockType BlockType;
};



// Declare and define a base_traits specialization for the nodes iterator:
template <typename OctreeT>
struct BaseTraits<NodesIterator<OctreeT> > {
    typedef          OctreeT            OctreeType;
    typedef typename OctreeT::NodeType  NodeType;
    typedef typename OctreeT::BlockType BlockType;
};



// Declare and define a base_traits specialization for the block iterator:
template <typename OctreeT>
struct BaseTraits<BlocksIterator<OctreeT> > {
    typedef          OctreeT            OctreeType;
    typedef typename OctreeT::NodeType  NodeType;
    typedef typename OctreeT::BlockType BlockType;
};



// Declare and define a base_traits specialization for the leaves iterator:
template <typename OctreeT>
struct BaseTraits<LeavesIterator<OctreeT> > {
    typedef          OctreeT            OctreeType;
    typedef typename OctreeT::NodeType  NodeType;
    typedef typename OctreeT::BlockType BlockType;
};



// Declare and define a base_traits specialization for the update iterator:
template <typename OctreeT>
struct BaseTraits<UpdateIterator<OctreeT> > {
    typedef          OctreeT            OctreeType;
    typedef typename OctreeT::NodeType  NodeType;
    typedef typename OctreeT::BlockType BlockType;
};


} // namespace se

#include "impl/iterator_impl.hpp"

#endif // SE_ITERATOR_HPP
