#ifndef SE_NODE_HPP
#define SE_NODE_HPP

namespace se
{


template <typename NodeT>
inline void get_child_idx(const Eigen::Vector3i& voxel_coord,
                        NodeT*                 node_ptr,
                        unsigned int&          child_idx);



// Forward Declaration
template <typename DataT,
          typename  DerivedT
>
class NodeMultiRes
{
};



template <Field     FldT,
          Colour    ColB,
          Semantics SemB,
          typename  DerivedT
>
class NodeMultiRes<se::Data<FldT, ColB, SemB>, DerivedT>
{
};

template <Colour    ColB,
          Semantics SemB,
          typename  DerivedT
>
class NodeMultiRes<se::Data<se::Field::TSDF, ColB, SemB>, DerivedT>
{
public:
  typedef se::Data<se::Field::TSDF, ColB, SemB> DataType;
  typedef se::DeltaData<se::Field::TSDF, ColB, SemB> PropDataType;

  NodeMultiRes(const DataType)
  {
  }

  inline const DataType getData() const { return DataType(); }
  inline       DataType getData()       { return DataType(); }
};



template <Colour    ColB,
          Semantics SemB,
          typename  DerivedT
>
class NodeMultiRes<se::Data<se::Field::Occupancy, ColB, SemB>, DerivedT> {
public:
  typedef se::Data<se::Field::Occupancy, ColB, SemB> DataType;

  NodeMultiRes(const DataType init_data)
  {
    data_ = init_data;
  }

  inline const DataType getData() const { return (data_.observed && this->underlying().children_mask_ == 0) ? data_ : DataType(); }

  inline const DataType getMaxData() const { return data_; }

  inline void setData(const DataType& data) { data_ = data; }

protected:
  DataType data_;

private:
  DerivedT& underlying() { return static_cast<DerivedT&>(*this); }
  const DerivedT& underlying() const { return static_cast<const DerivedT&>(*this); }
};



template <typename DataT>
class NodeSingleRes
{
public:
  NodeSingleRes(const DataT)
  {
  }

  inline const DataT getData() const { return DataT(); }
  inline       DataT getData()       { return DataT(); }
};



template <typename DataT,
          se::Res  ResT = se::Res::Single
>
class Node : public OctantBase,
             public std::conditional<ResT == Res::Single, NodeSingleRes<DataT>, NodeMultiRes<DataT, Node<DataT, ResT>>>::type
{
public:
  typedef DataT DataType;

  Node(const Eigen::Vector3i& coord,
       const int              size);

  Node(Node*     parent_ptr,
       const int child_idx);

  inline int getSize() const;

  inline se::OctantBase* getChild(const unsigned child_idx);

  inline const se::OctantBase* getChild(const unsigned child_idx) const;

  inline se::OctantBase* setChild(const unsigned child_idx, se::OctantBase* child_ptr);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  std::array<se::OctantBase*, 8> children_ptr_; ///< Pointers to the eight children (should be all nullptr at initialisation due to smart pointers)
  const int                      size_;         ///< The size in [voxel] of the node in comparision to the finest voxel
};



} // namespace se



#endif //SE_NODE_HPP



#include "impl/node_impl.hpp"
