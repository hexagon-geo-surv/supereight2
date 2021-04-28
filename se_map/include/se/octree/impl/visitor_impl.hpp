#ifndef SE_VISITOR_IMPL_HPP
#define SE_VISITOR_IMPL_HPP

#include "se/octree/octant.hpp"
#include "se/octree/allocator.hpp"
#include "se/octree/fetcher.hpp"
#include "se/data.hpp"

namespace se {
namespace visitor {

namespace {

/*
* Interpolation's value gather offsets
*/
static const Eigen::Vector3i interp_offsets[8] =
        {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {1, 1, 0},
         {0, 0, 1}, {1, 0, 1}, {0, 1, 1}, {1, 1, 1}};



template <typename BlockT, typename DataT>
inline void gather_local(const BlockT*           block_ptr,
                         const Eigen::Vector3i&  base_coord,
                         DataT                   neighbour_data[8])
{
  block_ptr->getData(base_coord + interp_offsets[0], neighbour_data[0]);
  block_ptr->getData(base_coord + interp_offsets[1], neighbour_data[1]);
  block_ptr->getData(base_coord + interp_offsets[2], neighbour_data[2]);
  block_ptr->getData(base_coord + interp_offsets[3], neighbour_data[3]);
  block_ptr->getData(base_coord + interp_offsets[4], neighbour_data[4]);
  block_ptr->getData(base_coord + interp_offsets[5], neighbour_data[5]);
  block_ptr->getData(base_coord + interp_offsets[6], neighbour_data[6]);
  block_ptr->getData(base_coord + interp_offsets[7], neighbour_data[7]);
  return;
}


template <typename BlockT, typename DataT>
inline void gather_4(const BlockT*           block_ptr,
                     const Eigen::Vector3i&  base_coord,
                     const unsigned int      offsets[4],
                     DataT                   neighbour_data[8])
{
  block_ptr->getData(base_coord + interp_offsets[offsets[0]], neighbour_data[offsets[0]]);
  block_ptr->getData(base_coord + interp_offsets[offsets[1]], neighbour_data[offsets[1]]);
  block_ptr->getData(base_coord + interp_offsets[offsets[2]], neighbour_data[offsets[2]]);
  block_ptr->getData(base_coord + interp_offsets[offsets[3]], neighbour_data[offsets[3]]);
  return;

}



template <typename BlockT, typename DataT>
inline void gather_2(const BlockT*           block_ptr,
                     const Eigen::Vector3i&  base_coord,
                     const unsigned int      offsets[2],
                     DataT                   neighbour_data[8])
{
  block_ptr->getData(base_coord + interp_offsets[offsets[0]], neighbour_data[offsets[0]]);
  block_ptr->getData(base_coord + interp_offsets[offsets[1]], neighbour_data[offsets[1]]);
}



template <typename OctreeT>
inline bool get_neighbours(const std::shared_ptr<OctreeT> octree_ptr,
                           const Eigen::Vector3i&         base_coord,
                           typename OctreeT::DataType     neighbour_data[8])
{

  const int stride = 1; // TODO:
  unsigned int block_size = OctreeT::BlockType::getSize();
  unsigned int crossmask
          = (((base_coord.x() & (block_size - 1)) == block_size - stride) << 2)
          | (((base_coord.y() & (block_size - 1)) == block_size - stride) << 1)
          |  ((base_coord.z() & (block_size - 1)) == block_size - stride);

  switch(crossmask) {
    case 0: /* all local */
    {
      const typename OctreeT::BlockType* block_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(base_coord, octree_ptr, octree_ptr->getRoot()));
      if (!block_ptr)
      {
        return false;
      }
      gather_local(block_ptr, base_coord, neighbour_data);
    }
      break;
    case 1: /* z crosses */
    {
      const unsigned int offs1[4] = {0, 1, 2, 3};
      const unsigned int offs2[4] = {4, 5, 6, 7};

      typename OctreeT::BlockType* block_1_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(base_coord, octree_ptr, octree_ptr->getRoot()));
      if (!block_1_ptr)
      {
        return false;
      }

      const Eigen::Vector3i offset_coord = base_coord + stride * interp_offsets[offs2[0]];
      typename OctreeT::BlockType* block_2_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(offset_coord, octree_ptr, octree_ptr->getRoot()));
      if (!block_2_ptr)
      {
        return false;
      }

      gather_4(block_1_ptr, base_coord, offs1, neighbour_data);
      gather_4(block_2_ptr, base_coord, offs2, neighbour_data);
    }
      break;
    case 2: /* y crosses */
    {
      const unsigned int offs1[4] = {0, 1, 4, 5};
      const unsigned int offs2[4] = {2, 3, 6, 7};

      typename OctreeT::BlockType* block_1_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(base_coord, octree_ptr, octree_ptr->getRoot()));
      if (!block_1_ptr)
      {
        return false;
      }

      const Eigen::Vector3i offset_coord = base_coord + stride * interp_offsets[offs2[0]];
      typename OctreeT::BlockType* block_2_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(offset_coord, octree_ptr, octree_ptr->getRoot()));
      if (!block_2_ptr)
      {
        return false;
      }

      gather_4(block_1_ptr, base_coord, offs1, neighbour_data);
      gather_4(block_2_ptr, base_coord, offs2, neighbour_data);
    }
      break;
    case 3: /* y, z cross */
    {
      const unsigned int offs1[2] = {0, 1};
      const unsigned int offs2[2] = {2, 3};
      const unsigned int offs3[2] = {4, 5};
      const unsigned int offs4[2] = {6, 7};

      const Eigen::Vector3i base_2_coord = base_coord + interp_offsets[offs2[0]];
      const Eigen::Vector3i base_3_coord = base_coord + interp_offsets[offs3[0]];
      const Eigen::Vector3i base_4_coord = base_coord + interp_offsets[offs4[0]];

      typename OctreeT::BlockType* block_1_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(base_coord, octree_ptr, octree_ptr->getRoot()));
      if (!block_1_ptr) { return false; }

      typename OctreeT::BlockType* block_2_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(base_2_coord, octree_ptr, octree_ptr->getRoot()));
      if (!block_2_ptr) { return false; }

      typename OctreeT::BlockType* block_3_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(base_3_coord, octree_ptr, octree_ptr->getRoot()));
      if (!block_3_ptr) { return false; }

      typename OctreeT::BlockType* block_4_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(base_4_coord, octree_ptr, octree_ptr->getRoot()));
      if (!block_4_ptr) { return false; }

      gather_2(block_1_ptr, base_coord,   offs1, neighbour_data);
      gather_2(block_2_ptr, base_2_coord, offs2, neighbour_data);
      gather_2(block_4_ptr, base_4_coord, offs4, neighbour_data);
      gather_2(block_3_ptr, base_3_coord, offs3, neighbour_data);
    }
      break;
    case 4: /* x crosses */
    {
      const unsigned int offs1[4] = {0, 2, 4, 6};
      const unsigned int offs2[4] = {1, 3, 5, 7};

      typename OctreeT::BlockType* block_1_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(base_coord, octree_ptr, octree_ptr->getRoot()));
      if (!block_1_ptr)
      {
        return false;
      }

      const Eigen::Vector3i offset_coord = base_coord + stride * interp_offsets[offs2[0]];
      typename OctreeT::BlockType* block_2_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(offset_coord, octree_ptr, octree_ptr->getRoot()));
      if (!block_2_ptr)
      {
        return false;
      }

      gather_4(block_1_ptr, base_coord, offs1, neighbour_data);
      gather_4(block_2_ptr, base_coord, offs2, neighbour_data);
    }
      break;
    case 5: /* x,z cross */
    {
      const unsigned int offs1[2] = {0, 2};
      const unsigned int offs2[2] = {1, 3};
      const unsigned int offs3[2] = {4, 6};
      const unsigned int offs4[2] = {5, 7};

      const Eigen::Vector3i base_2_coord = base_coord + interp_offsets[offs2[0]];
      const Eigen::Vector3i base_3_coord = base_coord + interp_offsets[offs3[0]];
      const Eigen::Vector3i base_4_coord = base_coord + interp_offsets[offs4[0]];

      typename OctreeT::BlockType* block_1_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(base_coord, octree_ptr, octree_ptr->getRoot()));
      if (!block_1_ptr) { return false; }

      typename OctreeT::BlockType* block_2_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(base_2_coord, octree_ptr, octree_ptr->getRoot()));
      if (!block_2_ptr) { return false; }

      typename OctreeT::BlockType* block_3_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(base_3_coord, octree_ptr, octree_ptr->getRoot()));
      if (!block_3_ptr) { return false; }

      typename OctreeT::BlockType* block_4_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(base_4_coord, octree_ptr, octree_ptr->getRoot()));
      if (!block_4_ptr) { return false; }

      gather_2(block_1_ptr, base_coord,   offs1, neighbour_data);
      gather_2(block_2_ptr, base_2_coord, offs2, neighbour_data);
      gather_2(block_4_ptr, base_4_coord, offs4, neighbour_data);
      gather_2(block_3_ptr, base_3_coord, offs3, neighbour_data);
    }
      break;
    case 6: /* x,y cross */
    {
      const unsigned int offs1[2] = {0, 4};
      const unsigned int offs2[2] = {1, 5};
      const unsigned int offs3[2] = {2, 6};
      const unsigned int offs4[2] = {3, 7};

      const Eigen::Vector3i base_2_coord = base_coord + interp_offsets[offs2[0]];
      const Eigen::Vector3i base_3_coord = base_coord + interp_offsets[offs3[0]];
      const Eigen::Vector3i base_4_coord = base_coord + interp_offsets[offs4[0]];

      typename OctreeT::BlockType* block_1_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(base_coord, octree_ptr, octree_ptr->getRoot()));
      if (!block_1_ptr) { return false; }

      typename OctreeT::BlockType* block_2_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(base_2_coord, octree_ptr, octree_ptr->getRoot()));
      if (!block_2_ptr) { return false; }

      typename OctreeT::BlockType* block_3_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(base_3_coord, octree_ptr, octree_ptr->getRoot()));
      if (!block_3_ptr) { return false; }

      typename OctreeT::BlockType* block_4_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(base_4_coord, octree_ptr, octree_ptr->getRoot()));
      if (!block_4_ptr) { return false; }

      gather_2(block_1_ptr, base_coord,   offs1, neighbour_data);
      gather_2(block_2_ptr, base_2_coord, offs2, neighbour_data);
      gather_2(block_4_ptr, base_4_coord, offs4, neighbour_data);
      gather_2(block_3_ptr, base_3_coord, offs3, neighbour_data);
    }
      break;

    case 7: /* x, y, z cross */
    {
      Eigen::Vector3i voxels_coord[8];
      voxels_coord[0] = base_coord + interp_offsets[0];
      voxels_coord[1] = base_coord + interp_offsets[1];
      voxels_coord[2] = base_coord + interp_offsets[2];
      voxels_coord[3] = base_coord + interp_offsets[3];
      voxels_coord[4] = base_coord + interp_offsets[4];
      voxels_coord[5] = base_coord + interp_offsets[5];
      voxels_coord[6] = base_coord + interp_offsets[6];
      voxels_coord[7] = base_coord + interp_offsets[7];

      for (int i = 0; i < 8; ++i)
      {
        typename OctreeT::BlockType* block_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(voxels_coord[i], octree_ptr, octree_ptr->getRoot()));

        if (!block_ptr) { return false; }

        block_ptr->getData(voxels_coord[i], neighbour_data[i]);
      }
    }
      break;
  }
  return true;
}

}



template <typename OctreeT, typename FieldT>
bool interpField(const std::shared_ptr<OctreeT> octree_ptr,
                 const Eigen::Vector3f&         voxel_coord_f,
                 FieldT&                        interp_field_value)
{
  typename OctreeT::DataType init_data;
  typename OctreeT::DataType neighbour_data[8] = { init_data };
  
  const unsigned int octree_size = octree_ptr->getSize();
  
  Eigen::Vector3f factor;

  const int stride = 1; // Single-res
  const Eigen::Vector3f scaled_voxel_coord_f = 1.f / stride * voxel_coord_f - se::sample_offset_frac;
  factor = math::fracf(scaled_voxel_coord_f);
  const Eigen::Vector3i base_coord = stride * scaled_voxel_coord_f.template cast<int>();
  
  if ((base_coord.array() < 0).any() ||
      ((base_coord + Eigen::Vector3i::Constant(stride)).array() >= octree_size).any())
  {
    interp_field_value = se::get_field(init_data);
    return false;
  }
  
  get_neighbours(octree_ptr, base_coord, neighbour_data);
  
  for (int n = 0; n < 8; n++) //< 8 neighbours
  {
    if (se::is_invalid(neighbour_data[n]))
    {
      interp_field_value = se::get_field(init_data);
      return false;
    }
  }

  interp_field_value =  (((se::get_field(neighbour_data[0]) * (1 - factor.x())
                         + se::get_field(neighbour_data[1]) * factor.x()) * (1 - factor.y())
                        + (se::get_field(neighbour_data[2]) * (1 - factor.x())
                         + se::get_field(neighbour_data[3]) * factor.x()) * factor.y()) * (1 - factor.z())
                       + ((se::get_field(neighbour_data[4]) * (1 - factor.x())
                         + se::get_field(neighbour_data[5]) * factor.x()) * (1 - factor.y())
                        + (se::get_field(neighbour_data[6]) * (1 - factor.x())
                         + se::get_field(neighbour_data[7]) * factor.x()) * factor.y()) * factor.z());

  // Interpolate the value based on the fractional part.
  return true;
}

template <typename OctreeT>
bool getData(std::shared_ptr<OctreeT>     octree_ptr,
             const Eigen::Vector3i&       voxel_coord,
             typename OctreeT::DataType&  data)
{

  se::OctantBase* octant_ptr = octree_ptr->getRoot();
  se::OctantBase* octant_tmp_ptr = nullptr;
  if(!octant_ptr) // Return false if the octree root isn't allocated, this should never be the case.
  {
    std::cerr << "Re-initalise octree. Root missing" << std::endl;
    return false;
  }

  unsigned int child_size = octree_ptr->getSize() >> 1;
  for (; child_size >= OctreeT::block_size; child_size = child_size >> 1)
  {
    typename OctreeT::NodeType* node_ptr = static_cast<typename OctreeT::NodeType*>(octant_ptr);

    unsigned int child_idx;
    get_child_idx(voxel_coord, node_ptr, child_idx);
    if (!node_ptr->getChild(child_idx, octant_tmp_ptr)) // Return false if the voxel is not allocated
    {
      return false;
    }
    octant_ptr = octant_tmp_ptr;
  }

  static_cast<typename OctreeT::BlockType*>(octant_ptr)->getData(voxel_coord, data);

  return true;
}

} // namespace visitor
} // namespace se

#endif // SE_VISITOR_IMPL_HPP
