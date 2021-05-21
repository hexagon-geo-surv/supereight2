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
  neighbour_data[0] = block_ptr->getData(base_coord + interp_offsets[0]);
  neighbour_data[1] = block_ptr->getData(base_coord + interp_offsets[1]);
  neighbour_data[2] = block_ptr->getData(base_coord + interp_offsets[2]);
  neighbour_data[3] = block_ptr->getData(base_coord + interp_offsets[3]);
  neighbour_data[4] = block_ptr->getData(base_coord + interp_offsets[4]);
  neighbour_data[5] = block_ptr->getData(base_coord + interp_offsets[5]);
  neighbour_data[6] = block_ptr->getData(base_coord + interp_offsets[6]);
  neighbour_data[7] = block_ptr->getData(base_coord + interp_offsets[7]);
}


template <typename BlockT, typename DataT>
inline void gather_4(const BlockT*           block_ptr,
                     const Eigen::Vector3i&  base_coord,
                     const unsigned int      offsets[4],
                     DataT                   neighbour_data[8])
{
  neighbour_data[offsets[0]] = block_ptr->getData(base_coord + interp_offsets[offsets[0]]);
  neighbour_data[offsets[1]] = block_ptr->getData(base_coord + interp_offsets[offsets[1]]);
  neighbour_data[offsets[2]] = block_ptr->getData(base_coord + interp_offsets[offsets[2]]);
  neighbour_data[offsets[3]] = block_ptr->getData(base_coord + interp_offsets[offsets[3]]);
}



template <typename BlockT, typename DataT>
inline void gather_2(const BlockT*           block_ptr,
                     const Eigen::Vector3i&  base_coord,
                     const unsigned int      offsets[2],
                     DataT                   neighbour_data[8])
{
  neighbour_data[offsets[0]] = block_ptr->getData(base_coord + interp_offsets[offsets[0]]);
  neighbour_data[offsets[1]] = block_ptr->getData(base_coord + interp_offsets[offsets[1]]);
}



template <typename OctreeT>
inline bool get_neighbours(const OctreeT&             octree,
                           const Eigen::Vector3i&     base_coord,
                           typename OctreeT::DataType neighbour_data[8])
{
  unsigned int stride = 1;
  unsigned int block_size = OctreeT::BlockType::getSize();
  unsigned int crossmask
          = (((base_coord.x() & (block_size - 1)) == block_size - stride) << 2)
          | (((base_coord.y() & (block_size - 1)) == block_size - stride) << 1)
          |  ((base_coord.z() & (block_size - 1)) == block_size - stride);

  switch(crossmask) {
    case 0: /* all local */
    {
      const typename OctreeT::BlockType* block_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(base_coord, octree, octree.getRoot()));
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

      typename OctreeT::BlockType* block_1_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(base_coord, octree, octree.getRoot()));
      if (!block_1_ptr)
      {
        return false;
      }

      typename OctreeT::BlockType* block_2_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(base_coord + interp_offsets[offs2[0]], octree, octree.getRoot()));
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

      typename OctreeT::BlockType* block_1_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(base_coord, octree, octree.getRoot()));
      if (!block_1_ptr)
      {
        return false;
      }

      typename OctreeT::BlockType* block_2_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(base_coord + interp_offsets[offs2[0]], octree, octree.getRoot()));
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

      typename OctreeT::BlockType* block_1_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(base_coord, octree, octree.getRoot()));
      if (!block_1_ptr) { return false; }

      typename OctreeT::BlockType* block_2_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(base_coord + interp_offsets[offs2[0]], octree, octree.getRoot()));
      if (!block_2_ptr) { return false; }

      typename OctreeT::BlockType* block_3_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(base_coord + interp_offsets[offs3[0]], octree, octree.getRoot()));
      if (!block_3_ptr) { return false; }

      typename OctreeT::BlockType* block_4_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(base_coord + interp_offsets[offs4[0]], octree, octree.getRoot()));
      if (!block_4_ptr) { return false; }

      gather_2(block_1_ptr, base_coord, offs1, neighbour_data);
      gather_2(block_2_ptr, base_coord, offs2, neighbour_data);
      gather_2(block_4_ptr, base_coord, offs4, neighbour_data);
      gather_2(block_3_ptr, base_coord, offs3, neighbour_data);
    }
      break;
    case 4: /* x crosses */
    {
      const unsigned int offs1[4] = {0, 2, 4, 6};
      const unsigned int offs2[4] = {1, 3, 5, 7};

      typename OctreeT::BlockType* block_1_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(base_coord, octree, octree.getRoot()));
      if (!block_1_ptr)
      {
        return false;
      }

      typename OctreeT::BlockType* block_2_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(base_coord + interp_offsets[offs2[0]], octree, octree.getRoot()));
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

      typename OctreeT::BlockType* block_1_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(base_coord, octree, octree.getRoot()));
      if (!block_1_ptr) { return false; }

      typename OctreeT::BlockType* block_2_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(base_coord + interp_offsets[offs2[0]], octree, octree.getRoot()));
      if (!block_2_ptr) { return false; }

      typename OctreeT::BlockType* block_3_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(base_coord + interp_offsets[offs3[0]], octree, octree.getRoot()));
      if (!block_3_ptr) { return false; }

      typename OctreeT::BlockType* block_4_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(base_coord + interp_offsets[offs4[0]], octree, octree.getRoot()));
      if (!block_4_ptr) { return false; }

      gather_2(block_1_ptr, base_coord, offs1, neighbour_data);
      gather_2(block_2_ptr, base_coord, offs2, neighbour_data);
      gather_2(block_3_ptr, base_coord, offs3, neighbour_data);
      gather_2(block_4_ptr, base_coord, offs4, neighbour_data);
    }
      break;
    case 6: /* x,y cross */
    {
      const unsigned int offs1[2] = {0, 4};
      const unsigned int offs2[2] = {1, 5};
      const unsigned int offs3[2] = {2, 6};
      const unsigned int offs4[2] = {3, 7};

      typename OctreeT::BlockType* block_1_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(base_coord, octree, octree.getRoot()));
      if (!block_1_ptr) { return false; }

      typename OctreeT::BlockType* block_2_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(base_coord + interp_offsets[offs2[0]], octree, octree.getRoot()));
      if (!block_2_ptr) { return false; }

      typename OctreeT::BlockType* block_3_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(base_coord + interp_offsets[offs3[0]], octree, octree.getRoot()));
      if (!block_3_ptr) { return false; }

      typename OctreeT::BlockType* block_4_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(base_coord + interp_offsets[offs4[0]], octree, octree.getRoot()));
      if (!block_4_ptr) { return false; }

      gather_2(block_1_ptr, base_coord, offs1, neighbour_data);
      gather_2(block_2_ptr, base_coord, offs2, neighbour_data);
      gather_2(block_3_ptr, base_coord, offs3, neighbour_data);
      gather_2(block_4_ptr, base_coord, offs4, neighbour_data);
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
        typename OctreeT::BlockType* block_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(voxels_coord[i], octree, octree.getRoot()));

        if (!block_ptr) { return false; }

        neighbour_data[i] = block_ptr->getData(voxels_coord[i]);
      }
    }
      break;
  }
  return true;
}


/////////////////////////////////
/// Multi-res value gathering ///
/////////////////////////////////

template <typename BlockT, typename DataT>
inline void gather_local(const BlockT*           block_ptr,
                         const Eigen::Vector3i&  base_coord,
                         const int               scale,
                         DataT                   neighbour_data[8])
{
  const int stride = 1 << scale;
  neighbour_data[0] = block_ptr->getData(base_coord + stride * interp_offsets[0], scale);
  neighbour_data[1] = block_ptr->getData(base_coord + stride * interp_offsets[1], scale);
  neighbour_data[2] = block_ptr->getData(base_coord + stride * interp_offsets[2], scale);
  neighbour_data[3] = block_ptr->getData(base_coord + stride * interp_offsets[3], scale);
  neighbour_data[4] = block_ptr->getData(base_coord + stride * interp_offsets[4], scale);
  neighbour_data[5] = block_ptr->getData(base_coord + stride * interp_offsets[5], scale);
  neighbour_data[6] = block_ptr->getData(base_coord + stride * interp_offsets[6], scale);
  neighbour_data[7] = block_ptr->getData(base_coord + stride * interp_offsets[7], scale);
  return;
}



template <typename BlockT, typename DataT>
inline void gather_4(const BlockT*           block_ptr,
                     const Eigen::Vector3i&  base_coord,
                     const unsigned int      offsets[4],
                     const int               scale,
                     DataT                   neighbour_data[8])
{
  const int stride = 1 << scale;
  neighbour_data[offsets[0]] = block_ptr->getData(base_coord + stride * interp_offsets[offsets[0]], scale);
  neighbour_data[offsets[1]] = block_ptr->getData(base_coord + stride * interp_offsets[offsets[1]], scale);
  neighbour_data[offsets[2]] = block_ptr->getData(base_coord + stride * interp_offsets[offsets[2]], scale);
  neighbour_data[offsets[3]] = block_ptr->getData(base_coord + stride * interp_offsets[offsets[3]], scale);
  return;

}



template <typename BlockT, typename DataT>
inline void gather_2(const BlockT*           block_ptr,
                     const Eigen::Vector3i&  base_coord,
                     const unsigned int      offsets[2],
                     const int               scale,
                     DataT                   neighbour_data[8])
{
  const int stride = 1 << scale;
  neighbour_data[offsets[0]] = block_ptr->getData(base_coord + stride * interp_offsets[offsets[0]], scale);
  neighbour_data[offsets[1]] = block_ptr->getData(base_coord + stride * interp_offsets[offsets[1]], scale);
}



template <typename OctreeT>
inline bool get_neighbours(const OctreeT&             octree,
                           const Eigen::Vector3i&     base_coord,
                           const int                  scale,
                           typename OctreeT::DataType neighbour_data[8])
{
  unsigned int stride = 1 << scale; // Multi-res
  unsigned int block_size = OctreeT::BlockType::getSize();
  unsigned int crossmask
          = (((base_coord.x() & (block_size - 1)) == block_size - stride) << 2)
          | (((base_coord.y() & (block_size - 1)) == block_size - stride) << 1)
          |  ((base_coord.z() & (block_size - 1)) == block_size - stride);

  switch(crossmask) {
    case 0: /* all local */
    {
      const typename OctreeT::BlockType* block_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(base_coord, octree, octree.getRoot()));
      if (!block_ptr || block_ptr->getCurrentScale() > scale) { return false; }
      gather_local(block_ptr, base_coord, scale, neighbour_data);
    }
      break;
    case 1: /* z crosses */
    {
      const unsigned int offs1[4] = {0, 1, 2, 3};
      const unsigned int offs2[4] = {4, 5, 6, 7};

      typename OctreeT::BlockType* block_1_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(base_coord, octree, octree.getRoot()));
      if (!block_1_ptr || block_1_ptr->getCurrentScale() > scale) { return false; }

      typename OctreeT::BlockType* block_2_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(base_coord + interp_offsets[offs2[0]], octree, octree.getRoot()));
      if (!block_2_ptr || block_2_ptr->getCurrentScale() > scale) { return false; }

      gather_4(block_1_ptr, base_coord, offs1, scale, neighbour_data);
      gather_4(block_2_ptr, base_coord, offs2, scale, neighbour_data);
    }
      break;
    case 2: /* y crosses */
    {
      const unsigned int offs1[4] = {0, 1, 4, 5};
      const unsigned int offs2[4] = {2, 3, 6, 7};

      typename OctreeT::BlockType* block_1_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(base_coord, octree, octree.getRoot()));
      if (!block_1_ptr || block_1_ptr->getCurrentScale() > scale) { return false; }

      typename OctreeT::BlockType* block_2_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(base_coord + interp_offsets[offs2[0]], octree, octree.getRoot()));
      if (!block_2_ptr || block_2_ptr->getCurrentScale() > scale) { return false; }

      gather_4(block_1_ptr, base_coord, offs1, scale, neighbour_data);
      gather_4(block_2_ptr, base_coord, offs2, scale, neighbour_data);
    }
      break;
    case 3: /* y, z cross */
    {
      const unsigned int offs1[2] = {0, 1};
      const unsigned int offs2[2] = {2, 3};
      const unsigned int offs3[2] = {4, 5};
      const unsigned int offs4[2] = {6, 7};

      typename OctreeT::BlockType* block_1_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(base_coord, octree, octree.getRoot()));
      if (!block_1_ptr || block_1_ptr->getCurrentScale() > scale) { return false; }

      typename OctreeT::BlockType* block_2_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(base_coord + interp_offsets[offs2[0]], octree, octree.getRoot()));
      if (!block_2_ptr || block_2_ptr->getCurrentScale() > scale) { return false; }

      typename OctreeT::BlockType* block_3_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(base_coord + interp_offsets[offs3[0]], octree, octree.getRoot()));
      if (!block_3_ptr || block_3_ptr->getCurrentScale() > scale) { return false; }

      typename OctreeT::BlockType* block_4_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(base_coord + interp_offsets[offs4[0]], octree, octree.getRoot()));
      if (!block_4_ptr || block_4_ptr->getCurrentScale() > scale) { return false; }

      gather_2(block_1_ptr, base_coord, offs1, scale, neighbour_data);
      gather_2(block_2_ptr, base_coord, offs2, scale, neighbour_data);
      gather_2(block_4_ptr, base_coord, offs4, scale, neighbour_data);
      gather_2(block_3_ptr, base_coord, offs3, scale, neighbour_data);
    }
      break;
    case 4: /* x crosses */
    {
      const unsigned int offs1[4] = {0, 2, 4, 6};
      const unsigned int offs2[4] = {1, 3, 5, 7};

      typename OctreeT::BlockType* block_1_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(base_coord, octree, octree.getRoot()));
      if (!block_1_ptr || block_1_ptr->getCurrentScale() > scale) { return false; }

      typename OctreeT::BlockType* block_2_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(base_coord + interp_offsets[offs2[0]], octree, octree.getRoot()));
      if (!block_2_ptr || block_2_ptr->getCurrentScale() > scale) { return false; }

      gather_4(block_1_ptr, base_coord, offs1, scale, neighbour_data);
      gather_4(block_2_ptr, base_coord, offs2, scale, neighbour_data);
    }
      break;
    case 5: /* x,z cross */
    {
      const unsigned int offs1[2] = {0, 2};
      const unsigned int offs2[2] = {1, 3};
      const unsigned int offs3[2] = {4, 6};
      const unsigned int offs4[2] = {5, 7};

      typename OctreeT::BlockType* block_1_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(base_coord, octree, octree.getRoot()));
      if (!block_1_ptr || block_1_ptr->getCurrentScale() > scale) { return false; }

      typename OctreeT::BlockType* block_2_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(base_coord + interp_offsets[offs2[0]], octree, octree.getRoot()));
      if (!block_2_ptr || block_2_ptr->getCurrentScale() > scale) { return false; }

      typename OctreeT::BlockType* block_3_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(base_coord + interp_offsets[offs3[0]], octree, octree.getRoot()));
      if (!block_3_ptr || block_3_ptr->getCurrentScale() > scale) { return false; }

      typename OctreeT::BlockType* block_4_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(base_coord + interp_offsets[offs4[0]], octree, octree.getRoot()));
      if (!block_4_ptr || block_4_ptr->getCurrentScale() > scale) { return false; }

      gather_2(block_1_ptr, base_coord, offs1, scale, neighbour_data);
      gather_2(block_2_ptr, base_coord, offs2, scale, neighbour_data);
      gather_2(block_3_ptr, base_coord, offs3, scale, neighbour_data);
      gather_2(block_4_ptr, base_coord, offs4, scale, neighbour_data);
    }
      break;
    case 6: /* x,y cross */
    {
      const unsigned int offs1[2] = {0, 4};
      const unsigned int offs2[2] = {1, 5};
      const unsigned int offs3[2] = {2, 6};
      const unsigned int offs4[2] = {3, 7};

      typename OctreeT::BlockType* block_1_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(base_coord, octree, octree.getRoot()));
      if (!block_1_ptr || block_1_ptr->getCurrentScale() > scale) { return false; }

      typename OctreeT::BlockType* block_2_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(base_coord + interp_offsets[offs2[0]], octree, octree.getRoot()));
      if (!block_2_ptr || block_2_ptr->getCurrentScale() > scale) { return false; }

      typename OctreeT::BlockType* block_3_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(base_coord + interp_offsets[offs3[0]], octree, octree.getRoot()));
      if (!block_3_ptr || block_3_ptr->getCurrentScale() > scale) { return false; }

      typename OctreeT::BlockType* block_4_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(base_coord + interp_offsets[offs4[0]], octree, octree.getRoot()));
      if (!block_4_ptr || block_4_ptr->getCurrentScale() > scale) { return false; }

      gather_2(block_1_ptr, base_coord, offs1, scale, neighbour_data);
      gather_2(block_2_ptr, base_coord, offs2, scale, neighbour_data);
      gather_2(block_3_ptr, base_coord, offs3, scale, neighbour_data);
      gather_2(block_4_ptr, base_coord, offs4, scale, neighbour_data);
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
        typename OctreeT::BlockType* block_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(voxels_coord[i], octree, octree.getRoot()));

        if (!block_ptr || block_ptr->getCurrentScale() > scale) { return false; }

        neighbour_data[i] = block_ptr->getData(voxels_coord[i], scale);
      }
    }
      break;
  }
  return true;
}

}



template <typename OctreeT>
inline typename std::enable_if_t<OctreeT::res_ == se::Res::Single, typename OctreeT::DataType>
getData(const OctreeT&         octree,
        const Eigen::Vector3i& voxel_coord)
{
  typename OctreeT::DataType data;

  const se::OctantBase* octant_ptr = se::fetcher::block(voxel_coord, octree, octree.getRoot());

  if (!octant_ptr)
  {
    return typename OctreeT::DataType(); // return init data
  }

  return static_cast<const typename OctreeT::BlockType*>(octant_ptr)->getData(voxel_coord);
}



template <typename OctreeT, typename BlockT>
inline typename std::enable_if_t<OctreeT::res_ == se::Res::Single, typename OctreeT::DataType>
getData(const OctreeT&         octree,
        BlockT*                block_ptr,
        const Eigen::Vector3i& voxel_coord)
{
  assert(block_ptr);

  const Eigen::Vector3i lower_coord = block_ptr->getCoord();
  const Eigen::Vector3i upper_coord = lower_coord + Eigen::Vector3i::Constant(BlockT::getSize() - 1);
  const bool is_contained = ((voxel_coord.array() >= lower_coord.array()) && (voxel_coord.array() <= upper_coord.array())).all();
  if (is_contained)
  {
    return block_ptr->getData(voxel_coord);
  }

  return se::visitor::getData(octree, voxel_coord);
}



template <typename OctreeT>
inline typename std::enable_if_t<OctreeT::res_ == se::Res::Multi, typename OctreeT::DataType>
getData(const OctreeT&         octree,
        const Eigen::Vector3i& voxel_coord)
{
  typename OctreeT::DataType data;

  const se::OctantBase* octant_ptr = se::fetcher::block(voxel_coord, octree, octree.getRoot());

  if (!octant_ptr)
  {
    return typename OctreeT::DataType(); // return init data
  }

  return static_cast<const typename OctreeT::BlockType*>(octant_ptr)->getData(voxel_coord);
}



template <typename OctreeT, typename BlockT>
inline typename std::enable_if_t<OctreeT::res_ == se::Res::Multi, typename OctreeT::DataType>
getData(const OctreeT&         octree,
        BlockT*                block_ptr,
        const Eigen::Vector3i& voxel_coord)
{
  assert(block_ptr);

  const Eigen::Vector3i lower_coord = block_ptr->getCoord();
  const Eigen::Vector3i upper_coord = lower_coord + Eigen::Vector3i::Constant(BlockT::getSize() - 1);
  const bool is_contained = ((voxel_coord.array() >= lower_coord.array()) && (voxel_coord.array() <= upper_coord.array())).all();
  if (is_contained)
  {
    return block_ptr->getData(voxel_coord);
  }

  return se::visitor::getData(octree, voxel_coord);
}



template <typename OctreeT>
inline typename std::enable_if_t<OctreeT::res_ == se::Res::Single, std::optional<se::field_t>>
getField(const OctreeT&         octree,
         const Eigen::Vector3i& voxel_coord)
{
  typename OctreeT::DataType data = getData(octree, voxel_coord);
  if (se::is_valid(data))
  {
    return se::get_field(data);
  }

  return {};
}



template <typename OctreeT, typename BlockT>
inline typename std::enable_if_t<OctreeT::res_ == se::Res::Single, std::optional<se::field_t>>
getField(const OctreeT&         octree,
         BlockT*                block_ptr,
         const Eigen::Vector3i& voxel_coord)
{
  typename OctreeT::DataType data = getData(octree, block_ptr, voxel_coord);
  if (se::is_valid(data))
  {
    return se::get_field(data);
  }

  return {};
}



template <typename OctreeT>
inline typename std::enable_if_t<OctreeT::res_ == se::Res::Multi, std::optional<se::field_t>>
getField(const OctreeT&         octree,
         const Eigen::Vector3i& voxel_coord)
{
  typename OctreeT::DataType data = getData(octree, voxel_coord);
  if (se::is_valid(data))
  {
    return se::get_field(data);
  }

  return {};
}



template <typename OctreeT, typename BlockT>
inline typename std::enable_if_t<OctreeT::res_ == se::Res::Multi, std::optional<se::field_t>>
getField(const OctreeT&         octree,
         BlockT*                block_ptr,
         const Eigen::Vector3i& voxel_coord)
{
  typename OctreeT::DataType data = getData(octree, block_ptr, voxel_coord);
  if (se::is_valid(data))
  {
    return se::get_field(data);
  }

  return {};
}



template <typename OctreeT>
inline typename std::enable_if_t<OctreeT::res_ == se::Res::Single, std::optional<se::field_t>>
getFieldInterp(const OctreeT&         octree,
               const Eigen::Vector3f& voxel_coord_f)
{
  typename OctreeT::DataType init_data;
  typename OctreeT::DataType neighbour_data[8] = {};

  const unsigned int octree_size = octree.getSize();

  Eigen::Vector3f factor;

  const int stride = 1; // Single-res
  const Eigen::Vector3f scaled_voxel_coord_f = 1.f / stride * voxel_coord_f - se::sample_offset_frac;
  factor = math::fracf(scaled_voxel_coord_f);
  const Eigen::Vector3i base_coord = stride * scaled_voxel_coord_f.template cast<int>();

  if ((base_coord.array() < 0).any() ||
      ((base_coord + Eigen::Vector3i::Constant(stride)).array() >= octree_size).any())
  {
    return {};
  }

  get_neighbours(octree, base_coord, neighbour_data);

  for (int n = 0; n < 8; n++) //< 8 neighbours
  {
    if (se::is_invalid(neighbour_data[n]))
    {
      return {};
    }
  }

  return (((se::get_field(neighbour_data[0]) * (1 - factor.x())
          + se::get_field(neighbour_data[1]) * factor.x()) * (1 - factor.y())
         + (se::get_field(neighbour_data[2]) * (1 - factor.x())
          + se::get_field(neighbour_data[3]) * factor.x()) * factor.y()) * (1 - factor.z())
        + ((se::get_field(neighbour_data[4]) * (1 - factor.x())
          + se::get_field(neighbour_data[5]) * factor.x()) * (1 - factor.y())
         + (se::get_field(neighbour_data[6]) * (1 - factor.x())
          + se::get_field(neighbour_data[7]) * factor.x()) * factor.y()) * factor.z());
}



template <typename OctreeT>
inline typename std::enable_if_t<OctreeT::res_ == se::Res::Multi, std::optional<se::field_t>>
getFieldInterp(const OctreeT&         octree,
               const Eigen::Vector3f& voxel_coord_f,
               const int              scale_desired,
               int&                   scale_returned)
{
  typename OctreeT::DataType init_data;
  typename OctreeT::DataType neighbour_data[8] = {};

  const unsigned int octree_size = octree.getSize();

  se::OctantBase* octant_ptr = se::fetcher::block(voxel_coord_f.cast<int>(), octree, octree.getRoot());
  if (!octant_ptr)
  {
    return {};
  }

  typedef typename OctreeT::BlockType BlockType;
  BlockType* block_ptr = static_cast<BlockType*>(octant_ptr);
  const int init_scale = std::max(block_ptr->getCurrentScale(), scale_desired);

  for (int scale = init_scale; scale <= BlockType::getMaxScale(); scale++)
  {
    // Reset the neighbours. Assigning {} only works during initialization
    std::fill(std::begin(neighbour_data), std::end(neighbour_data), init_data);
    scale_returned = scale;

    Eigen::Vector3f factor;
    const int stride = 1 << scale; // Multi-res
    const Eigen::Vector3f scaled_voxel_coord_f = 1.f / stride * voxel_coord_f - se::sample_offset_frac;
    factor = math::fracf(scaled_voxel_coord_f);
    const Eigen::Vector3i base_coord = stride * scaled_voxel_coord_f.template cast<int>();

    if ((base_coord.array() < 0).any() ||
        ((base_coord + Eigen::Vector3i::Constant(stride)).array() >= octree_size).any())
    {
      return {};
    }

    if (!get_neighbours(octree, base_coord, scale, neighbour_data))
    {
      continue;
    }

    for (int n = 0; n < 8; n++) //< 8 neighbours
    {
      if (se::is_invalid(neighbour_data[n]))
      {
        return {};
      }
    }

    return  (((se::get_field(neighbour_data[0]) * (1 - factor.x())
             + se::get_field(neighbour_data[1]) * factor.x()) * (1 - factor.y())
            + (se::get_field(neighbour_data[2]) * (1 - factor.x())
             + se::get_field(neighbour_data[3]) * factor.x()) * factor.y()) * (1 - factor.z())
           + ((se::get_field(neighbour_data[4]) * (1 - factor.x())
             + se::get_field(neighbour_data[5]) * factor.x()) * (1 - factor.y())
            + (se::get_field(neighbour_data[6]) * (1 - factor.x())
             + se::get_field(neighbour_data[7]) * factor.x()) * factor.y()) * factor.z());
  }
  return {};
}



template <typename OctreeT>
inline typename std::enable_if_t<OctreeT::res_ == se::Res::Multi, std::optional<se::field_t>>
getFieldInterp(const OctreeT&         octree,
               const Eigen::Vector3f& voxel_coord_f,
               int&                   scale_returned)
{
  return getFieldInterp(octree, voxel_coord_f, 0, scale_returned);
}



template <typename OctreeT>
inline typename std::enable_if_t<OctreeT::res_ == se::Res::Multi, std::optional<se::field_t>>
getFieldInterp(const OctreeT&         octree,
               const Eigen::Vector3f& voxel_coord_f)
{
  int scale_dummy;
  return getFieldInterp(octree, voxel_coord_f, 0, scale_dummy);
}



template <typename OctreeT>
inline typename std::enable_if_t<OctreeT::res_ == se::Res::Single, std::optional<se::field_vec_t>>
getFieldGrad(const OctreeT&         octree,
             const Eigen::Vector3f& voxel_coord_f)
{
  const Eigen::Vector3f scaled_voxel_coord_f = voxel_coord_f - se::sample_offset_frac;
  Eigen::Vector3f factor = se::math::fracf(scaled_voxel_coord_f);
  const Eigen::Vector3i base_coord = scaled_voxel_coord_f.template cast<int>();

  Eigen::Vector3i lower_lower_coord = (base_coord - Eigen::Vector3i::Constant(1)).cwiseMax(Eigen::Vector3i::Constant(0));
  Eigen::Vector3i lower_upper_coord = base_coord.cwiseMax(Eigen::Vector3i::Constant(0));

  Eigen::Vector3i upper_lower_coord = (base_coord + Eigen::Vector3i::Constant(1)).cwiseMin(
          Eigen::Vector3i::Constant(octree.getSize()) - Eigen::Vector3i::Constant(1));
  Eigen::Vector3i upper_upper_coord = (base_coord + Eigen::Vector3i::Constant(2)).cwiseMin(
          Eigen::Vector3i::Constant(octree.getSize()) - Eigen::Vector3i::Constant(1));

  const typename OctreeT::BlockType* block_ptr = static_cast<typename OctreeT::BlockType*>(se::fetcher::block(base_coord, octree, octree.getRoot()));
  if (!block_ptr)
  {
    return {};
  }

  const Eigen::Vector3i grad_coords[32] =
  {
    Eigen::Vector3i(lower_lower_coord.x(), lower_upper_coord.y(), lower_upper_coord.z()), //< Unique
    Eigen::Vector3i(lower_lower_coord.x(), upper_lower_coord.y(), lower_upper_coord.z()), //< Unique
    Eigen::Vector3i(lower_lower_coord.x(), lower_upper_coord.y(), upper_lower_coord.z()), //< Unique
    Eigen::Vector3i(lower_lower_coord.x(), upper_lower_coord.y(), upper_lower_coord.z()), //< Unique

    Eigen::Vector3i(lower_upper_coord.x(), lower_lower_coord.y(), lower_upper_coord.z()), //< Unique
    Eigen::Vector3i(lower_upper_coord.x(), lower_lower_coord.y(), upper_lower_coord.z()), //< Unique
    Eigen::Vector3i(lower_upper_coord.x(), lower_upper_coord.y(), lower_lower_coord.z()), //< Unique
    Eigen::Vector3i(lower_upper_coord.x(), lower_upper_coord.y(), lower_upper_coord.z()), //< Non-unique 3x
    Eigen::Vector3i(lower_upper_coord.x(), lower_upper_coord.y(), upper_lower_coord.z()), //< Non-unique 3x
    Eigen::Vector3i(lower_upper_coord.x(), lower_upper_coord.y(), upper_upper_coord.z()), //< Unique
    Eigen::Vector3i(lower_upper_coord.x(), upper_lower_coord.y(), lower_lower_coord.z()), //< Unique
    Eigen::Vector3i(lower_upper_coord.x(), upper_lower_coord.y(), lower_upper_coord.z()), //< Non-unique 3x
    Eigen::Vector3i(lower_upper_coord.x(), upper_lower_coord.y(), upper_lower_coord.z()), //< Non-unique 3x
    Eigen::Vector3i(lower_upper_coord.x(), upper_lower_coord.y(), upper_upper_coord.z()), //< Unique
    Eigen::Vector3i(lower_upper_coord.x(), upper_upper_coord.y(), lower_upper_coord.z()), //< Unique
    Eigen::Vector3i(lower_upper_coord.x(), upper_upper_coord.y(), upper_lower_coord.z()), //< Unique

    Eigen::Vector3i(upper_lower_coord.x(), lower_lower_coord.y(), lower_upper_coord.z()), //< Unique
    Eigen::Vector3i(upper_lower_coord.x(), lower_lower_coord.y(), upper_lower_coord.z()), //< Unique
    Eigen::Vector3i(upper_lower_coord.x(), lower_upper_coord.y(), lower_lower_coord.z()), //< Unique
    Eigen::Vector3i(upper_lower_coord.x(), lower_upper_coord.y(), lower_upper_coord.z()), //< Non-unique 3x
    Eigen::Vector3i(upper_lower_coord.x(), lower_upper_coord.y(), upper_lower_coord.z()), //< Non-unique 3x
    Eigen::Vector3i(upper_lower_coord.x(), lower_upper_coord.y(), upper_upper_coord.z()), //< Unique
    Eigen::Vector3i(upper_lower_coord.x(), upper_lower_coord.y(), lower_lower_coord.z()), //< Unique
    Eigen::Vector3i(upper_lower_coord.x(), upper_lower_coord.y(), lower_upper_coord.z()), //< Non-unique 3x
    Eigen::Vector3i(upper_lower_coord.x(), upper_lower_coord.y(), upper_lower_coord.z()), //< Non-unique 3x
    Eigen::Vector3i(upper_lower_coord.x(), upper_lower_coord.y(), upper_upper_coord.z()), //< Unique
    Eigen::Vector3i(upper_lower_coord.x(), upper_upper_coord.y(), lower_upper_coord.z()), //< Unique
    Eigen::Vector3i(upper_lower_coord.x(), upper_upper_coord.y(), upper_lower_coord.z()), //< Unique

    Eigen::Vector3i(upper_upper_coord.x(), lower_upper_coord.y(), lower_upper_coord.z()), //< Unique
    Eigen::Vector3i(upper_upper_coord.x(), upper_lower_coord.y(), lower_upper_coord.z()), //< Unique
    Eigen::Vector3i(upper_upper_coord.x(), lower_upper_coord.y(), upper_lower_coord.z()), //< Unique
    Eigen::Vector3i(upper_upper_coord.x(), upper_lower_coord.y(), upper_lower_coord.z())  //< Unique
  };

  se::field_t grad_field_values[32];

  for (unsigned int i = 0; i < 32; i++)
  {
    auto grad_field_value = se::visitor::getField(octree, block_ptr, grad_coords[i]);
    if (!grad_field_value)
    {
      return {};
    }
    grad_field_values[i] = *grad_field_value;
  }

  const float rev_factor_x = (1 - factor.x());
  const float rev_factor_y = (1 - factor.y());
  const float rev_factor_z = (1 - factor.z());

  se::field_vec_t gradient = Eigen::Vector3f::Constant(0);

  gradient.x() = (((grad_field_values[19]
                  - grad_field_values[0]) * rev_factor_x
                  +(grad_field_values[28]
                  - grad_field_values[7]) * factor.x()) * rev_factor_y
                + ((grad_field_values[23]
                  - grad_field_values[1]) * rev_factor_x
                  +(grad_field_values[29]
                  - grad_field_values[11]) * factor.x()) * factor.y()) * rev_factor_z
               + (((grad_field_values[20]
                  - grad_field_values[2]) * rev_factor_x
                  +(grad_field_values[30]
                  - grad_field_values[8]) * factor.x()) * rev_factor_y
                + ((grad_field_values[24]
                  - grad_field_values[3]) * rev_factor_x
                  +(grad_field_values[31]
                  - grad_field_values[12]) * factor.x()) * factor.y()) * factor.z();

  gradient.y() = (((grad_field_values[11]
                  - grad_field_values[4]) * rev_factor_x
                  +(grad_field_values[23]
                  - grad_field_values[16]) * factor.x()) * rev_factor_y
                + ((grad_field_values[14]
                  - grad_field_values[7]) * rev_factor_x
                  +(grad_field_values[26]
                  - grad_field_values[19]) * factor.x()) * factor.y()) * rev_factor_z
               + (((grad_field_values[12]
                  - grad_field_values[5]) * rev_factor_x
                  +(grad_field_values[24]
                  - grad_field_values[17]) * factor.x()) * rev_factor_y
                + ((grad_field_values[15]
                  - grad_field_values[8]) * rev_factor_x
                  +(grad_field_values[27]
                  - grad_field_values[20]) * factor.x()) * factor.y()) * factor.z();

  gradient.z() = (((grad_field_values[8]
                  - grad_field_values[6]) * rev_factor_x
                  +(grad_field_values[20]
                  - grad_field_values[18]) * factor.x()) * rev_factor_y
                + ((grad_field_values[12]
                  - grad_field_values[10]) * rev_factor_x
                  +(grad_field_values[24]
                  - grad_field_values[22]) * factor.x()) * factor.y()) * rev_factor_z
               + (((grad_field_values[9]
                  - grad_field_values[7]) * rev_factor_x
                  +(grad_field_values[21]
                  - grad_field_values[19]) * factor.x()) * rev_factor_y
                 +((grad_field_values[13]
                  - grad_field_values[11]) * rev_factor_x
                  +(grad_field_values[25]
                  - grad_field_values[23]) * factor.x()) * factor.y()) * factor.z();

  return 0.5f * gradient;
}



template <typename OctreeT>
inline typename std::enable_if_t<OctreeT::res_ == se::Res::Multi, std::optional<se::field_vec_t>>
getFieldGrad(const OctreeT&         octree,
             const Eigen::Vector3f& voxel_coord_f)
{
  typedef typename OctreeT::BlockType BlockType;

  BlockType* block_ptr = static_cast<BlockType*>(se::fetcher::block(voxel_coord_f.cast<int>(), octree, octree.getRoot()));
  if (!block_ptr) // If this block doesn't exist there's no way to compute a valid gradient
  {
    return {};
  }

  int init_scale = block_ptr->getCurrentScale(); // Get scale to start gradient computation from

  for (int scale = init_scale; scale <= BlockType::getMaxScale(); scale++)
  {
    const int stride = 1 << scale;
    const Eigen::Vector3f scaled_voxel_coord_f = 1.f / stride * voxel_coord_f - se::sample_offset_frac;
    const Eigen::Vector3f factor = math::fracf(scaled_voxel_coord_f);
    const Eigen::Vector3i base_coord = stride * scaled_voxel_coord_f.template cast<int>();
    Eigen::Vector3i lower_lower_coord = (base_coord - stride * Eigen::Vector3i::Constant(1)).cwiseMax(Eigen::Vector3i::Constant(0));
    Eigen::Vector3i lower_upper_coord = base_coord.cwiseMax(Eigen::Vector3i::Constant(0));
    Eigen::Vector3i upper_lower_coord = (base_coord + stride * Eigen::Vector3i::Constant(1)).cwiseMin(
            Eigen::Vector3i::Constant(octree.getSize()) - Eigen::Vector3i::Constant(1));
    Eigen::Vector3i upper_upper_coord = (base_coord + stride * Eigen::Vector3i::Constant(2)).cwiseMin(
            Eigen::Vector3i::Constant(octree.getSize()) - Eigen::Vector3i::Constant(1));

    block_ptr = static_cast<BlockType*>(se::fetcher::block(base_coord, octree, octree.getRoot()));
    if (!block_ptr) // If this block doesn't exist there's still a chance a gradient exist at a different scale
    {
      continue;
    }

    const Eigen::Vector3i grad_coords[32] =
    {
      Eigen::Vector3i(lower_lower_coord.x(), lower_upper_coord.y(), lower_upper_coord.z()), //< Unique
      Eigen::Vector3i(lower_lower_coord.x(), upper_lower_coord.y(), lower_upper_coord.z()), //< Unique
      Eigen::Vector3i(lower_lower_coord.x(), lower_upper_coord.y(), upper_lower_coord.z()), //< Unique
      Eigen::Vector3i(lower_lower_coord.x(), upper_lower_coord.y(), upper_lower_coord.z()), //< Unique

      Eigen::Vector3i(lower_upper_coord.x(), lower_lower_coord.y(), lower_upper_coord.z()), //< Unique
      Eigen::Vector3i(lower_upper_coord.x(), lower_lower_coord.y(), upper_lower_coord.z()), //< Unique
      Eigen::Vector3i(lower_upper_coord.x(), lower_upper_coord.y(), lower_lower_coord.z()), //< Unique
      Eigen::Vector3i(lower_upper_coord.x(), lower_upper_coord.y(), lower_upper_coord.z()), //< Non-unique 3x
      Eigen::Vector3i(lower_upper_coord.x(), lower_upper_coord.y(), upper_lower_coord.z()), //< Non-unique 3x
      Eigen::Vector3i(lower_upper_coord.x(), lower_upper_coord.y(), upper_upper_coord.z()), //< Unique
      Eigen::Vector3i(lower_upper_coord.x(), upper_lower_coord.y(), lower_lower_coord.z()), //< Unique
      Eigen::Vector3i(lower_upper_coord.x(), upper_lower_coord.y(), lower_upper_coord.z()), //< Non-unique 3x
      Eigen::Vector3i(lower_upper_coord.x(), upper_lower_coord.y(), upper_lower_coord.z()), //< Non-unique 3x
      Eigen::Vector3i(lower_upper_coord.x(), upper_lower_coord.y(), upper_upper_coord.z()), //< Unique
      Eigen::Vector3i(lower_upper_coord.x(), upper_upper_coord.y(), lower_upper_coord.z()), //< Unique
      Eigen::Vector3i(lower_upper_coord.x(), upper_upper_coord.y(), upper_lower_coord.z()), //< Unique

      Eigen::Vector3i(upper_lower_coord.x(), lower_lower_coord.y(), lower_upper_coord.z()), //< Unique
      Eigen::Vector3i(upper_lower_coord.x(), lower_lower_coord.y(), upper_lower_coord.z()), //< Unique
      Eigen::Vector3i(upper_lower_coord.x(), lower_upper_coord.y(), lower_lower_coord.z()), //< Unique
      Eigen::Vector3i(upper_lower_coord.x(), lower_upper_coord.y(), lower_upper_coord.z()), //< Non-unique 3x
      Eigen::Vector3i(upper_lower_coord.x(), lower_upper_coord.y(), upper_lower_coord.z()), //< Non-unique 3x
      Eigen::Vector3i(upper_lower_coord.x(), lower_upper_coord.y(), upper_upper_coord.z()), //< Unique
      Eigen::Vector3i(upper_lower_coord.x(), upper_lower_coord.y(), lower_lower_coord.z()), //< Unique
      Eigen::Vector3i(upper_lower_coord.x(), upper_lower_coord.y(), lower_upper_coord.z()), //< Non-unique 3x
      Eigen::Vector3i(upper_lower_coord.x(), upper_lower_coord.y(), upper_lower_coord.z()), //< Non-unique 3x
      Eigen::Vector3i(upper_lower_coord.x(), upper_lower_coord.y(), upper_upper_coord.z()), //< Unique
      Eigen::Vector3i(upper_lower_coord.x(), upper_upper_coord.y(), lower_upper_coord.z()), //< Unique
      Eigen::Vector3i(upper_lower_coord.x(), upper_upper_coord.y(), upper_lower_coord.z()), //< Unique

      Eigen::Vector3i(upper_upper_coord.x(), lower_upper_coord.y(), lower_upper_coord.z()), //< Unique
      Eigen::Vector3i(upper_upper_coord.x(), upper_lower_coord.y(), lower_upper_coord.z()), //< Unique
      Eigen::Vector3i(upper_upper_coord.x(), lower_upper_coord.y(), upper_lower_coord.z()), //< Unique
      Eigen::Vector3i(upper_upper_coord.x(), upper_lower_coord.y(), upper_lower_coord.z())  //< Unique
    };

    se::field_t grad_field_values[32];

    bool is_valid = true;

    for (unsigned int i = 0; i < 32; i++)
    {
      auto grad_field_value = se::visitor::getField(octree, block_ptr, grad_coords[i]);
      if (!grad_field_value)
      {
        is_valid = false;
        break;
      }
      grad_field_values[i] = *grad_field_value;
    }

    if (!is_valid)
    {
      continue;
    }

    const float rev_factor_x = (1 - factor.x());
    const float rev_factor_y = (1 - factor.y());
    const float rev_factor_z = (1 - factor.z());

    Eigen::Vector3f gradient = Eigen::Vector3f::Constant(0);

    gradient.x() = (((grad_field_values[19]
                    - grad_field_values[0]) * rev_factor_x
                    +(grad_field_values[28]
                    - grad_field_values[7]) * factor.x()) * rev_factor_y
                  + ((grad_field_values[23]
                    - grad_field_values[1]) * rev_factor_x
                    +(grad_field_values[29]
                    - grad_field_values[11]) * factor.x()) * factor.y()) * rev_factor_z
                 + (((grad_field_values[20]
                    - grad_field_values[2]) * rev_factor_x
                    +(grad_field_values[30]
                    - grad_field_values[8]) * factor.x()) * rev_factor_y
                  + ((grad_field_values[24]
                    - grad_field_values[3]) * rev_factor_x
                    +(grad_field_values[31]
                    - grad_field_values[12]) * factor.x()) * factor.y()) * factor.z();

    gradient.y() = (((grad_field_values[11]
                    - grad_field_values[4]) * rev_factor_x
                    +(grad_field_values[23]
                    - grad_field_values[16]) * factor.x()) * rev_factor_y
                  + ((grad_field_values[14]
                    - grad_field_values[7]) * rev_factor_x
                    +(grad_field_values[26]
                    - grad_field_values[19]) * factor.x()) * factor.y()) * rev_factor_z
                 + (((grad_field_values[12]
                    - grad_field_values[5]) * rev_factor_x
                    +(grad_field_values[24]
                    - grad_field_values[17]) * factor.x()) * rev_factor_y
                  + ((grad_field_values[15]
                    - grad_field_values[8]) * rev_factor_x
                    +(grad_field_values[27]
                    - grad_field_values[20]) * factor.x()) * factor.y()) * factor.z();

    gradient.z() = (((grad_field_values[8]
                    - grad_field_values[6]) * rev_factor_x
                    +(grad_field_values[20]
                    - grad_field_values[18]) * factor.x()) * rev_factor_y
                  + ((grad_field_values[12]
                    - grad_field_values[10]) * rev_factor_x
                    +(grad_field_values[24]
                    - grad_field_values[22]) * factor.x()) * factor.y()) * rev_factor_z
                 + (((grad_field_values[9]
                    - grad_field_values[7]) * rev_factor_x
                    +(grad_field_values[21]
                    - grad_field_values[19]) * factor.x()) * rev_factor_y
                   +((grad_field_values[13]
                    - grad_field_values[11]) * rev_factor_x
                    +(grad_field_values[25]
                    - grad_field_values[23]) * factor.x()) * factor.y()) * factor.z();

    return 0.5f * gradient;
  }

  return {};
}


} // namespace visitor
} // namespace se

#endif // SE_VISITOR_IMPL_HPP
