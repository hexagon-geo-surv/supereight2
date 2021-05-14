#ifndef SE_OCTREE_IO_HPP
#define SE_OCTREE_IO_HPP

namespace se {
namespace io {

template <typename OctreeT>
int save_3d_slice_vtk(const OctreeT&         octree_ptr,
                      const std::string&     filename,
                      const Eigen::Vector3i& lower_coord,
                      const Eigen::Vector3i& upper_coord);

template <typename OctreeT>
int save_octree_structure_ply(OctreeT&           octree_ptr,
                              const std::string& filename);

}



}

#include "se/io/impl/octree_io_impl.hpp"

#endif // SE_OCTREE_IO_HPP
