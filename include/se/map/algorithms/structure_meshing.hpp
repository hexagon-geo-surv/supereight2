/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_STRUCTURE_MESHING_HPP
#define SE_STRUCTURE_MESHING_HPP

#include "se/map/octree/octree.hpp"

namespace se {

/** Return a mesh of the octants of \p octree. Only leaf octants will be contained in the mesh if
 * \p only_leaves is true. */
template<typename OctreeT>
typename OctreeT::StructureMesh octree_structure_mesh(OctreeT& octree,
                                                      const bool only_leaves = true);

} // namespace se

#include "impl/structure_meshing_impl.hpp"

#endif // SE_STRUCTURE_MESHING_HPP
