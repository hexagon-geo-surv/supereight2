/*

Copyright 2016 Emanuele Vespa, Imperial College London

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef SE_NODE_ITERATOR_HPP
#define SE_NODE_ITERATOR_HPP

#include "octree.hpp"
#include "Eigen/Dense"

namespace se {

template<typename OctreeT>
using BlockType = typename OctreeT::BlockType;

/*! \brief Iterate through all the nodes (first Node and then VoxelBlock nodes)
 * of the Octree.
 */
template <typename OctreeT>
class octant_iterator {

  public:

  octant_iterator(const std::shared_ptr<OctreeT> octree_ptr): octree_ptr_(octree_ptr) {
    state_ = BRANCH_NODES;
    last = 0;
  };

  /*! \brief Get a pointer to the next node in the Octree.
   * Starting from the root node, each time this function is called will return
   * a pointer to the next non-leaf (Node) node. Once all non-leaf nodes have
   * been iterated through, it will start iterating through the leaf nodes
   * (VoxelBlock).
   *
   * \return A pointer to the next node. Returns nullptr if all nodes have been
   * iterated through.
   */
  std::shared_ptr<se::OctantBase> next() {
    switch(state_) {
      case BRANCH_NODES: {
        const auto &octant_buffer = octree_ptr_->getOctantBuffer();
        if (last < octant_buffer.size()) {
          std::shared_ptr<se::OctantBase> octant_ptr = octant_buffer[last++];
          return octant_ptr;
        } else {
          last = 0;
          state_ = FINISHED;
          return nullptr;
        }
        break;
      }
      case FINISHED: {
        return nullptr;
      }
    }
    return nullptr;
  };

  private:
  typedef enum ITER_STATE {
    BRANCH_NODES,
    LEAF_NODES,
    FINISHED
  } ITER_STATE;

  const std::shared_ptr<OctreeT> octree_ptr_;
  ITER_STATE state_;
  size_t last;
};
}
#endif
