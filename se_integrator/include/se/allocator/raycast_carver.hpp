#ifndef SE_RAYCAST_CARVER_HPP
#define SE_RAYCAST_CARVER_HPP



namespace se {
namespace fetcher {



template<typename MapT,
         typename SensorT
>
inline std::vector<se::OctantBase *> frustum(MapT&                  map,
                                             const SensorT&         sensor,
                                             const Eigen::Matrix4f& T_MS);



} // namespace fetcher




template<typename MapT,
         typename SensorT
>
class RaycastCarver
{
public:
  typedef typename MapT::OctreeType OctreeType;

  RaycastCarver(MapT&                   map,
                const SensorT&          sensor,
                const se::Image<float>& depth_img,
                const Eigen::Matrix4f&  T_MS,
                const int               frame);

  /**
   * \brief Allocate a band around the depth measurements using a raycasting approach
   *
   * \param[in] band    The band around the depth value to be allocated
   *
   * \retrun The allocated blocks
   */
  std::vector<se::OctantBase*> allocateBand(const float band);

  MapT&                   map_;
  OctreeType&             octree_;
  const SensorT&          sensor_;
  const se::Image<float>& depth_img_;
  const Eigen::Matrix4f&  T_MS_;
  const int               frame_;
};



} // namespace se



#include "./impl/raycast_carver_impl.hpp"



#endif //SE_RAYCAST_CARVER_HPP

