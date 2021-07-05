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

  struct RaycastCarverConfig
  {
    RaycastCarverConfig(const MapT& map) :
      truncation_boundary(map.getRes() * map.getDataConfig().truncation_boundary_factor),
      band(2 * truncation_boundary)
    {
    }

    const float truncation_boundary;
    const float band;
  };

  RaycastCarver(MapT&                   map,
                const SensorT&          sensor,
                const se::Image<float>& depth_img,
                const Eigen::Matrix4f&  T_MS,
                const int               frame);

  /**
   * \brief Allocate a band around the depth measurements using a raycasting approach
   *
   * \retrun The allocated blocks
   */
  std::vector<se::OctantBase*> operator()();

  MapT&                     map_;
  OctreeType&               octree_;
  const SensorT&            sensor_;
  const se::Image<float>&   depth_img_;
  const Eigen::Matrix4f&    T_MS_;
  const int                 frame_;
  const RaycastCarverConfig config_;
};



} // namespace se



#include "./impl/raycast_carver_impl.hpp"



#endif //SE_RAYCAST_CARVER_HPP

