#ifndef SE_MULTIRES_TSDF_UPDATER_HPP
#define SE_MULTIRES_TSDF_UPDATER_HPP



#include "se/map.hpp"
#include "se/sensor/sensor.hpp"
#include "se/octree/propagator.hpp"



namespace se {



// Multi-res TSDF updater
template<se::Colour    ColB,
         se::Semantics SemB,
         int           BlockSize,
         typename SensorT
>
class Updater<Map<Data<se::Field::TSDF, ColB, SemB>, se::Res::Multi, BlockSize>, SensorT>
{
public:
  typedef Map<Data<se::Field::TSDF, ColB, SemB>, se::Res::Multi, BlockSize> MapType;
  typedef typename MapType::DataType                                        DataType;
  typedef typename MapType::OctreeType                                      OctreeType;
  typedef typename MapType::OctreeType::NodeType                            NodeType;
  typedef typename MapType::OctreeType::BlockType                           BlockType;

  struct UpdaterConfig
  {
    UpdaterConfig(const MapType& map) :
            truncation_boundary(map.getRes() * map.getDataConfig().truncation_boundary_factor)
    {
    }

    const float truncation_boundary;
  };

  /**
   * \param[in]  map                  The reference to the map to be updated.
   * \param[in]  sensor               The sensor model.
   * \param[in]  depth_img            The depth image to be integrated.
   * \param[in]  T_MS                 The transformation from camera to map frame.
   * \param[in]  frame                The frame number to be integrated.
   */
  Updater(MapType&                               map,
          const SensorT&                         sensor,
          const se::Image<float>&                depth_img,
          const Eigen::Matrix4f&                 T_MS,
          const int                              frame);


  void operator()(std::vector<se::OctantBase*>& block_ptrs);

private:
  void updateVoxel(typename BlockType::DataUnion& data_union,
                   const field_t                  sdf_value);

  MapType&                map_;
  const SensorT&          sensor_;
  const se::Image<float>& depth_img_;
  const Eigen::Matrix4f&  T_MS_;
  const int               frame_;
  const UpdaterConfig     config_;
};



} // namespace se



#endif //SE_MULTIRES_TSDF_UPDATER_HPP



#include "./impl/multires_tsdf_updater_impl.hpp"