#ifndef SE_UPDATER_HPP
#define SE_UPDATER_HPP



namespace se {



template <typename MapT,
          typename SensorT
>
class Updater
{
public:
  Updater(MapT&                               map,
          const SensorT&                      sensor,
          const se::Image<float>&             depth_img,
          const Eigen::Matrix4f&              T_MS,
          const int                           frame);

  template <typename UpdateListT>
  void operator()(UpdateListT& updating_list);
};



// Single-res TSDF updater
template<se::Colour    ColB,
         se::Semantics SemB,
         int           BlockSize,
         typename      SensorT
>
class Updater<Map<Data<se::Field::TSDF, ColB, SemB>, se::Res::Single, BlockSize>, SensorT>;



// Multi-res TSDF updater
template<se::Colour    ColB,
         se::Semantics SemB,
         int           BlockSize,
         typename      SensorT
>
class Updater<Map<Data<se::Field::TSDF, ColB, SemB>, se::Res::Multi, BlockSize>, SensorT>;



// Multi-res Occupancy updater
template<se::Colour    ColB,
         se::Semantics SemB,
         int           BlockSize,
         typename      SensorT
>
class Updater<Map<Data<se::Field::Occupancy, ColB, SemB>, se::Res::Multi, BlockSize>, SensorT>;




} // namespace se



#endif //SE_UPDATER_HPP



#include "se/updater/singleres_tsdf_updater.hpp"
#include "se/updater/multires_tsdf_updater.hpp"
#include "se/updater/multires_ofusion_updater.hpp"