#ifndef SE_SENSOR_IMPL_HPP
#define SE_SENSOR_IMPL_HPP

namespace se {



template <typename DerivedT>
template <typename ValidPredicate>
bool SensorBase<DerivedT>::projectToPixelValue(const Eigen::Vector3f&  point_S,
                                               const se::Image<float>& img,
                                               float&                  img_value,
                                               ValidPredicate          valid_predicate) const
{
  Eigen::Vector2f pixel_f;
  if (this->underlying().model.project(point_S, &pixel_f) != srl::projection::ProjectionStatus::Successful)
  {
    return false;
  }
  const Eigen::Vector2i pixel = se::round_pixel(pixel_f);
  img_value = img(pixel.x(), pixel.y());
  // Return false for invalid depth measurement
  if (!valid_predicate(img_value))
  {
    return false;
  }
  return true;
}



template <typename DerivedT>
template <typename ValidPredicate>
bool SensorBase<DerivedT>::getPixelValue(const Eigen::Vector2f&  pixel_f,
                                         const se::Image<float>& img,
                                         float&                  img_value,
                                         ValidPredicate          valid_predicate) const
{
  if (!this->underlying().model.isInImage(pixel_f))
  {
    return false;
  }
  Eigen::Vector2i pixel = se::round_pixel(pixel_f);
  img_value = img(pixel.x(), pixel.y());
  // Return false for invalid depth measurement
  if (!valid_predicate(img_value))
  {
    return false;
  }
  return true;
}



} // namespace se

#endif // SE_SENSOR_IMPL_HPP

