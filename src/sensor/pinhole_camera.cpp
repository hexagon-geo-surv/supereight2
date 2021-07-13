// SPDX-FileCopyrightText: 2020 Sotiris Papatheodorou, Imperial College London
// SPDX-License-Identifier: BSD-3-Clause

#include "se/sensor/sensor.hpp"



se::PinholeCameraConfig::PinholeCameraConfig() :
    SensorConfigBase()
{
}



se::PinholeCameraConfig::PinholeCameraConfig(const std::string& yaml_file) :
    SensorConfigBase(yaml_file)
{
  // Open the file for reading.
  cv::FileStorage fs;
  fs.open(yaml_file, cv::FileStorage::READ | cv::FileStorage::FORMAT_YAML);

  // Get the node containing the sensor configuration.
  const cv::FileNode node = fs["sensor"];

  // Read the config parameters.
  se::yaml::subnode_as_float(node, "fx", fx);
  se::yaml::subnode_as_float(node, "fy", fy);
  se::yaml::subnode_as_float(node, "cx", cx);
  se::yaml::subnode_as_float(node, "cy", cy);
  left_hand_frame = fy < 0;
}



std::ostream& se::operator<<(std::ostream& os, const se::PinholeCameraConfig& c)
{
  os << "width:            " << c.width << " px\n";
  os << "height:           " << c.height << " px\n";
  os << "near_plane:       " << c.near_plane << " m\n";
  os << "far_plane:        " << c.far_plane << " m\n";
  os << "left_hand_frame:  " << (c.left_hand_frame ? "yes" : "no") << "\n";
  os << "fx:               " << c.fx << " px\n";
  os << "fy:               " << c.fy << " px\n";
  os << "cx:               " << c.cx << " px\n";
  os << "cy:               " << c.cy << " px\n";
  return os;
}



// Explicit template class instantiation
template class srl::projection::PinholeCamera<srl::projection::NoDistortion>;

// Used for initializing a PinholeCamera.
const srl::projection::NoDistortion _distortion;

// Static variables
constexpr int se::PinholeCamera::num_frustum_vertices_;
constexpr int se::PinholeCamera::num_frustum_normals_;



se::PinholeCamera::PinholeCamera(const PinholeCameraConfig& c) :
    model(c.width, c.height,
          c.fx, c.fy,
          c.cx, c.cy,
          _distortion),
    left_hand_frame(c.left_hand_frame),
    near_plane(c.near_plane), far_plane(c.far_plane),
    scaled_pixel(1 / c.fx)
{
  computeFrustumVertices();
  computeFrustumNormals();

  assert(c.width  > 0);
  assert(c.height > 0);
  assert(c.near_plane >= 0.f);
  assert(c.far_plane > c.near_plane);
  assert(!std::isnan(c.fx));
  assert(!std::isnan(c.fy));
  assert(!std::isnan(c.cx));
  assert(!std::isnan(c.cy));

  horizontal_fov = 2.0f * atanf(c.width  / (2.0f * c.fx));
  vertical_fov   = 2.0f * atanf(c.height / (2.0f * c.fy));
}



se::PinholeCamera::PinholeCamera(const PinholeCameraConfig& c,
                                 const float         dsf) :
    model(c.width / dsf, c.height / dsf,
          c.fx / dsf, c.fy / dsf,
          (c.cx + 0.5f) / dsf - 0.5f, (c.cy + 0.5f) / dsf - 0.5f,
          _distortion),
    left_hand_frame(c.left_hand_frame),
    near_plane(c.near_plane), far_plane(c.far_plane),
    scaled_pixel(1 / (c.fx / dsf))
{
  computeFrustumVertices();
  computeFrustumNormals();

  assert(c.width  > 0);
  assert(c.height > 0);
  assert(c.near_plane >= 0.f);
  assert(c.far_plane > c.near_plane);
  assert(!std::isnan(c.fx));
  assert(!std::isnan(c.fy));
  assert(!std::isnan(c.cx));
  assert(!std::isnan(c.cy));

  horizontal_fov = 2.0f * atanf(c.width  / (2.0f * c.fx));
  vertical_fov   = 2.0f * atanf(c.height / (2.0f * c.fy));
}



se::PinholeCamera::PinholeCamera(const PinholeCamera& pc,
                                 const float          dsf)
    : model(pc.model.imageWidth() / dsf, pc.model.imageHeight() / dsf,
            pc.model.focalLengthU() / dsf, pc.model.focalLengthV() / dsf,
            ((pc.model.imageCenterU() + 0.5f) / dsf - 0.5f),
            ((pc.model.imageCenterV() + 0.5f) / dsf - 0.5f),
            _distortion),
      left_hand_frame(pc.left_hand_frame),
      near_plane(pc.near_plane), far_plane(pc.far_plane)
{

  computeFrustumVertices();
  computeFrustumNormals();

  horizontal_fov = 2.0f * atanf(pc.model.imageWidth()  / (2.0f * pc.model.focalLengthU()));
  vertical_fov   = 2.0f * atanf(pc.model.imageHeight() / (2.0f * pc.model.focalLengthV()));
}



int se::PinholeCamera::computeIntegrationScaleImpl(const Eigen::Vector3f& block_centre_S,
                                                   const float            map_res,
                                                   const int              last_scale,
                                                   const int              min_scale,
                                                   const int              max_block_scale) const
{
  const float dist = block_centre_S.z();
  // Compute the side length in metres of a pixel projected dist metres from the camera
  const float pixel_dim = dist * scaled_pixel;
  const float pv_ratio = pixel_dim / map_res;
  int scale = 0;
  if (pv_ratio < 1.5)
  {
    scale = 0;
  } else if (pv_ratio < 3)
  {
    scale = 1;
  } else if (pv_ratio < 6)
  {
    scale = 2;
  } else
  {
    scale = 3;
  }
  scale = std::min(scale, max_block_scale);

  Eigen::Vector3f block_centre_S_hyst = block_centre_S;
  bool recompute = false;
  if (scale > last_scale && min_scale != -1)
  {
    block_centre_S_hyst.z() -= 0.25;
    recompute = true;
  } else if (scale < last_scale && min_scale != -1)
  {
    block_centre_S_hyst.z() += 0.25;
    recompute = true;
  }

  if (recompute)
  {
    return computeIntegrationScale(block_centre_S_hyst, map_res, last_scale, -1, max_block_scale);
  } else
  {
    return scale;
  }
}



float se::PinholeCamera::nearDistImpl(const Eigen::Vector3f& ray_S) const
{
  return near_plane / ray_S.normalized().z();
}



float se::PinholeCamera::farDistImpl(const Eigen::Vector3f& ray_S) const
{
  return far_plane / ray_S.normalized().z();
}



float se::PinholeCamera::measurementFromPointImpl(const Eigen::Vector3f& point_S) const
{
  return point_S.z();
}



bool se::PinholeCamera::pointInFrustumImpl(const Eigen::Vector3f& point_S) const
{
  for (size_t i = 0; i < num_frustum_normals_; ++i)
  {
    // Compute the signed distance between the point and the plane
    const float distance = point_S.homogeneous().dot(frustum_normals_.col(i));
    if (distance < 0.0f)
    {
      // A negative distance means that the point is located on the opposite
      // halfspace than the one the plane normal is pointing towards
      return false;
    }
  }
  return true;
}



bool se::PinholeCamera::pointInFrustumInfImpl(const Eigen::Vector3f& point_S) const
{
  // Skip the far plane normal
  for (size_t i = 0; i < num_frustum_normals_ - 1; ++i)
  {
    // Compute the signed distance between the point and the plane
    const float distance = point_S.homogeneous().dot(frustum_normals_.col(i));
    if (distance < 0.0f)
    {
      // A negative distance means that the point is located on the opposite
      // halfspace than the one the plane normal is pointing towards
      return false;
    }
  }
  return true;
}



bool se::PinholeCamera::sphereInFrustumImpl(const Eigen::Vector3f& center_S,
                                            const float            radius) const
{
  for (size_t i = 0; i < num_frustum_normals_; ++i)
  {
    // Compute the signed distance between the point and the plane
    const float distance = center_S.homogeneous().dot(frustum_normals_.col(i));
    if (distance < -radius)
    {
      // Instead of testing for negative distance as in
      // se::PinholeCamera::pointInFrustum, test for distance smaller than
      // -radius so that the test is essentially performed on the plane offset
      // by radius.
      return false;
    }
  }
  return true;
}



bool se::PinholeCamera::sphereInFrustumInfImpl(const Eigen::Vector3f& center_S,
                                               const float            radius) const
{
  // Skip the far plane normal
  for (size_t i = 0; i < num_frustum_normals_ - 1; ++i)
  {
    // Compute the signed distance between the point and the plane
    const float distance = center_S.homogeneous().dot(frustum_normals_.col(i));
    if (distance < -radius)
    {
      // Instead of testing for negative distance as in
      // se::PinholeCamera::pointInFrustum, test for distance smaller than
      // -radius so that the test is essentially performed on the plane offset
      // by radius.
      return false;
    }
  }
  return true;
}



void se::PinholeCamera::computeFrustumVertices()
{
  Eigen::Vector3f point_S;
  // Back-project the frame corners to get the frustum vertices
  // Top left
  model.backProject(Eigen::Vector2f(0.0f, 0.0f), &point_S);
  frustum_vertices_.col(0).head<3>() = point_S;
  frustum_vertices_.col(4).head<3>() = point_S;
  // Top right
  model.backProject(Eigen::Vector2f(model.imageWidth(), 0.0f), &point_S);
  frustum_vertices_.col(1).head<3>() = point_S;
  frustum_vertices_.col(5).head<3>() = point_S;
  // Bottom right
  model.backProject(Eigen::Vector2f(model.imageWidth(), model.imageHeight()), &point_S);
  frustum_vertices_.col(2).head<3>() = point_S;
  frustum_vertices_.col(6).head<3>() = point_S;
  // Bottom left
  model.backProject(Eigen::Vector2f(0.0f, model.imageHeight()), &point_S);
  frustum_vertices_.col(3).head<3>() = point_S;
  frustum_vertices_.col(7).head<3>() = point_S;
  // Scale the frustum vertices with the appropriate depth for near and far
  // plane vertices
  for (int i = 0; i < num_frustum_vertices_ / 2; ++i)
  {
    frustum_vertices_.col(i).head<3>() *= near_plane;
    frustum_vertices_.col(num_frustum_vertices_ / 2 + i).head<3>() *= far_plane;
  }
}



void se::PinholeCamera::computeFrustumNormals()
{
  // The w vector component corresponds to the distance of the plane from the
  // origin. It should be 0 for all planes other than the near and far planes.
  // Left plane vector.
  frustum_normals_.col(0) = se::math::plane_normal(
      frustum_vertices_.col(4), frustum_vertices_.col(0), frustum_vertices_.col(3));
  frustum_normals_.col(0).w() = 0.0f;
  // Right plane vector.
  frustum_normals_.col(1) = se::math::plane_normal(
      frustum_vertices_.col(1), frustum_vertices_.col(5), frustum_vertices_.col(6));
  frustum_normals_.col(1).w() = 0.0f;
  // Bottom plane vector.
  frustum_normals_.col(2) = se::math::plane_normal(
      frustum_vertices_.col(7), frustum_vertices_.col(3), frustum_vertices_.col(2));
  frustum_normals_.col(2).w() = 0.0f;
  // Top plane vector.
  frustum_normals_.col(3) = se::math::plane_normal(
      frustum_vertices_.col(5), frustum_vertices_.col(1), frustum_vertices_.col(0));
  frustum_normals_.col(3).w() = 0.0f;
  // Near plane vector.
  frustum_normals_.col(4) = Eigen::Vector4f(0.f, 0.f, 1.f, -near_plane);
  // Far plane vector.
  frustum_normals_.col(5) = Eigen::Vector4f(0.f, 0.f, -1.f, far_plane);
  // The directions of all normal vectors must be multiplied with sign(fx * fy) to account for
  // negative focal lengths. The near and far plane normals are already set with the correct
  // direction.
  float sign = model.focalLengthU() * model.focalLengthV();
  sign = sign / fabsf(sign);
  for (int i = 0; i < num_frustum_normals_ - 2; i++) {
    frustum_normals_.col(i).head<3>() *= sign;
  }
}
