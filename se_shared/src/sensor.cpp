// SPDX-FileCopyrightText: 2020 Sotiris Papatheodorou, Imperial College London
// SPDX-License-Identifier: BSD-3-Clause

#include "se/sensor.hpp"

#include <cassert>


// Explicit template class instantiation
template class srl::projection::PinholeCamera<srl::projection::NoDistortion>;

// Used for initializing a PinholeCamera.
const srl::projection::NoDistortion _distortion;

// Static variables
constexpr int se::PinholeCamera::num_frustum_vertices_;
constexpr int se::PinholeCamera::num_frustum_normals_;



se::PinholeCamera::PinholeCamera(const SensorConfig& c)
    : model(c.width, c.height, c.fx, c.fy, c.cx, c.cy, _distortion),
      left_hand_frame(c.left_hand_frame), near_plane(c.near_plane), far_plane(c.far_plane), scaled_pixel(1 / c.fx) {
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

  horizontal_fov = 2.0f * atanf(c.width / (2.0f * c.fx));
  vertical_fov = 2.0f * atanf(c.height / (2.0f * c.fy));
}

se::PinholeCamera::PinholeCamera(const PinholeCamera& pc, const float sf)
    : model(pc.model.imageWidth() * sf, pc.model.imageHeight() * sf,
            pc.model.focalLengthU() * sf, pc.model.focalLengthV() * sf,
            ((pc.model.imageCenterU() + 0.5f) * sf - 0.5f),
            ((pc.model.imageCenterV() + 0.5f) * sf - 0.5f),
            _distortion), left_hand_frame(pc.left_hand_frame), near_plane(pc.near_plane), far_plane(pc.far_plane) {

  computeFrustumVertices();
  computeFrustumNormals();
}

int se::PinholeCamera::computeIntegrationScale(
    const Eigen::Vector3f& block_centre,
    const float            voxel_dim,
    const int              last_scale,
    const int              min_scale,
    const int              max_block_scale) const {
  const float dist = block_centre.z();
  // Compute the side length in metres of a pixel projected dist metres from the camera
  const float pixel_dim = dist * scaled_pixel;
  const float pv_ratio = pixel_dim / voxel_dim;
  int scale = 0;
  if (pv_ratio < 1.5) {
    scale = 0;
  } else if (pv_ratio < 3) {
    scale = 1;
  } else if (pv_ratio < 6) {
    scale = 2;
  } else {
    scale = 3;
  }
  scale = std::min(scale, max_block_scale);

  Eigen::Vector3f block_centre_hyst = block_centre;
  bool recompute = false;
  if (scale > last_scale && min_scale != -1) {
    block_centre_hyst.z() -= 0.25;
    recompute = true;
  } else if (scale < last_scale && min_scale != -1) {
    block_centre_hyst.z() += 0.25;
    recompute = true;
  }

  if (recompute) {
    return computeIntegrationScale(block_centre_hyst, voxel_dim, last_scale, -1, max_block_scale);
  } else {
    return scale;
  }
}

float se::PinholeCamera::nearDist(const Eigen::Vector3f& ray_C) const {
  return near_plane / ray_C.normalized().z();
}

float se::PinholeCamera::farDist(const Eigen::Vector3f& ray_C) const {
  return far_plane / ray_C.normalized().z();
}

float se::PinholeCamera::measurementFromPoint(const Eigen::Vector3f& point_C) const {
  return point_C.z();
}

bool se::PinholeCamera::pointInFrustum(const Eigen::Vector3f& point_C) const {
  for (size_t i = 0; i < num_frustum_normals_; ++i) {
    // Compute the signed distance between the point and the plane
    const float distance = point_C.homogeneous().dot(frustum_normals_.col(i));
    if (distance < 0.0f) {
      // A negative distance means that the point is located on the opposite
      // halfspace than the one the plane normal is pointing towards
      return false;
    }
  }
  return true;
}

bool se::PinholeCamera::pointInFrustumInf(const Eigen::Vector3f& point_C) const {
  // Skip the far plane normal
  for (size_t i = 0; i < num_frustum_normals_ - 1; ++i) {
    // Compute the signed distance between the point and the plane
    const float distance = point_C.homogeneous().dot(frustum_normals_.col(i));
    if (distance < 0.0f) {
      // A negative distance means that the point is located on the opposite
      // halfspace than the one the plane normal is pointing towards
      return false;
    }
  }
  return true;
}

bool se::PinholeCamera::sphereInFrustum(const Eigen::Vector3f& center_C,
                                        const float            radius) const {
  for (size_t i = 0; i < num_frustum_normals_; ++i) {
    // Compute the signed distance between the point and the plane
    const float distance = center_C.homogeneous().dot(frustum_normals_.col(i));
    if (distance < -radius) {
      // Instead of testing for negative distance as in
      // se::PinholeCamera::pointInFrustum, test for distance smaller than
      // -radius so that the test is essentially performed on the plane offset
      // by radius.
      return false;
    }
  }
  return true;
}

bool se::PinholeCamera::sphereInFrustumInf(const Eigen::Vector3f& center_C,
                                           const float            radius) const {
  // Skip the far plane normal
  for (size_t i = 0; i < num_frustum_normals_ - 1; ++i) {
    // Compute the signed distance between the point and the plane
    const float distance = center_C.homogeneous().dot(frustum_normals_.col(i));
    if (distance < -radius) {
      // Instead of testing for negative distance as in
      // se::PinholeCamera::pointInFrustum, test for distance smaller than
      // -radius so that the test is essentially performed on the plane offset
      // by radius.
      return false;
    }
  }
  return true;
}

void se::PinholeCamera::computeFrustumVertices() {
  Eigen::Vector3f point_C;
  // Back-project the frame corners to get the frustum vertices
  // Top left
  model.backProject(Eigen::Vector2f(0.0f, 0.0f), &point_C);
  frustum_vertices_.col(0).head<3>() = point_C;
  frustum_vertices_.col(4).head<3>() = point_C;
  // Top right
  model.backProject(Eigen::Vector2f(model.imageWidth(), 0.0f), &point_C);
  frustum_vertices_.col(1).head<3>() = point_C;
  frustum_vertices_.col(5).head<3>() = point_C;
  // Bottom right
  model.backProject(Eigen::Vector2f(model.imageWidth(), model.imageHeight()), &point_C);
  frustum_vertices_.col(2).head<3>() = point_C;
  frustum_vertices_.col(6).head<3>() = point_C;
  // Bottom left
  model.backProject(Eigen::Vector2f(0.0f, model.imageHeight()), &point_C);
  frustum_vertices_.col(3).head<3>() = point_C;
  frustum_vertices_.col(7).head<3>() = point_C;
  // Scale the frustum vertices with the appropriate depth for near and far
  // plane vertices
  for (int i = 0; i < num_frustum_vertices_ / 2; ++i) {
    frustum_vertices_.col(i).head<3>() *= near_plane;
    frustum_vertices_.col(num_frustum_vertices_ / 2 + i).head<3>() *= far_plane;
  }
}

void se::PinholeCamera::computeFrustumNormals() {
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
}
