/*
 * SPDX-FileCopyrightText: 2019-2020 Smart Robotics Lab, Imperial College London
 * SPDX-FileCopyrightText: 2019-2020 Sotiris Papatheodorou, Imperial College London
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <memory>

#include <Eigen/Dense>

#include <se/common/image_utils.hpp>



class DepthImageIO : public ::testing::Test {
protected:
  DepthImageIO() {
    depth_image_data_16_ = std::unique_ptr<uint16_t[]>(new uint16_t[num_pixels_]());
    depth_image_data_f_ = std::unique_ptr<float[]>(new float[num_pixels_]());

    // Initialize the test images (one in scaled uint16_t, the other in float
    // metres) with a black (B), white (W) and gray (G) pattern.
    // G B B B
    // W G B B
    // W W G B
    // W W W G
    for (size_t x = 0; x < depth_image_width_; ++x) {
      for (size_t y = 0; y < depth_image_height_; ++y) {
        uint16_t depth_value_scaled;
        if (x > y) {
          depth_value_scaled = 0;
        } else if (x == y) {
          depth_value_scaled = UINT16_MAX / 2;
        } else {
          depth_value_scaled = UINT16_MAX;
        }
        depth_image_data_16_[x + depth_image_width_ * y] = depth_value_scaled;
        depth_image_data_f_[x + depth_image_width_ * y] = depth_inverse_scale_ * depth_value_scaled;
      }
    }
  }

  std::unique_ptr<uint16_t[]> depth_image_data_16_;
  std::unique_ptr<float[]> depth_image_data_f_;
  const size_t depth_image_width_  = 64;
  const size_t depth_image_height_ = 64;
  const float depth_scale_ = 5000.0f;
  const float depth_inverse_scale_ = 1.0f / depth_scale_;
  const size_t num_pixels_ = depth_image_width_ * depth_image_height_;
  const size_t depth_16_size_bytes_ = sizeof(uint16_t) * num_pixels_;
  const size_t depth_f_size_bytes_ = sizeof(float) * num_pixels_;
  const Eigen::Vector2i depth_image_res_
      = Eigen::Vector2i(depth_image_width_, depth_image_height_);
};





TEST_F(DepthImageIO, PNGSave16Load16) {
  // Save the image.
  const int save_ok = se::save_depth_png(depth_image_data_16_.get(), depth_image_res_, "/tmp/depth_16.png");
  EXPECT_EQ(save_ok, 0);

  // Load the image.
  uint16_t* loaded_depth_16;
  Eigen::Vector2i loaded_depth_image_res (0, 0);
  const int load_ok = se::load_depth_png(&loaded_depth_16, loaded_depth_image_res, "/tmp/depth_16.png");
  EXPECT_EQ(load_ok, 0);

  // Compare the loaded image with the saved one.
  EXPECT_EQ(static_cast<unsigned>(loaded_depth_image_res.x()), depth_image_width_);
  EXPECT_EQ(static_cast<unsigned>(loaded_depth_image_res.y()), depth_image_height_);
  EXPECT_EQ(memcmp(loaded_depth_16, depth_image_data_16_.get(), depth_16_size_bytes_), 0);

  delete[] loaded_depth_16;
}



TEST_F(DepthImageIO, PNGSaveFLoad16) {
  // Save the image.
  const int save_ok = se::save_depth_png(depth_image_data_f_.get(), depth_image_res_, "/tmp/depth_f.png", depth_scale_);
  EXPECT_EQ(save_ok, 0);

  // Load the image.
  uint16_t* loaded_depth_16;
  Eigen::Vector2i loaded_depth_image_res (0, 0);
  const int load_ok = se::load_depth_png(&loaded_depth_16, loaded_depth_image_res, "/tmp/depth_f.png");
  EXPECT_EQ(load_ok, 0);

  // Compare the loaded image with the saved one.
  EXPECT_EQ(static_cast<unsigned>(loaded_depth_image_res.x()), depth_image_width_);
  EXPECT_EQ(static_cast<unsigned>(loaded_depth_image_res.y()), depth_image_height_);
  EXPECT_EQ(memcmp(loaded_depth_16, depth_image_data_16_.get(), depth_16_size_bytes_), 0);

  delete[] loaded_depth_16;
}



TEST_F(DepthImageIO, PNGSave16LoadF) {
  // Save the image.
  const int save_ok = se::save_depth_png(depth_image_data_16_.get(), depth_image_res_, "/tmp/depth_16.png");
  EXPECT_EQ(save_ok, 0);

  // Load the image.
  float* loaded_depth_f;
  Eigen::Vector2i loaded_depth_image_res (0, 0);
  const int load_ok = se::load_depth_png(&loaded_depth_f, loaded_depth_image_res, "/tmp/depth_16.png", depth_inverse_scale_);
  EXPECT_EQ(load_ok, 0);

  // Compare the loaded image with the saved one.
  EXPECT_EQ(static_cast<unsigned>(loaded_depth_image_res.x()), depth_image_width_);
  EXPECT_EQ(static_cast<unsigned>(loaded_depth_image_res.y()), depth_image_height_);
  EXPECT_EQ(memcmp(loaded_depth_f, depth_image_data_f_.get(), depth_f_size_bytes_), 0);

  delete[] loaded_depth_f;
}



TEST_F(DepthImageIO, PNGSaveFLoadF) {
  // Save the image.
  const int save_ok = se::save_depth_png(depth_image_data_f_.get(), depth_image_res_, "/tmp/depth_f.png", depth_scale_);
  EXPECT_EQ(save_ok, 0);

  // Load the image.
  float* loaded_depth_f;
  Eigen::Vector2i loaded_depth_image_res (0, 0);
  const int load_ok = se::load_depth_png(&loaded_depth_f, loaded_depth_image_res, "/tmp/depth_f.png", depth_inverse_scale_);
  EXPECT_EQ(load_ok, 0);

  // Compare the loaded image with the saved one.
  EXPECT_EQ(static_cast<unsigned>(loaded_depth_image_res.x()), depth_image_width_);
  EXPECT_EQ(static_cast<unsigned>(loaded_depth_image_res.y()), depth_image_height_);
  EXPECT_EQ(memcmp(loaded_depth_f, depth_image_data_f_.get(), depth_f_size_bytes_), 0);

  delete[] loaded_depth_f;
}



TEST_F(DepthImageIO, PGMSave16Load16) {
  // Save the image.
  const int save_ok = se::save_depth_pgm(depth_image_data_16_.get(), depth_image_res_, "/tmp/depth_16.pgm");
  EXPECT_EQ(save_ok, 0);

  // Load the image.
  uint16_t* loaded_depth_16;
  Eigen::Vector2i loaded_depth_image_res (0, 0);
  const int load_ok = se::load_depth_pgm(&loaded_depth_16, loaded_depth_image_res, "/tmp/depth_16.pgm");
  EXPECT_EQ(load_ok, 0);

  // Compare the loaded image with the saved one.
  EXPECT_EQ(static_cast<unsigned>(loaded_depth_image_res.x()), depth_image_width_);
  EXPECT_EQ(static_cast<unsigned>(loaded_depth_image_res.y()), depth_image_height_);
  EXPECT_EQ(memcmp(loaded_depth_16, depth_image_data_16_.get(), depth_16_size_bytes_), 0);

  delete[] loaded_depth_16;
}



TEST_F(DepthImageIO, PGMSaveFLoad16) {
  // Save the image.
  const int save_ok = se::save_depth_pgm(depth_image_data_f_.get(), depth_image_res_, "/tmp/depth_f.pgm", depth_scale_);
  EXPECT_EQ(save_ok, 0);

  // Load the image.
  uint16_t* loaded_depth_16;
  Eigen::Vector2i loaded_depth_image_res (0, 0);
  const int load_ok = se::load_depth_pgm(&loaded_depth_16, loaded_depth_image_res, "/tmp/depth_f.pgm");
  EXPECT_EQ(load_ok, 0);

  // Compare the loaded image with the saved one.
  EXPECT_EQ(static_cast<unsigned>(loaded_depth_image_res.x()), depth_image_width_);
  EXPECT_EQ(static_cast<unsigned>(loaded_depth_image_res.y()), depth_image_height_);
  EXPECT_EQ(memcmp(loaded_depth_16, depth_image_data_16_.get(), depth_16_size_bytes_), 0);

  delete[] loaded_depth_16;
}



TEST_F(DepthImageIO, PGMSave16LoadF) {
  // Save the image.
  const int save_ok = se::save_depth_pgm(depth_image_data_16_.get(), depth_image_res_, "/tmp/depth_16.pgm");
  EXPECT_EQ(save_ok, 0);

  // Load the image.
  float* loaded_depth_f;
  Eigen::Vector2i loaded_depth_image_res (0, 0);
  const int load_ok = se::load_depth_pgm(&loaded_depth_f, loaded_depth_image_res, "/tmp/depth_16.pgm", depth_inverse_scale_);
  EXPECT_EQ(load_ok, 0);

  // Compare the loaded image with the saved one.
  EXPECT_EQ(static_cast<unsigned>(loaded_depth_image_res.x()), depth_image_width_);
  EXPECT_EQ(static_cast<unsigned>(loaded_depth_image_res.y()), depth_image_height_);
  EXPECT_EQ(memcmp(loaded_depth_f, depth_image_data_f_.get(), depth_f_size_bytes_), 0);

  delete[] loaded_depth_f;
}



TEST_F(DepthImageIO, PGMSaveFLoadF) {
  // Save the image.
  const int save_ok = se::save_depth_pgm(depth_image_data_f_.get(), depth_image_res_, "/tmp/depth_f.pgm", depth_scale_);
  EXPECT_EQ(save_ok, 0);

  // Load the image.
  float* loaded_depth_f;
  Eigen::Vector2i loaded_depth_image_res (0, 0);
  const int load_ok = se::load_depth_pgm(&loaded_depth_f, loaded_depth_image_res, "/tmp/depth_f.pgm", depth_inverse_scale_);
  EXPECT_EQ(load_ok, 0);

  // Compare the loaded image with the saved one.
  EXPECT_EQ(static_cast<unsigned>(loaded_depth_image_res.x()), depth_image_width_);
  EXPECT_EQ(static_cast<unsigned>(loaded_depth_image_res.y()), depth_image_height_);
  EXPECT_EQ(memcmp(loaded_depth_f, depth_image_data_f_.get(), depth_f_size_bytes_), 0);

  delete[] loaded_depth_f;
}

