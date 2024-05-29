/*
 * SPDX-FileCopyrightText: 2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2022 Nils Funk
 * SPDX-FileCopyrightText: 2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <se/supereight.hpp>

int main(int argc, char** argv)
{
    try {
        se::TSDFMap<se::Res::Multi> map(Eigen::Vector3f::Constant(10.0f), 0.1f);

        se::PinholeCamera::Config sensor_config;
        sensor_config.width = 320;
        sensor_config.height = 240;
        sensor_config.fx = 525.0f;
        sensor_config.fy = 525.0f;
        sensor_config.cx = sensor_config.width / 2.0f - 0.5f;
        sensor_config.cy = sensor_config.height / 2.0f - 0.5f;
        const se::PinholeCamera sensor(sensor_config);
        const se::Image<float> depth(sensor.model.imageWidth(), sensor.model.imageHeight(), 2.0f);

        se::MapIntegrator integrator(map);
        integrator.integrateDepth(
            0, se::Measurements{se::Measurement{depth, sensor, Eigen::Isometry3f::Identity()}});

        std::cout << "Supereight2 v" << SE_VERSION << " install successful\n";
        return EXIT_SUCCESS;
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << "\n";
        return EXIT_FAILURE;
    }
}
