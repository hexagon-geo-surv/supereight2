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
        se::TSDFMap<se::Res::Multi> map(se::MapConfig(), se::TSDFDataConfig());
        std::cout << "Supereight2 install successful\n";
        return EXIT_SUCCESS;
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << "\n";
        return EXIT_FAILURE;
    }
}
