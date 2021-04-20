/*
 * SPDX-FileCopyrightText: 2014 University of Edinburgh, Imperial College London, University of Manchester.
 * SPDX-FileCopyrightText: 2020 Smart Robotics Lab, Imperial College London
 * SPDX-FileCopyrightText: 2020 Sotiris Papatheodorou
 * SPDX-License-Identifier: MIT
 * Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1
 */

#ifndef __READER_HPP
#define __READER_HPP

#include "se/config.h"
#include "reader_base.hpp"



namespace se {

  /** Create the appropriate reader instance based on the configuration.
   *
   * \param[in] config The pipeline configuration.
   * \return A pointer to an instance of a class derived from Reader.
   */
  Reader* create_reader(const Configuration& config);

} // namespace se

#endif

