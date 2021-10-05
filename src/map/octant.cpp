/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2019-2021 Nils Funk
 * SPDX-FileCopyrightText: 2019-2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "se/map/octant/octant.hpp"

namespace se
{



/**
 * \brief Sets the Base for every node and block.
 */
OctantBase::OctantBase(const bool             is_block,
                       const Eigen::Vector3i& coord,
                       OctantBase*            parent_ptr)
        : is_block_(is_block), parent_ptr_(parent_ptr), coord_(coord), time_stamp_(-1), is_active_(true), children_mask_(0)
{
}



} // namespace se

