// SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London
// SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou, Imperial College London
// SPDX-License-Identifier: BSD-3-Clause

#ifndef SE_YAML_HPP
#define SE_YAML_HPP

#include <Eigen/Dense>
#include <iostream>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace se {
  namespace yaml {
    /** Interprets the data in the subnode of base_node named subnode_name as a boolean and saves it
     * in b. Shows a warning message on standard error and doesn't modify b if the YAML data is not
     * a boolean or doesn't exist.
     */
    void subnode_as_bool(const cv::FileNode& base_node,
                         const std::string&  subnode_name,
                         bool&               b);

    /** Interprets the data in the subnode of base_node named subnode_name as an int and saves it in
     * i. Shows a warning message on standard error and doesn't modify i if the YAML data is not an
     * int or doesn't exist.
     */
    void subnode_as_int(const cv::FileNode& base_node,
                        const std::string&  subnode_name,
                        int&                i);

    /** Interprets the data in the subnode of base_node named subnode_name as a float and saves it
     * in f. Shows a warning message on standard error and doesn't modify f if the YAML data is not
     * a float or doesn't exist.
     */
    void subnode_as_float(const cv::FileNode& base_node,
                          const std::string&  subnode_name,
                          float&              f);

    /** Interprets the data in the subnode of base_node named subnode_name as a string and saves it
     * in s. Shows a warning message on standard error and doesn't modify s if the YAML data is not
     * a string or doesn't exist.
     */
    void subnode_as_string(const cv::FileNode& base_node,
                           const std::string&  subnode_name,
                           std::string&        s);

    /** Interprets the data in the subnode of base_node named subnode_name as an std::vector and
     * saves it in v. Shows a warning message on standard error and doesn't modify v if the YAML
     * data is not a list or doesn't exist.
     */
    template<typename T>
    void subnode_as_vector(const cv::FileNode& base_node,
                           const std::string&  subnode_name,
                           std::vector<T>&     v);

    /** Interprets the data in the subnode of base_node named subnode_name as an Eigen::Vector3f and
     * saves it in v. Shows a warning message on standard error and doesn't modify v if the YAML
     * data is not a list or doesn't exist.
     */
    void subnode_as_vector3f(const cv::FileNode& base_node,
                             const std::string&  subnode_name,
                             Eigen::Vector3f&    v);
  } // namespace yaml
} // namespace se

#include "impl/yaml_impl.hpp"

#endif // SE_YAML_HPP

