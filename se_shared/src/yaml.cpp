// SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London
// SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou, Imperial College London
// SPDX-License-Identifier: BSD-3-Clause

#include "se/yaml.hpp"

#include "se/utils/str_utils.hpp"

namespace se {
  namespace yaml {
    void subnode_as_bool(const cv::FileNode& base_node,
                         const std::string&  subnode_name,
                         bool&               b)
    {
      const cv::FileNode subnode = base_node[subnode_name];
      if (subnode.isString()) {
        // The OpenCV YAML reader does not support booleans, we have to parse them as strings.
        std::string s = static_cast<std::string>(subnode);
        str_utils::to_lower(s);
        if (s == "true") {
          b = true;
        } else if (s == "false") {
          b = false;
        } else {
          std::cerr << "Warning: ignoring non-bool data in " << subnode_name
            << ", using default value \"" << b << "\"\n";
        }
      } else {
        std::cerr << "Warning: ignoring non-bool data in " << subnode_name
          << ", using default value \"" << b << "\"\n";
      }
    }



    void subnode_as_int(const cv::FileNode& base_node,
                        const std::string&  subnode_name,
                        int&                i)
    {
      const cv::FileNode subnode = base_node[subnode_name];
      if (subnode.isInt()) {
        i = static_cast<int>(subnode);
      } else {
        std::cerr << "Warning: ignoring non-int data in " << subnode_name
          << ", using default value \"" << i << "\"\n";
      }
    }



    void subnode_as_float(const cv::FileNode& base_node,
                          const std::string&  subnode_name,
                          float&              f)
    {
      const cv::FileNode subnode = base_node[subnode_name];
      if (subnode.isReal() || subnode.isInt()) {
        f = static_cast<float>(subnode);
      } else {
        std::cerr << "Warning: ignoring non-float data in " << subnode_name
          << ", using default value \"" << f << "\"\n";
      }
    }



    void subnode_as_string(const cv::FileNode& base_node,
                           const std::string&  subnode_name,
                           std::string&        s)
    {
      const cv::FileNode subnode = base_node[subnode_name];
      if (subnode.isString()) {
        s = static_cast<std::string>(subnode);
      } else {
        std::cerr << "Warning: ignoring non-string data in " << subnode_name
          << ", using default value \"" << s << "\"\n";
      }
    }



    void subnode_as_vector3f(const cv::FileNode& base_node,
                             const std::string&  subnode_name,
                             Eigen::Vector3f&    v)
    {
      const cv::FileNode subnode = base_node[subnode_name];
      if (subnode.isSeq() && subnode.size() == 3) {
        v = Eigen::Vector3f(subnode[0], subnode[1], subnode[2]);
      } else {
        // Show warnings on invalid data
        std::cerr << "Warning: ";
        if (subnode.isSeq() && subnode.size() != 3) {
          std::cerr << "expected list of length 3 for " << subnode_name
            << " but got length " << subnode.size();
        } else if (subnode.isNone()) {
          std::cerr << "no data for " << subnode_name;
        } else {
          std::cerr << "ignoring non-list data in " << subnode_name;
        }
        std::cerr << ", using default value [" << v.x() << ", " << v.y() << ", " << v.z() << "]\n";
      }
    }
  } // namespace yaml
} // namespace se

