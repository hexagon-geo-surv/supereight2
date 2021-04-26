#ifndef __STR_UTILS_HPP
#define __STR_UTILS_HPP

#include <string>
#include <vector>
#include <Eigen/Dense>



namespace str_utils {

  bool begins_with(const std::string& s, const std::string& prefix);

  bool ends_with(const std::string& s, const std::string& suffix);

/**
 * Split a string into a vector of substrings based on a delimiter.
 *
 * \param[in] s The string to split.
 * \param[in] delim The delimiter to use.
 * \param[in] ignore_consec Treat consecutive delimiters as a single delimiter.
 * \return The vector containing the substrings.
 */
  std::vector <std::string> split_str(const std::string &s,
                                      const char delim,
                                      const bool ignore_consec = false);

  void to_lower(std::string& s);

  void to_upper(std::string& s);

  static constexpr int default_width = 33;

  template<typename MatrixT>
  std::string matrix_to_pretty_str(const MatrixT &M,
                                   std::string M_name = "",
                                   const int width = default_width);

  template<typename VectorT>
  std::string vector_to_pretty_str(const VectorT &v,
                                   const std::string v_name = "",
                                   const std::vector <std::string> e_names = {},
                                   const int width = default_width);

  template<typename Vector3T>
  std::string volume_to_pretty_str(const Vector3T &vol,
                                   const std::string vol_name = "",
                                   const std::string vol_unit = "",
                                   const int width = default_width);

  std::string bool_to_pretty_str(const bool state,
                                 const std::string state_name = "",
                                 const int width = default_width);

  std::string str_to_pretty_str(const std::string string,
                                const std::string string_name = "",
                                const int width = default_width);

  template<typename ValueT>
  std::string value_to_pretty_str(const ValueT val,
                                  const std::string val_name = "",
                                  const std::string val_unit = "",
                                  const int width = default_width);

  std::string header_to_pretty_str(const std::string header_name,
                                   const int width = default_width);


} // namespace str_utils

#endif // STR_UTILS_HPP

#include "se/impl/str_utils_impl.hpp"
