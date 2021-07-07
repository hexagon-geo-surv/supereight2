#ifndef SE_STR_UTILS_HPP
#define SE_STR_UTILS_HPP

#include <Eigen/Dense>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace str_utils {

  bool begins_with(const std::string& s, const std::string& prefix);

  bool ends_with(const std::string& s, const std::string& suffix);

  bool is_int(const std::string& s, const bool accept_negative = true);

  bool is_float(const std::string& s, const bool accept_negative = true);

  void remove_prefix(std::string& s, const std::string& prefix);

  void remove_suffix(std::string& s, const std::string& suffix);

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
  std::string matrix_to_pretty_str(const MatrixT&     M,
                                   const std::string& M_name = "",
                                   const int          width = default_width);

  template<typename VectorT>
  std::string vector_to_pretty_str(const VectorT&                  v,
                                   const std::string&              v_name = "",
                                   const std::vector<std::string>& e_names = {},
                                   const int                       width = default_width);

  template<typename Vector3T>
  std::string volume_to_pretty_str(const Vector3T&    vol,
                                   const std::string& vol_name = "",
                                   const std::string& vol_unit = "",
                                   const int          width = default_width);

  std::string bool_to_pretty_str(const bool         state,
                                 const std::string& state_name = "",
                                 const int          width = default_width);

  std::string str_to_pretty_str(const std::string& string,
                                const std::string& string_name = "",
                                const int          width = default_width);

  template<typename ValueT>
  std::string value_to_pretty_str(const ValueT&      val,
                                  const std::string& val_name = "",
                                  const std::string& val_unit = "",
                                  const int          width = default_width);

  std::string header_to_pretty_str(const std::string& header_name,
                                   const int          width = default_width);

  /** Return the result of expanding a leading ~ in path.
   * \note Also expands environment variables since it uses wordexp(3) internally.
   * \note Currently only implemented for POSIX systems.
   */
  std::string expand_user(const std::string& path);

} // namespace str_utils

#include "impl/str_utils_impl.hpp"

#endif // SE_STR_UTILS_HPP

