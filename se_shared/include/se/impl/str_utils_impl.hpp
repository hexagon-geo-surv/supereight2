#ifndef __STR_UTILS_IMPL_HPP
#define __STR_UTILS_IMPL_HPP

#include <sstream>
#include <iomanip>



namespace str_utils {

  template<typename MatrixT>
  std::string matrix_to_pretty_str(const MatrixT &M,
                                   std::string M_name,
                                   const int width) {
    M_name = (M_name != "") ? M_name + ":\n" : "";
    std::string l_side = "";
    l_side.append(width, ' ');
    l_side += "[";
    Eigen::IOFormat MatrixFmt(6, 0,
                              ", ", "\n",
                              l_side, "]",
                              M_name, "");

    std::stringstream ss;
    ss << M.format(MatrixFmt);
    return ss.str();
  }



  template<typename VectorT>
  std::string vector_to_pretty_str(const VectorT &v,
                                   const std::string v_name,
                                   const std::vector <std::string> e_names,
                                   const int width) {
    std::string l_side = (v_name != "") ? v_name + ":" : "";
    int padding = (width - 1) - v_name.length();
    if (padding > 0) {
      l_side.append(padding, ' ');
    }

    std::stringstream ss;
    if (e_names.empty()) {
      l_side += "[";

      Eigen::IOFormat VectorFmt(6, Eigen::DontAlignCols,
                                "", ", ", "", "",
                                l_side, "]");
      ss << v.format(VectorFmt);
    } else {
      ss << l_side;
      int i_max = std::min((int) v.size(), (int) e_names.size());
      for (int i = 0; i < i_max; i++) {
        ss << e_names[i] << ": " << v[i];
        if (i < i_max - 1) {
          ss << ", ";
        }
      }
    }
    return ss.str();
  }


  template<typename Vector3T>
  std::string volume_to_pretty_str(const Vector3T &vol,
                                   const std::string vol_name,
                                   const std::string vol_unit,
                                   const int width) {
    std::string l_side = (vol_name != "") ? vol_name + ":" : "";
    int padding = (width - 1) - vol_name.length();
    if (padding > 0) {
      l_side.append(padding, ' ');
    }

    Eigen::IOFormat VolumeFmt(6, Eigen::DontAlignCols,
                              "", " x ", "", "",
                              l_side, " " + vol_unit);

    std::stringstream ss;
    ss << vol.format(VolumeFmt);
    return ss.str();
  }


  template<typename ValueT>
  std::string value_to_pretty_str(const ValueT val,
                                  const std::string val_name,
                                  const std::string val_unit,
                                  const int width) {
    std::string l_side = (val_name != "") ? val_name + ":" : "";
    std::string r_side = (val_unit != "") ? " " + val_unit : "";
    int padding = (width - 1) - val_name.length();
    if (padding > 0) {
      l_side.append(padding, ' ');
    }

    std::stringstream ss;
    ss << l_side << std::setprecision(6) << val << r_side;
    return ss.str();
  }

} // namespace str_utils



#endif // STR_UTILS_IMPL_HPP

