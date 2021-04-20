#include "se/str_utils.hpp"

#include <algorithm>



namespace str_utils {

  bool begins_with(const std::string& s, const std::string& prefix) {
    if (s.size() >= prefix.size()) {
      return (s.compare(0, prefix.length(), prefix) == 0);
    } else {
      return false;
    }
  }

  bool ends_with(const std::string& s, const std::string& suffix) {
    if (s.size() >= suffix.size()) {
      return (s.compare(s.length() - suffix.length(), suffix.length(), suffix) == 0);
    } else {
      return false;
    }
  }

  std::vector <std::string> split_str(
      const std::string &s,
      const char delim,
      const bool ignore_consec) {

    std::vector <std::string> elems;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
      // Empty items result from consecutive occurences of the delimiter.
      if (!ignore_consec || (ignore_consec && (item.size() > 0))) {
        elems.push_back(item);
      }
    }
    return elems;
  }


  void to_lower(std::string& s) {
    std::transform(s.begin(), s.end(), s.begin(),
        [](unsigned char c){ return std::tolower(c); });
  }


  void to_upper(std::string& s) {
    std::transform(s.begin(), s.end(), s.begin(),
        [](unsigned char c){ return std::toupper(c); });
  }


  std::string bool_to_pretty_str(const bool state,
                                 const std::string state_name,
                                 const int width) {
    std::string l_side = (state_name != "") ? state_name + ":" : "";
    int padding = (width - 1) - state_name.length();
    if (padding > 0) {
      l_side.append(padding, ' ');
    }

    return l_side + (state ? "true" : "false");
  }


  std::string str_to_pretty_str(const std::string string,
                                const std::string string_name,
                                const int width) {
    std::string l_side = (string_name != "") ? string_name + ":" : "";
    int padding = (width - 1) - string_name.length();
    if (padding > 0) {
      l_side.append(padding, ' ');
    }

    return l_side + string;
  }


  std::string header_to_pretty_str(const std::string header_name,
                                   const int width) {
    int l_width_1;
    float frac = (float) width / header_name.size();
    if (frac > 2) {
      l_width_1 = round((float) width / 4);
    } else if (frac > 1) {
      l_width_1 = (float) (width - header_name.size()) / 2 - 1;
    } else {
      return header_name;
    }
    int l_width_2 = round((float) (width - header_name.size()) / 2) - l_width_1;
    int r_width_1 = l_width_1;
    int r_width_2 = width - l_width_1 - r_width_1 - l_width_2 - header_name.size();

    std::string l_side;
    l_side.append(l_width_1, '=');
    l_side.append(l_width_2, ' ');

    std::string r_side;
    r_side.append(r_width_2, ' ');
    r_side.append(r_width_1, '=');

    return l_side + header_name + r_side;
  }

} // namespace str_utils


