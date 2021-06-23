#ifndef SE_DATA_HPP
#define SE_DATA_HPP

#include <cstdint>

#include "utils/type_util.hpp"
#include "utils/setup_util.hpp"

namespace se {

// Typedefs and defaults

static const se::field_t dflt_tsdf = 1.f;

typedef int weight_t;
static const weight_t dflt_weight = 0;

static const se::field_t dflt_occupancy = 0.f;

typedef float time_stamp_t;
static const time_stamp_t dflt_time_stamp = 0.f;

typedef uint32_t rgba_t;
static const rgba_t dflt_rgba = 0xFFFFFFFF; // White

typedef short semantics_t;
static const semantics_t  dflt_semantics = 0;

template<se::Field FieldT>
struct FieldData
{
};

template<>
struct FieldData<se::Field::Occupancy>
{
    FieldData() : occupancy(dflt_occupancy), time_stamp(dflt_time_stamp) {}
    se::field_t  occupancy;
    time_stamp_t time_stamp;
    static constexpr bool invert_normals = false;
};

template<>
struct FieldData<se::Field::TSDF>
{
    FieldData() : tsdf(dflt_tsdf), weight(dflt_weight) {}
    se::field_t tsdf;
    weight_t    weight; // TODO: int or float
    static constexpr bool invert_normals = true;
};

// Colour data
template<se::Colour ColB>
struct ColourData
{
};

template<>
struct ColourData<se::Colour::On>
{
    ColourData() : rgba(dflt_rgba) {}
    rgba_t rgba;
};

// Semantic data
template<se::Semantics SemB>
struct SemanticData
{
};

// Semantic data
template<>
struct SemanticData<se::Semantics::On>
{
    SemanticData() : sem(dflt_semantics) {}
    semantics_t sem;
};

template <se::Field     FldT = se::Field::TSDF,
          se::Colour    ColB = se::Colour::Off,
          se::Semantics SemB = se::Semantics::Off
>
struct Data : public FieldData<FldT>, ColourData<ColB>, SemanticData<SemB>
{
  static constexpr se::Field     fld_ = FldT;
  static constexpr se::Colour    col_ = ColB;
  static constexpr se::Semantics sem_ = SemB;
};



///////////////////
/// DELTA DATA  ///
///////////////////

template<se::Field FieldT>
struct FieldDeltaData
{
};

template<>
struct FieldDeltaData<se::Field::Occupancy>
{
    FieldDeltaData() : delta_occupancy(0) {}
    se::field_t  delta_occupancy;
};

template<>
struct FieldDeltaData<se::Field::TSDF>
{
    FieldDeltaData() : delta_tsdf(0), delta_weight(0) {}
    se::field_t delta_tsdf;
    weight_t    delta_weight; // TODO: int or float
};

// Colour data
template<se::Colour ColB>
struct ColourDeltaData
{
};

template<>
struct ColourDeltaData<se::Colour::On>
{
    ColourDeltaData() : delta_rgba(0) {}
    rgba_t delta_rgba;
};



template <se::Field     FldT = se::Field::TSDF,
        se::Colour    ColB = se::Colour::Off,
        se::Semantics SemB = se::Semantics::Off
>
struct DeltaData : public FieldDeltaData<FldT>, ColourDeltaData<ColB>
{
    static constexpr se::Field     fld_ = FldT;
    static constexpr se::Colour    col_ = ColB;
    static constexpr se::Semantics sem_ = SemB;
};



///////////////////
/// DATA CONFIG ///
///////////////////

template<se::Field FieldT>
struct FieldDataConfig
{
};

template<>
struct FieldDataConfig<se::Field::Occupancy>
{
    field_t min_occupancy;
    field_t max_occupancy;
    field_t surface_boundary;

    /** Initializes the config to some sensible defaults.
     */
    FieldDataConfig();

    /** Initializes the config from a YAML file. Data not present in the YAML file will be
     * initialized as in FieldDataConfig<se::Field::Occupancy>::FieldDataConfig().
     */
    FieldDataConfig(const std::string& yaml_file);

    static constexpr se::Field FldT = se::Field::Occupancy;
};

std::ostream& operator<<(std::ostream& os, const FieldDataConfig<se::Field::Occupancy>& c);

template<>
struct FieldDataConfig<se::Field::TSDF>
{
    float       truncation_boundary;
    weight_t    max_weight; // TODO: int or float

    /** Initializes the config to some sensible defaults.
     */
    FieldDataConfig();

    /** Initializes the config from a YAML file. Data not present in the YAML file will be
     * initialized as in FieldDataConfig<se::Field::TSDF>::FieldDataConfig().
     */
    FieldDataConfig(const std::string& yaml_file);

    static constexpr se::Field FldT = se::Field::TSDF;
};

std::ostream& operator<<(std::ostream& os, const FieldDataConfig<se::Field::TSDF>& c);



// Colour data
template<se::Colour ColB>
struct ColourDataConfig
{
  ColourDataConfig() {}
  ColourDataConfig(const std::string& /* yaml_file */) {}
};

template<se::Colour ColB>
std::ostream& operator<<(std::ostream& os, const ColourDataConfig<ColB>& /* c */)
{
  return os;
}

template<>
struct ColourDataConfig<se::Colour::On>
{
  /** Initializes the config to some sensible defaults.
   */
  ColourDataConfig();

  /** Initializes the config from a YAML file. Data not present in the YAML file will be
   * initialized as in ColourDataConfig<se::Colour::On>::ColourDataConfig().
   */
  ColourDataConfig(const std::string& yaml_file);
};

std::ostream& operator<<(std::ostream& os, const ColourDataConfig<se::Colour::On>& c);



// Semantic data
template<se::Semantics SemB>
struct SemanticDataConfig
{
  SemanticDataConfig() {}
  SemanticDataConfig(const std::string& /* yaml_file */) {}
};

template<se::Semantics SemB>
std::ostream& operator<<(std::ostream& os, const SemanticDataConfig<SemB>& /* c */)
{
  return os;
}

// Semantic data
template<>
struct SemanticDataConfig<se::Semantics::On>
{
  /** Initializes the config to some sensible defaults.
   */
  SemanticDataConfig();

  /** Initializes the config from a YAML file. Data not present in the YAML file will be
   * initialized as in SemanticDataConfig<se::Semantics::On>SemanticDataConfig().
   */
  SemanticDataConfig(const std::string& yaml_file);
};

std::ostream& operator<<(std::ostream& os, const SemanticDataConfig<se::Semantics::On>& c);



template <se::Field     FldT = se::Field::TSDF,
          se::Colour    ColB = se::Colour::Off,
          se::Semantics SemB = se::Semantics::Off
>
struct DataConfig : public FieldDataConfig<FldT>, ColourDataConfig<ColB>, SemanticDataConfig<SemB>
{
  static constexpr se::Field     fld_ = FldT;
  static constexpr se::Colour    col_ = ColB;
  static constexpr se::Semantics sem_ = SemB;

  /** Initializes all sub-configs to their sensible defaults.
   */
  DataConfig()
  {
  }

  /** Initializes the config from a YAML file. Data not present in the YAML file will be
   * initialized as in DataConfig::DataConfig().
   */
  DataConfig(const std::string& yaml_file)
    : FieldDataConfig<FldT>(yaml_file), ColourDataConfig<ColB>(yaml_file),
      SemanticDataConfig<SemB>(yaml_file)
  {
  }
};

template <se::Field     FldT,
          se::Colour    ColB,
          se::Semantics SemB
>
std::ostream& operator<<(std::ostream& os, const DataConfig<FldT, ColB, SemB>& c)
{
  // Call the operator<< of the base classes.
  os << *static_cast<const FieldDataConfig<FldT>*>(&c);
  os << *static_cast<const ColourDataConfig<ColB>*>(&c);
  os << *static_cast<const SemanticDataConfig<SemB>*>(&c);
  return os;
}



template <se::Field     FldT,
          se::Colour    ColB,
          se::Semantics SemB
>
inline void set_invalid(Data<FldT, ColB, SemB>& data);

template <se::Colour    ColB,
          se::Semantics SemB
>
inline void set_invalid(Data<se::Field::TSDF, ColB, SemB>& data) { data.weight = dflt_tsdf; }

template <se::Colour    ColB,
          se::Semantics SemB
>
inline void set_invalid(Data<se::Field::Occupancy, ColB, SemB>& data) { data.time_stamp != dflt_time_stamp; }



template <se::Field     FldT,
          se::Colour    ColB,
          se::Semantics SemB
>
inline bool is_valid(const Data<FldT, ColB, SemB>& data);

template <se::Colour    ColB,
          se::Semantics SemB
>
inline bool is_valid(const Data<se::Field::TSDF, ColB, SemB>& data) { return data.weight != dflt_weight; }

template <se::Colour    ColB,
          se::Semantics SemB
>
inline bool is_valid(const Data<se::Field::Occupancy, ColB, SemB>& data) { return data.weight != dflt_weight; }


template <se::Field     FldT,
          se::Colour    ColB,
          se::Semantics SemB
>
inline bool is_invalid(const Data<FldT, ColB, SemB>& data);

template <se::Colour    ColB,
          se::Semantics SemB
>
inline bool is_invalid(const Data<se::Field::TSDF, ColB, SemB>& data) { return data.weight == dflt_weight; }

template <se::Colour    ColB,
         se::Semantics SemB
>
inline bool is_invalid(const Data<se::Field::Occupancy, ColB, SemB>& data) { return data.weight == dflt_weight; }


template <se::Field     FldT,
          se::Colour    ColB,
          se::Semantics SemB
>
inline float get_field(const Data<FldT, ColB, SemB> data);

template <se::Colour    ColB,
          se::Semantics SemB
>
inline float get_field(const Data<se::Field::TSDF, ColB, SemB> data) { return data.tsdf; }

template <se::Colour    ColB,
        se::Semantics SemB
>
inline float get_field(const Data<se::Field::Occupancy, ColB, SemB> data) { return data.occupancy; }



template <se::Field     FldT,
          se::Colour    ColB,
          se::Semantics SemB
>
inline float is_inside(const Data<FldT, ColB, SemB>& data);

template <se::Colour    ColB,
          se::Semantics SemB
>
inline float is_inside(const Data<se::Field::TSDF, ColB, SemB>& data) { return data.tsdf < 0.f; }

template <se::Colour    ColB,
          se::Semantics SemB
>
inline float is_inside(const Data<se::Field::Occupancy, ColB, SemB>& data) { return data.occupancy > 0.f; }



// Occupancy data setups
typedef Data<se::Field::Occupancy, se::Colour::Off, se::Semantics::Off> OccData;
typedef Data<se::Field::Occupancy, se::Colour::On,  se::Semantics::Off> OccColData;
typedef Data<se::Field::Occupancy, se::Colour::Off, se::Semantics::On>  OccSemData;
typedef Data<se::Field::Occupancy, se::Colour::On,  se::Semantics::On>  OccColSemData;

// Occupancy data setups
typedef DataConfig<se::Field::Occupancy, se::Colour::Off, se::Semantics::Off> OccDataConfig;
typedef DataConfig<se::Field::Occupancy, se::Colour::On,  se::Semantics::Off> OccColDataConfig;
typedef DataConfig<se::Field::Occupancy, se::Colour::Off, se::Semantics::On>  OccSemDataConfig;
typedef DataConfig<se::Field::Occupancy, se::Colour::On,  se::Semantics::On>  OccColSemDataConfig;

// TSDF data setups
typedef Data<se::Field::TSDF, se::Colour::Off, se::Semantics::Off> TSDFData;
typedef Data<se::Field::TSDF, se::Colour::On,  se::Semantics::Off> TSDFColData;
typedef Data<se::Field::TSDF, se::Colour::Off, se::Semantics::On>  TSDFSemData;
typedef Data<se::Field::TSDF, se::Colour::On,  se::Semantics::On>  TSDFColSemData;

typedef DataConfig<se::Field::TSDF, se::Colour::Off, se::Semantics::Off> TSDFDataConfig;
typedef DataConfig<se::Field::TSDF, se::Colour::On,  se::Semantics::Off> TSDFColDataConfig;
typedef DataConfig<se::Field::TSDF, se::Colour::Off, se::Semantics::On>  TSDFSemDataConfig;
typedef DataConfig<se::Field::TSDF, se::Colour::On,  se::Semantics::On>  TSDFColSemDataConfig;

} // namespace se

#endif //SE_DATA_HPP
