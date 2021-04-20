#ifndef SE_DATA_HPP
#define SE_DATA_HPP

#include <cstdint>

#include "utils/setup_util.hpp"

namespace se {

// Typedefs and defaults

typedef float tsdf_t;
static const tsdf_t dflt_tsdf = 0.f;

typedef int weight_t;
static const weight_t dflt_weight = 0;

typedef float occupancy_t;
static const occupancy_t dflt_occupancy = 0.f;

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
    occupancy_t  occupancy;
    time_stamp_t time_stamp;
};

template<>
struct FieldData<se::Field::TSDF>
{
    FieldData() : tsdf(dflt_tsdf), weight(dflt_weight) {}
    tsdf_t   tsdf;
    weight_t weight; // TODO: int or float
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
public:
    static constexpr se::Field     fld_ = FldT;
    static constexpr se::Colour    col_ = ColB;
    static constexpr se::Semantics sem_ = SemB;
};


template <se::Field     FldT,
          se::Colour    ColB,
          se::Semantics SemB
>
void set_invalid(Data<FldT, ColB, SemB>& data);

template <se::Colour    ColB,
          se::Semantics SemB
>
void set_invalid(Data<se::Field::TSDF, ColB, SemB>& data) {
  data.weight = dflt_tsdf;
}

template <se::Colour    ColB,
          se::Semantics SemB
>
void set_invalid(Data<se::Field::Occupancy, ColB, SemB>& data) {
  data.time_stamp != dflt_time_stamp;
}



template <se::Field     FldT,
          se::Colour    ColB,
          se::Semantics SemB
>
bool is_valid(const Data<FldT, ColB, SemB>& data);

template <se::Colour    ColB,
          se::Semantics SemB
>
bool is_valid(const Data<se::Field::TSDF, ColB, SemB>& data) {
  return data.weight != dflt_weight;
}

template <se::Colour    ColB,
          se::Semantics SemB
>
void is_valid(const Data<se::Field::Occupancy, ColB, SemB>& data) {
  data.time_stamp != dflt_time_stamp;
}



template <se::Field     FldT,
        se::Colour    ColB,
        se::Semantics SemB
>
bool is_invalid(const Data<FldT, ColB, SemB>& data);

template <se::Colour    ColB,
        se::Semantics SemB
>
bool is_invalid(const Data<se::Field::TSDF, ColB, SemB>& data) {
  return data.weight == 0;
}

template <se::Colour    ColB,
        se::Semantics SemB
>
void is_invalid(const Data<se::Field::Occupancy, ColB, SemB>& data) {
  data.time_stamp == 0.f;
}



template <se::Field     FldT,
          se::Colour    ColB,
          se::Semantics SemB
>
float get_field(const Data<FldT, ColB, SemB>& data);

template <se::Colour    ColB,
          se::Semantics SemB
>
float get_field(const Data<se::Field::TSDF, ColB, SemB>& data) {
  return data.tsdf;
}

template <se::Colour    ColB,
        se::Semantics SemB
>
float get_field(const Data<se::Field::Occupancy, ColB, SemB>& data) {
  return data.occupancy;
}



template <se::Field     FldT,
          se::Colour    ColB,
          se::Semantics SemB
>
float is_inside(const Data<FldT, ColB, SemB>& data);

template <se::Colour    ColB,
          se::Semantics SemB
>
float is_inside(const Data<se::Field::TSDF, ColB, SemB>& data) {
  return data.tsdf < 0.f;
}

template <se::Colour    ColB,
          se::Semantics SemB
>
float is_inside(const Data<se::Field::Occupancy, ColB, SemB>& data) {
  return data.occupancy > 0.f;
}



// Occupancy data setups
typedef Data<se::Field::Occupancy, se::Colour::Off, se::Semantics::Off> OccData;
typedef Data<se::Field::Occupancy, se::Colour::On,  se::Semantics::Off>  OccColData;
typedef Data<se::Field::Occupancy, se::Colour::Off, se::Semantics::On>  OccSemData;
typedef Data<se::Field::Occupancy, se::Colour::On,  se::Semantics::On>   OccColSemData;

// TSDF data setups
typedef Data<se::Field::TSDF, se::Colour::Off, se::Semantics::Off> TSDFData;
typedef Data<se::Field::TSDF, se::Colour::On,  se::Semantics::Off>  TSDFColData;
typedef Data<se::Field::TSDF, se::Colour::Off, se::Semantics::On>  TSDFSemData;
typedef Data<se::Field::TSDF, se::Colour::On,  se::Semantics::On>   TSDFColSemData;

} // namespace se

#endif //SE_DATA_HPP
