/*
 * SPDX-FileCopyrightText: 2021-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021-2022 Nils Funk
 * SPDX-FileCopyrightText: 2021-2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_DATA_HPP
#define SE_DATA_HPP

#include "data_colour.hpp"
#include "data_field.hpp"
#include "data_semantics.hpp"
#include "utils/setup_util.hpp"
#include "utils/type_util.hpp"

namespace se {

template<se::Field FldT = se::Field::TSDF,
         se::Colour ColB = se::Colour::Off,
         se::Semantics SemB = se::Semantics::Off>
struct Data : public FieldData<FldT>, ColourData<ColB>, SemanticData<SemB> {
    static constexpr se::Field fld_ = FldT;
    static constexpr se::Colour col_ = ColB;
    static constexpr se::Semantics sem_ = SemB;
};



///////////////////
/// DELTA DATA  ///
///////////////////

template<se::Field FldT = se::Field::TSDF,
         se::Colour ColB = se::Colour::Off,
         se::Semantics SemB = se::Semantics::Off>
struct DeltaData : public FieldDeltaData<FldT>, ColourDeltaData<ColB> {
    static constexpr se::Field fld_ = FldT;
    static constexpr se::Colour col_ = ColB;
    static constexpr se::Semantics sem_ = SemB;
};



///////////////////
/// DATA CONFIG ///
///////////////////

template<se::Field FldT = se::Field::TSDF,
         se::Colour ColB = se::Colour::Off,
         se::Semantics SemB = se::Semantics::Off>
struct DataConfig : public FieldDataConfig<FldT>, ColourDataConfig<ColB>, SemanticDataConfig<SemB> {
    static constexpr se::Field fld_ = FldT;
    static constexpr se::Colour col_ = ColB;
    static constexpr se::Semantics sem_ = SemB;

    /** Initializes all sub-configs to their sensible defaults.
     */
    DataConfig()
    {
    }

    /** Initializes the config from a YAML file. Data not present in the YAML file will be
     * initialized as in DataConfig::DataConfig().
     */
    DataConfig(const std::string& yaml_file) :
            FieldDataConfig<FldT>(yaml_file),
            ColourDataConfig<ColB>(yaml_file),
            SemanticDataConfig<SemB>(yaml_file)
    {
    }
};

template<se::Field FldT, se::Colour ColB, se::Semantics SemB>
std::ostream& operator<<(std::ostream& os, const DataConfig<FldT, ColB, SemB>& c)
{
    // Call the operator<< of the base classes.
    os << *static_cast<const FieldDataConfig<FldT>*>(&c);
    os << *static_cast<const ColourDataConfig<ColB>*>(&c);
    os << *static_cast<const SemanticDataConfig<SemB>*>(&c);
    return os;
}



template<se::Field FldT, se::Colour ColB, se::Semantics SemB>
inline void set_invalid(Data<FldT, ColB, SemB>& data);

template<se::Colour ColB, se::Semantics SemB>
inline void set_invalid(Data<se::Field::TSDF, ColB, SemB>& data)
{
    data = Data<se::Field::TSDF, ColB, SemB>();
}

template<se::Colour ColB, se::Semantics SemB>
inline void set_invalid(Data<se::Field::Occupancy, ColB, SemB>& data)
{
    data = Data<se::Field::Occupancy, ColB, SemB>();
}



template<se::Field FldT, se::Colour ColB, se::Semantics SemB>
inline bool is_valid(const Data<FldT, ColB, SemB>& data);

template<se::Colour ColB, se::Semantics SemB>
inline bool is_valid(const Data<se::Field::TSDF, ColB, SemB>& data)
{
    return data.weight != dflt_weight;
}

template<se::Colour ColB, se::Semantics SemB>
inline bool is_valid(const Data<se::Field::Occupancy, ColB, SemB>& data)
{
    return data.weight != dflt_weight;
}


template<se::Field FldT, se::Colour ColB, se::Semantics SemB>
inline bool is_invalid(const Data<FldT, ColB, SemB>& data);

template<se::Colour ColB, se::Semantics SemB>
inline bool is_invalid(const Data<se::Field::TSDF, ColB, SemB>& data)
{
    return data.weight == dflt_weight;
}

template<se::Colour ColB, se::Semantics SemB>
inline bool is_invalid(const Data<se::Field::Occupancy, ColB, SemB>& data)
{
    return data.weight == dflt_weight;
}


template<se::Field FldT, se::Colour ColB, se::Semantics SemB>
inline float get_field(const Data<FldT, ColB, SemB> data);

template<se::Colour ColB, se::Semantics SemB>
inline float get_field(const Data<se::Field::TSDF, ColB, SemB> data)
{
    return data.tsdf;
}

template<se::Colour ColB, se::Semantics SemB>
inline float get_field(const Data<se::Field::Occupancy, ColB, SemB> data)
{
    return data.occupancy;
}



template<se::Field FldT, se::Colour ColB, se::Semantics SemB>
inline float is_inside(const Data<FldT, ColB, SemB>& data);

template<se::Colour ColB, se::Semantics SemB>
inline float is_inside(const Data<se::Field::TSDF, ColB, SemB>& data)
{
    return data.tsdf < 0.f;
}

template<se::Colour ColB, se::Semantics SemB>
inline float is_inside(const Data<se::Field::Occupancy, ColB, SemB>& data)
{
    return data.occupancy > 0.f;
}



// Occupancy data setups
typedef Data<se::Field::Occupancy, se::Colour::Off, se::Semantics::Off> OccupancyData;
typedef Data<se::Field::Occupancy, se::Colour::On, se::Semantics::Off> OccupancyColData;
typedef Data<se::Field::Occupancy, se::Colour::Off, se::Semantics::On> OccupancySemData;
typedef Data<se::Field::Occupancy, se::Colour::On, se::Semantics::On> OccupancyColSemData;

// Occupancy data setups
typedef DataConfig<se::Field::Occupancy, se::Colour::Off, se::Semantics::Off> OccupancyDataConfig;
typedef DataConfig<se::Field::Occupancy, se::Colour::On, se::Semantics::Off> OccupancyColDataConfig;
typedef DataConfig<se::Field::Occupancy, se::Colour::Off, se::Semantics::On> OccupancySemDataConfig;
typedef DataConfig<se::Field::Occupancy, se::Colour::On, se::Semantics::On>
    OccupancyColSemDataConfig;

// TSDF data setups
typedef Data<se::Field::TSDF, se::Colour::Off, se::Semantics::Off> TSDFData;
typedef Data<se::Field::TSDF, se::Colour::On, se::Semantics::Off> TSDFColData;
typedef Data<se::Field::TSDF, se::Colour::Off, se::Semantics::On> TSDFSemData;
typedef Data<se::Field::TSDF, se::Colour::On, se::Semantics::On> TSDFColSemData;

typedef DataConfig<se::Field::TSDF, se::Colour::Off, se::Semantics::Off> TSDFDataConfig;
typedef DataConfig<se::Field::TSDF, se::Colour::On, se::Semantics::Off> TSDFColDataConfig;
typedef DataConfig<se::Field::TSDF, se::Colour::Off, se::Semantics::On> TSDFSemDataConfig;
typedef DataConfig<se::Field::TSDF, se::Colour::On, se::Semantics::On> TSDFColSemDataConfig;

} // namespace se

#endif // SE_DATA_HPP
