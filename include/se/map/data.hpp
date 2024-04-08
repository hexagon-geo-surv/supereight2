/*
 * SPDX-FileCopyrightText: 2021-2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021-2022 Nils Funk
 * SPDX-FileCopyrightText: 2021-2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_DATA_HPP
#define SE_DATA_HPP

#include <array>
#include <se/common/bounded_vector.hpp>

#include "data_colour.hpp"
#include "data_field.hpp"
#include "data_semantics.hpp"
#include "utils/setup_util.hpp"
#include "utils/type_util.hpp"

namespace se {

template<Field FldT = Field::TSDF, Colour ColB = Colour::Off, Semantics SemB = Semantics::Off>
struct Data {
    typedef FieldData<FldT> FieldType;
    typedef ColourData<ColB> ColourType;
    typedef SemanticData<SemB> SemanticType;

    FieldType field;
    ColourType colour;
    SemanticType semantic;

    struct Config {
        typename FieldType::Config field;
        typename ColourType::Config colour;
        typename SemanticType::Config semantic;

        /** Reads the struct members from the "data" node of a YAML file. Members not present in the
         * YAML file aren't modified.
         */
        void readYaml(const std::string& yaml_file)
        {
            field.readYaml(yaml_file);
            colour.readYaml(yaml_file);
            semantic.readYaml(yaml_file);
        }
    };

    static constexpr Field fld_ = FldT;
    static constexpr Colour col_ = ColB;
    static constexpr Semantics sem_ = SemB;
    static constexpr bool invert_normals = FieldData<FldT>::invert_normals;
    static constexpr field_t surface_boundary = FieldData<FldT>::surface_boundary;
};

template<Field FldT, Colour ColB, Semantics SemB>
std::ostream& operator<<(std::ostream& os, const typename Data<FldT, ColB, SemB>::Config& c)
{
    os << c.field;
    operator<<<ColB>(os, c.colour);
    operator<<<SemB>(os, c.semantic);
    return os;
}



///////////////////
/// DELTA DATA  ///
///////////////////

template<Field FldT = Field::TSDF, Colour ColB = Colour::Off, Semantics SemB = Semantics::Off>
struct DeltaData {
    FieldDeltaData<FldT> field;
    ColourDeltaData<ColB> colour;

    static constexpr Field fld_ = FldT;
    static constexpr Colour col_ = ColB;
    static constexpr Semantics sem_ = SemB;
};



namespace data {

/** Up-propagate the mean of the valid \p child_data into \p parent_data and return the number of
 * children with valid data.
 */
template<Field FldT, Colour ColB, Semantics SemB>
int up_prop_mean(Data<FldT, ColB, SemB>& parent_data,
                 const std::array<Data<FldT, ColB, SemB>, 8>& child_data);

/** Up-propagate the minimum of the valid \p child_data into \p parent_data and return the number of
 * children with valid data.
 */
template<Field FldT, Colour ColB, Semantics SemB>
int up_prop_min(Data<FldT, ColB, SemB>& parent_min_data,
                const std::array<Data<FldT, ColB, SemB>, 8>& child_min_data);

/** Up-propagate the maximum of the valid \p child_data into \p parent_data and return the number of
 * children with valid data.
 */
template<Field FldT, Colour ColB, Semantics SemB>
int up_prop_max(Data<FldT, ColB, SemB>& parent_max_data,
                const std::array<Data<FldT, ColB, SemB>, 8>& child_max_data);

} // namespace data

template<Field FldT, Colour ColB, Semantics SemB>
inline void set_invalid(Data<FldT, ColB, SemB>& data);

template<Colour ColB, Semantics SemB>
inline void set_invalid(Data<Field::TSDF, ColB, SemB>& data)
{
    data = Data<Field::TSDF, ColB, SemB>();
}

template<Colour ColB, Semantics SemB>
inline void set_invalid(Data<Field::Occupancy, ColB, SemB>& data)
{
    data = Data<Field::Occupancy, ColB, SemB>();
}



template<Field FldT, Colour ColB, Semantics SemB>
inline bool is_valid(const Data<FldT, ColB, SemB>& data)
{
    return data.field.valid();
}



template<Field FldT, Colour ColB, Semantics SemB>
inline field_t get_field(const Data<FldT, ColB, SemB>& data);

template<Colour ColB, Semantics SemB>
inline field_t get_field(const Data<Field::TSDF, ColB, SemB>& data)
{
    return data.field.tsdf;
}

template<Colour ColB, Semantics SemB>
inline field_t get_field(const Data<Field::Occupancy, ColB, SemB>& data)
{
    return data.field.occupancy * data.field.weight;
}



template<Field FldT, Colour ColB, Semantics SemB>
inline bool is_inside(const Data<FldT, ColB, SemB>& data);

template<Colour ColB, Semantics SemB>
inline bool is_inside(const Data<Field::TSDF, ColB, SemB>& data)
{
    return get_field(data) < Data<Field::TSDF, ColB, SemB>::surface_boundary;
}

template<Colour ColB, Semantics SemB>
inline bool is_inside(const Data<Field::Occupancy, ColB, SemB>& data)
{
    return get_field(data) > Data<Field::Occupancy, ColB, SemB>::surface_boundary;
}



// Occupancy data setups
typedef Data<Field::Occupancy, Colour::Off, Semantics::Off> OccupancyData;
typedef Data<Field::Occupancy, Colour::On, Semantics::Off> OccupancyColData;
typedef Data<Field::Occupancy, Colour::Off, Semantics::On> OccupancySemData;
typedef Data<Field::Occupancy, Colour::On, Semantics::On> OccupancyColSemData;

// TSDF data setups
typedef Data<Field::TSDF, Colour::Off, Semantics::Off> TSDFData;
typedef Data<Field::TSDF, Colour::On, Semantics::Off> TSDFColData;
typedef Data<Field::TSDF, Colour::Off, Semantics::On> TSDFSemData;
typedef Data<Field::TSDF, Colour::On, Semantics::On> TSDFColSemData;

} // namespace se

#include "impl/data_impl.hpp"

#endif // SE_DATA_HPP
