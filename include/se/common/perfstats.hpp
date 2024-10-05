/*
 * SPDX-FileCopyrightText: 2011-2013 Gerhard Reitmayr, TU Graz
 * SPDX-FileCopyrightText: 2014 University of Edinburgh, Imperial College, University of Manchester
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021-2024 Sotiris Papatheodorou
 * SPDX-License-Identifier: MIT
 */

#ifndef SE_PERFSTATS_HPP
#define SE_PERFSTATS_HPP

#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <map>
#include <mutex>
#include <numeric>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>

namespace se {

struct PerfStats {
    enum Type {
        BOOL,
        COORDINATES,
        COUNT,
        CURRENT,
        DISTANCE,
        DOUBLE,
        DURATION,
        ENERGY,
        FRAME,
        FREQUENCY,
        INT,
        ITERATION,
        MEMORY,
        ORIENTATION,
        PERCENTAGE,
        POSITION,
        POWER,
        TIME,
        UNDEFINED,
        VOLTAGE,
        VOLUME,
    };

    struct Stats {
        /// Static functions to summarise data of one iteration
        /**
         * \brief Compute the mean value of the stats data of an iteration.
         *
         * \param[in] iter_data_vec The vector containing all values to be processed.
         *
         * \return The mean of the values in the iter_data_vec.
         */
        static double meanIter(const std::vector<double>& iter_data_vec);

        /**
         * \brief Provide the last value of the stats data of an iteration.
         *
         * \param[in] iter_data_vec The vector containing all values to be processed.
         *
         * \return The last value in the iter_data_vec.
         */
        static double lastIter(const std::vector<double>& iter_data_vec);

        /**
         * \brief Compute the minimum value of the stats data of an iteration.
         *
         * \param[in] iter_data_vec The vector containing all values to be processed.
         *
         * \return The minimum value of the values in the iter_data_vec.
         */
        static double minIter(const std::vector<double>& iter_data_vec);

        /**
         * \brief Compute the maximum value of the stats data of an iteration.
         *
         * \param[in] iter_data_vec The vector containing all values to be processed.
         *
         * \return The maximum value of the values in the iter_data_vec.
         */
        static double maxIter(const std::vector<double>& iter_data_vec);

        /**
         * \brief Compute the sum of the values of the stats data of an iteration.
         *
         * \param[in] iter_data_vec The vector containing all values to be processed.
         *
         * \return The sum of the values in the iter_data_vec.
         */
        static double sumIter(const std::vector<double>& iter_data_vec);

        /**
         * \brief Merge the stats data of one interation into a scalar value.
         *        The merging strategy depends on the PerfStats::Type.
         *
         * \param[in] iter_data_vec The vector containing all values to be processed.
         *
         * \return The merged scalar value of the values in the iter_data_vec.
         */
        static double mergeIter(const std::vector<double>& iter_data_vec, const Type type);


        /// Functions to summarise data of one iteration
        /**
         * \brief Compute the mean value of the stats data of a given iteration.
         *
         * \param[in] iter The iteration to processed.
         *
         * \return The mean of the values of the given iteration.
         */
        double meanIter(const size_t iter);

        /**
         * \brief Provide the last value of the stats data of a given iteration.
         *
         * \param[in] iter The iteration to processed.
         *
         * \return The last value of the given iteration.
         */
        double lastIter(const size_t iter);

        /**
         * \brief Compute the minimum value of the stats data of a given iteration.
         *
         * \param[in] iter The iteration to processed.
         *
         * \return The minimum value of the given iteration.
         */
        double minIter(const size_t iter);

        /**
         * \brief Compute the maximum value of the stats data of a given iteration.
         *
         * \param[in] iter The iteration to processed.
         *
         * \return The maximum value of the given iteration.
         */
        double maxIter(const size_t iter);

        /**
         * \brief Compute the sum value of the stats data of a given iteration.
         *
         * \param[in] iter The iteration to processed.
         *
         * \return The sum of the values of the given iteration.
         */
        double sumIter(const size_t iter);

        /**
         * \brief Merge the stats data a given interation into a scalar value.
         *        The merging strategy depends on the PerfStats::Type.
         *
         * \param[in] iter The iteration to processed.
         *
         * \return The merged scalar value of the values of the given iteration.
         */
        double mergeIter(const size_t iter);


        /// Functions to summarise data of the entire stats
        /**
         * \brief Compute the mean value of all stats data stored so far.
         *
         * \return The mean of all stats data
         */
        double mean() const;

        /**
         * \brief Provide the last stored value to the stats data.
         *
         * \return The last value of the stats data
         */
        double last() const;

        /**
         * \brief Compute the minimum value of all stats data stored so far.
         *
         * \return The minimum value of all stats data.
         */
        double min() const;

        /**
         * \brief Compute the maximum value of all stats data stored so far.
         *
         * \return The maximum value of all stats data.
         */
        double max() const;

        /**
         * \brief Compute the sum of all stats data stored so far.
         *
         * \return The sum of all stats data.
         */
        double sum() const;

        /**
         * \brief Merge all stats data into a scalar value.
         *        The merging strategy depends on the PerfStats::Type.
         *
         * \return The merged scalar value of all stats data.
         */
        double merge() const;


        /**
         * \return The unit of the stats type as a std::string, e.g. "[V]" for PerfStats::Voltage.
         */
        std::string unitString();

        // <iteration/frame, vector of values at iteration/frame>
        std::map<size_t, std::vector<double>> data_;
        double last_absolute_; ///< The last absolute time the stat data was updated.
        std::mutex mutex_;
        Type type_; ///< The type of data stored in the stat struct.
    };

    struct Results {
        double mean;
        double min;
        double max;
        double sum;
    };

    PerfStats();

    /**
     * \brief
     *
     * \return The tab separated stats names with units.
     */
    std::string createHeaderString();

    std::string createDataIterString();

    std::string createDataIterString(const size_t iter);

    std::string createDataString();

    /**
     * \brief
     *
     * \param[in] key The key to the requested stats.
     * \return The stats cooresponding
     */
    const Stats& get(const std::string& key) const
    {
        return stats_.find(key)->second;
    }

    double sample(const std::string& key, const double value, const Type type = COUNT);

    double sampleT_WB(const Eigen::Isometry3f& T_WB);

    double sampleDurationStart(const std::string& key);

    double sampleDurationEnd(const std::string& key);

    void setFilestream(std::ofstream* filestream);

    /**
     * \brief Set the current iteration and add it to the stats.
     *
     * \param[in] iter
     */
    void setIter(const size_t iter)
    {
        iter_ = iter;
        sample("iteration", iter, ITERATION);
    };

    /**
     * \brief Write performance stats to filestream.
     *        The first time the function is called the header will be added.
     *        If the
     */
    void writeToFilestream();

    void writeToOStream(std::ostream& ostream);

    void writeSummaryToOStream(std::ostream& ostream, const bool include_iter_data = true);

    /** Return the current seconds since the epoch, as measured by std::chrono::steady_clock. */
    static double timeNow();

    /** The order the different types are added to the output. */
    std::vector<PerfStats::Type> header_order_ = {
        FRAME,    ITERATION, TIME, DURATION, MEMORY,     POSITION, ORIENTATION,
        DISTANCE, FREQUENCY, BOOL, POWER,    ENERGY,     CURRENT,  VOLTAGE,
        VOLUME,   COUNT,     INT,  DOUBLE,   PERCENTAGE, UNDEFINED};

    int insertion_idx_; ///< The index of the next stat to be inserted to performance stats
    size_t iter_;       ///< The current iteration
    std::map<int, std::string>
        order_; ///< The order the stats are added to the stats_ map | map idx -> stat name
    std::map<std::string, Stats> stats_; ///< The map stat name -> stat

    std::ofstream* filestream_;
    bool filestream_aligned_;
    size_t filestream_last_iter_;
    std::streampos filestream_pos_;

    bool ostream_aligned_;
    size_t ostream_last_iter_;
};

extern PerfStats perfstats;

} // namespace se

#include "impl/perfstats_impl.hpp"

#endif // SE_PERFSTATS_HPP
