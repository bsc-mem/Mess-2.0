/*
 * Copyright (c) 2026, Barcelona Supercomputing Center
 * Contact: mess             [at] bsc [dot] es
 *          victor.xirau     [at] bsc [dot] es
 *          petar.radojkovic [at] bsc [dot] es
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright notice,
 *       this list of conditions and the following disclaimer.
 *
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 *     * Neither the name of the copyright holder nor the names
 *       of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "Measurement.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

OutlierDetector::OutlierDetector(const BenchmarkConfig& config)
    : config_(config) {}

namespace {

double bw_threshold_pct(double median_mbps) {
    if (median_mbps >= 20000.0) return  5.0;  // >= 20 GB/s
    if (median_mbps >=  8000.0) return  8.0;  //  8-20 GB/s
    if (median_mbps >=  2000.0) return 12.0;  //  2- 8 GB/s
    return 18.0;                              // <  2 GB/s
}

double lat_threshold_pct(double /*median_ns*/) { return 5.0; }

constexpr double BW_ABS_FLOOR_MBPS = 200.0;
constexpr double LAT_ABS_FLOOR_NS  = 2.0;

}

std::vector<int> OutlierDetector::find_outliers(const std::vector<double>& measurements,
                                                double threshold_pct,
                                                const std::string& unit) const {
    std::vector<int> outliers;
    if (measurements.size() < 3) {
        return outliers;
    }

    std::vector<double> sorted = measurements;
    std::sort(sorted.begin(), sorted.end());
    double median = sorted[sorted.size() / 2];

    const bool is_latency = (unit == "ns");
    const double adaptive_threshold = is_latency ? lat_threshold_pct(median)
                                                 : bw_threshold_pct(median);
    const double abs_floor = is_latency ? LAT_ABS_FLOOR_NS : BW_ABS_FLOOR_MBPS;

    if (config_.verbosity >= 3) {
        std::cout << "    Outlier detection: median=" << std::fixed << std::setprecision(2)
                  << median << " " << unit << ", caller_threshold=" << std::setprecision(1)
                  << threshold_pct << "% -> load_aware=" << std::setprecision(1)
                  << adaptive_threshold << "%, abs_floor=" << std::setprecision(2)
                  << abs_floor << " " << unit << std::endl;
    }

    for (size_t i = 0; i < measurements.size(); ++i) {
        double dev = std::abs(measurements[i] - median);
        double diff_pct = dev / median * 100.0;
        if (diff_pct > adaptive_threshold && dev > abs_floor) {
            outliers.push_back(static_cast<int>(i));
        }
    }

    return outliers;
}

std::vector<int> OutlierDetector::find_outliers(const std::vector<BenchmarkResult>& results,
                                                double threshold_pct) const {
    if (results.size() < 3) {
        return {};
    }

    std::vector<double> bw_values;
    std::vector<double> lat_values;
    bw_values.reserve(results.size());
    lat_values.reserve(results.size());
    for (const auto& result : results) {
        bw_values.push_back(result.bandwidth_mbps);
        lat_values.push_back(result.latency_ns);
    }

    std::vector<int> bw_outliers = find_outliers(bw_values, threshold_pct, "MB/s");
    std::vector<int> lat_outliers = find_outliers(lat_values, threshold_pct, "ns");

    std::vector<int> all_outliers = bw_outliers;
    for (int idx : lat_outliers) {
        if (std::find(all_outliers.begin(), all_outliers.end(), idx) == all_outliers.end()) {
            all_outliers.push_back(idx);
        }
    }

    std::sort(all_outliers.begin(), all_outliers.end());
    return all_outliers;
}
