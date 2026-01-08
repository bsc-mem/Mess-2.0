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

#include "measurement.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <iomanip>

namespace {

std::vector<std::string> extract_blocks(std::ifstream& input) {
    std::vector<std::string> blocks;
    std::string line;
    std::string current_block;
    while (std::getline(input, line)) {
        current_block += line + "\n";
        if (line.find("seconds time elapsed") != std::string::npos) {
            blocks.push_back(current_block);
            current_block.clear();
        }
    }
    if (!current_block.empty()) {
        blocks.push_back(current_block);
    }
    return blocks;
}

void write_blocks(std::ofstream& output,
                  const std::vector<std::string>& blocks,
                  size_t start_index) {
    for (size_t i = start_index; i < blocks.size(); ++i) {
        output << blocks[i];
    }
}

} 

OutlierDetector::OutlierDetector(const BenchmarkConfig& config,
                                 MeasurementStorage* storage,
                                 double cached_tlb_hit_latency_ns)
    : config_(config),
      storage_(storage),
      cached_tlb_hit_latency_ns_(cached_tlb_hit_latency_ns) {}

std::vector<double> OutlierDetector::load_bandwidth(double ratio_pct, int pause, double scaling_factor) const {
    return storage_->parse_bw_measurements(storage_->bw_file_path(ratio_pct, pause), scaling_factor);
}

std::vector<double> OutlierDetector::load_latency(double ratio_pct, int pause) const {
    return storage_->parse_lat_measurements(storage_->lat_file_path(ratio_pct, pause),
                                            cached_tlb_hit_latency_ns_);
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

    double adaptive_threshold = threshold_pct;
    if (median < 4000.0) {
        adaptive_threshold = threshold_pct + (4000.0 - median) / 4000.0 * 10.0;
        adaptive_threshold = std::min(adaptive_threshold, 20.0);
    }

    if (config_.verbosity >= 3) {
        std::cout << "    Outlier detection: median=" << std::fixed << std::setprecision(2)
                  << median << " " << unit << ", threshold=" << std::setprecision(1)
                  << threshold_pct << "% → adaptive=" << std::setprecision(1)
                  << adaptive_threshold << "%" << std::endl;
    }

    for (size_t i = 0; i < measurements.size(); i++) {
        double diff_pct = std::abs((measurements[i] - median) / median) * 100.0;
        if (diff_pct > adaptive_threshold) {
            outliers.push_back(static_cast<int>(i));
        }
    }

    return outliers;
}

std::vector<int> OutlierDetector::find_outliers(const std::vector<BenchmarkResult>& results,
                                                double threshold_pct) const {
    std::vector<int> outliers;
    if (results.size() < 3) {
        return outliers;
    }

    std::vector<double> bw_values;
    std::vector<double> lat_values;
    for (const auto& res : results) {
        bw_values.push_back(res.bandwidth_mbps);
        lat_values.push_back(res.latency_ns);
    }

    std::vector<int> bw_outliers = find_outliers(bw_values, threshold_pct, "MB/s");
    
    std::vector<int> lat_outliers = find_outliers(lat_values, threshold_pct, "ns");

    std::vector<int> all_outliers = bw_outliers;
    for (int idx : lat_outliers) {
        bool exists = false;
        for (int existing : all_outliers) {
            if (existing == idx) {
                exists = true;
                break;
            }
        }
        if (!exists) {
            all_outliers.push_back(idx);
        }
    }

    std::sort(all_outliers.begin(), all_outliers.end());
    return all_outliers;
}

bool OutlierDetector::align_measurement_files(double ratio_pct,
                                              int pause,
                                              size_t keep) const {
    std::string bw_file = storage_->bw_file_path(ratio_pct, pause);
    std::string lat_file = storage_->lat_file_path(ratio_pct, pause);

    auto align_file = [&](const std::string& path) -> bool {
        std::ifstream input(path);
        if (!input.is_open()) {
            return false;
        }
        auto blocks = extract_blocks(input);
        input.close();
        if (blocks.size() <= keep) {
            return true;
        }
        std::ofstream output(path, std::ios::trunc);
        if (!output.is_open()) {
            return false;
        }
        write_blocks(output, blocks, blocks.size() - keep);
        output.close();
        return true;
    };

    bool bw_ok = align_file(bw_file);
    bool lat_ok = align_file(lat_file);
    return bw_ok && lat_ok;
}

void OutlierDetector::remove_measurements_by_indices(
    double ratio_pct,
    int pause,
    const std::vector<int>& indices) const {
    if (indices.empty()) {
        return;
    }

    auto remove_from_file = [&](const std::string& path) {
        std::ifstream input(path);
        if (!input.is_open()) {
            return;
        }
        auto blocks = extract_blocks(input);
        input.close();

        std::vector<std::string> filtered;
        filtered.reserve(blocks.size());
        for (size_t i = 0; i < blocks.size(); ++i) {
            if (std::find(indices.begin(), indices.end(), static_cast<int>(i)) ==
                indices.end()) {
                filtered.push_back(blocks[i]);
            }
        }

        std::ofstream output(path, std::ios::trunc);
        if (!output.is_open()) {
            return;
        }
        for (const auto& block : filtered) {
            output << block;
        }
        output.close();
    };

    remove_from_file(storage_->bw_file_path(ratio_pct, pause));
    remove_from_file(storage_->lat_file_path(ratio_pct, pause));
}

void OutlierDetector::trim_to_last_measurements(double ratio_pct,
                                                int pause,
                                                int expected_reps) const {
    std::string bw_file = storage_->bw_file_path(ratio_pct, pause);
    std::string lat_file = storage_->lat_file_path(ratio_pct, pause);

    auto trim_file = [&](const std::string& path) {
        std::ifstream input(path);
        if (!input.is_open()) {
            return;
        }
        auto blocks = extract_blocks(input);
        input.close();
        if (blocks.size() <= static_cast<size_t>(expected_reps)) {
            return;
        }
        std::ofstream output(path, std::ios::trunc);
        if (!output.is_open()) {
            return;
        }
        size_t start =
            blocks.size() - static_cast<size_t>(expected_reps);
        write_blocks(output, blocks, start);
        output.close();
    };

    trim_file(bw_file);
    trim_file(lat_file);
}

