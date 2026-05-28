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

#include "ResultsProcessor.h"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <filesystem>
#include <cstdlib>
#include <cstdio>
#include <memory>
#include <set>
#include <limits>
#include <map>

namespace {

struct NumericRange {
    bool valid = false;
    double min_value = 0.0;
    double max_value = 0.0;

    void add(double v) {
        if (!valid) {
            min_value = max_value = v;
            valid = true;
            return;
        }
        min_value = std::min(min_value, v);
        max_value = std::max(max_value, v);
    }
};



struct ModeAggregation {
    std::set<double> ratios;
    std::map<double, NumericRange> bw_by_ratio;
    std::map<double, NumericRange> lat_by_ratio;

    NumericRange bw_global;
    NumericRange lat_global;

    void add(const BenchmarkResult& result) {
        ratios.insert(result.ratio_pct);

        if (result.bandwidth_mbps > 0.0) {
            bw_global.add(result.bandwidth_mbps);
            bw_by_ratio[result.ratio_pct].add(result.bandwidth_mbps);
        }

        if (result.latency_ns > 0.0) {
            lat_global.add(result.latency_ns);
            lat_by_ratio[result.ratio_pct].add(result.latency_ns);
        }
    }
};



void print_bw_range_line(const char* prefix, const NumericRange& r, const char* indent) {
    std::cout << indent << prefix;
    if (r.valid) {
        std::cout << std::fixed << std::setprecision(2)
                  << (r.min_value / 1000.0) << " - " << (r.max_value / 1000.0);
    } else {
        std::cout << "0.00 - 0.00";
    }
    std::cout << " GB/s" << std::endl;
}



void print_lat_range_line(const char* prefix, const NumericRange& r, const char* indent) {
    std::cout << indent << prefix;
    if (r.valid && r.min_value > 0.0 && r.max_value < 1e9) {
        std::cout << std::fixed << std::setprecision(2)
                  << r.min_value << " - " << r.max_value;
    } else {
        std::cout << "0.00 - 0.00";
    }
    std::cout << " ns" << std::endl;
}

} // namespace

ResultsProcessor::ResultsProcessor(const BenchmarkConfig& config, const BenchmarkExecutor* executor)
    : config_(config), total_runtime_(0.0), executor_(executor) {
}

void ResultsProcessor::print_timing_statistics() const {
    if (results_.empty()) {
        return;
    }
    
    std::set<double> ratios;
    std::set<int> pauses;
    bool has_valid_measurements = false;
    std::vector<const BenchmarkResult*> seq_results;
    seq_results.reserve(results_.size());

    for (const auto& result : results_) {
        ratios.insert(result.ratio_pct);
        pauses.insert(result.pause);
        if (result.bandwidth_mbps > 0.0 || result.latency_ns > 0.0) {
            has_valid_measurements = true;
        }
        if (result.mode == ExecutionMode::MULTISEQUENTIAL) {
            seq_results.push_back(&result);
        }
    }

    double avg_time = 0.0;
    double std_dev = 0.0;
    if (!iteration_times_.empty()) {
        avg_time = std::accumulate(iteration_times_.begin(), iteration_times_.end(), 0.0) / iteration_times_.size();
        
        double variance = 0.0;
        for (double t : iteration_times_) {
            variance += (t - avg_time) * (t - avg_time);
        }
        std_dev = std::sqrt(variance / iteration_times_.size());
    }
    
    std::cout << "\n\n╔══════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║              Benchmark Execution Statistics              ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════╝" << std::endl;
    std::cout << std::endl;
    
    std::cout << "Execution Summary:" << std::endl;
    std::cout << "  Total ratios run:        " << ratios.size() << std::endl;
    std::cout << "  Pause values per ratio:  " << pauses.size() << std::endl;
    std::cout << "  Total measurements:      " << results_.size() << std::endl;
    std::cout << std::endl;
    
    std::cout << "Time Statistics:" << std::endl;
    std::cout << "  Avg time per point:      " << std::fixed << std::setprecision(1) << avg_time << " ± " << std_dev << " s" << std::endl;
    std::cout << "  Total runtime:           " << std::fixed << std::setprecision(1) << total_runtime_ << " s" << std::endl;
    std::cout << std::endl;

    auto print_mode_summary = [&](const std::string& title, const std::vector<const BenchmarkResult*>& mode_results) {
        if (mode_results.empty()) return;

        ModeAggregation agg;
        for (const BenchmarkResult* result : mode_results) {
            agg.add(*result);
        }

        std::cout << title << ":" << std::endl;

        if (agg.ratios.size() > 1) {
            for (double ratio : agg.ratios) {
                const NumericRange bw = (agg.bw_by_ratio.count(ratio) > 0) ? agg.bw_by_ratio.at(ratio) : NumericRange{};
                const NumericRange lat = (agg.lat_by_ratio.count(ratio) > 0) ? agg.lat_by_ratio.at(ratio) : NumericRange{};

                std::cout << "  ┌─ Ratio " << std::fixed << std::setprecision(0) << ratio << "%:" << std::endl;
                print_bw_range_line("Bandwidth range:       ", bw, "  │  ");
                print_lat_range_line("Latency range:         ", lat, "  │  ");
                std::cout << "  └─" << std::endl;
            }
            
            std::cout << "  Global Metrics:" << std::endl;
            if (agg.lat_global.valid && agg.lat_global.min_value > 0.0 && agg.lat_global.min_value < 1e9) {
                std::cout << "    Lead-off Latency:      " << std::fixed << std::setprecision(2) << agg.lat_global.min_value << " ns" << std::endl;
            } else {
                std::cout << "    Lead-off Latency:      " << "N/A (invalid measurement)" << std::endl;
            }
            std::cout << "    Max BW Achieved:       ";
            if (agg.bw_global.valid) {
                std::cout << std::fixed << std::setprecision(2) << (agg.bw_global.max_value / 1000.0);
            } else {
                std::cout << "0.00";
            }
            std::cout << " GB/s" << std::endl;
            
        } else {
            print_bw_range_line("Bandwidth range:         ", agg.bw_global, "  ");
            print_lat_range_line("Latency range:           ", agg.lat_global, "  ");
            std::cout << "  Lead-off Latency:        ";
            if (agg.lat_global.valid && agg.lat_global.min_value > 0.0 && agg.lat_global.min_value < 1e9) {
                std::cout << std::fixed << std::setprecision(2) << agg.lat_global.min_value;
            } else {
                std::cout << "0.00";
            }
            std::cout << " ns" << std::endl;
            std::cout << "  Max BW Achieved:         ";
            if (agg.bw_global.valid) {
                std::cout << std::fixed << std::setprecision(2) << (agg.bw_global.max_value / 1000.0);
            } else {
                std::cout << "0.00";
            }
            std::cout << " GB/s" << std::endl;

        }
        std::cout << std::endl;
    };
    
    print_mode_summary("MultiSequential Execution Summary", seq_results);
    
    if (has_valid_measurements) {
        std::cout << "\033[1;32mBenchmark completed successfully\033[0m" << std::endl;
    } else {
        std::cout << "\033[1;31mError: Benchmark completed but no valid measurements were recorded.\033[0m" << std::endl;
        std::cout << "Please check your system configuration and try again with appropriate permissions." << std::endl;
    }
    std::cout << std::endl;
}

bool ResultsProcessor::process(const std::vector<BenchmarkResult>& results, double total_runtime, const std::vector<double>& iteration_times) {
    results_ = results;
    total_runtime_ = total_runtime;
    iteration_times_ = iteration_times;

    if (config_.verbosity >= 1) {
        print_timing_statistics();
    }

    return true;
}
