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

#include "results_processor.h"
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

std::string ResultsProcessor::detect_available_bw_tool() const {
    if (run_command_success("which likwid-perfctr > /dev/null 2>&1")) {
        return "likwid";
    }
    
    if (run_command_success("which perf > /dev/null 2>&1")) {
        return "perf";
    }
    
    return "";
}

std::string ResultsProcessor::detect_available_lat_tool() const {
    if (run_command_success("which perf > /dev/null 2>&1")) {
        return "perf";
    }
    
    return "";
}

ResultsProcessor::ResultsProcessor(const BenchmarkConfig& config, const BenchmarkExecutor* executor)
    : config_(config), total_runtime_(0.0), executor_(executor) {
}

void ResultsProcessor::print_timing_statistics() const {
    if (results_.empty()) {
        return;
    }
    
    std::set<double> ratios;
    std::set<int> pauses;
    for (const auto& result : results_) {
        ratios.insert(result.ratio_pct);
        pauses.insert(result.pause);
    }
    
    double min_bw = 0.0, max_bw = 0.0, min_lat = 0.0, max_lat = 0.0;
    std::map<double, std::pair<double, double>> ratio_bw_ranges;
    std::map<double, std::pair<double, double>> ratio_lat_ranges;
    
    if (!results_.empty()) {
        for (const auto& result : results_) {
            if (result.bandwidth_mbps > 0) {
                min_bw = max_bw = result.bandwidth_mbps;
                break;
            }
        }
        
        for (const auto& result : results_) {
            if (result.bandwidth_mbps > 0) {
                min_bw = std::min(min_bw, result.bandwidth_mbps);
                max_bw = std::max(max_bw, result.bandwidth_mbps);
                
                if (ratio_bw_ranges.find(result.ratio_pct) == ratio_bw_ranges.end()) {
                    ratio_bw_ranges[result.ratio_pct] = {result.bandwidth_mbps, result.bandwidth_mbps};
                } else {
                    auto& range = ratio_bw_ranges[result.ratio_pct];
                    range.first = std::min(range.first, result.bandwidth_mbps);
                    range.second = std::max(range.second, result.bandwidth_mbps);
                }
            }
            
            if (result.latency_ns > 0) {
                if (min_lat == 0.0 || result.latency_ns < min_lat) {
                    min_lat = result.latency_ns;
                }
                if (result.latency_ns > max_lat) {
                    max_lat = result.latency_ns;
                }
                
                if (ratio_lat_ranges.find(result.ratio_pct) == ratio_lat_ranges.end()) {
                    ratio_lat_ranges[result.ratio_pct] = {result.latency_ns, result.latency_ns};
                } else {
                    auto& range = ratio_lat_ranges[result.ratio_pct];
                    range.first = std::min(range.first, result.latency_ns);
                    range.second = std::max(range.second, result.latency_ns);
                }
            }
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
    
    bool has_valid_measurements = false;
    for (const auto& result : results_) {
        if (result.bandwidth_mbps > 0 || result.latency_ns > 0) {
            has_valid_measurements = true;
            break;
        }
    }
    
    std::vector<BenchmarkResult> seq_results;
    
    for (const auto& result : results_) {
        if (result.mode == ExecutionMode::MULTISEQUENTIAL) {
            seq_results.push_back(result);
        }
    }
    
    auto print_mode_summary = [&](const std::string& title, const std::vector<BenchmarkResult>& mode_results) {
        if (mode_results.empty()) return;
        
        std::cout << title << ":" << std::endl;
        
        std::map<double, std::pair<double, double>> mode_ratio_bw_ranges;
        std::map<double, std::pair<double, double>> mode_ratio_lat_ranges;
        double mode_min_bw = 0.0, mode_max_bw = 0.0, mode_min_lat = 0.0, mode_max_lat = 0.0;
        std::set<double> mode_ratios;
        
        bool first_bw = true;
        bool first_lat = true;
        
        for (const auto& result : mode_results) {
            mode_ratios.insert(result.ratio_pct);
            
            if (result.bandwidth_mbps > 0) {
                if (first_bw) {
                    mode_min_bw = mode_max_bw = result.bandwidth_mbps;
                    first_bw = false;
                } else {
                    mode_min_bw = std::min(mode_min_bw, result.bandwidth_mbps);
                    mode_max_bw = std::max(mode_max_bw, result.bandwidth_mbps);
                }
                
                if (mode_ratio_bw_ranges.find(result.ratio_pct) == mode_ratio_bw_ranges.end()) {
                    mode_ratio_bw_ranges[result.ratio_pct] = {result.bandwidth_mbps, result.bandwidth_mbps};
                } else {
                    auto& range = mode_ratio_bw_ranges[result.ratio_pct];
                    range.first = std::min(range.first, result.bandwidth_mbps);
                    range.second = std::max(range.second, result.bandwidth_mbps);
                }
            }
            
            if (result.latency_ns > 0) {
                if (first_lat) {
                    mode_min_lat = mode_max_lat = result.latency_ns;
                    first_lat = false;
                } else {
                    mode_min_lat = std::min(mode_min_lat, result.latency_ns);
                    mode_max_lat = std::max(mode_max_lat, result.latency_ns);
                }
                
                if (mode_ratio_lat_ranges.find(result.ratio_pct) == mode_ratio_lat_ranges.end()) {
                    mode_ratio_lat_ranges[result.ratio_pct] = {result.latency_ns, result.latency_ns};
                } else {
                    auto& range = mode_ratio_lat_ranges[result.ratio_pct];
                    range.first = std::min(range.first, result.latency_ns);
                    range.second = std::max(range.second, result.latency_ns);
                }
            }
        }
        
        if (mode_ratios.size() > 1) {
            for (auto ratio : mode_ratios) {
                std::cout << "  ┌─ Ratio " << std::fixed << std::setprecision(0) << ratio << "%:" << std::endl;
                std::cout << "  │  Bandwidth range:       " << std::fixed << std::setprecision(2) 
                         << mode_ratio_bw_ranges[ratio].first / 1000.0 << " - " 
                         << mode_ratio_bw_ranges[ratio].second / 1000.0 << " GB/s" << std::endl;
                std::cout << "  │  Latency range:         " << std::fixed << std::setprecision(2) 
                         << mode_ratio_lat_ranges[ratio].first << " - " << mode_ratio_lat_ranges[ratio].second << " ns" << std::endl;
                std::cout << "  └─" << std::endl;
            }
            
            std::cout << "  Global Metrics:" << std::endl;
            if (mode_min_lat > 0 && mode_min_lat < 1e9) { 
                std::cout << "    Lead-off Latency:      " << std::fixed << std::setprecision(2) << mode_min_lat << " ns" << std::endl;
            } else {
                std::cout << "    Lead-off Latency:      " << "N/A (invalid measurement)" << std::endl;
            }
            std::cout << "    Max BW Achieved:       ";
            if (mode_max_bw > 0) {
                std::cout << std::fixed << std::setprecision(2) << (mode_max_bw / 1000.0);
            } else {
                std::cout << "0.00";
            }
            std::cout << " GB/s" << std::endl;
        } else {
            std::cout << "  Bandwidth range:         ";
            if (mode_max_bw > 0) {
                std::cout << std::fixed << std::setprecision(2) << (mode_min_bw / 1000.0) << " - " << (mode_max_bw / 1000.0);
            } else {
                std::cout << "0.00 - 0.00";
            }
            std::cout << " GB/s" << std::endl;
            std::cout << "  Latency range:           ";
            if (mode_min_lat < 1e9 && mode_min_lat > 0) {
                std::cout << std::fixed << std::setprecision(2) << mode_min_lat << " - " << mode_max_lat;
            } else {
                std::cout << "0.00 - 0.00";
            }
            std::cout << " ns" << std::endl;
            std::cout << "  Lead-off Latency:        ";
            if (mode_min_lat > 0 && mode_min_lat < 1e9) {
                std::cout << std::fixed << std::setprecision(2) << mode_min_lat;
            } else {
                std::cout << "0.00";
            }
            std::cout << " ns" << std::endl;
            std::cout << "  Max BW Achieved:         ";
            if (mode_max_bw > 0) {
                std::cout << std::fixed << std::setprecision(2) << (mode_max_bw / 1000.0);
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


