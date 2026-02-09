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

#ifndef UTILS_PROGRESS_TRACKER_H
#define UTILS_PROGRESS_TRACKER_H

#include "benchmark_config.h"

#include <chrono>
#include <map>
#include <string>
#include <vector>

class ProgressTracker {
public:
    explicit ProgressTracker(int total_steps);

    void start_benchmark(int total_iterations);
    void start_iteration();
    double finish_iteration();

    void increment_completed_iterations();
    void reset_completed_iterations(int count) { completed_iterations_ = count; }
    int completed_iterations() const { return completed_iterations_; }
    
    void update_estimated_total(int new_total) { total_global_iterations_ = new_total; }
    int estimated_total() const { return total_global_iterations_; }

    std::string format_progress(int completed,int total_measurements,int remaining_iterations) const;
    
    void record_iteration_time(double seconds);

    double total_runtime() const { return cumulative_iteration_time_; }
    const std::vector<double>& iteration_times() const { return iteration_times_; }

    void update_ratio_stats(double ratio_pct, double avg_bw, double avg_lat);

    double min_bandwidth_mbps() const { return min_bandwidth_mbps_; }
    double max_bandwidth_mbps() const { return max_bandwidth_mbps_; }
    double min_latency_ns() const { return min_latency_ns_; }
    double max_latency_ns() const { return max_latency_ns_; }

    const std::map<double, std::pair<double, double>>& ratio_bw_ranges() const {
        return ratio_bw_ranges_;
    }
    const std::map<double, std::pair<double, double>>& ratio_lat_ranges() const {
        return ratio_lat_ranges_;
    }

    double ema_iteration_time() const { return ema_iteration_time_; }
    bool show_eta() const { return show_eta_; }

private:

    std::chrono::steady_clock::time_point benchmark_start_time_;
    std::chrono::steady_clock::time_point iteration_start_time_;
    double cumulative_iteration_time_;
    int completed_iterations_;
    int total_global_iterations_;
    bool show_eta_;
    double ema_iteration_time_;
    std::vector<double> iteration_times_;
    double min_bandwidth_mbps_;
    double max_bandwidth_mbps_;
    double min_latency_ns_;
    double max_latency_ns_;
    std::map<double, std::pair<double, double>> ratio_bw_ranges_;
    std::map<double, std::pair<double, double>> ratio_lat_ranges_;
};

#endif

