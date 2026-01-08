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

#include "utils/progress_tracker.h"

#include <algorithm>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>

ProgressTracker::ProgressTracker(int total_steps)
    : cumulative_iteration_time_(0.0),
      completed_iterations_(0),
      total_global_iterations_(total_steps),
      show_eta_(true),
      ema_iteration_time_(0.0),
      min_bandwidth_mbps_(std::numeric_limits<double>::max()),
      max_bandwidth_mbps_(0.0),
      min_latency_ns_(std::numeric_limits<double>::max()),
      max_latency_ns_(0.0) {}

void ProgressTracker::start_benchmark(int total_iterations) {
    benchmark_start_time_ = std::chrono::steady_clock::now();
    cumulative_iteration_time_ = 0.0;
    completed_iterations_ = 0;
    total_global_iterations_ = total_iterations;
    ema_iteration_time_ = 0.0;
    iteration_times_.clear();
}

void ProgressTracker::start_iteration() {
    iteration_start_time_ = std::chrono::steady_clock::now();
}

double ProgressTracker::finish_iteration() {
    auto iteration_end_time = std::chrono::steady_clock::now();
    auto iteration_duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(iteration_end_time -
                                                              iteration_start_time_);
    double iteration_time = iteration_duration.count() / 1000.0;
    cumulative_iteration_time_ += iteration_time;
    return iteration_time;
}

void ProgressTracker::increment_completed_iterations() {
    completed_iterations_++;
}

std::string ProgressTracker::format_progress(int completed,
                                             int total_measurements,
                                             int remaining_iterations) const {
    const int bar_width = 30;
    float progress =
        total_measurements > 0
            ? static_cast<float>(completed) / static_cast<float>(total_measurements)
            : 0.0f;
    progress = std::clamp(progress, 0.0f, 1.0f);
    int pos = static_cast<int>(bar_width * progress);
    if (pos < 0) pos = 0;
    if (pos > bar_width) pos = bar_width;

    std::string progress_bar = "[";
    progress_bar += std::string(static_cast<size_t>(pos), '#');
    progress_bar += std::string(static_cast<size_t>(bar_width - pos), ' ');
    progress_bar += "]";

    char buffer[64];
    std::snprintf(buffer,
                  sizeof(buffer),
                  " %3d%% (%d/%d)",
                  static_cast<int>(progress * 100.0f),
                  std::max(0, std::min(completed, total_measurements)),
                  total_measurements);
    progress_bar += buffer;

    std::string eta_str = " ERT x ratio: calculating";
    if (show_eta_ && completed_iterations_ > 0 && ema_iteration_time_ > 0) {
        double estimated_remaining_seconds = ema_iteration_time_ * remaining_iterations;
        int hours = static_cast<int>(estimated_remaining_seconds) / 3600;
        int minutes = (static_cast<int>(estimated_remaining_seconds) % 3600) / 60;
        int seconds = static_cast<int>(estimated_remaining_seconds) % 60;

        std::ostringstream oss;
        oss << " ERT x ratio: ";
        if (hours > 0) {
            oss << hours << "h " << minutes << "m " << seconds << "s";
        } else if (minutes > 0) {
            oss << minutes << "m " << seconds << "s";
        } else {
            oss << seconds << "s";
        }
        oss << " (" << static_cast<int>(ema_iteration_time_) << " s/point)";
        eta_str = oss.str();
    }

    return progress_bar + eta_str + "   ";
}

void ProgressTracker::record_iteration_time(double seconds) {
    iteration_times_.push_back(seconds);
    if (ema_iteration_time_ == 0.0) {
        ema_iteration_time_ = seconds;
    } else {
        ema_iteration_time_ = 0.3 * seconds + 0.7 * ema_iteration_time_;
    }
}

void ProgressTracker::update_ratio_stats(double ratio_pct,
                                         double avg_bw,
                                         double avg_lat) {
    auto update_map = [](std::map<double, std::pair<double, double>>& ranges,
                         double key,
                         double value) {
        if (ranges.find(key) == ranges.end()) {
            ranges[key] = {value, value};
        } else {
            ranges[key].first = std::min(ranges[key].first, value);
            ranges[key].second = std::max(ranges[key].second, value);
        }
    };

    update_map(ratio_bw_ranges_, ratio_pct, avg_bw);
    update_map(ratio_lat_ranges_, ratio_pct, avg_lat);

    min_bandwidth_mbps_ = std::min(min_bandwidth_mbps_, ratio_bw_ranges_[ratio_pct].first);
    max_bandwidth_mbps_ = std::max(max_bandwidth_mbps_, ratio_bw_ranges_[ratio_pct].second);
    min_latency_ns_ = std::min(min_latency_ns_, ratio_lat_ranges_[ratio_pct].first);
    max_latency_ns_ = std::max(max_latency_ns_, ratio_lat_ranges_[ratio_pct].second);
}

