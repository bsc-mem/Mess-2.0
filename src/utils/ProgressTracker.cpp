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

#include "utils/ProgressTracker.h"

#include <algorithm>
#include <cmath>
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
      trimmed_mean_iteration_time_(0.0),
      max_displayed_progress_(0.0f),
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
    trimmed_mean_iteration_time_ = 0.0;
    max_displayed_progress_ = 0.0f;
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

double ProgressTracker::elapsed_seconds() const {
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - benchmark_start_time_).count() / 1000.0;
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
    if (progress < max_displayed_progress_) {
        progress = max_displayed_progress_;
    }
    max_displayed_progress_ = progress;

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
                  " %3d%%",
                  static_cast<int>(progress * 100.0f));
    progress_bar += buffer;

    double elapsed = elapsed_seconds();
    std::ostringstream elapsed_oss;
    {
        int eh = static_cast<int>(elapsed) / 3600;
        int em = (static_cast<int>(elapsed) % 3600) / 60;
        int es = static_cast<int>(elapsed) % 60;
        if (eh > 0) {
            elapsed_oss << eh << "h " << em << "m " << es << "s";
        } else if (em > 0) {
            elapsed_oss << em << "m " << es << "s";
        } else {
            elapsed_oss << es << "s";
        }
    }

    double rate = trimmed_mean_iteration_time_ > 0 ? trimmed_mean_iteration_time_ : ema_iteration_time_;

    std::string eta_str;
    if (!show_eta_ || completed_iterations_ < 2 || rate <= 0) {
        eta_str = " | ETA: calculating";
    } else {
        double estimated_remaining_seconds = std::max(0.0, rate * remaining_iterations);
        int hours = static_cast<int>(estimated_remaining_seconds) / 3600;
        int minutes = (static_cast<int>(estimated_remaining_seconds) % 3600) / 60;
        int seconds = static_cast<int>(estimated_remaining_seconds) % 60;

        std::ostringstream oss;
        oss << " | ETA: ";
        if (hours > 0) {
            oss << hours << "h " << minutes << "m " << seconds << "s";
        } else if (minutes > 0) {
            oss << minutes << "m " << seconds << "s";
        } else {
            oss << seconds << "s";
        }
        oss << " (" << std::fixed << std::setprecision(1) << rate << " s/pt)";
        eta_str = oss.str();
    }

    std::string pts_str;
    std::string ratio_str;
    if (total_ratios_ > 0) {
        std::ostringstream rs;
        rs << " ratio: " << current_ratio_index_ << "/" << total_ratios_;
        ratio_str = rs.str();
    }

    if (adaptive_current_run_est_total_ > 0) {
        std::ostringstream ps;
        ps << " pts: " << adaptive_current_run_points_
           << "/" << adaptive_current_run_est_total_;
        pts_str = ps.str();
    }

    std::string time_prefix = " [" + elapsed_oss.str() + "]";
    std::string line = progress_bar + ratio_str + pts_str + time_prefix + eta_str;
    if (line.size() < 140) {
        line.append(140 - line.size(), ' ');
    } else if (line.size() > 140) {
        line = line.substr(0, 140);
    }
    return line;
}

void ProgressTracker::set_adaptive_info(int total_mode_runs,
                                        int point_reps,
                                        int max_points_per_run,
                                        int modes_per_ratio) {
    adaptive_total_mode_runs_ = total_mode_runs;
    adaptive_point_reps_ = point_reps;
    adaptive_max_points_per_run_ = max_points_per_run;
    adaptive_modes_per_ratio_ = modes_per_ratio;
    adaptive_completed_runs_ = 0;
    adaptive_total_discovered_ = 0;
    ema_discovered_points_ = 0.0;
    adaptive_current_run_points_ = 0;
    adaptive_current_run_est_total_ = 0;
}

int ProgressTracker::record_adaptive_run(int discovered_points) {
    adaptive_completed_runs_++;
    adaptive_total_discovered_ += discovered_points;
    adaptive_current_run_points_ = 0;
    adaptive_current_run_est_total_ = 0;

    if (ema_discovered_points_ <= 0.0) {
        ema_discovered_points_ = static_cast<double>(discovered_points);
    } else {
        ema_discovered_points_ = 0.4 * discovered_points + 0.6 * ema_discovered_points_;
    }

    if (adaptive_max_points_per_run_ > 0) {
        return total_global_iterations_;
    }

    int remaining_runs = adaptive_total_mode_runs_ - adaptive_completed_runs_;
    int new_estimate = completed_iterations_ + static_cast<int>(
        std::ceil(remaining_runs * ema_discovered_points_ * adaptive_point_reps_));

    if (new_estimate > total_global_iterations_) {
        total_global_iterations_ = completed_iterations_ +
            (new_estimate - completed_iterations_ + total_global_iterations_ - completed_iterations_) / 2;
    } else {
        total_global_iterations_ = new_estimate;
    }

    return total_global_iterations_;
}

int ProgressTracker::estimate_with_active_runs(int active_runs,
                                               int points_in_progress,
                                               int segments_above) {
    (void)segments_above;
    adaptive_current_run_points_ = points_in_progress;
    if (adaptive_max_points_per_run_ > 0) {
        const int modes_for_budget = (adaptive_modes_per_ratio_ > 0)
            ? adaptive_modes_per_ratio_
            : std::max(active_runs, 1);
        adaptive_current_run_est_total_ = modes_for_budget * adaptive_max_points_per_run_;
    } else {
        adaptive_current_run_est_total_ = points_in_progress + segments_above;
    }

    int future_runs = adaptive_total_mode_runs_ - adaptive_completed_runs_ - active_runs;
    if (future_runs < 0) {
        future_runs = 0;
    }

    if (adaptive_max_points_per_run_ > 0) {
        return total_global_iterations_;
    }

    int remaining_active = segments_above * adaptive_point_reps_;

    double avg_points_per_run = 0.0;
    if (active_runs > 0) {
        avg_points_per_run =
            static_cast<double>(points_in_progress + segments_above) / active_runs;
    }

    if (ema_discovered_points_ > 0.0) {
        avg_points_per_run = (avg_points_per_run > 0.0)
            ? 0.5 * avg_points_per_run + 0.5 * ema_discovered_points_
            : ema_discovered_points_;
    } else if (avg_points_per_run <= 0.0 && adaptive_completed_runs_ > 0) {
        avg_points_per_run = static_cast<double>(adaptive_total_discovered_) /
                             adaptive_completed_runs_;
    }

    int new_estimate = completed_iterations_ + remaining_active +
        static_cast<int>(std::ceil(future_runs * avg_points_per_run * adaptive_point_reps_));

    if (new_estimate > total_global_iterations_) {
        int raw_remaining = new_estimate - completed_iterations_;
        int old_remaining = total_global_iterations_ - completed_iterations_;
        total_global_iterations_ = completed_iterations_ + (raw_remaining + old_remaining) / 2;
    } else {
        total_global_iterations_ = new_estimate;
    }

    return total_global_iterations_;
}

void ProgressTracker::set_current_ratio(double ratio_pct, int current_ratio, int total_ratios) {
    current_ratio_pct_ = ratio_pct;
    current_ratio_index_ = current_ratio;
    total_ratios_ = total_ratios;
}

void ProgressTracker::record_iteration_time(double seconds) {
    iteration_times_.push_back(seconds);
    if (ema_iteration_time_ == 0.0) {
        ema_iteration_time_ = seconds;
    } else {
        ema_iteration_time_ = 0.3 * seconds + 0.7 * ema_iteration_time_;
    }

    const size_t window = 20;
    size_t n = iteration_times_.size();
    size_t start = (n > window) ? n - window : 0;
    std::vector<double> recent(iteration_times_.begin() + static_cast<long>(start),
                               iteration_times_.end());
    std::sort(recent.begin(), recent.end());

    if (recent.size() >= 5) {
        size_t trim = recent.size() / 5;
        double sum = 0.0;
        size_t count = 0;
        for (size_t i = trim; i < recent.size() - trim; ++i) {
            sum += recent[i];
            count++;
        }
        trimmed_mean_iteration_time_ = (count > 0) ? sum / count : ema_iteration_time_;
    } else {
        double sum = 0.0;
        for (double v : recent) sum += v;
        trimmed_mean_iteration_time_ = sum / recent.size();
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
