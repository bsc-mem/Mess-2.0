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

#pragma once

#include <deque>
#include <vector>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <functional>
#include <string>
#include <cstdio>
#include <cstdlib>
#include <sstream>

#include "measurement/TheoreticalPeakCalculator.h"

struct BWSample {
    long long cas_rd;
    long long cas_wr;
    double bw_gb_s = 0.0;
};

enum class StabilizationResult {
    PENDING,
    STABLE,
    ZOMBIE_DETECTED,
    WARMUP_REJECTED,
    TIMEOUT
};

struct TrafficGenHealthChecker {
    std::function<bool(int)> is_pid_alive;
    std::function<int()> count_running_instances;
    int expected_instance_count = 0;
    
    bool is_healthy() const {
        if (!count_running_instances) return true;
        int running = count_running_instances();
        return running >= expected_instance_count;
    }
    
    static int count_traffic_gen_processes(const std::string& pattern) {
        std::string cmd = "pgrep -c -f '" + pattern + "' 2>/dev/null || echo 0";
        FILE* pipe = popen(cmd.c_str(), "r");
        if (!pipe) return 0;
        
        char buffer[64];
        int count = 0;
        if (fgets(buffer, sizeof(buffer), pipe)) {
            count = std::atoi(buffer);
        }
        pclose(pipe);
        return count;
    }
    
    static int count_taskset_traffic_gen() {
        return count_traffic_gen_processes("taskset.*traffic_gen");
    }
};

class BandwidthStabilizer {
public:
    BandwidthStabilizer(
        double theoretical_peak_gb_s,
        int pause_value,
        TrafficGenHealthChecker health_checker,
        size_t window_size = 7,
        double threshold_pct = 0.05,
        int verbosity = 0)
        : theoretical_peak_gb_s_(theoretical_peak_gb_s),
          pause_value_(pause_value),
          health_checker_(health_checker),
          window_size_(window_size),
          threshold_pct_(threshold_pct),
          verbosity_(verbosity)
    {
        noise_floor_gb_s_ = TheoreticalPeakCalculator::get_noise_floor(theoretical_peak_gb_s_);
        warmup_threshold_gb_s_ = TheoreticalPeakCalculator::get_warmup_threshold(theoretical_peak_gb_s_);
    }

    StabilizationResult add_sample(long long cas_rd, long long cas_wr, double bw_gb_s) {
        last_result_ = StabilizationResult::PENDING;
        total_samples_seen_++;
        
        if (!gate1_zombie_check(bw_gb_s)) {
            last_result_ = StabilizationResult::ZOMBIE_DETECTED;
            return last_result_;
        }
        
        if (!gate2_warmup_check(bw_gb_s)) {
            warmup_samples_discarded_++;
            if (verbosity_ >= 3) {
                std::cout << "      [WARMUP] Discarding sample " << std::fixed << std::setprecision(2) 
                          << bw_gb_s << " GB/s (< threshold " << warmup_threshold_gb_s_ << " GB/s)" << '\n';
            }
            last_result_ = StabilizationResult::WARMUP_REJECTED;
            return last_result_;
        }
        
        samples_.push_back({cas_rd, cas_wr, bw_gb_s});
        if (samples_.size() > window_size_) {
            samples_.pop_front();
        }
        
        update_bw_stats();
        
        if (gate3_stability_check() && gate4_trend_check()) {
            last_result_ = StabilizationResult::STABLE;
        } else if (total_samples_seen_ >= max_samples_before_force_stable_) {
            if (verbosity_ >= 2) {
                std::cout << "      [FORCE STABLE] Max samples (" << max_samples_before_force_stable_ 
                          << ") reached, accepting current BW" << '\n';
            }
            last_result_ = StabilizationResult::STABLE;
        }
        
        return last_result_;
    }

    bool is_stable() const {
        return last_result_ == StabilizationResult::STABLE;
    }

    StabilizationResult get_last_result() const { return last_result_; }

    bool validate_post_measurement(double initial_bw_gb_s, double final_bw_gb_s) const {
        bool initial_active = initial_bw_gb_s > noise_floor_gb_s_;
        bool final_active = final_bw_gb_s > noise_floor_gb_s_;
        
        if (initial_active && !final_active) {
            return false;
        }
        
        if (initial_active && final_active) {
            double drift = std::abs(final_bw_gb_s - initial_bw_gb_s) / initial_bw_gb_s;
            if (drift > 0.10) {
                return false;
            }
        }
        
        return true;
    }

    void reset() {
        samples_.clear();
        mean_bw_gb_s_ = 0.0;
        stddev_bw_gb_s_ = 0.0;
        cv_ = 0.0;
        warmup_samples_discarded_ = 0;
        total_samples_seen_ = 0;
        last_result_ = StabilizationResult::PENDING;
        mean_history_.clear();
        has_active_trend_ = false;
    }

    double get_mean_bw() const { return mean_bw_gb_s_; }
    double get_stddev_bw() const { return stddev_bw_gb_s_; }
    double get_cv() const { return cv_; }
    double get_noise_floor() const { return noise_floor_gb_s_; }
    double get_theoretical_peak() const { return theoretical_peak_gb_s_; }
    size_t get_sample_count() const { return samples_.size(); }
    int get_warmup_discarded() const { return warmup_samples_discarded_; }
    const std::deque<BWSample>& get_samples() const { return samples_; }

    void print_status(double read_ratio, double sample_bw_gb_s = -1.0, bool has_distinct_rw = true) const {
        std::string bw_str = "";
        if (sample_bw_gb_s >= 0) {
            std::ostringstream oss;
            oss << " | BW: " << std::fixed << std::setprecision(2) << sample_bw_gb_s << " GB/s";
            bw_str = oss.str();
        }
        
        std::string rd_str = "";
        if (has_distinct_rw) {
            std::ostringstream oss;
            oss << " | RD%: " << std::fixed << std::setprecision(1) << (read_ratio * 100.0) << "%";
            rd_str = oss.str();
        }
        
        if (samples_.size() < window_size_) {
            std::cout << bw_str
                      << " | Filling window (" << samples_.size() << "/" << window_size_ << ")"
                      << " | Mean: " << std::fixed << std::setprecision(2) << mean_bw_gb_s_ << " GB/s"
                      << rd_str
                      << '\n';
        } else {
            bool in_active_mode = mean_bw_gb_s_ > noise_floor_gb_s_;
            std::cout << bw_str
                      << " | Mean: " << std::fixed << std::setprecision(2) << mean_bw_gb_s_ << " GB/s"
                      << " | CV: " << std::fixed << std::setprecision(2) << (cv_ * 100.0) << "%"
                      << " | " << (in_active_mode ? "ACTIVE" : "SILENT")
                      << rd_str
                      << (is_stable() ? " STABLE" : (has_active_trend_ ? " ✗ trending" : " ✗ unstable"))
                      << '\n';
        }
    }

private:
    double theoretical_peak_gb_s_;
    double noise_floor_gb_s_;
    double warmup_threshold_gb_s_;
    int pause_value_;
    TrafficGenHealthChecker health_checker_;
    size_t window_size_;
    double threshold_pct_;
    int verbosity_;
    
    std::deque<BWSample> samples_;
    double mean_bw_gb_s_ = 0.0;
    double stddev_bw_gb_s_ = 0.0;
    double cv_ = 0.0;
    int warmup_samples_discarded_ = 0;
    int total_samples_seen_ = 0;
    static constexpr int max_samples_before_force_stable_ = 50;
    StabilizationResult last_result_ = StabilizationResult::PENDING;

    std::deque<double> mean_history_;
    bool has_active_trend_ = false;
    static constexpr size_t trend_lookback_ = 3;
    static constexpr double trend_drift_threshold_ = 0.03;

    bool gate1_zombie_check(double bw_gb_s) {
        if (bw_gb_s < noise_floor_gb_s_) {
            if (!health_checker_.is_healthy()) {
                if (verbosity_ >= 2) {
                    int running = health_checker_.count_running_instances ? 
                                  health_checker_.count_running_instances() : 0;
                    std::cout << "      [ZOMBIE] BW " << std::fixed << std::setprecision(3) << bw_gb_s 
                              << " GB/s < floor " << noise_floor_gb_s_ 
                              << " GB/s | TrafficGen instances: " << running 
                              << "/" << health_checker_.expected_instance_count << '\n';
                }
                return false;
            }
        }
        return true;
    }

    bool gate2_warmup_check(double bw_gb_s) {
        if (pause_value_ == 0 && bw_gb_s < warmup_threshold_gb_s_) {
            return false;
        }
        return true;
    }

    bool gate3_stability_check() const {
        if (samples_.size() < window_size_) return false;
        
        bool in_active_mode = mean_bw_gb_s_ > noise_floor_gb_s_;
        
        if (in_active_mode) {
            double low_bw_threshold = theoretical_peak_gb_s_ * 0.05;
            
            if (mean_bw_gb_s_ < low_bw_threshold) {
                double abs_stddev_threshold = 0.5;
                return stddev_bw_gb_s_ < abs_stddev_threshold;
            }
            
            return cv_ < threshold_pct_;
        } else {
            double strict_deviation = noise_floor_gb_s_ / 2.0;
            return stddev_bw_gb_s_ < strict_deviation;
        }
    }

    bool gate4_trend_check() {
        has_active_trend_ = false;
        if (mean_history_.size() <= trend_lookback_) return true;

        double old_mean = mean_history_[mean_history_.size() - 1 - trend_lookback_];
        double current_mean = mean_history_.back();
        if (old_mean <= 0) return true;

        double drift = (current_mean - old_mean) / old_mean;
        if (std::abs(drift) > trend_drift_threshold_) {
            has_active_trend_ = true;
            return false;
        }
        return true;
    }

    void update_bw_stats() {
        if (samples_.empty()) return;
        
        double sum = 0.0;
        for (const auto& s : samples_) {
            sum += s.bw_gb_s;
        }
        mean_bw_gb_s_ = sum / samples_.size();
        
        double sq_sum = 0.0;
        for (const auto& s : samples_) {
            sq_sum += (s.bw_gb_s - mean_bw_gb_s_) * (s.bw_gb_s - mean_bw_gb_s_);
        }
        stddev_bw_gb_s_ = std::sqrt(sq_sum / samples_.size());
        
        if (mean_bw_gb_s_ > 0) {
            cv_ = stddev_bw_gb_s_ / mean_bw_gb_s_;
        } else {
            cv_ = 0.0;
        }

        if (samples_.size() >= window_size_) {
            mean_history_.push_back(mean_bw_gb_s_);
        }
    }
};
