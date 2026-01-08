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

struct BWSample {
    long long cas_rd;
    long long cas_wr;
};

class BandwidthStabilizer {
public:
    BandwidthStabilizer(size_t window_size = 7, double threshold_pct = 0.05, double noise_floor = 5000.0)
        : window_size_(window_size), threshold_pct_(threshold_pct), noise_floor_(noise_floor) {}

    void add_sample(long long cas_rd, long long cas_wr) {
        samples_.push_back({cas_rd, cas_wr});
        if (samples_.size() > window_size_) {
            samples_.pop_front();
        }
        update_stats();
    }

    bool is_stable() const {
        if (samples_.size() < window_size_) return false;
        
        if (stddev_ <= (mean_ * threshold_pct_) + noise_floor_) {
            return true;
        }

        if (cv_history_.size() >= 3) {
            double max_cv = -1.0;
            double min_cv = 1000.0;
            for (double c : cv_history_) {
                if (c > max_cv) max_cv = c;
                if (c < min_cv) min_cv = c;
            }
            if ((max_cv - min_cv) < 0.02) {
                return true;
            }
        }
        return false;
    }

    void reset() {
        samples_.clear();
        cv_history_.clear();
        mean_ = 0.0;
        stddev_ = 0.0;
        cv_ = 0.0;
    }

    double get_mean() const { return mean_; }
    double get_stddev() const { return stddev_; }
    double get_cv() const { return cv_; }
    size_t get_sample_count() const { return samples_.size(); }
    const std::deque<BWSample>& get_samples() const { return samples_; }

    void print_status(double read_ratio) const {
        if (samples_.size() < window_size_) {
            std::cout << " | Filling window (" << samples_.size() << "/" << window_size_ << ")"
                      << " | Meas. Read Ratio: " << std::fixed << std::setprecision(2) << (read_ratio * 100.0) << "%"
                      << std::endl;
        } else {
            bool stable = is_stable();
            std::cout << " | Win Mean: " << std::fixed << std::setprecision(0) << mean_
                      << " | StdDev: " << stddev_
                      << " | CV: " << std::fixed << std::setprecision(2) << (cv_ * 100.0) << "%"
                      << " | Meas. Read Ratio: " << std::fixed << std::setprecision(2) << (read_ratio * 100.0) << "%"
                      << (stable ? " STABLE" : " ✗ unstable")
                      << std::endl;
        }
    }

private:
    size_t window_size_;
    double threshold_pct_;
    double noise_floor_;
    
    std::deque<BWSample> samples_;
    std::deque<double> cv_history_;
    
    double mean_ = 0.0;
    double stddev_ = 0.0;
    double cv_ = 0.0;

    void update_stats() {
        if (samples_.empty()) return;

        double sum = 0.0;
        for (const auto& s : samples_) {
            sum += (s.cas_rd + s.cas_wr);
        }
        mean_ = sum / samples_.size();

        double sq_sum = 0.0;
        for (const auto& s : samples_) {
            double val = (s.cas_rd + s.cas_wr);
            sq_sum += (val - mean_) * (val - mean_);
        }
        stddev_ = std::sqrt(sq_sum / samples_.size());
        
        if (mean_ > 0) {
            cv_ = stddev_ / mean_;
        } else {
            cv_ = 0.0;
        }

        if (samples_.size() == window_size_) {
             if (stddev_ <= (mean_ * threshold_pct_) + noise_floor_) {
                 cv_history_.clear();
             } else {
                 cv_history_.push_back(cv_);
                 if (cv_history_.size() > 3) {
                     cv_history_.pop_front();
                 }
             }
        }
    }
};
