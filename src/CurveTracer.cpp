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

#include "CurveTracer.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>

namespace {
constexpr double SUSPICION_ALPHA = 0.25;
constexpr double SUSPICION_MIN_THRESHOLD = 0.03;
}

const double CurveTracer::DEFAULT_TOLERANCE = 0.00005;


const AdaptiveTier CurveTracer::TIER_LITE = {
    "lite",
    CurveTracer::DEFAULT_TOLERANCE,
    0.03,
    15
};

const AdaptiveTier CurveTracer::TIER_STANDARD = {
    "standard",
    CurveTracer::DEFAULT_TOLERANCE,
    0.03,
    50
};

const AdaptiveTier CurveTracer::TIER_DETAILED = {
    "detailed",
    CurveTracer::DEFAULT_TOLERANCE,
    0.03,
    200
};

const AdaptiveTier& CurveTracer::get_tier(const std::string& name) {
    if (name == "lite") {
        return TIER_LITE;
    }
    if (name == "detailed") {
        return TIER_DETAILED;
    }
    return TIER_STANDARD;
}

AdaptiveTier CurveTracer::resolve_tier(const std::string& name, int point_count_override) {
    if (point_count_override > 0) {
        return {"custom", DEFAULT_TOLERANCE, 0.03, point_count_override};
    }
    return get_tier(name);
}

std::vector<int> CurveTracer::compute_skeleton_candidates(const std::vector<int>& sorted_points) {
    std::vector<int> candidates;
    for (size_t i = 0; i + 1 < sorted_points.size(); ++i) {
        int left = sorted_points[i];
        int right = sorted_points[i + 1];
        if (left == 0) {
            continue;
        }
        if (std::log10(static_cast<double>(right) / left) > 2.0) {
            candidates.push_back(
                static_cast<int>(std::sqrt(static_cast<double>(left) * right)));
        }
    }
    return candidates;
}

CurveTracer::CurveTracer(const AdaptiveTier& tier,
                         MeasureFn measure_fn,
                         int verbosity,
                         ScoringVariant variant)
    : tier_(tier),
      measure_(std::move(measure_fn)),
      verbosity_(verbosity),
      scoring_variant_(variant) {}

ProbeStats CurveTracer::probe(int pause) {
    auto it = points_.find(pause);
    if (it != points_.end()) {
        return it->second;
    }

    ProbeStats stats = measure_(pause);
    stats.pause = pause;
    points_[pause] = stats;
    return stats;
}

void CurveTracer::upsert_point(const ProbeStats& stats, bool count_towards_budget) {
    points_[stats.pause] = stats;
    if (count_towards_budget) {
        budgeted_pauses_.insert(stats.pause);
    }
    sorted_ps_.clear();
    pq_ = std::priority_queue<Segment>();
    segments_above_ = 0;
}

void CurveTracer::seed_point(const ProbeStats& stats, bool count_towards_budget) {
    upsert_point(stats, count_towards_budget);
}

void CurveTracer::replace_point(const ProbeStats& stats) {
    upsert_point(stats, budgeted_pauses_.find(stats.pause) != budgeted_pauses_.end());
}

void CurveTracer::promote_budget_point(int pause) {
    if (points_.find(pause) == points_.end()) {
        return;
    }
    budgeted_pauses_.insert(pause);
    sorted_ps_ = sorted_points();
    rebuild_normalization();
    rebuild_refinement_queue();
}

void CurveTracer::prepare_seeded_refinement() {
    sorted_ps_ = sorted_points();
    rebuild_normalization();
    rebuild_refinement_queue();
}

bool CurveTracer::has_pending_refinement() const {
    return point_count() < tier_.max_budget &&
           !pq_.empty() &&
           pq_.top().error >= tier_.tolerance;
}

double CurveTracer::current_worst_error() const {
    return pq_.empty() ? 0.0 : pq_.top().error;
}

int CurveTracer::next_refinement_pause() {
    if (!has_pending_refinement()) {
        return -1;
    }

    while (!pq_.empty()) {
        Segment segment = pq_.top();
        pq_.pop();

        if (segment.left_idx + 1 >= sorted_ps_.size()) {
            continue;
        }

        int left_pause = sorted_ps_[segment.left_idx];
        int right_pause = sorted_ps_[segment.left_idx + 1];
        int pause = split_pause(left_pause, right_pause);
        if (pause < 0 || points_.find(pause) != points_.end()) {
            if (segment.error >= tier_.tolerance && segments_above_ > 0) {
                --segments_above_;
            }
            continue;
        }

        if (verbosity_ >= 3) {
            std::cout << "\r" << std::string(140, ' ') << "\r";
            std::cout << " CurveTracer Split [" << left_pause
                      << ", " << right_pause << "] -> p=" << pause
                      << " (E=" << std::scientific << std::setprecision(3)
                      << segment.error << ")" << std::endl;
        }

        return pause;
    }

    segments_above_ = 0;
    return -1;
}

std::vector<int> CurveTracer::current_points() const {
    return sorted_ps_.empty() ? sorted_points() : sorted_ps_;
}

SuspiciousTriplet CurveTracer::compute_triplet(int left_pause,
                                               int center_pause,
                                               int right_pause) const {
    const ProbeStats& left = points_.at(left_pause);
    const ProbeStats& center = points_.at(center_pause);
    const ProbeStats& right = points_.at(right_pause);

    const double x_left = log_x(left_pause);
    const double x_center = log_x(center_pause);
    const double x_right = log_x(right_pause);
    const double span = x_right - x_left;
    if (span < 1e-12) {
        return {left_pause, center_pause, right_pause, 0.0, 0.0};
    }

    const double lambda = (x_right - x_center) / span;
    const double left_bw = normalize_bw(left.bw);
    const double center_bw = normalize_bw(center.bw);
    const double right_bw = normalize_bw(right.bw);
    const double pred_bw = lambda * left_bw + (1.0 - lambda) * right_bw;

    double left_lat = 0.0;
    double center_lat = 0.0;
    double right_lat = 0.0;
    double pred_lat = 0.0;
    if (lat_range_ > 1e-12) {
        left_lat = normalize_lat(left.lat);
        center_lat = normalize_lat(center.lat);
        right_lat = normalize_lat(right.lat);
        pred_lat = lambda * left_lat + (1.0 - lambda) * right_lat;
    }

    const double residual_bw = center_bw - pred_bw;
    const double residual_lat = center_lat - pred_lat;
    const double residual_norm =
        std::sqrt(residual_bw * residual_bw + residual_lat * residual_lat);

    const double dx_left = x_center - x_left;
    const double dx_right = x_right - x_center;
    if (dx_left < 1e-12 || dx_right < 1e-12) {
        return {left_pause, center_pause, right_pause, residual_norm, 0.0};
    }

    const double slope1_bw = (center_bw - left_bw) / dx_left;
    const double slope2_bw = (right_bw - center_bw) / dx_right;
    const double slope1_lat = (center_lat - left_lat) / dx_left;
    const double slope2_lat = (right_lat - center_lat) / dx_right;
    const double slope_dot = slope1_bw * slope2_bw + slope1_lat * slope2_lat;

    return {left_pause, center_pause, right_pause, residual_norm, slope_dot};
}

double CurveTracer::suspicious_threshold(const SuspiciousTriplet& triplet) const {
    const ProbeStats& left = points_.at(triplet.left_pause);
    const ProbeStats& center = points_.at(triplet.center_pause);
    const ProbeStats& right = points_.at(triplet.right_pause);

    const double sigma_bw_norm =
        (bw_max_ > 1e-12)
            ? std::max({left.sigma_bw, center.sigma_bw, right.sigma_bw}) / bw_max_
            : 0.0;
    const double sigma_lat_norm =
        (lat_range_ > 1e-12)
            ? std::max({left.sigma_lat, center.sigma_lat, right.sigma_lat}) / lat_range_
            : 0.0;
    const double sigma_norm =
        std::sqrt(sigma_bw_norm * sigma_bw_norm + sigma_lat_norm * sigma_lat_norm);

    const double span_bw =
        std::abs(normalize_bw(right.bw) - normalize_bw(left.bw));
    const double span_lat =
        (lat_range_ > 1e-12)
            ? std::abs(normalize_lat(right.lat) - normalize_lat(left.lat))
            : 0.0;
    const double span_norm = std::sqrt(span_bw * span_bw + span_lat * span_lat);

    return std::max({3.0 * sigma_norm, SUSPICION_ALPHA * span_norm, SUSPICION_MIN_THRESHOLD});
}

SuspiciousTriplet CurveTracer::triplet_metrics(int left_pause,
                                               int center_pause,
                                               int right_pause) const {
    return compute_triplet(left_pause, center_pause, right_pause);
}

std::optional<SuspiciousTriplet> CurveTracer::suspicious_triplet_around(int pause) const {
    if (points_.find(pause) == points_.end()) {
        return std::nullopt;
    }

    const std::vector<int> ordered = current_points();
    auto pos_it = std::find(ordered.begin(), ordered.end(), pause);
    if (pos_it == ordered.end()) {
        return std::nullopt;
    }

    const size_t idx = static_cast<size_t>(std::distance(ordered.begin(), pos_it));
    if (idx == 0 || idx + 1 >= ordered.size()) {
        return std::nullopt;
    }

    SuspiciousTriplet triplet =
        compute_triplet(ordered[idx - 1], ordered[idx], ordered[idx + 1]);
    if (!triplet.is_suspicious(suspicious_threshold(triplet))) {
        return std::nullopt;
    }

    return triplet;
}

double CurveTracer::log_x(int pause) const {
    return std::log10(std::max(pause, 1));
}

double CurveTracer::normalize_bw(double bw) const {
    return (bw_max_ < 1e-12) ? 0.0 : bw / bw_max_;
}

double CurveTracer::normalize_lat(double lat) const {
    return (lat_range_ < 1e-12) ? 0.0 : (lat - lat_min_) / lat_range_;
}

void CurveTracer::rebuild_normalization() {
    bw_max_ = 0.0;
    lat_min_ = 1e18;
    lat_max_ = 0.0;
    max_reps_ = 1;

    for (const auto& kv : points_) {
        bw_max_ = std::max(bw_max_, kv.second.bw);
        lat_min_ = std::min(lat_min_, kv.second.lat);
        lat_max_ = std::max(lat_max_, kv.second.lat);
        max_reps_ = std::max(max_reps_, std::max(kv.second.n_reps, 1));
    }

    lat_range_ = lat_max_ - lat_min_;
}

std::vector<int> CurveTracer::sorted_points() const {
    std::vector<int> sorted;
    sorted.reserve(budgeted_pauses_.size());
    for (int pause : budgeted_pauses_) {
        sorted.push_back(pause);
    }
    std::sort(sorted.begin(), sorted.end());
    return sorted;
}

double CurveTracer::kappa_pause_space(
    size_t left_idx,
    const std::vector<const ProbeStats*>& ordered_points) const {
    double kappa_bw = 0.0;
    double kappa_lat = 0.0;

    auto accumulate = [&](size_t idx) {
        if (idx == 0 || idx + 1 >= ordered_points.size()) {
            return;
        }

        int pA = sorted_ps_[idx - 1];
        int pB = sorted_ps_[idx];
        int pC = sorted_ps_[idx + 1];
        if (pA == 0 && pB <= 1) {
            return;
        }

        double xA = log_x(pA);
        double xB = log_x(pB);
        double xC = log_x(pC);
        double h1 = xB - xA;
        double h2 = xC - xB;
        if (h1 < 1e-12 || h2 < 1e-12) {
            return;
        }

        const ProbeStats& a = *ordered_points[idx - 1];
        const ProbeStats& b = *ordered_points[idx];
        const ProbeStats& c = *ordered_points[idx + 1];

        double bA = normalize_bw(a.bw);
        double bB = normalize_bw(b.bw);
        double bC = normalize_bw(c.bw);
        double kb = std::abs(2.0 *
                             (bA / h1 -
                              bB * (1.0 / h1 + 1.0 / h2) +
                              bC / h2) /
                             (h1 + h2));

        double kl = 0.0;
        if (lat_range_ > 1e-12) {
            double lA = normalize_lat(a.lat);
            double lB = normalize_lat(b.lat);
            double lC = normalize_lat(c.lat);
            kl = std::abs(2.0 *
                          (lA / h1 -
                           lB * (1.0 / h1 + 1.0 / h2) +
                           lC / h2) /
                          (h1 + h2));
        }

        int min_n = std::min({a.n_reps, b.n_reps, c.n_reps});
        double reliability = (max_reps_ > 0)
            ? static_cast<double>(std::max(min_n, 0)) / max_reps_
            : 0.0;

        kappa_bw = std::max(kappa_bw, kb * reliability);
        kappa_lat = std::max(kappa_lat, kl * reliability);
    };

    accumulate(left_idx);
    accumulate(left_idx + 1);

    double kappa_max = std::max(kappa_bw, kappa_lat);
    double kappa_min = std::min(kappa_bw, kappa_lat);
    return kappa_max + 0.01 * kappa_min;
}

double CurveTracer::kappa_bw_space(
    size_t left_idx,
    const std::vector<const ProbeStats*>& ordered_points) const {
    double kappa = 0.0;

    auto accumulate = [&](size_t idx) {
        if (idx == 0 || idx + 1 >= ordered_points.size()) {
            return;
        }

        const ProbeStats& a = *ordered_points[idx - 1];
        const ProbeStats& b = *ordered_points[idx];
        const ProbeStats& c = *ordered_points[idx + 1];

        double h1_bw = b.bw - a.bw;
        double h2_bw = c.bw - b.bw;
        int min_n = std::min({a.n_reps, b.n_reps, c.n_reps});
        double reliability = (max_reps_ > 0)
            ? static_cast<double>(std::max(min_n, 0)) / max_reps_
            : 0.0;

        if (std::abs(h1_bw) < 1e-6 || std::abs(h2_bw) < 1e-6) {
            double xA = log_x(sorted_ps_[idx - 1]);
            double xB = log_x(sorted_ps_[idx]);
            double xC = log_x(sorted_ps_[idx + 1]);
            double h1 = xB - xA;
            double h2 = xC - xB;
            if (h1 > 1e-12 && h2 > 1e-12 && lat_range_ > 1e-12) {
                double lA = normalize_lat(a.lat);
                double lB = normalize_lat(b.lat);
                double lC = normalize_lat(c.lat);
                double local_kappa = std::abs(2.0 *
                                              (lA / h1 -
                                               lB * (1.0 / h1 + 1.0 / h2) +
                                               lC / h2) /
                                              (h1 + h2));
                kappa = std::max(kappa, local_kappa * reliability);
            }
            return;
        }

        if (lat_range_ > 1e-12 && bw_max_ > 1e-12) {
            double lA = normalize_lat(a.lat);
            double lB = normalize_lat(b.lat);
            double lC = normalize_lat(c.lat);

            double bA = a.bw / bw_max_;
            double bB = b.bw / bw_max_;
            double bC = c.bw / bw_max_;
            double h1n = bB - bA;
            double h2n = bC - bB;

            if (std::abs(h1n) > 1e-12 && std::abs(h2n) > 1e-12) {
                double local_kappa = std::abs(2.0 *
                                              (lA / h1n -
                                               lB * (1.0 / h1n + 1.0 / h2n) +
                                               lC / h2n) /
                                              (h1n + h2n));
                kappa = std::max(kappa, local_kappa * reliability);
            }
        }
    };

    accumulate(left_idx);
    accumulate(left_idx + 1);
    return kappa;
}

double CurveTracer::coupled_transition_ambiguity(
    size_t left_idx,
    const std::vector<const ProbeStats*>& ordered_points) const {
    const ProbeStats& left = *ordered_points[left_idx];
    const ProbeStats& right = *ordered_points[left_idx + 1];

    const double du = normalize_bw(right.bw) - normalize_bw(left.bw);
    const double dv = (lat_range_ > 1e-12)
        ? (normalize_lat(right.lat) - normalize_lat(left.lat))
        : 0.0;
    const double denom = std::sqrt(du * du + dv * dv);
    if (denom < 1e-12) {
        return 0.0;
    }

    // This is the maximum corner-to-chord deviation for the normalized endpoint
    // rectangle. It is zero for pure horizontal or vertical motion.
    return std::abs(du * dv) / denom;
}

double CurveTracer::segment_error(
    size_t left_idx,
    const std::vector<const ProbeStats*>& ordered_points) const {
    const ProbeStats& left = *ordered_points[left_idx];
    const ProbeStats& right = *ordered_points[left_idx + 1];

    double w_left = static_cast<double>(std::max(left.n_reps, 1));
    double w_right = static_cast<double>(std::max(right.n_reps, 1));
    double weight_sum = w_left + w_right;
    double sigma_bw = (w_left * left.sigma_bw + w_right * right.sigma_bw) / weight_sum;
    double sigma_lat = (w_left * left.sigma_lat + w_right * right.sigma_lat) / weight_sum;
    if (std::abs(right.bw - left.bw) < 3.0 * sigma_bw &&
        std::abs(right.lat - left.lat) < 3.0 * sigma_lat) {
        return 0.0;
    }

    double dx = log_x(sorted_ps_[left_idx + 1]) - log_x(sorted_ps_[left_idx]);
    if (dx < 1e-12) {
        return 0.0;
    }

    double kappa = (scoring_variant_ == ScoringVariant::BWSpace)
        ? kappa_bw_space(left_idx, ordered_points)
        : kappa_pause_space(left_idx, ordered_points);
    double shape_error = 0.0;
    if (kappa > 1e-12) {
        shape_error = dx * dx / 8.0 * kappa;
    }

    const double ambiguity = coupled_transition_ambiguity(left_idx, ordered_points);
    const double ambiguity_error =
        tier_.tolerance * (ambiguity / std::max(tier_.ambiguity_tolerance, 1e-12));

    return std::max(shape_error, ambiguity_error);
}

void CurveTracer::rebuild_refinement_queue() {
    pq_ = std::priority_queue<Segment>();
    segments_above_ = 0;
    if (sorted_ps_.size() < 2) {
        return;
    }

    std::vector<const ProbeStats*> ordered_points;
    ordered_points.reserve(sorted_ps_.size());
    for (int pause : sorted_ps_) {
        ordered_points.push_back(&points_.at(pause));
    }

    for (size_t i = 0; i + 1 < ordered_points.size(); ++i) {
        double error = segment_error(i, ordered_points);
        pq_.push({i, error});
        if (error >= tier_.tolerance) {
            segments_above_++;
        }
    }
}

int CurveTracer::split_pause(int p_left, int p_right) {
    if (p_left == 0) {
        int pause = std::max(1, p_right / 2);
        return (pause < p_right) ? pause : -1;
    }

    int pause = static_cast<int>(
        std::sqrt(static_cast<double>(p_left) * p_right));
    return (pause > p_left && pause < p_right) ? pause : -1;
}
