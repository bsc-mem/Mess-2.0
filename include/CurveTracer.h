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

#ifndef CURVE_TRACER_H
#define CURVE_TRACER_H

#include <functional>
#include <map>
#include <optional>
#include <queue>
#include <string>
#include <unordered_set>
#include <vector>

enum class ScoringVariant { PauseSpace, BWSpace };

struct AdaptiveTier {
    std::string name;
    double tolerance;
    double ambiguity_tolerance;
    int max_budget;
};

struct ProbeStats {
    int pause = 0;
    double bw = 0.0;
    double lat = 0.0;
    double sigma_bw = 0.0;
    double sigma_lat = 0.0;
    int n_reps = 0;
};

struct SuspiciousTriplet {
    int left_pause = -1;
    int center_pause = -1;
    int right_pause = -1;
    double residual_norm = 0.0;
    double slope_dot = 0.0;

    bool is_suspicious(double threshold) const {
        return residual_norm > threshold && slope_dot < 0.0;
    }
};

class CurveTracer {
public:
    using MeasureFn = std::function<ProbeStats(int pause)>;

    CurveTracer(const AdaptiveTier& tier,
                MeasureFn measure_fn,
                int verbosity = 0,
                ScoringVariant variant = ScoringVariant::PauseSpace);

    void seed_point(const ProbeStats& stats, bool count_towards_budget = true);
    void replace_point(const ProbeStats& stats);
    void promote_budget_point(int pause);
    void prepare_seeded_refinement();
    bool has_pending_refinement() const;
    int next_refinement_pause();
    double current_worst_error() const;
    int segments_above_threshold() const { return segments_above_; }
    int point_count() const { return static_cast<int>(budgeted_pauses_.size()); }
    std::vector<int> current_points() const;
    std::optional<SuspiciousTriplet> suspicious_triplet_around(int pause) const;
    double suspicious_threshold(const SuspiciousTriplet& triplet) const;
    SuspiciousTriplet triplet_metrics(int left_pause, int center_pause, int right_pause) const;

    const std::map<int, ProbeStats>& measured_points() const { return points_; }

    static const AdaptiveTier& get_tier(const std::string& name);
    static AdaptiveTier resolve_tier(const std::string& name, int point_count_override = 0);
    static std::vector<int> compute_skeleton_candidates(const std::vector<int>& sorted_points);
    static int split_pause(int p_left, int p_right);

private:
    struct Segment {
        size_t left_idx = 0;
        double error = 0.0;
        bool operator<(const Segment& other) const { return error < other.error; }
    };

    AdaptiveTier tier_;
    MeasureFn measure_;
    int verbosity_;
    ScoringVariant scoring_variant_;

    std::map<int, ProbeStats> points_;
    std::unordered_set<int> budgeted_pauses_;
    std::vector<int> sorted_ps_;
    std::priority_queue<Segment> pq_;
    int segments_above_ = 0;

    double bw_max_ = 0.0;
    double lat_min_ = 1e18;
    double lat_max_ = 0.0;
    double lat_range_ = 0.0;
    int max_reps_ = 1;

    ProbeStats probe(int pause);
    void rebuild_normalization();
    void rebuild_refinement_queue();
    void upsert_point(const ProbeStats& stats, bool count_towards_budget);

    double log_x(int pause) const;
    double normalize_bw(double bw) const;
    double normalize_lat(double lat) const;
    std::vector<int> sorted_points() const;
    SuspiciousTriplet compute_triplet(int left_pause, int center_pause, int right_pause) const;
    double kappa_pause_space(size_t left_idx,
                             const std::vector<const ProbeStats*>& ordered_points) const;
    double kappa_bw_space(size_t left_idx,
                          const std::vector<const ProbeStats*>& ordered_points) const;
    double coupled_transition_ambiguity(
        size_t left_idx,
        const std::vector<const ProbeStats*>& ordered_points) const;
    double segment_error(size_t left_idx,
                         const std::vector<const ProbeStats*>& ordered_points) const;

    static const double DEFAULT_TOLERANCE;
    static const AdaptiveTier TIER_LITE;
    static const AdaptiveTier TIER_STANDARD;
    static const AdaptiveTier TIER_DETAILED;
};

#endif
