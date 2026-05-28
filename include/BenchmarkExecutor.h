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

#ifndef BENCHMARK_EXECUTOR_H
#define BENCHMARK_EXECUTOR_H

#include "BenchmarkConfig.h"
#include "Codegen.h"
#include <vector>
#include <map>
#include <string>
#include <chrono>
#include <future>
#include <thread>
#include <semaphore.h>
#include <memory>

#include "Utils.h"
#include "SystemInfo.h"
#include "PtrchasePerfHelper.h"
#include "Measurement.h"
#include "ProcessManager.h"
#include "utils/ProgressTracker.h"
#include "architecture/BandwidthCounterStrategy.h"
#include "CurveTracer.h"
#include "measurement/InstructionSampler.h"

/**
 * @file BenchmarkExecutor.h
 * @brief Main benchmark orchestration entry point.
 */

/**
 * @brief Coordinates traffic generation, bandwidth measurement, latency measurement, and result aggregation.
 */
/**
 * @brief Coordinates the full execution flow of the memory benchmark.
 * 
 * The `BenchmarkExecutor` acts as the master controller for the test lifecycle.
 * For each traffic ratio point configured, it:
 * 1. Synchronizes the traffic generation worker processes (`TrafficGenProcessManager`).
 * 2. Synchronizes the latency probing process (`PtrChaseProcessManager`).
 * 3. Manages the lifecycle of hardware performance monitors (`BandwidthMeasurer`, `PowerMeasurer`).
 * 4. Filters out noise and hardware anomalies via the `OutlierDetector`.
 * 5. Collects the stable measurement loops into `BenchmarkResult` instances.
 * 
 * It abstracts away the heavy lifting of IPC (semaphores, pipes) and thread lifecycle management.
 */
class BenchmarkExecutor {
private:
    const BenchmarkConfig& config_;
    const system_info& sys_info_;
    const CPUCapabilities& caps_;
    std::vector<BenchmarkResult> results_;
    int cache_line_size_;                          

    static const int LAT_REQUIRED_STABLE_COUNT = 3;
    BandwidthCounterStrategy* counter_strategy_;

    std::unique_ptr<TrafficGenProcessManager> traffic_gen_manager_;
    std::unique_ptr<PtrChaseProcessManager> ptrchase_manager_;
    
    std::unique_ptr<MeasurementStorage> measurement_storage_multiseq_;
    std::unique_ptr<BandwidthMeasurer> bandwidth_measurer_multiseq_;
    std::unique_ptr<LatencyMeasurer> latency_measurer_multiseq_;
    std::unique_ptr<OutlierDetector> outlier_detector_multiseq_;

    
    std::unique_ptr<InstructionSampler> instruction_sampler_;

    static double cached_tlb_hit_latency_ns_;
    static bool frequency_written_to_plotter_; 
    static double cached_cpuinfo_freq_ghz_;

    double max_cpu_freq_ghz_ = 0.0;
    
    std::unique_ptr<ProgressTracker> progress_tracker_;

    mutable BandwidthCounterSelection cached_bw_counters_;
    mutable bool bw_counters_discovered_ = false;
    void discover_bandwidth_counters_if_needed() const;

    enum class BenchmarkStatus {
        SUCCESS,
        FAILURE,
        RETRY_BW_GROWTH
    };

    struct AdaptiveMeasuredPoint {
        ProbeStats stats;
        std::vector<BenchmarkResult> results;
    };

    BenchmarkStatus run_single_benchmark(double ratio_pct, int pause, int rep, BenchmarkResult& result, ExecutionMode mode);
    void write_frequency_to_plotter(double cpu_freq_ghz);
    
    double calculate_latency_from_perf_values(double cycles, double accesses, double tlb1miss, double tlb2miss, double interval_duration, bool using_hugepages = false);
    
    std::vector<int> get_memory_binding_nodes() const;    

    void cleanup_measurement_files(double ratio_pct, int pause) const;

    std::string mode_label(ExecutionMode mode) const;
    MeasurementStorage* storage_for_mode(ExecutionMode mode) const;
    OutlierDetector* outlier_detector_for_mode(ExecutionMode mode) const;
    BandwidthMeasurer* bandwidth_measurer_for_mode(ExecutionMode mode) const;
    void flush_point_results(double ratio_pct,
                             int pause,
                             ExecutionMode mode,
                             const std::vector<BenchmarkResult>& final_results);
    void remove_results_for_point(double ratio_pct, int pause, ExecutionMode mode);
    void account_measured_point_progress(const std::vector<BenchmarkResult>& final_results);
    bool resolve_point_measurements(double ratio_pct,
                                    int pause,
                                    ExecutionMode mode,
                                    int& total_global_measurements,
                                    std::vector<BenchmarkResult>& final_results,
                                    double bw_peak = 0.0,
                                    bool skip_outliers_for_low_bw = false,
                                    bool count_progress = true,
                                    bool replace_existing = false);
    ProbeStats summarize_probe_stats(int pause,
                                     const std::vector<BenchmarkResult>& final_results) const;

    std::string format_bandwidth_output(const BenchmarkResult& result, const BandwidthCounterSelection& selection) const;
    std::string format_latency_output(const BenchmarkResult& result, pid_t ptrchase_pid) const;

    double get_upi_scaling_factor() const;
    std::vector<int> resolve_monitored_nodes(int src_cpu) const;

    bool run_adaptive_for_ratio(double ratio_pct,
                                const std::vector<ExecutionMode>& modes,
                                int& total_global_measurements);
    AdaptiveMeasuredPoint adaptive_probe(double ratio_pct, int pause,
                                         ExecutionMode mode,
                                         int& total_global_measurements,
                                         double bw_peak,
                                         bool count_progress = true,
                                         bool replace_existing = false);

public:
    /**
     * @brief Builds an executor bound to a resolved runtime configuration.
     * @param config Benchmark configuration to execute.
     * @param sys_info Detected static system information.
     * @param caps Detected architectural capabilities.
     * @param tlb_latency_ns Cached TLB-hit penalty used when correcting latency.
     * @param cache_line_size Cache-line size used for bandwidth conversion.
     */
    void force_cleanup();
    BenchmarkExecutor(const BenchmarkConfig& config, const system_info& sys_info, const CPUCapabilities& caps,
                     double tlb_latency_ns = 7.0, int cache_line_size = 64);

    /**
     * @brief Releases managed subprocesses and measurement resources.
     */
    ~BenchmarkExecutor();
    
    BenchmarkExecutor(const BenchmarkExecutor&) = delete;
    BenchmarkExecutor& operator=(const BenchmarkExecutor&) = delete;

    /**
     * @brief Executes the configured benchmark sweep.
     * @return `true` when the run completed successfully.
     */
    bool run();

    /**
     * @brief Returns the accumulated benchmark points.
     * @return Immutable view of all recorded results.
     */
    const std::vector<BenchmarkResult>& get_results() const { return results_; }

    /**
     * @brief Returns the total measured benchmark runtime.
     * @return Sum of recorded iteration times in seconds.
     */
    double get_total_runtime() const { return progress_tracker_->total_runtime(); }

    /**
     * @brief Returns the per-iteration runtime history.
     * @return Immutable view of iteration durations in seconds.
     */
    const std::vector<double>& get_iteration_times() const { return progress_tracker_->iteration_times(); }
    
    /** @brief Returns the smallest bandwidth observed by the progress tracker. */
    double get_min_bandwidth_mbps() const { return progress_tracker_->min_bandwidth_mbps(); }
    /** @brief Returns the largest bandwidth observed by the progress tracker. */
    double get_max_bandwidth_mbps() const { return progress_tracker_->max_bandwidth_mbps(); }
    /** @brief Returns the smallest latency observed by the progress tracker. */
    double get_min_latency_ns() const { return progress_tracker_->min_latency_ns(); }
    /** @brief Returns the largest latency observed by the progress tracker. */
    double get_max_latency_ns() const { return progress_tracker_->max_latency_ns(); }
    
    /**
     * @brief Returns per-ratio bandwidth ranges observed during the run.
     * @return Map keyed by requested ratio percentage.
     */
    const std::map<double, std::pair<double, double>>& get_ratio_bw_ranges() const { return progress_tracker_->ratio_bw_ranges(); }

    /**
     * @brief Returns per-ratio latency ranges observed during the run.
     * @return Map keyed by requested ratio percentage.
     */
    const std::map<double, std::pair<double, double>>& get_ratio_lat_ranges() const { return progress_tracker_->ratio_lat_ranges(); }
};

#endif
