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

#include "benchmark_config.h"
#include "codegen.h"
#include <vector>
#include <map>
#include <string>
#include <chrono>
#include <future>
#include <thread>
#include <semaphore.h>
#include <memory>

#include "utils.h"
#include "system_info.h"
#include "ptrchase_perf_helper.h"
#include "measurement.h"
#include "process_manager.h"
#include "utils/progress_tracker.h"
#include "architecture/BandwidthCounterStrategy.h"

class BenchmarkExecutor {
private:
    const BenchmarkConfig& config_;
    const system_info& sys_info_;
    const CPUCapabilities& caps_;
    std::vector<BenchmarkResult> results_;
    std::vector<double> candidate_ratios_;         
    int cache_line_size_;                          

    static const int LAT_REQUIRED_STABLE_COUNT = 3;

    std::unique_ptr<TrafficGenProcessManager> traffic_gen_manager_;
    std::unique_ptr<PtrChaseProcessManager> ptrchase_manager_;
    
    std::unique_ptr<MeasurementStorage> measurement_storage_multiseq_;
    std::unique_ptr<BandwidthMeasurer> bandwidth_measurer_multiseq_;
    std::unique_ptr<LatencyMeasurer> latency_measurer_multiseq_;
    std::unique_ptr<OutlierDetector> outlier_detector_multiseq_;


    static double cached_tlb_hit_latency_ns_;
    static bool frequency_written_to_plotter_; 

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

    BenchmarkStatus run_single_benchmark(double ratio_pct, int pause, int rep, BenchmarkResult& result, ExecutionMode mode);
    void write_frequency_to_plotter(double cpu_freq_ghz);
    
    double calculate_latency_from_perf_values(double cycles, double accesses, double tlb1miss, double tlb2miss, double interval_duration, bool using_hugepages = false);
    
    std::vector<int> get_memory_binding_nodes() const;    

    void cleanup_measurement_files(double ratio_pct, int pause) const;

    bool check_and_rerun_outliers(double ratio_pct, int pause, ExecutionMode mode);
    
    std::string format_bandwidth_output(const BenchmarkResult& result, const BandwidthCounterSelection& selection) const;
    std::string format_latency_output(const BenchmarkResult& result, pid_t ptrchase_pid) const;

    double get_upi_scaling_factor() const;
    std::vector<int> resolve_monitored_nodes(int src_cpu) const;

public:
    void force_cleanup();
    BenchmarkExecutor(const BenchmarkConfig& config, const system_info& sys_info, const CPUCapabilities& caps,
                     double tlb_latency_ns = 7.0, int cache_line_size = 64);
    ~BenchmarkExecutor();
    
    BenchmarkExecutor(const BenchmarkExecutor&) = delete;
    BenchmarkExecutor& operator=(const BenchmarkExecutor&) = delete;

    bool run();
    const std::vector<BenchmarkResult>& get_results() const { return results_; }
    double get_total_runtime() const { return progress_tracker_->total_runtime(); }
    const std::vector<double>& get_iteration_times() const { return progress_tracker_->iteration_times(); }
    
    double get_min_bandwidth_mbps() const { return progress_tracker_->min_bandwidth_mbps(); }
    double get_max_bandwidth_mbps() const { return progress_tracker_->max_bandwidth_mbps(); }
    double get_min_latency_ns() const { return progress_tracker_->min_latency_ns(); }
    double get_max_latency_ns() const { return progress_tracker_->max_latency_ns(); }
    
    const std::map<double, std::pair<double, double>>& get_ratio_bw_ranges() const { return progress_tracker_->ratio_bw_ranges(); }
    const std::map<double, std::pair<double, double>>& get_ratio_lat_ranges() const { return progress_tracker_->ratio_lat_ranges(); }
};

#endif
