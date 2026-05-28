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

#ifndef MEASUREMENT_H
#define MEASUREMENT_H

#include "BenchmarkConfig.h"
#include "SystemInfo.h"
#include "ProcessManager.h"
#include <functional>
#include <chrono>
#include <string>
#include <unordered_set>
#include <vector>

/**
 * @file Measurement.h
 * @brief Storage and measurement abstractions used by benchmark execution and profiling.
 */

/**
 * @brief Owns the filesystem layout for one measurement campaign.
 */
/**
 * @brief Manages the persistence of raw and processed benchmark data to disk.
 * 
 * This class ensures that all intermediate logs (from PMU tools, ptr_chase, traffic generators) 
 * are safely written to the specified output directory structure. 
 */
class MeasurementStorage {
public:
    /**
     * @brief Creates measurement storage rooted at either the configured output directory or a temporary directory.
     */
    MeasurementStorage(const BenchmarkConfig& config,
                       int cache_line_size,
                       int verbosity,
                       const std::string& output_root_override = "");
    ~MeasurementStorage();

    MeasurementStorage(const MeasurementStorage&) = delete;
    MeasurementStorage& operator=(const MeasurementStorage&) = delete;

    const std::string& root() const { return measurement_root_; }
    const std::string& bandwidth_dir() const { return measurement_bw_dir_; }
    const std::string& latency_dir() const { return measurement_lat_dir_; }
    const std::string& logs_dir() const { return measurement_logs_dir_; }
    const std::string& sampler_dir() const { return measurement_sampler_dir_; }

    std::string bw_file_path(double ratio_pct, int pause) const;
    std::string lat_file_path(double ratio_pct, int pause) const;
    std::string sampler_file_path(double ratio_pct, int pause) const;

    std::string traffic_gen_log_file_path(double ratio_pct, int pause) const;

    void cleanup_measurement_files(double ratio_pct, int pause) const;
    void ensure_directories_exist() const;
    void write_plotter_file(const CPUCapabilities& caps, const std::string& binding_type, double tlb_ns, int cache_line_size, double upi_scaling_factor = 0.0, double page_walk_lat_ns = -1.0) const;

    bool append_to_file_with_lock(const std::string& filepath, const std::string& content) const;
    bool append_sampler_samples(double ratio_pct, int pause, const std::vector<uint64_t>& latencies_cycles) const;

    std::vector<double> parse_bw_measurements(const std::string& filepath, double scaling_factor) const;

    bool is_temp_root() const { return measurement_root_is_temp_; }

private:
    const BenchmarkConfig& config_;
    int cache_line_size_;
    int verbosity_;
    std::string output_root_override_;

    std::string measurement_root_;
    std::string measurement_bw_dir_;
    std::string measurement_lat_dir_;
    std::string measurement_sampler_dir_;

    std::string measurement_logs_dir_;
    bool measurement_root_is_temp_;
    mutable bool directories_created_ = false;
    mutable std::unordered_set<std::string> sampler_initialized_files_;

    void initialize_storage();
    std::string create_temp_measurement_root() const;
    void copy_plotter_to(const std::string& source_plotter,
                         const std::string& destination_dir) const;
};

/**
 * @brief Abstract bandwidth measurement backend used by the benchmark and profiler.
 */
/**
 * @brief Coordinates hardware performance monitoring tools to measure memory bandwidth.
 * 
 * This class abstracts the actual PMU collection backend (`perf`, `vtune`, `likwid`, or `pcm`).
 * It receives the target NUMA topology from the `BenchmarkExecutor`, queries the `BandwidthCounterStrategy`
 * for the exact PMU events to track, and spawns the monitoring process alongside the traffic generators.
 * Once the traffic phase completes, it parses the tool's raw text output back into unified memory metrics.
 */
class BandwidthMeasurer {
public:
    BandwidthMeasurer(const BenchmarkConfig& config,
                      const system_info& sys_info,
                      const CPUCapabilities& caps,
                      MeasurementStorage* storage,
                      TrafficGenProcessManager* traffic_gen_manager,
                      std::function<std::vector<int>()> numa_resolver,
                      ExecutionMode mode = ExecutionMode::MULTISEQUENTIAL);

    virtual ~BandwidthMeasurer() = default;

    void set_sampling_interval_ms(double interval) { sampling_interval_ms_ = interval; }
    double sampling_interval_ms() const { return sampling_interval_ms_; }

    /**
     * @brief Collects one raw bandwidth sample.
     * @return `true` on success.
     */
    virtual bool sample_bandwidth(long long& cas_rd,
                          long long& cas_wr,
                          double& elapsed,
                          const std::vector<int>& mem_nodes) const = 0;

    /**
     * @brief Samples until the traffic generator is considered stable.
     * @return `true` when a stable point was collected.
     */
    virtual bool wait_for_stabilization(int& samples_collected,
                                long long& bw_cas_rd,
                                long long& bw_cas_wr,
                                double& bw_elapsed,
                                int pause,
                                int ratio_pct,
                                bool reuse_existing_traffic_gen,
                                std::function<void()> on_sample_callback = nullptr) = 0;

    void set_counter_selection(const BandwidthCounterSelection& selection) {
        counter_selection_ = selection;
        scaling_factor_initialized_ = false;
        cached_scaling_factor_ = 1.0;
        if (sys_info_.socket_count > 0 && sys_info_.sockets[0].cache_count > 0) {
            cached_cache_line_size_ = sys_info_.sockets[0].caches[0].line_size_bytes;
        }
        if (selection.cas.uses_read_subtract_formula) {
            cached_cache_line_size_ = 256;
        }
    }

    const BandwidthCounterSelection& get_counter_selection() const {
        return counter_selection_;
    }

    const std::map<std::string, long long>& get_extra_perf_values() const {
        return extra_perf_values_;
    }

    void clear_extra_perf_values() {
        extra_perf_values_.clear();
    }

    /**
     * @brief Converts raw counter values into GB/s.
     * @return Calculated bandwidth in GB/s.
     */
    static double calculate_bandwidth_gbps(long long cas_rd, long long cas_wr, double elapsed_s,
                                           CounterType type, int cache_line_size, double scaling_factor);

protected:
    const BenchmarkConfig& config_;
    const system_info& sys_info_;
    const CPUCapabilities& caps_;
    MeasurementStorage* storage_;
    TrafficGenProcessManager* traffic_gen_manager_;
    std::function<std::vector<int>()> numa_resolver_;
    ExecutionMode mode_;
    double sampling_interval_ms_;
    BandwidthCounterSelection counter_selection_;
    mutable std::map<std::string, long long> extra_perf_values_;
    
    int cached_cache_line_size_ = 64;
    double cached_scaling_factor_ = 1.0;
    mutable bool scaling_factor_initialized_ = false;

    bool relaunch_traffic_gen(int ratio, int pause, int traffic_gen_cores);
    void ensure_scaling_factor_cached() const;
    int get_traffic_gen_cores() const;
    
public:
    using MonitorSampleExtras = std::map<std::string, long long>;
    using MonitorCallback = std::function<void(double timestamp,
                                               double bw_gbps,
                                               long long raw_rd,
                                               long long raw_wr,
                                               const MonitorSampleExtras& extra_values)>;

    /**
     * @brief Profiles an external command and streams bandwidth samples through a callback.
     * @return `true` when the command was monitored successfully.
     */
    virtual bool monitor_command(const std::string& command, 
                                 MonitorCallback callback,
                                 bool summary_mode) {
        (void)command; (void)callback; (void)summary_mode;
        return false; 
    }
};

/**
 * @brief Factory for the configured bandwidth measurement backend.
 */
std::unique_ptr<BandwidthMeasurer> create_bandwidth_measurer(
    const BenchmarkConfig& config,
    const system_info& sys_info,
    const CPUCapabilities& caps,
    MeasurementStorage* storage,
    TrafficGenProcessManager* traffic_gen_manager,
    std::function<std::vector<int>()> numa_resolver,
    ExecutionMode mode);

/**
 * @brief Raw counter values emitted by one pointer-chase burst.
 */
struct PerfBurstCounters {
    double cycles;
    double instructions;
    double tlb1miss;
    double tlb2miss;
    double duration_s;
    bool using_hugepages;
    
    std::string tlb1_event_name;
    std::string tlb2_event_name;
};

/**
 * @brief Manages pointer-chase latency bursts and their associated counter collection.
 *
 */
class LatencyMeasurer {
public:
    LatencyMeasurer(const BenchmarkConfig& config,
                    MeasurementStorage* storage,
                    PtrChaseProcessManager* ptrchase_manager,
                    std::function<std::vector<int>()> numa_resolver,
                    int cache_line_size);

    void set_tlb_hit_latency_ns(double ns) { tlb_hit_latency_ns_ = ns; }
    double get_accesses_per_burst() const { return ptrchase_accesses_per_burst_; }
    void init_ptrchase_scaling();

    void setExpectedTlbEvents(const std::string& tlb1_event, const std::string& tlb2_event) {
        expected_tlb1_event_ = tlb1_event;
        expected_tlb2_event_ = tlb2_event;
    }

    int pipe_fd_ = -1;
    bool burst_running_ = false;
    std::chrono::steady_clock::time_point burst_start_time_;

    bool start_burst_async();
    bool try_collect_burst_async(PerfBurstCounters& out);
    bool is_burst_running() const { return burst_running_; }
    void reset();

private:
    const BenchmarkConfig& config_;
    PtrChaseProcessManager* ptrchase_manager_;
    std::function<std::vector<int>()> numa_resolver_;
    double ptrchase_accesses_per_burst_;
    double ptrchase_insts_per_access_;
    uint64_t ptrchase_burst_iters_;
    uint64_t ptrchase_insts_per_iter_;
    double tlb_hit_latency_ns_ = 0.0;
    
    std::string expected_tlb1_event_;
    std::string expected_tlb2_event_;
};

/**
 * @brief Detects and removes measurement outliers from persisted raw data.
 */
/**
 * @brief Validates benchmark measurements to discard noisy or invalid samples.
 * 
 * Performance measuring on shared systems is prone to noise (e.g., OS jitter, thermal throttling, 
 * scheduling artifacts). This class analyzes the gathered `BenchmarkResult` (comparing measured
 * BW/latency against previous samples or theoretical peaks). If the variance is too high or 
 * cache hits instead of DRAM accesses are detected, it signals the `BenchmarkExecutor` to retry
 * the measurement point.
 */
class OutlierDetector {
public:
    explicit OutlierDetector(const BenchmarkConfig& config);

    std::vector<int> find_outliers(const std::vector<double>& measurements,
                                   double threshold_pct,
                                   const std::string& unit) const;
                                   
    std::vector<int> find_outliers(const std::vector<BenchmarkResult>& results,
                                   double threshold_pct = 10.0) const;

private:
    const BenchmarkConfig& config_;
};

#endif
