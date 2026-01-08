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

#include "benchmark_config.h"
#include "system_info.h"
#include "process_manager.h"
#include <functional>
#include <chrono>
#include <string>
#include <vector>

class MeasurementStorage {
public:
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

    std::string bw_file_path(double ratio_pct, int pause) const;
    std::string lat_file_path(double ratio_pct, int pause) const;

    std::string traffic_gen_log_file_path(double ratio_pct, int pause) const;

    void cleanup_measurement_files(double ratio_pct, int pause) const;
    void ensure_directories_exist() const;
    void sync_plotter_file() const;
    void write_plotter_file(const CPUCapabilities& caps, const std::string& binding_type, double tlb_ns, int cache_line_size, double upi_scaling_factor = 0.0) const;

    std::vector<double> parse_bw_measurements(const std::string& filepath, double scaling_factor = 1.0) const;
    std::vector<double> parse_lat_measurements(const std::string& filepath,
                                               double cached_tlb_hit_latency_ns) const;

    bool append_to_file_with_lock(const std::string& filepath, const std::string& content) const;

    bool is_temp_root() const { return measurement_root_is_temp_; }

private:
    const BenchmarkConfig& config_;
    int cache_line_size_;
    int verbosity_;
    std::string output_root_override_;

    std::string measurement_root_;
    std::string measurement_bw_dir_;
    std::string measurement_lat_dir_;

    std::string measurement_logs_dir_;
    bool measurement_root_is_temp_;
    mutable bool directories_created_ = false;

    void initialize_storage();
    std::string create_temp_measurement_root() const;
    void copy_plotter_to(const std::string& source_plotter,
                         const std::string& destination_dir) const;
};

class BandwidthMeasurer {
public:
    BandwidthMeasurer(const BenchmarkConfig& config,
                      const system_info& sys_info,
                      MeasurementStorage* storage,
                      TrafficGenProcessManager* traffic_gen_manager,
                      std::function<std::vector<int>()> numa_resolver,
                      ExecutionMode mode = ExecutionMode::MULTISEQUENTIAL);

    virtual ~BandwidthMeasurer() = default;

    void set_sampling_interval_ms(double interval) { sampling_interval_ms_ = interval; }
    double sampling_interval_ms() const { return sampling_interval_ms_; }

    virtual bool sample_bandwidth(long long& cas_rd,
                          long long& cas_wr,
                          double& elapsed,
                          const std::vector<int>& mem_nodes) const = 0;

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

protected:
    const BenchmarkConfig& config_;
    const system_info& sys_info_;
    MeasurementStorage* storage_;
    TrafficGenProcessManager* traffic_gen_manager_;
    std::function<std::vector<int>()> numa_resolver_;
    ExecutionMode mode_;
    double sampling_interval_ms_;
    BandwidthCounterSelection counter_selection_;
    std::map<std::string, long long> extra_perf_values_;

    bool relaunch_traffic_gen(int ratio, int pause, int traffic_gen_cores);
    
public:
    virtual bool monitor_command(const std::string& command, 
                                 std::function<void(double timestamp, double bw_gbps, long long raw_rd, long long raw_wr)> callback, 
                                 bool summary_mode) {
        (void)command; (void)callback; (void)summary_mode;
        return false; 
    }
};

std::unique_ptr<BandwidthMeasurer> create_bandwidth_measurer(
    const BenchmarkConfig& config,
    const system_info& sys_info,
    const CPUCapabilities& caps,
    MeasurementStorage* storage,
    TrafficGenProcessManager* traffic_gen_manager,
    std::function<std::vector<int>()> numa_resolver,
    ExecutionMode mode);

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

class LatencyMeasurer {
public:
    LatencyMeasurer(const BenchmarkConfig& config,
                    MeasurementStorage* storage,
                    PtrChaseProcessManager* ptrchase_manager,
                    std::function<std::vector<int>()> numa_resolver,
                    int cache_line_size);

    bool measure_latency_with_bursts(int burst_count,
                                     PerfBurstCounters& totals,
                                     double& total_accesses);

    void set_tlb_hit_latency_ns(double ns) { tlb_hit_latency_ns_ = ns; }
    double get_accesses_per_burst() const { return ptrchase_accesses_per_burst_; }
    void init_ptrchase_scaling();

    void setExpectedTlbEvents(const std::string& tlb1_event, const std::string& tlb2_event) {
        expected_tlb1_event_ = tlb1_event;
        expected_tlb2_event_ = tlb2_event;
    }

    bool run_single_ptrchase_burst(PerfBurstCounters& out);
    bool parse_perf_burst_output(const std::string& filepath, PerfBurstCounters& out);

    int pipe_fd_ = -1;
    bool burst_running_ = false;
    std::chrono::steady_clock::time_point burst_start_time_;

    bool start_burst_async();
    bool try_collect_burst_async(PerfBurstCounters& out);
    bool is_burst_running() const { return burst_running_; }
    void reset();

private:
    const BenchmarkConfig& config_;
    MeasurementStorage* storage_;
    PtrChaseProcessManager* ptrchase_manager_;
    std::function<std::vector<int>()> numa_resolver_;
    int cache_line_size_;
    double ptrchase_accesses_per_burst_;
    double ptrchase_insts_per_access_;
    uint64_t ptrchase_burst_iters_;
    uint64_t ptrchase_insts_per_iter_;
    double tlb_hit_latency_ns_ = 0.0;
    
    std::string expected_tlb1_event_;
    std::string expected_tlb2_event_;
};

class OutlierDetector {
public:
    OutlierDetector(const BenchmarkConfig& config,
                    MeasurementStorage* storage,
                    double cached_tlb_hit_latency_ns);

    std::vector<double> load_bandwidth(double ratio_pct, int pause, double scaling_factor = 1.0) const;
    std::vector<double> load_latency(double ratio_pct, int pause) const;

    std::vector<int> find_outliers(const std::vector<double>& measurements,
                                   double threshold_pct,
                                   const std::string& unit) const;
                                   
    std::vector<int> find_outliers(const std::vector<BenchmarkResult>& results,
                                   double threshold_pct = 10.0) const;

    bool align_measurement_files(double ratio_pct, int pause, size_t keep) const;
    void remove_measurements_by_indices(double ratio_pct,
                                        int pause,
                                        const std::vector<int>& indices) const;
    void trim_to_last_measurements(double ratio_pct, int pause, int expected_reps) const;

private:
    const BenchmarkConfig& config_;
    MeasurementStorage* storage_;
    double cached_tlb_hit_latency_ns_;
};

#endif
