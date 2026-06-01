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

#include "BenchmarkExecutor.h"
#include "Measurement.h"
#include "ProcessManager.h"
#include "architecture/ArchitectureRegistry.h"
#include "Utils.h"
#include <unistd.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <signal.h>
#include <sys/file.h>
#include <fstream>
#include <sstream>
#include <cstring>
#include <iostream>
#include <filesystem>
#include <random>
#include <future>
#include <thread>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <limits>
#include <map>
#include <cstdlib>
#include <cstdio>
#include <memory>
#include <set>
#include <iomanip>
#include <vector>
#include <utility>
#include <csignal>
#include <deque>
#include <stdexcept>
#include <system_error>
#include <semaphore.h>
#include <cctype>
#include <cerrno>
#include <sys/ioctl.h>
#include <sys/syscall.h>
#ifdef __linux__
#include <linux/perf_event.h>
#endif
#ifdef __linux__
#include <asm/unistd.h>
#endif
#include "PtrchasePerfHelper.h"
#include "architecture/BandwidthCounterStrategy.h"
#include "measurement/InstructionSamplerFactory.h"

double BenchmarkExecutor::cached_tlb_hit_latency_ns_ = 10.0;
bool BenchmarkExecutor::frequency_written_to_plotter_ = false; 
double BenchmarkExecutor::cached_cpuinfo_freq_ghz_ = 0.0;

BenchmarkExecutor::BenchmarkExecutor(const BenchmarkConfig& config, const system_info& sys_info, const CPUCapabilities& caps,
                                   double tlb_latency_ns, int cache_line_size)
    : config_(config),
      sys_info_(sys_info),
      caps_(caps),
      cache_line_size_(cache_line_size),
      counter_strategy_(&BandwidthCounterStrategy::instance()),
      traffic_gen_manager_(std::make_unique<TrafficGenProcessManager>(config, sys_info)),
      ptrchase_manager_(std::make_unique<PtrChaseProcessManager>(config, sys_info)),
      measurement_storage_multiseq_(config.generate_multiseq
          ? std::make_unique<MeasurementStorage>(config, cache_line_size, config.verbosity, config.output_root + "/multisequential")
          : nullptr),
      bandwidth_measurer_multiseq_(nullptr),
      latency_measurer_multiseq_(config.generate_multiseq
          ? std::make_unique<LatencyMeasurer>(config, measurement_storage_multiseq_.get(), ptrchase_manager_.get(), [this]() { return get_memory_binding_nodes(); }, cache_line_size)
          : nullptr),
      outlier_detector_multiseq_(config.generate_multiseq
          ? std::make_unique<OutlierDetector>(config)
          : nullptr),
      max_cpu_freq_ghz_(0.0),
      progress_tracker_(std::make_unique<ProgressTracker>(0)) {
    
    if (config_.instruction_sampling) {
        instruction_sampler_ = InstructionSamplerFactory::create(
            SamplerBackend::AUTO, 50, 0, config_.verbosity);
        if (!instruction_sampler_ && config_.verbosity >= 1) {
            std::cerr << "    Warning: No instruction sampler backend available (PEBS/SPE)\n";
        }
    }

    if (config_.generate_multiseq) {
        bandwidth_measurer_multiseq_ = create_bandwidth_measurer(
            config, sys_info, caps,
            measurement_storage_multiseq_.get(),
            traffic_gen_manager_.get(),
            [this]() { return get_memory_binding_nodes(); },
            ExecutionMode::MULTISEQUENTIAL);
        if (!bandwidth_measurer_multiseq_) {
            throw std::runtime_error("Failed to create MultiSequential bandwidth measurer");
        }
    }

    
    cached_tlb_hit_latency_ns_ = tlb_latency_ns;

    uint64_t tlb1_raw = 0, tlb2_raw = 0;
    bool use_tlb1 = false, use_tlb2 = false;
    ptrchase_perf::select_tlb_events_for_ptrchase(sys_info, tlb1_raw, tlb2_raw, use_tlb1, use_tlb2);
    
    auto arch_for_fmt = ArchitectureRegistry::instance().getArchitecture(caps);
    auto fmt_strategy = arch_for_fmt ? arch_for_fmt->createCounterStrategy(caps) : nullptr;
    const BandwidthCounterStrategy* fmt = fmt_strategy ? fmt_strategy.get() : counter_strategy_;
    std::string tlb1_str = use_tlb1 ? fmt->formatTlbEventName(0, tlb1_raw) : "";
    std::string tlb2_str = use_tlb2 ? fmt->formatTlbEventName(1, tlb2_raw) : "";

    if (latency_measurer_multiseq_) {
        latency_measurer_multiseq_->setExpectedTlbEvents(tlb1_str, tlb2_str);
    }


    std::string binding_type = "local";
    if (!config_.memory_bind_nodes.empty()) {
        int src_cpu = 0;
        if (!config_.traffic_gen_explicit_cores.empty()) {
            try {
                src_cpu = std::stoi(config_.traffic_gen_explicit_cores[0]);
            } catch (...) {}
        }
        
        int src_node = get_numa_node_of_cpu(src_cpu);
        for (int node : config_.memory_bind_nodes) {
            if (node != src_node) {
                binding_type = "remote";
                break;
            }
        }
    }

    double upi_scaling_factor = 0.0;
    auto arch = ArchitectureRegistry::instance().getArchitecture(caps);
    if (arch) {
        double factor = arch->getUpiScalingFactor(caps);
        if (factor > 0) {
            upi_scaling_factor = 1.0 / factor;
        }
    }

    const double page_walk_lat_ns = fmt ? fmt->getPageWalkLatencyNs() : -1.0;
    if (measurement_storage_multiseq_) {
        measurement_storage_multiseq_->write_plotter_file(caps, binding_type, tlb_latency_ns, cache_line_size, upi_scaling_factor, page_walk_lat_ns);
    }


    discover_bandwidth_counters_if_needed();
}

BenchmarkExecutor::~BenchmarkExecutor() {
    force_cleanup();
}

void BenchmarkExecutor::discover_bandwidth_counters_if_needed() const {
    if (bw_counters_discovered_) {
        return;
    }
    bw_counters_discovered_ = true;

    auto& strategy = BandwidthCounterStrategy::instance();
    if (strategy.is_initialized()) {
        cached_bw_counters_ = strategy.get_selection();
        return;
    }

    const auto& cache = SystemToolsCache::instance();
    int src_cpu = cache.initialized ? cache.ptr_chase_cpu : 0;
    if (!config_.traffic_gen_explicit_cores.empty()) {
        try {
            src_cpu = std::stoi(config_.traffic_gen_explicit_cores[0]);
        } catch (...) {}
    }

    std::vector<int> monitored_nodes = resolve_monitored_nodes(src_cpu);

    strategy.set_measurer_type(string_to_measurer_type(config_.measurer));
    strategy.set_extra_counters(config_.add_counters);
    strategy.set_memory_type(caps_.memory_type);
    strategy.initialize(src_cpu, monitored_nodes, caps_);
    cached_bw_counters_ = strategy.get_selection();
}

void BenchmarkExecutor::force_cleanup() {
    if (ptrchase_manager_) {
        ptrchase_manager_->cleanup();
    }
    if (traffic_gen_manager_) {
        traffic_gen_manager_->kill_all_traffic_gen();
    }
}

double BenchmarkExecutor::get_upi_scaling_factor() const {
    auto arch = ArchitectureRegistry::instance().getArchitecture(caps_);
    if (arch) {
        return arch->getUpiScalingFactor(caps_);
    }
    return 1.0 / 9.0;
}

/**
 * @brief Determines which physical NUMA nodes or sockets should be monitored by PMU counters.
 * 
 * Hardware bandwidth counters are often tied to specific memory controllers (IMCs), which are
 * associated with NUMA nodes. This function analyzes the benchmark's execution binding
 * (e.g., local, remote, or specific NUMA node) and the CPU executing the traffic generator
 * (`src_cpu`) to resolve the list of NUMA nodes whose IMCs must be monitored.
 * 
 * For instance, on AMD systems, memory channels are scoped tightly per node; on Intel, it may
 * be per socket.
 * 
 * @param src_cpu The CPU core ID where the primary traffic generator thread is pinned.
 * @return std::vector<int> A list of NUMA node IDs to attach performance counters to.
 */
std::vector<int> BenchmarkExecutor::resolve_monitored_nodes(int src_cpu) const {
    std::vector<int> monitored_nodes = get_memory_binding_nodes();
    if (monitored_nodes.empty()) {
        const auto& cpu_topo = SystemToolsCache::instance().cpu_topology;
        if (cpu_topo.count(src_cpu)) {
            int socket = cpu_topo.at(src_cpu).socket;
            std::map<int, std::set<int>> socket_nodes = get_socket_to_nodes_map(cpu_topo);
            if (socket_nodes.count(socket)) {
                for (int node : socket_nodes[socket]) {
                    monitored_nodes.push_back(node);
                }
            }
        }
        if (monitored_nodes.empty() && sys_info_.numa_node_count > 0) {
            monitored_nodes.push_back(sys_info_.numa_nodes[0].id);
        }
    }
    return monitored_nodes;
}

void BenchmarkExecutor::cleanup_measurement_files(double ratio_pct, int pause) const {

    MeasurementStorage* storage = measurement_storage_multiseq_.get();
    storage->cleanup_measurement_files(ratio_pct, pause);
}

std::string BenchmarkExecutor::mode_label(ExecutionMode mode) const {
    (void)mode;
    return "MULTISEQ";
}

MeasurementStorage* BenchmarkExecutor::storage_for_mode(ExecutionMode mode) const {
    (void)mode;
    return measurement_storage_multiseq_.get();
}

OutlierDetector* BenchmarkExecutor::outlier_detector_for_mode(ExecutionMode mode) const {
    (void)mode;
    return outlier_detector_multiseq_.get();
}

BandwidthMeasurer* BenchmarkExecutor::bandwidth_measurer_for_mode(ExecutionMode mode) const {
    (void)mode;
    return bandwidth_measurer_multiseq_.get();
}

std::string BenchmarkExecutor::format_bandwidth_output(const BenchmarkResult& result, const BandwidthCounterSelection& selection) const {
    std::stringstream bw_ss;
    
    std::string read_event = "unc_m_cas_count.rd";
    std::string write_event = "unc_m_cas_count.wr";
    
    if (selection.type == CounterType::UPI_FLITS) {
        if (!selection.upi.rxl_data_events.empty()) read_event = selection.upi.rxl_data_events[0];
        if (!selection.upi.txl_data_events.empty()) write_event = selection.upi.txl_data_events[0];
    } else if (selection.type == CounterType::CAS_COUNT || selection.type == CounterType::NVIDIA_GRACE) {
        if (!selection.cas.read_events.empty()) {
            std::string evt = selection.cas.read_events[0];
            if (evt.find("event=") == std::string::npos) read_event = evt;
        }
        if (!selection.cas.write_events.empty()) {
            std::string evt = selection.cas.write_events[0];
            if (evt.find("event=") == std::string::npos) write_event = evt;
        }
        if (!selection.cas.combined_events.empty() && selection.cas.read_events.empty()) {
            read_event = selection.cas.combined_events[0];
            write_event = selection.cas.combined_events[0];
        }
    }

    const bool is_pcm_cxl_mode =
        read_event.find("pcm_cxl/") != std::string::npos ||
        write_event.find("pcm_cxl/") != std::string::npos;
    
    bw_ss << "\n Performance counter stats for system wide:\n\n";
    if (is_pcm_cxl_mode) {
        auto get_extra_scaled = [&](const std::string& key) -> double {
            auto it = result.extra_perf_values.find(key);
            if (it == result.extra_perf_values.end()) {
                return 0.0;
            }
            return static_cast<double>(it->second) / 100.0;
        };

        double read_mbps = get_extra_scaled("pcm_cxl_read_mbps_x100");
        double write_mbps = get_extra_scaled("pcm_cxl_write_mbps_x100");
        double total_mbps = get_extra_scaled("pcm_cxl_total_mbps_x100");

        if (read_mbps <= 0.0 && write_mbps <= 0.0 && result.bw_elapsed > 0.0) {
            read_mbps = (static_cast<double>(result.bw_cas_rd) * cache_line_size_) / (result.bw_elapsed * 1e6);
            write_mbps = (static_cast<double>(result.bw_cas_wr) * cache_line_size_) / (result.bw_elapsed * 1e6);
            total_mbps = read_mbps + write_mbps;
        } else if (total_mbps <= 0.0) {
            total_mbps = read_mbps + write_mbps;
        }

        bw_ss << "S0           " << result.traffic_gen_samples << "                  "
              << std::fixed << std::setprecision(2) << read_mbps
              << "      pcm_cxl_read_mbps\n";
        bw_ss << "S0           " << result.traffic_gen_samples << "                  "
              << std::fixed << std::setprecision(2) << write_mbps
              << "      pcm_cxl_write_mbps\n";
        bw_ss << "S0           " << result.traffic_gen_samples << "                  "
              << std::fixed << std::setprecision(2) << total_mbps
              << "      pcm_cxl_total_mbps\n";
    } else if (selection.cas.uses_read_subtract_formula) {
        auto get_extra_value = [&](const std::string& event) -> long long {
            auto it = result.extra_perf_values.find(event);
            return (it != result.extra_perf_values.end()) ? it->second : 0LL;
        };

        long long subtract_total = 0;
        for (const auto& event : selection.cas.read_subtract_events) {
            subtract_total += get_extra_value(event);
        }

        const long long raw_read = result.bw_cas_rd + subtract_total;
        const long long raw_write = result.bw_cas_wr;

        bw_ss << "S0           " << result.traffic_gen_samples << "                  " << result.bw_cas_rd
              << "      a64fx_read_effective\n";
        bw_ss << "S0           " << result.traffic_gen_samples << "                  " << result.bw_cas_wr
              << "      a64fx_write_effective\n";
        bw_ss << "S0           " << result.traffic_gen_samples << "                  " << raw_read
              << "      r17\n";
        bw_ss << "S0           " << result.traffic_gen_samples << "                  " << raw_write
              << "      r18\n";
        for (const auto& event : selection.cas.read_subtract_events) {
            bw_ss << "S0           " << result.traffic_gen_samples << "                  " << get_extra_value(event)
                  << "      " << event << "\n";
        }
    } else if (read_event.find("dram_channel_data_controller") != std::string::npos || 
        write_event.find("dram_channel_data_controller") != std::string::npos) {
        long long total_cas = result.bw_cas_rd + result.bw_cas_wr;
        bw_ss << "S0           " << result.traffic_gen_samples << "                  " << total_cas << "      dram_channel_data_controller_total\n";
    } else {
        bw_ss << "S0           " << result.traffic_gen_samples << "                  " << result.bw_cas_rd << "      " << read_event << "\n";
        bw_ss << "S0           " << result.traffic_gen_samples << "                  " << result.bw_cas_wr << "      " << write_event << "\n";
    }

    for (const auto& kv : result.extra_perf_values) {
        if (is_pcm_cxl_mode &&
            (kv.first == "pcm_cxl_read_mbps_x100" ||
             kv.first == "pcm_cxl_write_mbps_x100" ||
             kv.first == "pcm_cxl_total_mbps_x100")) {
            continue;
        }
        if (selection.cas.uses_read_subtract_formula &&
            (kv.first == "r17" || kv.first == "r18" ||
             std::find(selection.cas.read_subtract_events.begin(),
                       selection.cas.read_subtract_events.end(),
                       kv.first) != selection.cas.read_subtract_events.end())) {
            continue;
        }
        bw_ss << "S0           " << result.traffic_gen_samples << "                  " << kv.second << "      " << kv.first << "\n";
    }

    bw_ss << "\n       " << std::fixed << std::setprecision(9) << result.bw_elapsed << " seconds time elapsed\n\n";
    
    return bw_ss.str();
}

std::string BenchmarkExecutor::format_latency_output(const BenchmarkResult& result, pid_t ptrchase_pid) const {
    std::stringstream lat_ss;
    lat_ss << "\n Performance counter stats for process id '" << ptrchase_pid << "':\n\n";

    if (config_.instruction_sampling) {
        lat_ss << std::fixed << std::setprecision(0);
        lat_ss << "   " << result.sampler_samples << "      samples\n";
        lat_ss << std::fixed << std::setprecision(2);
        lat_ss << "   " << result.sampler_mean_ns << "      mean_ns\n";
        lat_ss << "   " << result.sampler_iqr_mean_ns << "      iqr_mean_ns\n";
        lat_ss << "   " << result.sampler_min_ns << "      min_ns\n";
        lat_ss << "   " << result.sampler_median_ns << "      median_ns\n";
        lat_ss << "   " << result.sampler_p90_ns << "      p90_ns\n";
        lat_ss << "   " << result.sampler_p95_ns << "      p95_ns\n";
        lat_ss << "   " << result.sampler_p99_ns << "      p99_ns\n";
        lat_ss << "   " << result.sampler_p99_9_ns << "      p99_9_ns\n";
        lat_ss << "   " << result.sampler_max_ns << "      max_ns\n";
        lat_ss << "   " << result.sampler_mean_cycles << "      mean_cycles\n";
        lat_ss << "   " << result.sampler_iqr_mean_cycles << "      iqr_mean_cycles\n";
        lat_ss << "   " << result.sampler_min_cycles << "      min_cycles\n";
        lat_ss << "   " << result.sampler_median_cycles << "      median_cycles\n";
        lat_ss << "   " << result.sampler_p90_cycles << "      p90_cycles\n";
        lat_ss << "   " << result.sampler_p95_cycles << "      p95_cycles\n";
        lat_ss << "   " << result.sampler_p99_cycles << "      p99_cycles\n";
        lat_ss << "   " << result.sampler_p99_9_cycles << "      p99_9_cycles\n";
        lat_ss << "   " << result.sampler_max_cycles << "      max_cycles\n";
        lat_ss << std::fixed << std::setprecision(9);
        lat_ss << "\n       " << result.bw_elapsed << " seconds time elapsed\n\n";
    } else {
        lat_ss << std::fixed << std::setprecision(0);
        lat_ss << "   " << result.cycles << "      cycles\n";
        lat_ss << "   " << result.instructions << "      instructions\n";
        lat_ss << "   " << result.accesses << "      accesses\n";
        lat_ss << "   " << result.tlb1miss << "      " << result.tlb1_event_name << "\n";
        lat_ss << "   " << result.tlb2miss << "      " << result.tlb2_event_name << "\n";
        lat_ss << std::fixed << std::setprecision(9);
        lat_ss << "\n       " << result.duration_s << " seconds time elapsed\n\n";
    }

    return lat_ss.str();
}


void BenchmarkExecutor::flush_point_results(double ratio_pct,
                                            int pause,
                                            ExecutionMode mode,
                                            const std::vector<BenchmarkResult>& final_results) {
    MeasurementStorage* storage = storage_for_mode(mode);
    BandwidthMeasurer* bandwidth_measurer = bandwidth_measurer_for_mode(mode);

    std::stringstream bw_buffer;
    std::stringstream lat_buffer;
    std::vector<uint64_t> sampler_cycles;
    const auto& selection = bandwidth_measurer->get_counter_selection();

    for (const auto& result : final_results) {
        bw_buffer << format_bandwidth_output(result, selection);
        lat_buffer << format_latency_output(result, ptrchase_manager_->pid());
        sampler_cycles.insert(sampler_cycles.end(),
                              result.sampler_raw_latencies.begin(),
                              result.sampler_raw_latencies.end());
    }

    const std::string bw_file = storage->bw_file_path(ratio_pct, pause);
    storage->append_to_file_with_lock(bw_file, bw_buffer.str());

    const std::string lat_file = storage->lat_file_path(ratio_pct, pause);
    if (!storage->append_to_file_with_lock(lat_file, lat_buffer.str())) {
        std::cerr << "  Failed to write latency data: " << lat_file << std::endl;
    }

    if (!sampler_cycles.empty()) {
        storage->append_sampler_samples(ratio_pct, pause, sampler_cycles);
    }

}

void BenchmarkExecutor::remove_results_for_point(double ratio_pct,
                                                 int pause,
                                                 ExecutionMode mode) {
    results_.erase(
        std::remove_if(results_.begin(), results_.end(),
                       [&](const BenchmarkResult& result) {
                           return std::abs(result.ratio_pct - ratio_pct) < 1e-9 &&
                                  result.pause == pause &&
                                  result.mode == mode;
                       }),
        results_.end());
}

void BenchmarkExecutor::account_measured_point_progress(
    const std::vector<BenchmarkResult>& final_results) {
    for (const auto& result : final_results) {
        progress_tracker_->record_iteration_time(result.iteration_time_s);
        progress_tracker_->increment_completed_iterations();
    }
}

ProbeStats BenchmarkExecutor::summarize_probe_stats(
    int pause,
    const std::vector<BenchmarkResult>& final_results) const {
    ProbeStats point_stats;
    point_stats.pause = pause;
    point_stats.n_reps = static_cast<int>(final_results.size());
    if (final_results.empty()) {
        return point_stats;
    }

    double sum_bw = 0.0;
    double sum_lat = 0.0;
    for (const auto& result : final_results) {
        sum_bw += result.bandwidth_mbps / 1000.0;
        sum_lat += result.latency_ns;
    }

    point_stats.bw = sum_bw / final_results.size();
    point_stats.lat = sum_lat / final_results.size();

    double bw_sq_sum = 0.0;
    double lat_sq_sum = 0.0;
    for (const auto& result : final_results) {
        double bw_gbps = result.bandwidth_mbps / 1000.0;
        bw_sq_sum += (bw_gbps - point_stats.bw) * (bw_gbps - point_stats.bw);
        lat_sq_sum += (result.latency_ns - point_stats.lat) * (result.latency_ns - point_stats.lat);
    }

    point_stats.sigma_bw = std::sqrt(bw_sq_sum / final_results.size());
    point_stats.sigma_lat = std::sqrt(lat_sq_sum / final_results.size());
    return point_stats;
}

bool BenchmarkExecutor::resolve_point_measurements(
    double ratio_pct,
    int pause,
    ExecutionMode mode,
    int& total_global_measurements,
    std::vector<BenchmarkResult>& final_results,
    double bw_peak,
    bool skip_outliers_for_low_bw,
    bool count_progress,
    bool replace_existing) {
    OutlierDetector* outlier_detector = outlier_detector_for_mode(mode);
    const std::string mode_str = mode_label(mode);

    auto refresh_progress_line = [&]() {
        int remaining = total_global_measurements - progress_tracker_->completed_iterations();
        std::string progress_line = progress_tracker_->format_progress(
            progress_tracker_->completed_iterations(), total_global_measurements, remaining);
        std::cout << "\r" << progress_line << std::flush;
    };

    auto print_progress_checkpoint = [&]() {
        refresh_progress_line();
        if (config_.verbosity >= 2) {
            std::cout << std::endl << std::endl;
        }
    };

    auto print_point_measurements = [&](const std::vector<BenchmarkResult>& point_results,
                                        const std::string& header) {
        if (config_.verbosity < 2) {
            return;
        }

        std::cout << header << std::endl;
        std::cout << "    BW measurements: ";
        for (size_t i = 0; i < point_results.size(); ++i) {
            std::cout << std::fixed << std::setprecision(2)
                      << point_results[i].bandwidth_mbps / 1000.0 << " GB/s";
            if (i + 1 < point_results.size()) {
                std::cout << ", ";
            }
        }
        std::cout << std::endl << "    Latency measurements: ";
        for (size_t i = 0; i < point_results.size(); ++i) {
            std::cout << std::fixed << std::setprecision(1)
                      << point_results[i].latency_ns << " ns";
            if (i + 1 < point_results.size()) {
                std::cout << ", ";
            }
        }
        std::cout << std::endl;
    };

    int point_restart_count = 0;
    const int max_point_restarts = 3;
    bool needs_restart = true;
    while (needs_restart) {
        needs_restart = false;
        cleanup_measurement_files(ratio_pct, pause);

        final_results.clear();
        final_results.reserve(static_cast<size_t>(config_.point_reps));
        int successful_measurements = 0;

        auto measure_rep = [&](int rep, BenchmarkResult& result) -> bool {
            int consecutive_failures = 0;

            while (true) {
                result = BenchmarkResult{};
                result.ratio_pct = ratio_pct;
                result.pause = pause;
                result.repetition = rep;
                result.iteration_time_s = 0.0;

                progress_tracker_->start_iteration();

                if (config_.verbosity >= 2) {
                    std::cout << "\r" << std::string(140, ' ') << "\r";
                    std::cout << " Issued Load Ratio: " << std::fixed << std::setprecision(0)
                              << ratio_pct << "\n Pause: " << pause
                              << "\n Execution Mode: " << mode_str << std::endl;
                }

                BenchmarkStatus status =
                    run_single_benchmark(ratio_pct, pause, rep, result, mode);
                if (status == BenchmarkStatus::SUCCESS) {
                    double iteration_time = progress_tracker_->finish_iteration();
                    result.iteration_time_s = iteration_time;
                    if (count_progress) {
                        progress_tracker_->record_iteration_time(iteration_time);
                        progress_tracker_->increment_completed_iterations();
                        successful_measurements++;
                    }
                    if (count_progress && config_.verbosity < 2) {
                        refresh_progress_line();
                    }
                    return true;
                }

                progress_tracker_->finish_iteration();

                if (status == BenchmarkStatus::RETRY_BW_GROWTH) {
                    if (config_.verbosity >= 2) {
                        std::cout << "    [Retry] Re-measuring same point due to BW growth...\n";
                    }
                    continue;
                }

                consecutive_failures++;
                if (consecutive_failures >= 5) {
                    std::cerr << "\nERROR: Too many consecutive failures ("
                              << consecutive_failures
                              << "). Restarting point." << std::endl;
                    return false;
                }

                if (config_.verbosity >= 2) {
                    std::cout << "    Retrying measurement (failure "
                              << consecutive_failures << "/5)...\n";
                }

                if (traffic_gen_manager_) {
                    traffic_gen_manager_->kill_all_traffic_gen();
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }
        };

        for (int rep = 0; rep < config_.point_reps; ++rep) {
            BenchmarkResult result;
            if (!measure_rep(rep, result)) {
                needs_restart = true;
                break;
            }
            final_results.push_back(result);
        }

        if (needs_restart || final_results.size() != static_cast<size_t>(config_.point_reps)) {
            if (count_progress) {
                progress_tracker_->reset_completed_iterations(
                    progress_tracker_->completed_iterations() - successful_measurements);
                print_progress_checkpoint();
            }
            point_restart_count++;
            if (point_restart_count >= max_point_restarts) {
                std::cerr << "\nERROR: Failed to complete point after "
                          << max_point_restarts << " restarts." << std::endl;
                return false;
            }
            continue;
        }

        double max_bw_now = 0.0;
        for (const auto& result : final_results) {
            max_bw_now = std::max(max_bw_now, result.bandwidth_mbps / 1000.0);
        }

        bool should_report_outliers = config_.point_reps > 1;
        bool skip_outliers = should_report_outliers &&
                             skip_outliers_for_low_bw &&
                             bw_peak > 1e-12 &&
                             max_bw_now < 0.02 * bw_peak;
        if (config_.verbosity >= 2 && should_report_outliers) {
            std::cout << "\r" << std::string(140, ' ') << "\r" << std::flush;
            print_point_measurements(
                final_results,
                "  Outlier Detection for " + std::to_string(static_cast<int>(ratio_pct)) +
                    "% | " + std::to_string(pause));
        }

        if (skip_outliers) {
            if (config_.verbosity >= 2 && should_report_outliers) {
                std::cout << "    [Adaptive] Skipping outlier detection (max BW="
                          << std::fixed << std::setprecision(2) << max_bw_now
                          << " GB/s is <2% of peak " << bw_peak << " GB/s)" << std::endl;
            }
        } else if (should_report_outliers) {
            std::vector<int> outliers = outlier_detector->find_outliers(final_results, 10.0);
            int retry_count = 0;
            const int max_retries = 3;

            while (!outliers.empty() && retry_count < max_retries) {
                retry_count++;

                if (config_.verbosity >= 2) {
                    std::cout << "    Outliers in reps: ";
                    for (int rep : outliers) {
                        std::cout << (rep + 1) << " ";
                    }
                    std::cout << "-> retry " << retry_count << "/" << max_retries
                              << std::endl;
                }

                bool rerun_failed = false;
                for (int rep : outliers) {
                    BenchmarkResult rerun_result;
                    if (!measure_rep(rep, rerun_result)) {
                        rerun_failed = true;
                        break;
                    }
                    final_results[static_cast<size_t>(rep)] = std::move(rerun_result);
                }

                if (rerun_failed) {
                    needs_restart = true;
                    break;
                }

                if (config_.verbosity >= 2) {
                    print_point_measurements(
                        final_results,
                        "    After retry " + std::to_string(retry_count) + ":");
                }

                outliers = outlier_detector->find_outliers(final_results, 10.0);
            }

            if (needs_restart || !outliers.empty()) {
                if (!needs_restart && config_.verbosity >= 2) {
                    std::cout << "    Failed to eliminate all outliers after "
                              << max_retries << " retries. Restarting point."
                              << std::endl;
                }
                needs_restart = true;
                if (count_progress) {
                    progress_tracker_->reset_completed_iterations(
                        progress_tracker_->completed_iterations() - successful_measurements);
                    print_progress_checkpoint();
                }
                point_restart_count++;
                if (point_restart_count >= max_point_restarts) {
                    std::cerr << "\nERROR: Failed to stabilize point after "
                              << max_point_restarts << " restarts." << std::endl;
                    return false;
                }
                continue;
            }
        }

        if (replace_existing) {
            remove_results_for_point(ratio_pct, pause, mode);
        }
        flush_point_results(ratio_pct, pause, mode, final_results);
        results_.insert(results_.end(), final_results.begin(), final_results.end());

        if (!frequency_written_to_plotter_) {
            write_frequency_to_plotter(max_cpu_freq_ghz_);
            frequency_written_to_plotter_ = true;
        }

        ProbeStats summary = summarize_probe_stats(pause, final_results);
        progress_tracker_->update_ratio_stats(ratio_pct, summary.bw * 1000.0, summary.lat);
        if (count_progress) {
            print_progress_checkpoint();
        }
    }

    if (count_progress && config_.verbosity < 2) {
        int remaining = total_global_measurements - progress_tracker_->completed_iterations();
        std::string progress_line = progress_tracker_->format_progress(
            progress_tracker_->completed_iterations(), total_global_measurements, remaining);
        std::cout << "\r" << progress_line << std::flush;
    }
    return true;
}

BenchmarkExecutor::AdaptiveMeasuredPoint BenchmarkExecutor::adaptive_probe(
    double ratio_pct,
    int pause,
    ExecutionMode mode,
    int& total_global_measurements,
    double bw_peak,
    bool count_progress,
    bool replace_existing) {
    std::vector<BenchmarkResult> final_results;
    if (!resolve_point_measurements(ratio_pct, pause, mode, total_global_measurements,
                                    final_results, bw_peak, true,
                                    count_progress, replace_existing)) {
        throw std::runtime_error("Failed to resolve adaptive point measurements");
    }
    return {summarize_probe_stats(pause, final_results), std::move(final_results)};
}

bool BenchmarkExecutor::run_adaptive_for_ratio(double ratio_pct,
                                                const std::vector<ExecutionMode>& modes,
                                                int& total_global_measurements) {
    if (modes.empty()) {
        return true;
    }

    AdaptiveTier tier = CurveTracer::resolve_tier(config_.adaptive_tier, config_.adaptive_point_count);

    struct AdaptiveModeState {
        ExecutionMode mode;
        std::string label;
        double bw_peak = 0.0;
        std::unique_ptr<CurveTracer> tracer;
        bool completed = false;
    };

    std::vector<AdaptiveModeState> states(modes.size());
    for (size_t i = 0; i < modes.size(); ++i) {
        states[i].mode = modes[i];
        states[i].label = "MULTISEQ";
    }

    auto display_progress = [&]() {
        int remaining = total_global_measurements - progress_tracker_->completed_iterations();
        std::string line = progress_tracker_->format_progress(
            progress_tracker_->completed_iterations(), total_global_measurements, remaining);
        std::cout << "\r" << line << std::flush;
    };

    auto update_progress_estimate = [&]() {
        int active_runs = 0;
        int active_points = 0;
        int active_segments = 0;
        for (const auto& state : states) {
            if (state.completed || !state.tracer) {
                continue;
            }
            active_runs++;
            active_points += state.tracer->point_count();
            active_segments += state.tracer->segments_above_threshold();
        }

        total_global_measurements = progress_tracker_->estimate_with_active_runs(
            active_runs, active_points, active_segments);
        if (config_.verbosity < 2) {
            display_progress();
        }
    };

    auto measure_point_for_state = [&](AdaptiveModeState& state,
                                       int pause,
                                       bool count_progress,
                                       bool replace_existing) {
        AdaptiveMeasuredPoint measured = adaptive_probe(
            ratio_pct, pause, state.mode, total_global_measurements, state.bw_peak,
            count_progress, replace_existing);
        state.bw_peak = std::max(state.bw_peak, measured.stats.bw);
        return measured;
    };

    try {
        for (auto& state : states) {
            auto* state_ptr = &state;
            auto measure_fn = [&, state_ptr](int pause) -> ProbeStats {
                return measure_point_for_state(*state_ptr, pause, true, false).stats;
            };
            state_ptr->tracer = std::make_unique<CurveTracer>(tier, measure_fn, config_.verbosity);
        }

        if (config_.verbosity >= 2) {
            std::cout << "\r" << std::string(140, ' ') << "\r";
            std::cout << " Adaptive Setup:" << std::endl;
            std::cout << "   Modes:";
            for (size_t i = 0; i < states.size(); ++i) {
                std::cout << (i == 0 ? " " : ", ") << states[i].label;
            }
            std::cout << std::endl;
            std::cout << "   Budget: " << tier.max_budget << " points/mode";
            if (!states.empty()) {
                std::cout << " (" << (tier.max_budget * static_cast<int>(states.size()))
                          << " total)";
            }
            std::cout << std::endl << std::endl;
            display_progress();
        }

        auto measure_shared_pause = [&](int pause) {
            for (auto& state : states) {
                AdaptiveMeasuredPoint measured =
                    measure_point_for_state(state, pause, true, false);
                state.tracer->seed_point(measured.stats, true);
            }
            update_progress_estimate();
        };

        auto validate_suspicious_point = [&](AdaptiveModeState& state, int center_pause) {
            auto suspicious = state.tracer->suspicious_triplet_around(center_pause);
            if (!suspicious.has_value()) {
                return;
            }

            const int left_pause = suspicious->left_pause;
            const int right_pause = suspicious->right_pause;

            if (config_.verbosity >= 2) {
                std::cout << "\r" << std::string(140, ' ') << "\r";
                std::cout << " Adaptive Validating suspicious " << state.label
                          << " pause " << center_pause << " between ["
                          << left_pause << ", " << right_pause << "]" << std::endl;
            }

            AdaptiveMeasuredPoint repeated =
                measure_point_for_state(state, center_pause, false, true);
            state.tracer->replace_point(repeated.stats);

            std::optional<AdaptiveMeasuredPoint> left_guard;
            std::optional<AdaptiveMeasuredPoint> right_guard;
            bool left_guard_new = false;
            bool right_guard_new = false;

            const int guard_left_pause = CurveTracer::split_pause(left_pause, center_pause);
            if (guard_left_pause > 0 &&
                state.tracer->measured_points().find(guard_left_pause) ==
                    state.tracer->measured_points().end()) {
                left_guard = measure_point_for_state(state, guard_left_pause, false, false);
                state.tracer->seed_point(left_guard->stats, false);
                left_guard_new = true;
            }

            const int guard_right_pause = CurveTracer::split_pause(center_pause, right_pause);
            if (guard_right_pause > 0 &&
                state.tracer->measured_points().find(guard_right_pause) ==
                    state.tracer->measured_points().end()) {
                right_guard = measure_point_for_state(state, guard_right_pause, false, false);
                state.tracer->seed_point(right_guard->stats, false);
                right_guard_new = true;
            }

            state.tracer->prepare_seeded_refinement();

            const SuspiciousTriplet outer =
                state.tracer->triplet_metrics(left_pause, center_pause, right_pause);
            const bool outer_is_suspicious =
                outer.is_suspicious(state.tracer->suspicious_threshold(outer));

            bool left_supported = true;
            if (guard_left_pause > 0 &&
                state.tracer->measured_points().find(guard_left_pause) !=
                    state.tracer->measured_points().end()) {
                const SuspiciousTriplet left_triplet =
                    state.tracer->triplet_metrics(left_pause, guard_left_pause, center_pause);
                left_supported =
                    !left_triplet.is_suspicious(state.tracer->suspicious_threshold(left_triplet));
            }

            bool right_supported = true;
            if (guard_right_pause > 0 &&
                state.tracer->measured_points().find(guard_right_pause) !=
                    state.tracer->measured_points().end()) {
                const SuspiciousTriplet right_triplet =
                    state.tracer->triplet_metrics(center_pause, guard_right_pause, right_pause);
                right_supported = !right_triplet.is_suspicious(
                    state.tracer->suspicious_threshold(right_triplet));
            }

            const bool validated_feature =
                outer_is_suspicious && left_supported && right_supported;

            if (validated_feature) {
                if (left_guard_new && left_guard.has_value()) {
                    state.tracer->promote_budget_point(guard_left_pause);
                    account_measured_point_progress(left_guard->results);
                }
                if (right_guard_new && right_guard.has_value()) {
                    state.tracer->promote_budget_point(guard_right_pause);
                    account_measured_point_progress(right_guard->results);
                }
            }

            if (config_.verbosity >= 2) {
                std::cout << "\r" << std::string(140, ' ') << "\r";
                std::cout << " Adaptive Validation " << state.label << " pause "
                          << center_pause << ": "
                          << (validated_feature ? "feature" : "corrected")
                          << std::endl;
            }
        };

        auto validate_existing_suspicious_points = [&](AdaptiveModeState& state) {
            bool changed = false;
            do {
                changed = false;
                std::vector<int> pauses = state.tracer->current_points();
                for (size_t i = 1; i + 1 < pauses.size(); ++i) {
                    if (state.tracer->suspicious_triplet_around(pauses[i]).has_value()) {
                        validate_suspicious_point(state, pauses[i]);
                        changed = true;
                        break;
                    }
                }
            } while (changed);
        };

        measure_shared_pause(0);
        measure_shared_pause(1);

        const int step_factor = 10;
        const long long max_pause = 1000000000LL;
        std::vector<int> horizons(states.size(), 1);
        std::vector<bool> horizon_fixed(states.size(), false);
        long long p = 10;
        long long last_discovery_pause = 1;

        while (p <= max_pause) {
            measure_shared_pause(static_cast<int>(p));
            last_discovery_pause = p;

            bool all_fixed = true;
            for (size_t i = 0; i < states.size(); ++i) {
                if (horizon_fixed[i]) {
                    continue;
                }

                const auto& measured_points = states[i].tracer->measured_points();
                auto it = measured_points.find(static_cast<int>(p));
                if (it == measured_points.end()) {
                    all_fixed = false;
                    continue;
                }

                const ProbeStats& sample = it->second;
                if (sample.sigma_bw / std::max(sample.bw, 1e-12) > 0.3) {
                    horizons[i] = std::max(1, static_cast<int>(p / step_factor));
                    horizon_fixed[i] = true;
                } else if (sample.bw < 0.01 * states[i].bw_peak) {
                    horizons[i] = static_cast<int>(p);
                    horizon_fixed[i] = true;
                } else {
                    all_fixed = false;
                }
            }

            if (all_fixed) {
                break;
            }

            p *= step_factor;
        }

        for (size_t i = 0; i < states.size(); ++i) {
            if (!horizon_fixed[i]) {
                horizons[i] = static_cast<int>(std::min(last_discovery_pause, max_pause));
            }
        }

        if (tier.tolerance < 0.05) {
            long long extended = static_cast<long long>(*std::max_element(horizons.begin(), horizons.end())) * 10;
            if (extended <= max_pause) {
                measure_shared_pause(static_cast<int>(extended));
            }
        }

        std::vector<int> skeleton_candidates =
            CurveTracer::compute_skeleton_candidates(states.front().tracer->current_points());
        for (int skeleton_pause : skeleton_candidates) {
            measure_shared_pause(skeleton_pause);
        }

        for (auto& state : states) {
            validate_existing_suspicious_points(state);
        }

        for (auto& state : states) {
            state.tracer->prepare_seeded_refinement();
        }
        update_progress_estimate();

        int completed_modes = 0;
        while (completed_modes < static_cast<int>(states.size())) {
            int best_idx = -1;
            double best_error = -1.0;

            for (size_t i = 0; i < states.size(); ++i) {
                auto& state = states[i];
                if (state.completed) {
                    continue;
                }

                if (!state.tracer->has_pending_refinement()) {
                    state.completed = true;
                    completed_modes++;
                    total_global_measurements = progress_tracker_->record_adaptive_run(
                        state.tracer->point_count());
                    update_progress_estimate();
                    continue;
                }

                double error = state.tracer->current_worst_error();
                if (error > best_error) {
                    best_error = error;
                    best_idx = static_cast<int>(i);
                }
            }

            if (best_idx < 0) {
                break;
            }

            if (config_.verbosity >= 3) {
                std::cout << "\r" << std::string(140, ' ') << "\r";
                std::cout << " Adaptive Refining " << states[best_idx].label
                          << " (worst E=" << std::scientific << std::setprecision(3)
                          << best_error << std::fixed << ")" << std::endl;
            }

            auto& state = states[static_cast<size_t>(best_idx)];
            int inserted_pause = state.tracer->next_refinement_pause();
            if (inserted_pause < 0) {
                state.completed = true;
                completed_modes++;
                total_global_measurements = progress_tracker_->record_adaptive_run(
                    state.tracer->point_count());
            } else {
                AdaptiveMeasuredPoint measured =
                    measure_point_for_state(state, inserted_pause, true, false);
                state.tracer->seed_point(measured.stats, true);
                validate_suspicious_point(state, inserted_pause);
                state.tracer->prepare_seeded_refinement();
            }
            update_progress_estimate();
        }

        for (auto& state : states) {
            if (!state.completed) {
                state.completed = true;
                total_global_measurements = progress_tracker_->record_adaptive_run(
                    state.tracer->point_count());
            }

            if (config_.verbosity >= 2) {
                std::vector<int> discovered = state.tracer->current_points();
                std::cout << "\r" << std::string(140, ' ') << "\r";
                std::cout << " Adaptive " << state.label << " pauses (" << discovered.size()
                          << "): ";
                for (size_t i = 0; i < discovered.size(); ++i) {
                    if (i > 0) {
                        std::cout << ", ";
                    }
                    std::cout << discovered[i];
                }
                std::cout << std::endl;
            }
        }
        return true;
    } catch (const std::exception& ex) {
        std::cerr << "ERROR: Adaptive ratio execution failed: " << ex.what() << std::endl;
        return false;
    }
}

bool BenchmarkExecutor::run() {

    results_.clear();

    std::vector<double> ratios = config_.ratios_pct;
    if (ratios.empty()) {
        for (int i = 100; i >= 0; i -= 2) {
            ratios.push_back(static_cast<double>(i));
        }
    }

    std::vector<ExecutionMode> modes;
    bool do_multiseq = config_.generate_multiseq;
    if (do_multiseq) modes.push_back(ExecutionMode::MULTISEQUENTIAL);


    if (!ptrchase_manager_->ensure_running()) {
        std::cerr << "ERROR: Failed to launch persistent pointer chase process" << std::endl;
        return false;
    }

    int total_ratios = static_cast<int>(ratios.size());
    int current_ratio = 0;

    if (config_.pauses.empty()) {
        AdaptiveTier tier = CurveTracer::resolve_tier(config_.adaptive_tier, config_.adaptive_point_count);
        int num_modes = static_cast<int>(modes.size());
        int estimated_total = total_ratios * tier.max_budget * config_.point_reps * num_modes;
        int total_global_measurements = estimated_total;
        progress_tracker_->start_benchmark(total_global_measurements);
        progress_tracker_->set_adaptive_info(
            total_ratios * num_modes,
            config_.point_reps,
            tier.max_budget,
            num_modes);

        for (double ratio_pct : ratios) {
            current_ratio++;
            progress_tracker_->set_current_ratio(ratio_pct, current_ratio, total_ratios);
            if (config_.verbosity >= 2) {
                std::cout << "\r" << std::string(140, ' ') << "\r";
                std::cout << "\n\n=== " << ratio_pct << "% loads ("
                          << current_ratio << "/" << total_ratios << " load ratios) ===" << std::endl;
            }

            if (!run_adaptive_for_ratio(ratio_pct, modes, total_global_measurements)) {
                return false;
            }
        }
    } else {
        std::vector<int> pauses = config_.pauses;

        int total_global_measurements = ratios.size() * pauses.size() * config_.point_reps * modes.size();
        progress_tracker_->start_benchmark(total_global_measurements);

        for (double ratio_pct : ratios) {
            current_ratio++;
            progress_tracker_->set_current_ratio(ratio_pct, current_ratio, total_ratios);
            if (config_.verbosity >= 2) {
                std::cout << "\n\n=== " << ratio_pct << "% loads (" 
                          << current_ratio << "/" << total_ratios << " load ratios) ===" << std::endl;
            }

            std::map<ExecutionMode, std::vector<double>> bw_history;
            std::set<int> backtracked_pauses;

            for (int pause_idx = 0; pause_idx < static_cast<int>(pauses.size()); ++pause_idx) {
                int pause = pauses[pause_idx];
                    bool backtrack_triggered = false;

                    for (ExecutionMode mode : modes) {
                        std::vector<BenchmarkResult> current_point_results;
                        if (!resolve_point_measurements(ratio_pct, pause, mode,
                                                        total_global_measurements,
                                                        current_point_results)) {
                            return false;
                        }

                        if (pause_idx > 0 && pauses[pause_idx] > pauses[pause_idx - 1]) {
                            double total_bw = 0;
                            for (const auto& res : current_point_results) {
                                total_bw += res.bandwidth_mbps;
                            }
                            double avg_bw = total_bw / current_point_results.size();

                            if (bw_history[mode].size() <= static_cast<size_t>(pause_idx)) {
                                bw_history[mode].resize(pause_idx + 1, 0.0);
                            }
                            bw_history[mode][pause_idx] = avg_bw;

                            double prev_bw = bw_history[mode][pause_idx - 1];
                            
                            if (prev_bw > 0.1 && avg_bw > prev_bw * 1.5) {
                                if (backtracked_pauses.find(pause) == backtracked_pauses.end()) {
                                    if (config_.verbosity >= 1) {
                                        std::cout << "    ⚠ Anomaly detected: Higher pause (" << pause 
                                                << ") has significantly higher BW (" << std::fixed << std::setprecision(2) << avg_bw/1000.0 
                                                << " GB/s) than previous pause (" << pauses[pause_idx-1] 
                                                << ", " << prev_bw/1000.0 << " GB/s). (+50% threshold exceeded)\n";
                                        std::cout << "    ↺ Backtracking to repeat pause " << pauses[pause_idx-1] << "...\n";
                                    }
                                    
                                    backtracked_pauses.insert(pause);
                                    pause_idx -= 2;
                                    backtrack_triggered = true;
                                    break; 
                                } else {
                                    if (config_.verbosity >= 1) {
                                        std::cout << "    ⚠ Anomaly persisted after retry: Higher pause (" << pause 
                                                << ") still has higher BW. Ignoring and continuing...\n";
                                    }
                                }
                            }
                        } else {
                            double total_bw = 0;
                            for (const auto& res : current_point_results) {
                                total_bw += res.bandwidth_mbps;
                            }
                            double avg_bw = total_bw / current_point_results.size();
                            
                            if (bw_history[mode].size() <= static_cast<size_t>(pause_idx)) {
                                bw_history[mode].resize(pause_idx + 1, 0.0);
                            }
                            bw_history[mode][pause_idx] = avg_bw;
                        }

                        if (backtrack_triggered) break;
                    }
                }
            }
        }

    if (config_.verbosity >= 2) {
        std::cout << "\n\n=== Final Cleanup ===" << std::endl;
        std::cout << "Terminating processes..." << std::endl;
    }
        
    if (ptrchase_manager_) {
        ptrchase_manager_->cleanup();
    }
    
    if (traffic_gen_manager_) {
        traffic_gen_manager_->kill_all_traffic_gen();
    }
    
    if (config_.verbosity >= 2) {
        std::cout << "Cleanup complete" << std::endl;
    }

    return true;
}

std::vector<int> BenchmarkExecutor::get_memory_binding_nodes() const {
    return config_.memory_bind_nodes;
}



/**
 * @brief Executes a single iteration of the benchmark suite for a specific traffic mix and mode.
 * 
 * 
 * @param ratio_pct The current mix of reads/writes being tested (0.0 to 100.0).
 * @param pause The configured pause or delay between memory operations.
 * @param rep The current repetition number.
 * @param result An output struct where the collected metrics are appended.
 * @param mode The execution mode.
 * @return BenchmarkStatus Indicates whether the run succeeded or requires a retry due to outliers.
 */
BenchmarkExecutor::BenchmarkStatus BenchmarkExecutor::run_single_benchmark(double ratio_pct, int pause, int rep, BenchmarkResult& result, ExecutionMode mode) {
    MeasurementStorage* measurement_storage_ = measurement_storage_multiseq_.get();
    BandwidthMeasurer* bandwidth_measurer_ = bandwidth_measurer_multiseq_.get();
    LatencyMeasurer* latency_measurer_ = latency_measurer_multiseq_.get();

    result.repetitions = config_.point_reps;
    result.mode = mode;

    measurement_storage_->ensure_directories_exist();

    if (config_.instruction_sampling && instruction_sampler_) {
        instruction_sampler_->cleanup();
    }
    
    if (ratio_pct < 0 || ratio_pct > 100) {
        std::cerr << "ERROR: Invalid ratio_pct value: " << ratio_pct << std::endl;
        return BenchmarkStatus::FAILURE;
    }
    
    if (pause < 0) {
        std::cerr << "ERROR: Invalid pause value: " << pause << std::endl;
        return BenchmarkStatus::FAILURE;
    }
    
    std::string traffic_gen_log_file = measurement_storage_->traffic_gen_log_file_path(ratio_pct, pause);
    
    int traffic_gen_cores = config_.instruction_sampling
        ? sys_info_.sockets[0].core_count
        : get_default_traffic_gen_cores(sys_info_);
    static bool warned_cross_socket_total_cores = false;
    if (config_.traffic_gen_cores > sys_info_.sockets[0].core_count && !warned_cross_socket_total_cores) {
        std::cerr << "WARNING: --total-cores is larger than one socket. Mess profiles single-socket curves; spreading traffic across sockets affects the measurements and is not the expected way of running." << std::endl;
        warned_cross_socket_total_cores = true;
    }
    if (config_.traffic_gen_cores > 0 && config_.traffic_gen_cores <= sys_info_.sockets[0].core_count) {
        traffic_gen_cores = config_.traffic_gen_cores;
    }
    
    if (traffic_gen_cores <= 0 || traffic_gen_cores > 1024) {
        std::cerr << "ERROR: Invalid traffic_gen_cores value: " << traffic_gen_cores << std::endl;
        return BenchmarkStatus::FAILURE;
    }

    std::vector<int> traffic_gen_mem_nodes = get_memory_binding_nodes();

    TrafficGenPreparation traffic_gen_prep;
    if (!traffic_gen_manager_->prepare_traffic_gen(ratio_pct,
                                         pause,
                                         traffic_gen_cores,
                                         traffic_gen_mem_nodes,
                                         traffic_gen_log_file,
                                         traffic_gen_prep,
                                         mode)) {
        return BenchmarkStatus::FAILURE;
    }

    bool reuse_existing_traffic_gen = traffic_gen_prep.reused_existing;
    long long prev_cas_rd = traffic_gen_prep.prev_cas_rd;
    long long prev_cas_wr = traffic_gen_prep.prev_cas_wr;
    double prev_elapsed = traffic_gen_prep.prev_elapsed;
    double prev_bw_gb_s = traffic_gen_prep.prev_bw_gb_s;
    int traffic_gen_pid = traffic_gen_prep.pid;

    if (!reuse_existing_traffic_gen) {
        if (!traffic_gen_manager_->wait_for_traffic_gen_ready(120)) {
            if (!traffic_gen_manager_->is_traffic_gen_running(traffic_gen_pid)) {
                std::cerr << "    ERROR: TrafficGen failed during initialization" << std::endl;
                return BenchmarkStatus::FAILURE;
            }
            if (config_.verbosity >= 1) {
                std::cerr << "    Warning: TrafficGen ready wait failed, proceeding anyway" << std::endl;
            }
        }
    }

    discover_bandwidth_counters_if_needed();
    BandwidthCounterSelection bw_counters = cached_bw_counters_;
    for (const auto& counter : config_.add_counters) {
        if (std::find(bw_counters.extra_counters.begin(), bw_counters.extra_counters.end(), counter) ==
            bw_counters.extra_counters.end()) {
            bw_counters.extra_counters.push_back(counter);
        }
    }
    bandwidth_measurer_->set_counter_selection(bw_counters);

    const bool verbose1 = config_.verbosity >= 1;
    const bool verbose2 = config_.verbosity >= 2;
    const bool verbose3 = config_.verbosity >= 3;


    if (verbose2) {
        std::cout << "    Current measurement is " << (rep + 1) << "/" << config_.point_reps << '\n';
    }
    
    if (!config_.instruction_sampling) {
        if (!ptrchase_manager_->ensure_running()) {
            std::cerr << "ERROR: Cannot ensure ptr_chase is running" << std::endl;
            return BenchmarkStatus::FAILURE;
        }
                
        if (verbose3) {
            std::cout << "      Still active with pid: " << ptrchase_manager_->pid() << '\n';
        }
    }

    
    if (traffic_gen_pid > 0) {
        int traffic_gen_samples = 0;
        
        if (verbose2) {
            std::cout << "    Waiting for TrafficGen to stabilize...\n";
        }

        long long bw_cas_rd = 0, bw_cas_wr = 0;
        double bw_elapsed = 0;
        bool traffic_gen_stabilized = false;
        const double sampling_interval_ms = bandwidth_measurer_->sampling_interval_ms();
        
        if (reuse_existing_traffic_gen && prev_cas_rd > 0 && prev_cas_wr > 0 && prev_elapsed > 0.0) {
            bw_cas_rd = prev_cas_rd;
            bw_cas_wr = prev_cas_wr;
            bw_elapsed = prev_elapsed;
            traffic_gen_stabilized = true;
            
            if (verbose2) {
                std::cout << "    Using previous BW measurement: " << std::fixed << std::setprecision(2) 
                          << prev_bw_gb_s << " GB/s (TrafficGen already stable, just re-measuring latency)\n";
            }
        }
        

        
        // If BW didn't stabilize (60s timeout), abort this measurement and retry
        double stabilization_timeout_samples = (60.0 * 1000.0) / sampling_interval_ms;
        if (!traffic_gen_stabilized && traffic_gen_samples > stabilization_timeout_samples) {
            if (verbose1) {
                std::cout << "    ⚠ Aborting measurement due to BW instability (Time limit)\n";
            }
            return BenchmarkStatus::FAILURE;
        }

        std::deque<PerfBurstCounters> latency_samples;
        
        auto print_burst_stats = [&](const PerfBurstCounters& b, int current_count) {
            if (verbose3) {
                double accesses_per_burst = latency_measurer_->get_accesses_per_burst();
                double latency_ns = calculate_latency_from_perf_values(
                    b.cycles, accesses_per_burst, b.tlb1miss, b.tlb2miss,
                    b.duration_s, b.using_hugepages);
                
                std::cout << "      [Latency Burst " << current_count << "] "
                          << "cycles: " << static_cast<long long>(b.cycles) << " | "
                          << "instr: " << static_cast<long long>(b.instructions) << " | "
                          << "tlb1: " << static_cast<long long>(b.tlb1miss) << " | "
                          << "tlb2: " << static_cast<long long>(b.tlb2miss) << " | "
                          << "dur: " << std::fixed << std::setprecision(3) << b.duration_s << " s | "
                          << "lat: " << std::fixed << std::setprecision(1) << latency_ns << " ns"
                          << '\n';
            }
        };

        bool sampler_started = false;
        int sampler_callback_count = 0;
        auto on_sample_callback = [&]() {
            if (config_.instruction_sampling && instruction_sampler_) {
                ++sampler_callback_count;
                if (!sampler_started && sampler_callback_count >= 3) {
                    std::vector<int> cores;
                    if (config_.traffic_gen_explicit_cores.empty()) {
                        const int reserved_cpu = config_.instruction_sampling ? -1 : SystemToolsCache::instance().ptr_chase_cpu;
                        cores = select_worker_cpus(traffic_gen_cores, reserved_cpu);
                    } else {
                        for (const auto& c : config_.traffic_gen_explicit_cores) {
                            cores.push_back(std::stoi(c));
                        }
                    }
                    std::vector<pid_t> pids = traffic_gen_manager_->traffic_gen_worker_pids();
                    if (instruction_sampler_->start_async(cores, pids)) {
                        sampler_started = true;
                    }
                }
            } else {
                PerfBurstCounters burst_result;
                if (latency_measurer_->is_burst_running()) {
                    if (latency_measurer_->try_collect_burst_async(burst_result)) {
                        latency_samples.push_back(burst_result);
                        if (latency_samples.size() > 10) {
                            latency_samples.pop_front();
                        }
                        
                        print_burst_stats(burst_result, latency_samples.size());
                        
                        latency_measurer_->start_burst_async();
                    }
                } else {
                    latency_measurer_->start_burst_async();
                }
            }
        };
       

        if (!traffic_gen_stabilized) {

            if (!config_.instruction_sampling) {
                latency_measurer_->init_ptrchase_scaling();
                latency_measurer_->start_burst_async();
            }

            traffic_gen_stabilized = bandwidth_measurer_->wait_for_stabilization(
                traffic_gen_samples, bw_cas_rd, bw_cas_wr, bw_elapsed,
                pause, ratio_pct, reuse_existing_traffic_gen, on_sample_callback);
        }
        
        // If BW didn't stabilize (60s timeout), abort this measurement and retry
        if (!traffic_gen_stabilized && traffic_gen_samples > stabilization_timeout_samples) {
            if (verbose1) {
                std::cout << "    ⚠ Aborting measurement due to BW instability (Time limit)\n";
            }
            if (config_.instruction_sampling && instruction_sampler_) {
                instruction_sampler_->stop_and_process(0.0);
            }
            return BenchmarkStatus::FAILURE;
        }

        if (verbose2) {
            if (traffic_gen_stabilized) {
                double wait_time = (sampling_interval_ms * traffic_gen_samples + 100.0);
                const auto& stab_selection = bandwidth_measurer_->get_counter_selection();
                const bool is_pcm_counter_mode =
                    (!stab_selection.cas.read_events.empty() &&
                     stab_selection.cas.read_events[0].find("pcm_cxl/") != std::string::npos) ||
                    (!stab_selection.cas.write_events.empty() &&
                     stab_selection.cas.write_events[0].find("pcm_cxl/") != std::string::npos);
                if (is_pcm_counter_mode && bw_elapsed > 0.0) {
                    wait_time = bw_elapsed * 1000.0;
                }
                std::cout << "    Bandwidth stabilized after " << traffic_gen_samples << " samples";
                if (verbose3 && config_.measurer != "vtune") {
                    std::string time_str = (wait_time >= 1000.0)
                        ? (std::to_string(static_cast<int>(std::round(wait_time/1000.0))) + "s")
                        : (std::to_string(static_cast<int>(std::round(wait_time))) + "ms");
                    std::cout << " ( ~" << time_str << " )";
                }
                std::cout << '\n';
            } else {
                std::cout << "    Timeout, measuring last value\n";
            }
        }

        if (!traffic_gen_stabilized && traffic_gen_samples == 0) {
            std::cerr << "    Error: Failed to collect any bandwidth samples for this point." << std::endl;
            return BenchmarkStatus::FAILURE;
        }
        
        PerfBurstCounters burst_totals{0, 0, 0, 0, 0, false, "", ""};
        double total_accesses = 0.0;
        int bursts_aggregated = 0;
        InstructionSamplerStats sampler_stats;
        
        const int min_bursts = 3;
        const int max_bursts = 10;
        
        auto are_last_3_stable = [&](const std::deque<PerfBurstCounters>& samples) -> bool {
            if (samples.size() < 3) return false;

            std::vector<double> latencies;
            double sum = 0.0;
            int count = 0;
            const double accesses_per_burst = latency_measurer_->get_accesses_per_burst();
            for (auto it = samples.rbegin(); it != samples.rend() && count < 3; ++it) {
                double lat = calculate_latency_from_perf_values(
                    it->cycles, accesses_per_burst, it->tlb1miss, it->tlb2miss,
                    it->duration_s, it->using_hugepages);
                if (lat <= 0.0) {
                    double accesses_per_s = (it->duration_s > 0)
                        ? (accesses_per_burst / it->duration_s) : 0.0;
                    lat = (accesses_per_s > 0) ? (1e9 / accesses_per_s) : 0.0;
                }
                latencies.push_back(lat);
                sum += lat;
                count++;
            }

            double mean = sum / 3.0;
            double sq_sum = 0.0;
            for (double lat : latencies) {
                sq_sum += (lat - mean) * (lat - mean);
            }
            double std_dev = std::sqrt(sq_sum / 3.0);
            double cv = (mean > 0) ? (std_dev / mean) : 0.0;

            return cv < 0.02;
        };

        int latency_retries = 0;
        const int max_latency_retries = 3;
        
        bool latency_success = false;

        if (config_.instruction_sampling) {
            double freq_ghz = cached_cpuinfo_freq_ghz_ > 0.0 ? cached_cpuinfo_freq_ghz_
                : (sys_info_.cpu_base_mhz > 0
                    ? sys_info_.cpu_base_mhz / 1000.0 : 3.0);

            sampler_stats = instruction_sampler_->stop_and_process(freq_ghz);

            if (!frequency_written_to_plotter_ && cached_cpuinfo_freq_ghz_ > 0.0) {
                write_frequency_to_plotter(freq_ghz);
                frequency_written_to_plotter_ = true;
            }

            if (sampler_stats.samples > 0) {
                latency_success = true;
                if (verbose2) {
                    std::cout << "    Sampler collected " << sampler_stats.samples << " latency samples"
                              << " | mean=" << std::fixed << std::setprecision(1) << sampler_stats.mean_ns << " ns"
                              << " | median=" << sampler_stats.median_ns << " ns"
                              << " | p99=" << sampler_stats.p99_ns << " ns" << std::endl;
                }
            } else {
                if (verbose1) {
                    std::cout << "    Warning: No latency samples collected during BW stabilization window" << std::endl;
                }
            }

        } else {
            while (latency_retries < max_latency_retries && !latency_success) {
                if (latency_retries > 0) {
                    latency_samples.clear();
                } else {
                    while (latency_samples.size() >= max_bursts) {
                        latency_samples.pop_front();
                    }
                }
                
                if (!ptrchase_manager_->ensure_running()) {
                    std::cerr << "ERROR: Cannot ensure ptr_chase is running" << std::endl;
                    latency_retries++;
                    continue;
                }

                if (latency_retries > 0) {
                     if (verbose1) {
                        std::cout << "    Retrying latency measurement (attempt " << (latency_retries + 1) << "/" << max_latency_retries << ")...\n";
                     }
                }

                if (verbose2) {
                    std::cout << "    Waiting for stable latency bursts...\n";
                }
                 
                while (latency_samples.size() < max_bursts) {
                     PerfBurstCounters burst_result;
                     if (latency_measurer_->is_burst_running()) {
                         if (latency_measurer_->try_collect_burst_async(burst_result)) {
                            latency_samples.push_back(burst_result);
                            
                            print_burst_stats(burst_result, latency_samples.size());
                            if (latency_samples.size() >= min_bursts && are_last_3_stable(latency_samples)) {
                                if (verbose2) {
                                    std::cout << "    Latency bursts stabilized\n";
                                }
                                latency_success = true;
                                break;
                            }

                             if (latency_samples.size() < max_bursts) {
                                 latency_measurer_->start_burst_async();
                             }
                         } else {
                            std::this_thread::sleep_for(std::chrono::milliseconds(10));
                         }
                     } else {
                         latency_measurer_->start_burst_async();
                     }
                }

                if (latency_success) {
                    break;
                }

                std::cerr << "    Warning: Failed to collect valid latency bursts." << std::endl;
                
                if (ptrchase_manager_) {
                    std::cerr << "    Forcing ptr_chase restart..." << std::endl;
                    ptrchase_manager_->cleanup();
                    latency_measurer_->reset();
                }
                latency_retries++;
            }
        }

        if (!latency_success) {
            std::cerr << "    Error: Failed to measure latency after " << max_latency_retries << " attempts." << std::endl;
            return BenchmarkStatus::FAILURE;
        }

        if (!config_.instruction_sampling) {
            int count = 0;
            for (auto it = latency_samples.rbegin(); it != latency_samples.rend() && count < 3; ++it) {
                burst_totals.cycles += it->cycles;
                burst_totals.instructions += it->instructions;
                burst_totals.tlb1miss += it->tlb1miss;
                burst_totals.tlb2miss += it->tlb2miss;
                burst_totals.duration_s += it->duration_s;
                
                if (count == 0) {
                    burst_totals.tlb1_event_name = it->tlb1_event_name;
                    burst_totals.tlb2_event_name = it->tlb2_event_name;
                }
                count++;
            }
            bursts_aggregated = count;
            
            double accesses_per_burst = latency_measurer_->get_accesses_per_burst();
            total_accesses = accesses_per_burst * bursts_aggregated;

            if (verbose3) {
                std::cout << "      averaged last " << bursts_aggregated << " bursts\n";
            }
        }

        
        
        bool bw_increased = false;
        
        long long final_bw_cas_rd = 0, final_bw_cas_wr = 0;
        double final_bw_elapsed = 0;

        const auto& final_selection = bandwidth_measurer_->get_counter_selection();
        const bool is_pcm_counter_mode =
            (!final_selection.cas.read_events.empty() &&
             final_selection.cas.read_events[0].find("pcm_cxl/") != std::string::npos) ||
            (!final_selection.cas.write_events.empty() &&
             final_selection.cas.write_events[0].find("pcm_cxl/") != std::string::npos);
        const bool skip_final_bw_recheck = (config_.measurer == "vtune") || is_pcm_counter_mode;
        if (skip_final_bw_recheck) {
            if (verbose3) {
                if (is_pcm_counter_mode) {
                    std::cout << "    BW re-check skipped for PCM (using stabilized streaming sample)\n";
                } else {
                    std::cout << "    BW re-check skipped for VTune to avoid extra collection overhead\n";
                }
            }
        } else if (bandwidth_measurer_->sample_bandwidth(final_bw_cas_rd,
                                                    final_bw_cas_wr,
                                                    final_bw_elapsed,
                                                    traffic_gen_mem_nodes)) {
            CounterType ctype = bandwidth_measurer_->get_counter_selection().type;
            double scaling_factor = (ctype == CounterType::UPI_FLITS) ? get_upi_scaling_factor() : 1.0;

            double initial_bw = BandwidthMeasurer::calculate_bandwidth_gbps(
                bw_cas_rd, bw_cas_wr, bw_elapsed, ctype, cache_line_size_, scaling_factor);
            double final_bw = BandwidthMeasurer::calculate_bandwidth_gbps(
                final_bw_cas_rd, final_bw_cas_wr, final_bw_elapsed, ctype, cache_line_size_, scaling_factor);

            double bw_change_pct = (initial_bw > 0) ? std::abs((final_bw - initial_bw) / initial_bw * 100.0) : 0.0;
            
            const double BW_GROWTH_THRESHOLD = 5.0;
            
            if (final_bw > initial_bw && bw_change_pct > BW_GROWTH_THRESHOLD) {
                bw_increased = true;
                if (verbose2) {
                    std::cout << "    ⚠ BW increased from " << std::fixed << std::setprecision(2) 
                                << initial_bw << " to " << final_bw << " GB/s (+" 
                                << bw_change_pct << "%) - TrafficGen still ramping up!\n";
                    std::cout << "    Using higher BW value and re-measuring latency...\n";
                }
                
                bw_cas_rd = final_bw_cas_rd;
                bw_cas_wr = final_bw_cas_wr;
                bw_elapsed = final_bw_elapsed;
                
                traffic_gen_manager_->stash_bandwidth_retry(traffic_gen_pid,
                                                        final_bw_cas_rd,
                                                        final_bw_cas_wr,
                                                        final_bw_elapsed,
                                                        final_bw);
                
                return BenchmarkStatus::RETRY_BW_GROWTH;
            } else {
                bw_cas_rd = final_bw_cas_rd;
                bw_cas_wr = final_bw_cas_wr;
                bw_elapsed = final_bw_elapsed;
                
                if (verbose3) {
                    std::cout << "    BW re-check: " << std::fixed << std::setprecision(2) 
                                << final_bw << " GB/s (change: " << bw_change_pct << "%) - STABLE\n";
                }
            }
        } else if (!skip_final_bw_recheck && bw_elapsed <= 0.0 && bw_cas_rd == 0 && bw_cas_wr == 0) {
            std::cerr << "    Error: Final bandwidth sampling failed and no valid bandwidth measurement is available." << std::endl;
            return BenchmarkStatus::FAILURE;
        }
        
        if (verbose2 && !bw_increased) {
            std::cout << "    Both BW and latency stabilized\n";
        }
        
        double total_cycles = burst_totals.cycles;
        double total_instructions = burst_totals.instructions;
        double total_tlb1miss = burst_totals.tlb1miss;
        double total_tlb2miss = burst_totals.tlb2miss;
        double aggregate_duration = burst_totals.duration_s;
        bool using_hugepages = burst_totals.using_hugepages;
        
        double measured_ratio_pct = ratio_pct;
        if (bw_cas_rd + bw_cas_wr > 0) {
            measured_ratio_pct = (double)bw_cas_rd / (double)(bw_cas_rd + bw_cas_wr) * 100.0;
        }
        
        
        result.measured_ratio_pct = measured_ratio_pct;
        result.cycles = total_cycles;
        result.instructions = total_instructions;
        result.accesses = total_accesses;
        result.tlb1miss = total_tlb1miss;
        result.tlb2miss = total_tlb2miss;
        result.duration_s = aggregate_duration;
        result.using_hugepages = using_hugepages;
        result.bw_cas_rd = bw_cas_rd;
        result.bw_cas_wr = bw_cas_wr;
        result.bw_elapsed = bw_elapsed;
        result.traffic_gen_samples = traffic_gen_samples;
        
        result.tlb1_event_name = burst_totals.tlb1_event_name;
        result.tlb2_event_name = burst_totals.tlb2_event_name;

        result.extra_perf_values = bandwidth_measurer_->get_extra_perf_values();

                
        double measured_bw_gb_s = 0.0;
        if (bw_elapsed > 0) {
            CounterType counter_type = bandwidth_measurer_->get_counter_selection().type;
            double scaling_factor = (counter_type == CounterType::UPI_FLITS) ? get_upi_scaling_factor() : 1.0;
            measured_bw_gb_s = BandwidthMeasurer::calculate_bandwidth_gbps(
                bw_cas_rd, bw_cas_wr, bw_elapsed, counter_type, cache_line_size_, scaling_factor);
            result.bandwidth_mbps = measured_bw_gb_s * 1000.0;
        }
        
        if (verbose2) {
            std::cout << "      Measured Bandwidth: " << std::fixed << std::setprecision(2) 
                      << measured_bw_gb_s << " GB/s\n";
            if (verbose3) {
                const auto& selection = bandwidth_measurer_->get_counter_selection();
                bool has_distinct_rw = selection.cas.has_read_write;
                if (has_distinct_rw) {
                    std::string rd_label = "rd", wr_label = "wr";
                    if (selection.type == CounterType::UPI_FLITS) {
                        rd_label = "RXL"; wr_label = "TXL";
                    } else if (selection.type == CounterType::NVIDIA_GRACE) {
                        rd_label = "rd_data"; wr_label = "wr_bytes";
                    } else if (selection.cas.uses_read_subtract_formula) {
                        rd_label = "r17-(r325+r326)";
                        wr_label = "r18";
                    }
                    std::cout << "         (" << rd_label << ": " << bw_cas_rd << ", " << wr_label << ": " << bw_cas_wr 
                          << ", elapsed: " << std::fixed << std::setprecision(2) << bw_elapsed << "s)\n";
                } else {
                    std::cout << "         (DRAM count: " << (bw_cas_rd + bw_cas_wr)
                          << ", elapsed: " << std::fixed << std::setprecision(2) << bw_elapsed << "s)\n";
                }
            }
        }
    
        if (total_accesses > 0 && aggregate_duration > 0) {
            result.latency_ns = calculate_latency_from_perf_values(total_cycles, total_accesses, 
                                                                   total_tlb1miss, total_tlb2miss, 
                                                                   aggregate_duration, using_hugepages);
            
            if (verbose2) {
                std::cout << "      Measured Latency:   " << std::fixed << std::setprecision(2)
                          << result.latency_ns << " ns\n";
                if (verbose3) {
                        std::cout << "         (aggregated over " << LAT_REQUIRED_STABLE_COUNT
                          << " bursts = " << std::fixed << std::setprecision(2)
                          << aggregate_duration << "s, cycles: "
                          << std::fixed << std::setprecision(0) << total_cycles
                          << ", instr: " << total_instructions
                          << ", tlb1miss: " << total_tlb1miss
                          << ", tlb2miss: " << total_tlb2miss
                          << ", Freq: " << std::fixed << std::setprecision(2)
                          << max_cpu_freq_ghz_ << " GHz)\n";
                }
            }
        } else if (config_.instruction_sampling && sampler_stats.samples > 0) {
            result.latency_ns = sampler_stats.iqr_mean_ns > 0.0
                ? sampler_stats.iqr_mean_ns : sampler_stats.mean_ns;

            result.sampler_samples = sampler_stats.samples;
            result.sampler_mean_ns = sampler_stats.mean_ns;
            result.sampler_iqr_mean_ns = sampler_stats.iqr_mean_ns;
            result.sampler_min_ns = sampler_stats.min_ns;
            result.sampler_median_ns = sampler_stats.median_ns;
            result.sampler_p90_ns = sampler_stats.p90_ns;
            result.sampler_p95_ns = sampler_stats.p95_ns;
            result.sampler_p99_ns = sampler_stats.p99_ns;
            result.sampler_p99_9_ns = sampler_stats.p99_9_ns;
            result.sampler_max_ns = sampler_stats.max_ns;

            result.sampler_mean_cycles = sampler_stats.mean_cycles;
            result.sampler_iqr_mean_cycles = sampler_stats.iqr_mean_cycles;
            result.sampler_min_cycles = sampler_stats.min_cycles;
            result.sampler_median_cycles = sampler_stats.median_cycles;
            result.sampler_p90_cycles = sampler_stats.p90_cycles;
            result.sampler_p95_cycles = sampler_stats.p95_cycles;
            result.sampler_p99_cycles = sampler_stats.p99_cycles;
            result.sampler_p99_9_cycles = sampler_stats.p99_9_cycles;
            result.sampler_max_cycles = sampler_stats.max_cycles;

            result.sampler_raw_latencies = std::move(sampler_stats.raw_latencies);

            if (verbose2) {
                double freq_ghz = cached_cpuinfo_freq_ghz_ > 0.0 ? cached_cpuinfo_freq_ghz_
                    : (sys_info_.cpu_base_mhz > 0
                        ? sys_info_.cpu_base_mhz / 1000.0 : 3.0);
                std::cout << "      Sampler Latency Stats (freq=" << std::fixed << std::setprecision(2) << freq_ghz << " GHz):\n";
                std::cout << "        Samples : " << sampler_stats.samples << "\n";
                std::cout << "        Mean    : " << std::fixed << std::setprecision(1) << sampler_stats.mean_cycles << " cycles (" << std::fixed << std::setprecision(2) << sampler_stats.mean_ns << " ns)\n";
                std::cout << "        IQR Mean: " << std::fixed << std::setprecision(1) << sampler_stats.iqr_mean_cycles << " cycles (" << std::fixed << std::setprecision(2) << sampler_stats.iqr_mean_ns << " ns)\n";
                std::cout << "        Min     : " << std::fixed << std::setprecision(1) << sampler_stats.min_cycles << " cycles (" << std::fixed << std::setprecision(2) << sampler_stats.min_ns << " ns)\n";
                std::cout << "        Median  : " << std::fixed << std::setprecision(1) << sampler_stats.median_cycles << " cycles (" << std::fixed << std::setprecision(2) << sampler_stats.median_ns << " ns)\n";
                std::cout << "        P90     : " << std::fixed << std::setprecision(1) << sampler_stats.p90_cycles << " cycles (" << std::fixed << std::setprecision(2) << sampler_stats.p90_ns << " ns)\n";
                std::cout << "        P95     : " << std::fixed << std::setprecision(1) << sampler_stats.p95_cycles << " cycles (" << std::fixed << std::setprecision(2) << sampler_stats.p95_ns << " ns)\n";
                std::cout << "        P99     : " << std::fixed << std::setprecision(1) << sampler_stats.p99_cycles << " cycles (" << std::fixed << std::setprecision(2) << sampler_stats.p99_ns << " ns)\n";
                std::cout << "        P99.9   : " << std::fixed << std::setprecision(1) << sampler_stats.p99_9_cycles << " cycles (" << std::fixed << std::setprecision(2) << sampler_stats.p99_9_ns << " ns)\n";
                std::cout << "        Max     : " << std::fixed << std::setprecision(1) << sampler_stats.max_cycles << " cycles (" << std::fixed << std::setprecision(2) << sampler_stats.max_ns << " ns)\n";
            }
        } else {
            std::cerr << "Warning: No valid latency measurements found" << std::endl;
            result.latency_ns = 0.0;
            if (ptrchase_manager_) {
                std::cerr << "    Forcing ptr_chase restart due to missing latency data" << std::endl;
                ptrchase_manager_->cleanup();
            }
            std::cerr << "    Returning false to trigger retry..." << std::endl;
            return BenchmarkStatus::FAILURE;
        }
        
        
        if (verbose2) {
            std::cout << "    Cleaning up processes...";
            std::cout << "\n\n";
        }
        traffic_gen_manager_->terminate_traffic_gen(traffic_gen_pid);
        
        int cleanup_wait = 0;
        while (traffic_gen_manager_->is_traffic_gen_running(traffic_gen_pid) && cleanup_wait < 20) {
            traffic_gen_manager_->terminate_traffic_gen(traffic_gen_pid);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            cleanup_wait++;
        }
        
        if (cleanup_wait >= 20) {
            std::cerr << "    ERROR: Failed to kill TrafficGen processes after 4 seconds" << std::endl;
        }
    } else {
        std::cerr << "Warning: Failed to launch TrafficGen stress for ratio " << ratio_pct << "%, pause " << pause << std::endl;
    }

    return BenchmarkStatus::SUCCESS;
}


double BenchmarkExecutor::calculate_latency_from_perf_values(double cycles, double accesses, double tlb1miss, double tlb2miss, double interval_duration, bool using_hugepages) {
    (void)using_hugepages;
    if (interval_duration <= 0.0 || accesses <= 0.0) {
        return 0.0;
    }
    
    // Measured frequency from cycles/time. This is the most accurate when valid,
    // but on some systems (e.g. NVIDIA Grace) cycles/duration does not track the
    // real core frequency under memory-stall-heavy workloads. We therefore only
    // trust the measurement when it lies in a plausible band around the
    // datasheet base/max frequencies; otherwise we fall back to base.
    const double measured_freq_ghz = (cycles / interval_duration) / 1e9;
    const double base_ghz = sys_info_.cpu_base_mhz / 1000.0;
    const double max_ghz  = sys_info_.cpu_max_mhz  / 1000.0;

    double freq_ghz = cached_cpuinfo_freq_ghz_;
    const char* freq_source = "cached";
    if (freq_ghz <= 0.0) {
        double lo = (base_ghz > 0.0) ? base_ghz * 0.7 : 0.0;
        double hi = (max_ghz  > 0.0) ? max_ghz  * 1.1 : 0.0;
        const bool has_hi_bound = (max_ghz > 0.0);

        if (measured_freq_ghz >= lo &&
            (!has_hi_bound || measured_freq_ghz <= hi) &&
            measured_freq_ghz > 0.0) {
            freq_ghz = measured_freq_ghz;
            freq_source = "measured";
        } else if (base_ghz > 0.0) {
            freq_ghz = base_ghz;
            freq_source = "datasheet-base (measured out of range)";
        } else if (max_ghz > 0.0) {
            freq_ghz = max_ghz;
            freq_source = "datasheet-max (no base)";
        } else {
            freq_ghz = 3.0; 
            freq_source = "fallback 3.0 GHz";
        }
        cached_cpuinfo_freq_ghz_ = freq_ghz;
    }

    if (config_.verbosity >= 4) {
        std::cout << "      [Freq] measured=" << std::fixed << std::setprecision(2)
                  << measured_freq_ghz << " GHz, base=" << base_ghz
                  << ", max=" << max_ghz
                  << " -> using " << freq_ghz << " GHz (" << freq_source << ")"
                  << std::endl;
    }

    max_cpu_freq_ghz_ = freq_ghz;

    const double total_ns = cycles / freq_ghz;
    const double tlb_overhead_ns = counter_strategy_->computeTlbOverheadNs(
        tlb1miss, tlb2miss, freq_ghz, cached_tlb_hit_latency_ns_);
    return (total_ns - tlb_overhead_ns) / accesses;
}


void BenchmarkExecutor::write_frequency_to_plotter(double cpu_freq_ghz) {
    if (!config_.profile_output) {
        return;
    }
    
    auto update_plotter = [&](const std::string& plotter_file) {
        int fd = open(plotter_file.c_str(), O_RDWR | O_CREAT, 0644);
        if (fd == -1) {
            return;
        }
        
        if (flock(fd, LOCK_EX) == -1) {
            close(fd);
            return;
        }
        
        std::ifstream current_file(plotter_file);
        std::string content;
        std::string line;
        bool cpu_freq_exists = false;
        bool cpu_base_freq_exists = false;
        bool cpu_max_freq_exists = false;
        
        while (std::getline(current_file, line)) {
            content += line + "\n";
            if (line.find("CPU_FREQ=") == 0) {
                cpu_freq_exists = true;
            } else if (line.find("CPU_BASE_FREQ=") == 0) {
                cpu_base_freq_exists = true;
            } else if (line.find("CPU_MAX_FREQ=") == 0) {
                cpu_max_freq_exists = true;
            }
        }
        current_file.close();
        
        if (!cpu_freq_exists || !cpu_base_freq_exists || !cpu_max_freq_exists) {
            if (!content.empty() && content.back() == '\n') {
                content.pop_back();
            }
            
            if (!cpu_freq_exists) {
                content += "\nCPU_FREQ=" + std::to_string(cpu_freq_ghz);
            }
            if (!cpu_base_freq_exists && sys_info_.cpu_base_mhz > 0) {
                content += "\nCPU_BASE_FREQ=" + std::to_string(sys_info_.cpu_base_mhz / 1000.0);
            }
            if (!cpu_max_freq_exists && sys_info_.cpu_max_mhz > 0) {
                content += "\nCPU_MAX_FREQ=" + std::to_string(sys_info_.cpu_max_mhz / 1000.0);
            }
            content += "\n";
            
            std::ofstream out_file(plotter_file);
            out_file << content;
            out_file.close();
        }
        
        flock(fd, LOCK_UN);
        close(fd);
    };
    
    update_plotter(config_.output_root + "/multisequential/plotter.txt");
    
}
