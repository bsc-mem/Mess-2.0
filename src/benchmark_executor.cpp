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

#include "benchmark_executor.h"
#include "measurement.h"
#include "process_manager.h"
#include "architecture/ArchitectureRegistry.h"
#include "utils.h"
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
#include <thread>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <numeric>
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
#include "ptrchase_perf_helper.h"
#include "architecture/PerformanceCounterStrategy.h"
#include "architecture/ArchitectureRegistry.h"

double BenchmarkExecutor::cached_tlb_hit_latency_ns_ = 10.0;
bool BenchmarkExecutor::frequency_written_to_plotter_ = false; 

BenchmarkExecutor::BenchmarkExecutor(const BenchmarkConfig& config, const system_info& sys_info, const CPUCapabilities& caps,
                                   double tlb_latency_ns, int cache_line_size)
    : config_(config),
      sys_info_(sys_info),
      caps_(caps),
      cache_line_size_(cache_line_size),
      traffic_gen_manager_(std::make_unique<TrafficGenProcessManager>(config, sys_info)),
      ptrchase_manager_(std::make_unique<PtrChaseProcessManager>(config, sys_info)),
      measurement_storage_multiseq_(std::make_unique<MeasurementStorage>(config, cache_line_size, config.verbosity, config.output_root + "/multisequential")),
      bandwidth_measurer_multiseq_(nullptr),
      latency_measurer_multiseq_(std::make_unique<LatencyMeasurer>(config, measurement_storage_multiseq_.get(), ptrchase_manager_.get(), [this]() { return get_memory_binding_nodes(); }, cache_line_size)),
      outlier_detector_multiseq_(std::make_unique<OutlierDetector>(config, measurement_storage_multiseq_.get(), tlb_latency_ns)),
      max_cpu_freq_ghz_(0.0),
      progress_tracker_(std::make_unique<ProgressTracker>(0)) {
    bandwidth_measurer_multiseq_ = create_bandwidth_measurer(
        config, sys_info, caps,
        measurement_storage_multiseq_.get(),
        traffic_gen_manager_.get(),
        [this]() { return get_memory_binding_nodes(); },
        ExecutionMode::MULTISEQUENTIAL);
    
    
    for (int i = 0; i <= 100; i += 2) {
        candidate_ratios_.push_back(static_cast<double>(i));
    }
    candidate_ratios_.insert(candidate_ratios_.end(), config_.ratios_pct.begin(), config_.ratios_pct.end());
    
    cached_tlb_hit_latency_ns_ = tlb_latency_ns;

    uint64_t tlb1_raw = 0, tlb2_raw = 0;
    bool use_tlb1 = false, use_tlb2 = false;
    ptrchase_perf::select_tlb_events_for_ptrchase(sys_info, tlb1_raw, tlb2_raw, use_tlb1, use_tlb2);

    std::string tlb1_str = "";
    std::string tlb2_str = "";

    if (use_tlb1) {
        tlb1_str = PerformanceCounterStrategy::formatTlbEventString(tlb1_raw);
    }
    if (use_tlb2) {
        tlb2_str = PerformanceCounterStrategy::formatTlbEventString(tlb2_raw);
    }

    latency_measurer_multiseq_->setExpectedTlbEvents(tlb1_str, tlb2_str);


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

    measurement_storage_multiseq_->write_plotter_file(caps, binding_type, tlb_latency_ns, cache_line_size, upi_scaling_factor);
}

BenchmarkExecutor::~BenchmarkExecutor() {
    force_cleanup();
}

void BenchmarkExecutor::force_cleanup() {
    if (ptrchase_manager_) {
        ptrchase_manager_->cleanup();
    }
    if (traffic_gen_manager_) {
        traffic_gen_manager_->kill_all_traffic_gen();
    }
}

void BenchmarkExecutor::cleanup_measurement_files(double ratio_pct, int pause) const {

    MeasurementStorage* storage = measurement_storage_multiseq_.get();
    storage->cleanup_measurement_files(ratio_pct, pause);
}

std::string BenchmarkExecutor::format_bandwidth_output(const BenchmarkResult& result, const BandwidthCounterSelection& selection) const {
    std::stringstream bw_ss;
    
    std::string read_event = "unc_m_cas_count.rd";
    std::string write_event = "unc_m_cas_count.wr";
    
    if (selection.type == CounterType::UPI_FLITS) {
        if (!selection.upi.rxl_data_events.empty()) read_event = selection.upi.rxl_data_events[0];
        if (!selection.upi.txl_data_events.empty()) write_event = selection.upi.txl_data_events[0];
    } else if (selection.type == CounterType::CAS_COUNT || selection.type == CounterType::NVIDIA_GRACE) {
        if (!selection.cas.read_events.empty()) read_event = selection.cas.read_events[0];
        if (!selection.cas.write_events.empty()) write_event = selection.cas.write_events[0];
        if (!selection.cas.combined_events.empty() && read_event == "unc_m_cas_count.rd") {
            read_event = selection.cas.combined_events[0];
            write_event = selection.cas.combined_events[0];
        }
    }
    
    bw_ss << "\n Performance counter stats for system wide:\n\n";
    if (read_event.find("dram_channel_data_controller") != std::string::npos || 
        write_event.find("dram_channel_data_controller") != std::string::npos) {
        long long total_cas = result.bw_cas_rd + result.bw_cas_wr;
        bw_ss << "S0           " << result.traffic_gen_samples << "                  " << total_cas << "      dram_channel_data_controller_total\n";
    } else {
        bw_ss << "S0           " << result.traffic_gen_samples << "                  " << result.bw_cas_rd << "      " << read_event << "\n";
        bw_ss << "S0           " << result.traffic_gen_samples << "                  " << result.bw_cas_wr << "      " << write_event << "\n";
    }

    for (const auto& kv : result.extra_perf_values) {
        bw_ss << "S0           " << result.traffic_gen_samples << "                  " << kv.second << "      " << kv.first << "\n";
    }

    bw_ss << "\n       " << std::fixed << std::setprecision(9) << result.bw_elapsed << " seconds time elapsed\n\n";
    
    return bw_ss.str();
}

std::string BenchmarkExecutor::format_latency_output(const BenchmarkResult& result, pid_t ptrchase_pid) const {
    std::stringstream lat_ss;
    lat_ss << "\n Performance counter stats for process id '" << ptrchase_pid << "':\n\n";
    lat_ss << std::fixed << std::setprecision(0);
    lat_ss << "   " << result.cycles << "      cycles\n";
    lat_ss << "   " << result.instructions << "      instructions\n";
    lat_ss << "   " << result.accesses << "      accesses\n";
    lat_ss << "   " << result.tlb1miss << "      " << result.tlb1_event_name << "\n";
    lat_ss << "   " << result.tlb2miss << "      " << result.tlb2_event_name << "\n";
    lat_ss << std::fixed << std::setprecision(9);
    lat_ss << "\n       " << result.duration_s << " seconds time elapsed\n\n";
    return lat_ss.str();
}

bool BenchmarkExecutor::run() {

    results_.clear();

    std::vector<double> ratios = config_.ratios_pct;
    if (ratios.empty()) {
        for (int i = 0; i <= 100; i += 2) {
            ratios.push_back(static_cast<double>(i));
        }
    }

    std::vector<int> pauses = config_.pauses;
    if (pauses.empty()) {
        pauses = {0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 80, 90, 100,
                  120, 140, 160, 180, 200, 220, 260, 300, 340, 380, 450, 550, 600, 700,
                  800, 900, 1000, 1500, 2000, 3000, 5000, 40000, 100000};
    }
    


    std::vector<ExecutionMode> modes;
    bool do_multiseq = config_.generate_multiseq;
    if (do_multiseq) modes.push_back(ExecutionMode::MULTISEQUENTIAL);


    int total_global_measurements = ratios.size() * pauses.size() * config_.point_reps * modes.size();
    progress_tracker_->start_benchmark(total_global_measurements);

    if (!ptrchase_manager_->ensure_running()) {
        std::cerr << "ERROR: Failed to launch persistent pointer chase process" << std::endl;
        return false;
    }
    
    int total_ratios = ratios.size();
    int current_ratio = 0;
    
    for (double ratio_pct : ratios) {
        current_ratio++;
        if (config_.verbosity >= 1) {
            std::cout << "\n\n=== " << ratio_pct << "% reads (" 
                      << current_ratio << "/" << total_ratios << " read ratios) ===" << std::endl;
        }
        
        std::map<ExecutionMode, std::vector<double>> bw_history;
        std::set<int> backtracked_pauses;

        for (int pause_idx = 0; pause_idx < static_cast<int>(pauses.size()); ++pause_idx) {
            int pause = pauses[pause_idx];
            bool backtrack_triggered = false;
            int consecutive_failures = 0;

            for (ExecutionMode mode : modes) {
                std::string mode_str = "MULTISEQ";
                MeasurementStorage* storage = measurement_storage_multiseq_.get();
                OutlierDetector* outlier_detector = outlier_detector_multiseq_.get();
                BandwidthMeasurer* bandwidth_measurer = bandwidth_measurer_multiseq_.get();

                bool needs_restart = true;
                int total_measurements = config_.point_reps; 
                
                std::vector<BenchmarkResult> current_point_results;

                while (needs_restart) {
                    needs_restart = false;
                    current_point_results.clear();
                    
                    std::string bw_file = storage->bw_file_path(ratio_pct, pause);
                    std::string lat_file = storage->lat_file_path(ratio_pct, pause);
                    std::string log_file = storage->traffic_gen_log_file_path(ratio_pct, pause);
                    
                    cleanup_measurement_files(ratio_pct, pause);

                    if (progress_tracker_->completed_iterations() == 0) {
                        std::string initial_progress = progress_tracker_->format_progress(
                            0, total_measurements, total_global_measurements);
                        std::cout << initial_progress << std::flush;
                    }
                    
                    std::stringstream bw_buffer;
                    std::stringstream lat_buffer;

                    for (int rep = 0; rep < config_.point_reps; rep++) {
                        BenchmarkResult result;
                        result.ratio_pct = ratio_pct;
                        result.pause = pause;
                        result.repetition = rep;
                        result.iteration_time_s = 0.0;

                        progress_tracker_->start_iteration();

                        if (config_.verbosity >= 2) {
                            std::cout << "\n Read Ratio: " << ratio_pct << "\n Pause: " << pause << "\n Execution Mode: " << mode_str << std::endl;
                        }
                        
                        BenchmarkStatus status = run_single_benchmark(ratio_pct, pause, rep, result, mode);
                        
                        if (status != BenchmarkStatus::SUCCESS) {
                            progress_tracker_->finish_iteration();
                            
                            if (status == BenchmarkStatus::RETRY_BW_GROWTH) {
                                // For BW growth, we just retry the same point immediately without killing traffic gen
                                // and without incrementing consecutive_failures count (or maybe reset it?)
                                if (config_.verbosity >= 2) {
                                    std::cout << "    [Retry] Re-measuring same point due to BW growth..." << std::endl;
                                }
                                rep--;
                                continue;
                            }

                            consecutive_failures++;
                            if (consecutive_failures >= 5) {
                                std::cerr << "\nERROR: Too many consecutive failures (" << consecutive_failures << "). Aborting benchmark to prevent infinite loop." << std::endl;
                                return false;
                            }

                            if (config_.verbosity >= 2) {
                                std::cout << "    Retrying measurement (failure " << consecutive_failures << "/5)..." << std::endl;
                            }
                            
                            if (traffic_gen_manager_) {
                                traffic_gen_manager_->kill_all_traffic_gen();
                            }
                            
                            std::this_thread::sleep_for(std::chrono::milliseconds(200));
                            
                            rep--;
                            continue;
                        }
                        consecutive_failures = 0;

                        double iteration_time = progress_tracker_->finish_iteration();
                        progress_tracker_->record_iteration_time(iteration_time);
                        result.iteration_time_s = iteration_time;
                        current_point_results.push_back(result);
                        
                        progress_tracker_->increment_completed_iterations();
                        
                        int remaining_iterations =
                            total_global_measurements - progress_tracker_->completed_iterations();
                        std::string progress_line = progress_tracker_->format_progress(
                            progress_tracker_->completed_iterations(), total_global_measurements, remaining_iterations);
                        
                        if (rep < config_.point_reps - 1) {
                            std::cout << "\r" << progress_line << std::flush;
                        }
                        
                        const auto& selection = bandwidth_measurer->get_counter_selection();
                        bw_buffer << format_bandwidth_output(result, selection);
                        
                        lat_buffer << format_latency_output(result, ptrchase_manager_->pid());

                        results_.push_back(result);
                    }

                    // Flush buffers to disk
                    bw_file = storage->bw_file_path(ratio_pct, pause);
                    storage->append_to_file_with_lock(bw_file, bw_buffer.str());
                    
                    lat_file = storage->lat_file_path(ratio_pct, pause);
                    if (!storage->append_to_file_with_lock(lat_file, lat_buffer.str())) {
                        std::cerr << "  Failed to write latency data: " << lat_file << std::endl;
                    }
                    
                    if (check_and_rerun_outliers(ratio_pct, pause, mode)) {
                         if (config_.verbosity >= 1) {
                            std::cout << "\n    ⚠ Failed to resolve outliers after retries. Restarting point..." << std::endl;
                        }
                        needs_restart = true;
                        progress_tracker_->reset_completed_iterations(progress_tracker_->completed_iterations() - current_point_results.size());
                        continue;
                    }
                    
                    int remaining_iterations =
                            total_global_measurements - progress_tracker_->completed_iterations();
                    std::string progress_line = progress_tracker_->format_progress(
                            progress_tracker_->completed_iterations(), total_global_measurements, remaining_iterations);
                    std::cout << "\r" << progress_line << std::flush;
                    
                    double scaling_factor = 1.0;
                    if (bandwidth_measurer->get_counter_selection().type == CounterType::UPI_FLITS) {
                        auto arch = ArchitectureRegistry::instance().getArchitecture(caps_);
                        if (arch) {
                            scaling_factor = arch->getUpiScalingFactor(caps_);
                        } else {
                            scaling_factor = 1.0 / 9.0; // Fallback
                        }
                    }
                    std::vector<double> final_bw = outlier_detector->load_bandwidth(ratio_pct, pause, scaling_factor);


                    current_point_results.clear();
                    for (double bw : final_bw) {
                        BenchmarkResult res;
                        res.bandwidth_mbps = bw * 1000.0;
                        current_point_results.push_back(res);
                    }
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
                    
                    // If previous BW was valid (non-zero) and current BW is > 50% higher
                    if (prev_bw > 0.1 && avg_bw > prev_bw * 1.5) {
                        if (backtracked_pauses.find(pause) == backtracked_pauses.end()) {
                            if (config_.verbosity >= 1) {
                                std::cout << "    ⚠ Anomaly detected: Higher pause (" << pause 
                                        << ") has significantly higher BW (" << std::fixed << std::setprecision(2) << avg_bw/1000.0 
                                        << " GB/s) than previous pause (" << pauses[pause_idx-1] 
                                        << ", " << prev_bw/1000.0 << " GB/s). (+50% threshold exceeded)" << std::endl;
                                std::cout << "    ↺ Backtracking to repeat pause " << pauses[pause_idx-1] << "..." << std::endl;
                            }
                            
                            backtracked_pauses.insert(pause);
                            pause_idx -= 2;
                            backtrack_triggered = true;
                            break; 
                        } else {
                            if (config_.verbosity >= 1) {
                                std::cout << "    ⚠ Anomaly persisted after retry: Higher pause (" << pause 
                                        << ") still has higher BW. Ignoring and continuing..." << std::endl;
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

            if (!backtrack_triggered && !config_.explicit_pauses && pause_idx == static_cast<int>(pauses.size()) - 1) {
                double max_bw = 0.0;
                for (ExecutionMode mode : modes) {
                    if (bw_history[mode].size() > static_cast<size_t>(pause_idx)) {
                        max_bw = std::max(max_bw, bw_history[mode][pause_idx]);
                    }
                }

                if (max_bw > 1000.0) {
                    int current_pause = pauses[pause_idx];
                    int next_pause = (current_pause == 0) ? 100 : current_pause * 2;
                    
                    if (next_pause < 200000000) {
                        pauses.push_back(next_pause);
                        if (config_.verbosity >= 2) {
                            std::cout << "    [Dynamic] BW (" << std::fixed << std::setprecision(2) << max_bw/1000.0 
                                      << " GB/s) > 1 GB/s. Extending pause range: added " << next_pause << std::endl;
                        }
                    }
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
    // Universal cleanup for traffic_gen processes
    system("pkill -9 -f 'traffic_gen_.*\\.x' 2>/dev/null");
    system("pkill -9 '^traffic_gen_' 2>/dev/null");
    
    system("rm -f /tmp/traffic_gen_pid_* 2>/dev/null");
    
    if (config_.verbosity >= 2) {
        std::cout << "Cleanup complete" << std::endl;
    }

    return true;
}

std::vector<int> BenchmarkExecutor::get_memory_binding_nodes() const {
    return config_.memory_bind_nodes;
}



BenchmarkExecutor::BenchmarkStatus BenchmarkExecutor::run_single_benchmark(double ratio_pct, int pause, int rep, BenchmarkResult& result, ExecutionMode mode) {
    MeasurementStorage* measurement_storage_ = measurement_storage_multiseq_.get();
    BandwidthMeasurer* bandwidth_measurer_ = bandwidth_measurer_multiseq_.get();
    LatencyMeasurer* latency_measurer_ = latency_measurer_multiseq_.get();

    result.repetitions = config_.point_reps;
    result.mode = mode;

    measurement_storage_->ensure_directories_exist();
    
    if (ratio_pct < 0 || ratio_pct > 100) {
        std::cerr << "ERROR: Invalid ratio_pct value: " << ratio_pct << std::endl;
        return BenchmarkStatus::FAILURE;
    }
    
    if (pause < 0) {
        std::cerr << "ERROR: Invalid pause value: " << pause << std::endl;
        return BenchmarkStatus::FAILURE;
    }
    
    std::string bw_file = measurement_storage_->bw_file_path(ratio_pct, pause);
    std::string lat_file = measurement_storage_->lat_file_path(ratio_pct, pause);
    std::string traffic_gen_log_file = measurement_storage_->traffic_gen_log_file_path(ratio_pct, pause);
        
    int traffic_gen_cores = sys_info_.sockets[0].core_count - 1;
    if (config_.traffic_gen_cores > 0 && config_.traffic_gen_cores <= sys_info_.sockets[0].core_count - 1) {
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

    int src_cpu = 0;
    if (!config_.traffic_gen_explicit_cores.empty()) {
        try {
            src_cpu = std::stoi(config_.traffic_gen_explicit_cores[0]);
        } catch (...) {}
    }

    std::vector<int> monitored_nodes = traffic_gen_mem_nodes;
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

    auto bw_counters = PerformanceCounterStrategy::discoverBandwidthCounters(src_cpu, monitored_nodes, config_.force_likwid);
    bw_counters.extra_counters = config_.extra_perf_counters;
    bandwidth_measurer_->set_counter_selection(bw_counters);


    if (config_.verbosity >= 2) {
        std::cout << "    Current measurement is " << (rep+1) << "/" << config_.point_reps << "" << std::endl;
    }

    auto step_start = std::chrono::steady_clock::now();
    auto prev_step = step_start;
    
    if (!ptrchase_manager_->ensure_running()) {
        std::cerr << "ERROR: Cannot ensure ptr_chase is running" << std::endl;
        return BenchmarkStatus::FAILURE;
    }
    
    if (config_.verbosity >= 2) {
        std::cout << "    Checking ptr_chase status..." << std::endl;
    }
    
    if (config_.verbosity >= 3) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - prev_step).count();
        std::string time_str = (elapsed_ms > 1000) ? ("~" + std::to_string(elapsed_ms/1000) + "s") : ("~" + std::to_string(elapsed_ms) + "ms");
        std::cout << "        Still active with pid: " << ptrchase_manager_->pid() << std::endl;
        prev_step = now;
    }
    
    if (traffic_gen_pid > 0) {
        int traffic_gen_samples = 0;
        auto now = std::chrono::steady_clock::now();
        prev_step = now;
        
        std::string bw_file = measurement_storage_->bw_file_path(ratio_pct, pause);
        
        std::string lat_file = measurement_storage_->lat_file_path(ratio_pct, pause);
        
        if (config_.verbosity >= 2) {
            std::cout << "    Waiting for TrafficGen to stabilize..." << std::endl;
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
            
            if (config_.verbosity >= 2) {
                std::cout << "    Using previous BW measurement: " << std::fixed << std::setprecision(2) 
                          << prev_bw_gb_s << " GB/s (TrafficGen already stable, just re-measuring latency)" << std::endl;
            }
        }
        

        
        // If BW didn't stabilize (60s timeout), abort this measurement and retry
        double stabilization_timeout_samples = (60.0 * 1000.0) / sampling_interval_ms;
        if (!traffic_gen_stabilized && traffic_gen_samples > stabilization_timeout_samples) {
            if (config_.verbosity >= 1) {
                std::cout << "    ⚠ Aborting measurement due to BW instability (Time limit)" << std::endl;
            }
            return BenchmarkStatus::FAILURE;  // Trigger retry
        }
        
        std::deque<PerfBurstCounters> latency_samples;
        
        auto print_burst_stats = [&](const PerfBurstCounters& b, int current_count) {
            if (config_.verbosity >= 3) {
                double accesses_per_burst = latency_measurer_->get_accesses_per_burst();
                double accesses_per_s = (b.duration_s > 0) ? (accesses_per_burst / b.duration_s) : 0;
                double latency_ns = (accesses_per_s > 0) ? (1e9 / accesses_per_s) : 0;
                
                std::cout << "      [Latency Burst " << current_count << "] "
                          << "cycles: " << static_cast<long long>(b.cycles) << " | "
                          << "instr: " << static_cast<long long>(b.instructions) << " | "
                          << "tlb1: " << static_cast<long long>(b.tlb1miss) << " | "
                          << "tlb2: " << static_cast<long long>(b.tlb2miss) << " | "
                          << "dur: " << std::fixed << std::setprecision(3) << b.duration_s << " s | "
                          << "lat: " << std::fixed << std::setprecision(1) << latency_ns << " ns"
                          << std::endl;
            }
        };

        auto on_sample_callback = [&]() {
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
        };

        if (!traffic_gen_stabilized) {
            latency_measurer_->init_ptrchase_scaling();
            latency_measurer_->start_burst_async();

            traffic_gen_stabilized = bandwidth_measurer_->wait_for_stabilization(
                traffic_gen_samples, bw_cas_rd, bw_cas_wr, bw_elapsed,
                pause, ratio_pct, reuse_existing_traffic_gen, on_sample_callback);
        }
        
        // If BW didn't stabilize (60s timeout), abort this measurement and retry
        if (!traffic_gen_stabilized && traffic_gen_samples > stabilization_timeout_samples) {
            if (config_.verbosity >= 1) {
                std::cout << "    ⚠ Aborting measurement due to BW instability (Time limit)" << std::endl;
            }
            return BenchmarkStatus::FAILURE;  // Trigger retry
        }
        
        if (config_.verbosity >= 2) {
            if (traffic_gen_stabilized) {
                double wait_time = (sampling_interval_ms * traffic_gen_samples + 100.0);
                std::cout << "    Bandwidth stabilized after " << traffic_gen_samples << " samples";
                if (config_.verbosity >= 3) {
                    std::string time_str = (wait_time >= 1000.0)
                        ? (std::to_string(static_cast<int>(std::round(wait_time/1000.0))) + "s")
                        : (std::to_string(static_cast<int>(std::round(wait_time))) + "ms");
                    std::cout << " ( ~" << time_str << " )";
                }
                std::cout << std::endl;
            } else {
                std::cout << "    Timeout, measuring last value" << std::endl;
            }
        }
        
        PerfBurstCounters burst_totals{0, 0, 0, 0, 0, false, "", ""};
        double total_accesses = 0.0;
        int bursts_aggregated = 0;
        
        const int min_bursts = 3;
        const int max_bursts = 15;
        
        auto are_last_3_stable = [&](const std::deque<PerfBurstCounters>& samples) -> bool {
            if (samples.size() < 3) return false;
            
            std::vector<double> latencies;
            double sum = 0.0;
            int count = 0;
            for (auto it = samples.rbegin(); it != samples.rend() && count < 3; ++it) {
                double accesses_per_burst = latency_measurer_->get_accesses_per_burst();
                double accesses_per_s = (it->duration_s > 0) ? (accesses_per_burst / it->duration_s) : 0;
                double lat = (accesses_per_s > 0) ? (1e9 / accesses_per_s) : 0;
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
                 if (config_.verbosity >= 1) {
                    std::cout << "    Retrying latency measurement (attempt " << (latency_retries + 1) << "/" << max_latency_retries << ")..." << std::endl;
                 }
            }

            if (config_.verbosity >= 2) {
                std::cout << "    Waiting for stable latency bursts..." << std::endl;
            }
             
            while (latency_samples.size() < max_bursts) {
                 PerfBurstCounters burst_result;
                 if (latency_measurer_->is_burst_running()) {
                     if (latency_measurer_->try_collect_burst_async(burst_result)) {
                        latency_samples.push_back(burst_result);
                        
                        print_burst_stats(burst_result, latency_samples.size());
                        if (latency_samples.size() >= min_bursts && are_last_3_stable(latency_samples)) {
                            if (config_.verbosity >= 2) {
                                std::cout << "    Latency bursts stabilized" << std::endl;
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

        if (!latency_success) {
            std::cerr << "    Error: Failed to measure latency after " << max_latency_retries << " attempts." << std::endl;
            return BenchmarkStatus::FAILURE;
        }
        
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

        if (config_.verbosity >= 3) {
            std::cout << "        averaged last " << bursts_aggregated << " bursts" << std::endl;
        }
        
        bool bw_increased = false;
        
        long long final_bw_cas_rd = 0, final_bw_cas_wr = 0;
        double final_bw_elapsed = 0;
        
        if (bandwidth_measurer_->sample_bandwidth(final_bw_cas_rd,
                                                    final_bw_cas_wr,
                                                    final_bw_elapsed,
                                                    traffic_gen_mem_nodes)) {
            double scaling_factor = 1.0;
            double initial_bw = 0.0;

            if (bandwidth_measurer_->get_counter_selection().type == CounterType::UPI_FLITS) {
                auto arch = ArchitectureRegistry::instance().getArchitecture(caps_);
                if (arch) {
                    scaling_factor = arch->getUpiScalingFactor(caps_);
                } else {
                    scaling_factor = 1.0 / 9.0;
                }
                initial_bw = (bw_cas_rd + bw_cas_wr) * cache_line_size_ * scaling_factor / (bw_elapsed * 1e9);
            } else if (bandwidth_measurer_->get_counter_selection().type == CounterType::NVIDIA_GRACE) {
                double bytes_rd = static_cast<double>(bw_cas_rd) * 32.0;
                double bytes_wr = static_cast<double>(bw_cas_wr);
                initial_bw = (bytes_rd + bytes_wr) / (bw_elapsed * 1e9);
            } else {
                initial_bw = (bw_cas_rd + bw_cas_wr) * cache_line_size_ * scaling_factor / (bw_elapsed * 1e9);
            }
            double final_bw = 0.0;
            if (bandwidth_measurer_->get_counter_selection().type == CounterType::UPI_FLITS) {
                final_bw = (final_bw_cas_rd + final_bw_cas_wr) * cache_line_size_ * scaling_factor / (final_bw_elapsed * 1e9);
            } else if (bandwidth_measurer_->get_counter_selection().type == CounterType::NVIDIA_GRACE) {
                double final_bytes_rd = static_cast<double>(final_bw_cas_rd) * 32.0;
                double final_bytes_wr = static_cast<double>(final_bw_cas_wr);
                final_bw = (final_bytes_rd + final_bytes_wr) / (final_bw_elapsed * 1e9);
            } else {
                final_bw = (final_bw_cas_rd + final_bw_cas_wr) * cache_line_size_ * scaling_factor / (final_bw_elapsed * 1e9);
            }
            double bw_change_pct = (initial_bw > 0) ? std::abs((final_bw - initial_bw) / initial_bw * 100.0) : 0.0;
            
            const double BW_GROWTH_THRESHOLD = 5.0;
            
            if (final_bw > initial_bw && bw_change_pct > BW_GROWTH_THRESHOLD) {
                bw_increased = true;
                if (config_.verbosity >= 2) {
                    std::cout << "    ⚠ BW increased from " << std::fixed << std::setprecision(2) 
                                << initial_bw << " to " << final_bw << " GB/s (+" 
                                << bw_change_pct << "%) - TrafficGen still ramping up!" << std::endl;
                    std::cout << "    Using higher BW value and re-measuring latency..." << std::endl;
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
                
                if (config_.verbosity >= 3) {
                    std::cout << "    BW re-check: " << std::fixed << std::setprecision(2) 
                                << final_bw << " GB/s (change: " << bw_change_pct << "%) - STABLE" << std::endl;
                }
            }
        }
        
        if (config_.verbosity >= 2 && !bw_increased) {
            std::cout << "    Both BW and latency stabilized" << std::endl;
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

                
        traffic_gen_manager_->terminate_traffic_gen(traffic_gen_pid);
        double measured_bw_gb_s = 0.0;
        if (bw_elapsed > 0) {
            double scaling_factor = 1.0;
            if (bandwidth_measurer_->get_counter_selection().type == CounterType::UPI_FLITS) {
                auto arch = ArchitectureRegistry::instance().getArchitecture(caps_);
                if (arch) {
                    scaling_factor = arch->getUpiScalingFactor(caps_);
                } else {
                    scaling_factor = 1.0 / 9.0;
                }
                measured_bw_gb_s = (bw_cas_rd + bw_cas_wr) * cache_line_size_ * scaling_factor / (bw_elapsed * 1e9);
            } else if (bandwidth_measurer_->get_counter_selection().type == CounterType::NVIDIA_GRACE) {
                double bytes_rd = static_cast<double>(bw_cas_rd) * 32.0;
                double bytes_wr = static_cast<double>(bw_cas_wr);
                measured_bw_gb_s = (bytes_rd + bytes_wr) / (bw_elapsed * 1e9);
            } else {
                measured_bw_gb_s = (bw_cas_rd + bw_cas_wr) * cache_line_size_ * scaling_factor / (bw_elapsed * 1e9);
            }
            result.bandwidth_mbps = measured_bw_gb_s * 1000.0;
        }
        
        if (config_.verbosity >= 2) {
            std::cout << "        Measured Bandwidth: " << std::fixed << std::setprecision(2) 
                      << measured_bw_gb_s << " GB/s" << std::endl;
            std::cout << "           (CAS rd: " << bw_cas_rd << ", wr: " << bw_cas_wr 
                      << ", elapsed: " << std::fixed << std::setprecision(2) << bw_elapsed << "s)" << std::endl;
        }
    
        if (total_accesses > 0 && aggregate_duration > 0) {
            double measured_freq_ghz = total_cycles / (aggregate_duration * 1e9);
            result.latency_ns = calculate_latency_from_perf_values(total_cycles, total_accesses, 
                                                                   total_tlb1miss, total_tlb2miss, 
                                                                   aggregate_duration, using_hugepages);
            
            if (config_.verbosity >= 3) {
                std::cout << "        Measured Latency: " << std::fixed << std::setprecision(2)
                          << result.latency_ns << " ns" << std::endl;
                std::cout << "           (aggregated over " << LAT_REQUIRED_STABLE_COUNT
                          << " bursts = " << std::fixed << std::setprecision(2)
                          << aggregate_duration << "s, cycles: "
                          << std::fixed << std::setprecision(0) << total_cycles
                          << ", instr: " << total_instructions
                          << ", tlb1miss: " << total_tlb1miss
                          << ", tlb2miss: " << total_tlb2miss
                          << ", Freq: " << std::fixed << std::setprecision(2)
                          << measured_freq_ghz << " GHz)" << std::endl;
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
        
        if (config_.verbosity >= 2) {
            std::cout << "    Cleaning up processes...";
            std::cout << "\n" << std::endl;
        }
        
        system("pkill -9 traffic_gen_multiseq.x 2>/dev/null");
        system("killall -9 traffic_gen_multiseq.x 2>/dev/null");
        
        int cleanup_wait = 0;
        while (run_command_success("pgrep -x traffic_gen_multiseq.x >/dev/null 2>&1") && cleanup_wait < 20) {
            system("pkill -9 traffic_gen_multiseq.x 2>/dev/null");
            system("killall -9 traffic_gen_multiseq.x 2>/dev/null");
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
    if (interval_duration <= 0.0 || accesses <= 0.0) {
        return 0.0;
    }
    
    double cpu_freq_hz = cycles / interval_duration;
    double cpu_freq_ghz = cpu_freq_hz / 1e9;
    
    max_cpu_freq_ghz_ = std::max(max_cpu_freq_ghz_, cpu_freq_ghz);
    
    double stlb_hit_latency_ns = cached_tlb_hit_latency_ns_;
    double total_ns = interval_duration * 1e9;
    
    double pagewalk_ns = (cpu_freq_ghz > 0.0) ? (tlb2miss / cpu_freq_ghz) : 0.0;
    double stlb_ns = stlb_hit_latency_ns * tlb1miss;
    
    double effective_ns = total_ns - pagewalk_ns;
    
    if (!using_hugepages) {
        effective_ns -= stlb_ns;
    }

    return effective_ns / accesses;
}

// Check for outliers and re-run if needed
bool BenchmarkExecutor::check_and_rerun_outliers(double ratio_pct, int pause, ExecutionMode mode) {  
    MeasurementStorage* measurement_storage_ = measurement_storage_multiseq_.get();
    OutlierDetector* outlier_detector_ = outlier_detector_multiseq_.get();

    std::string bw_file = measurement_storage_->bw_file_path(ratio_pct, pause);
    std::string lat_file = measurement_storage_->lat_file_path(ratio_pct, pause);
    
    BandwidthMeasurer* bandwidth_measurer_ = bandwidth_measurer_multiseq_.get();
    double scaling_factor = 1.0;
    if (bandwidth_measurer_->get_counter_selection().type == CounterType::UPI_FLITS) {
        auto arch = ArchitectureRegistry::instance().getArchitecture(caps_);
        if (arch) {
            scaling_factor = arch->getUpiScalingFactor(caps_);
        } else {
            scaling_factor = 1.0 / 9.0;
        }
    }
    
    std::vector<double> bw_measurements = outlier_detector_->load_bandwidth(ratio_pct, pause, scaling_factor);
    std::vector<double> lat_measurements = outlier_detector_->load_latency(ratio_pct, pause);
    
    if (bw_measurements.size() != static_cast<size_t>(config_.point_reps) || lat_measurements.size() != static_cast<size_t>(config_.point_reps)) {
        if (config_.verbosity >= 2) {
            std::cout << "  ⚠ Warning: Expected " << config_.point_reps << " measurements, got BW=" << bw_measurements.size() 
                      << ", LAT=" << lat_measurements.size() << std::endl;
        }
        size_t keep = std::min({static_cast<size_t>(config_.point_reps),
                                bw_measurements.size(),
                                lat_measurements.size()});
        if (keep > 0 &&
            outlier_detector_->align_measurement_files(ratio_pct, pause, keep) &&
            config_.verbosity >= 2) {
            std::cout << "    Aligned BW/LAT files to last " << keep << " measurements"
                      << std::endl;
        }
        return false;
    }
    
    if (config_.verbosity >= 2) {
        std::cout << "  Outlier Detection for " << ratio_pct << "% | " << pause << std::endl;
        std::cout << "    BW measurements: ";
        for (size_t i = 0; i < bw_measurements.size(); i++) {
            std::cout << std::fixed << std::setprecision(2) << bw_measurements[i] << " GB/s";
            if (i < bw_measurements.size() - 1) std::cout << ", ";
        }
        std::cout << std::endl;
        
        std::cout << "    Latency measurements: ";
        for (size_t i = 0; i < lat_measurements.size(); i++) {
            std::cout << std::fixed << std::setprecision(1) << lat_measurements[i] << " ns";
            if (i < lat_measurements.size() - 1) std::cout << ", ";
        }
        std::cout << std::endl;
    }
    
    std::vector<int> bw_outliers = outlier_detector_->find_outliers(bw_measurements, 10.0, "GB/s");
    std::vector<int> lat_outliers = outlier_detector_->find_outliers(lat_measurements, 10.0, "ns");
    
    std::set<int> reps_to_rerun;
    for (int idx : bw_outliers) {
        reps_to_rerun.insert(idx);
    }
    for (int idx : lat_outliers) {
        reps_to_rerun.insert(idx);
    }
    
    if (reps_to_rerun.empty()) {
        if (config_.verbosity >= 2) {
            std::cout << "    No outliers detected" << std::endl;
            
            if (!frequency_written_to_plotter_) {
                write_frequency_to_plotter(max_cpu_freq_ghz_);
                frequency_written_to_plotter_ = true;
            }

            std::cout << std::endl;
        }
        
        if (!bw_measurements.empty() && !lat_measurements.empty()) {
            double avg_bw = std::accumulate(bw_measurements.begin(), bw_measurements.end(), 0.0) /
                            bw_measurements.size();
            double avg_lat = std::accumulate(lat_measurements.begin(), lat_measurements.end(), 0.0) /
                             lat_measurements.size();
            progress_tracker_->update_ratio_stats(ratio_pct, avg_bw, avg_lat);
        }
        
        return false;
    }
    
    if (config_.verbosity >= 2) {
        std::cout << "    ⚠ Outliers detected in reps: ";
        for (int rep : reps_to_rerun) {
            std::cout << (rep + 1) << " ";
        }
        std::cout << "    → Re-running..." << std::endl;
    }
    
    int max_retries = 3;
    int retry_count = 0;
    
    while (!reps_to_rerun.empty() && retry_count < max_retries) {
        retry_count++;
        
        std::vector<int> outliers_vec(reps_to_rerun.begin(), reps_to_rerun.end());
        std::sort(outliers_vec.rbegin(), outliers_vec.rend());
        outlier_detector_->remove_measurements_by_indices(ratio_pct, pause, outliers_vec);
        if (config_.verbosity >= 2) {
            std::cout << "    Removed outlier measurements at indices: ";
            for (int rep : reps_to_rerun) {
                std::cout << (rep + 1) << " ";
            }
            std::cout << std::endl;
        }
        
        for (int rep : reps_to_rerun) {
            if (config_.verbosity >= 2) {
                std::cout << "\n  Retry " << retry_count << "/3: rep " << (rep + 1) << std::endl;
            }
            
            BenchmarkResult result;
            result.ratio_pct = ratio_pct;
            result.pause = pause;
            result.repetition = rep;
            
            if (run_single_benchmark(ratio_pct, pause, rep, result, mode) == BenchmarkStatus::SUCCESS) {
                std::string bw_file_retry = measurement_storage_->bw_file_path(result.ratio_pct, result.pause);
                BandwidthMeasurer* bandwidth_measurer_ = bandwidth_measurer_multiseq_.get();
                const auto& selection = bandwidth_measurer_->get_counter_selection();
                measurement_storage_->append_to_file_with_lock(bw_file_retry, format_bandwidth_output(result, selection));
                
                std::string lat_file_retry = measurement_storage_->lat_file_path(result.ratio_pct, result.pause);
                if (!measurement_storage_->append_to_file_with_lock(lat_file_retry, format_latency_output(result, ptrchase_manager_->pid()))) {
                    std::cerr << "  Failed to write latency data: " << lat_file_retry << std::endl;
                }
            }
        }
        
        bw_measurements = outlier_detector_->load_bandwidth(ratio_pct, pause, scaling_factor);
        lat_measurements = outlier_detector_->load_latency(ratio_pct, pause);
        
        if (config_.verbosity >= 2) {
            std::cout << "  Outlier Detection for " << ratio_pct << "% | " << pause << std::endl;
            std::cout << "    After retry " << retry_count << ":" << std::endl;
            std::cout << "      BW (last " << config_.point_reps << "): ";
            for (size_t i = 0; i < bw_measurements.size(); i++) {
                std::cout << std::fixed << std::setprecision(2) << bw_measurements[i] << " GB/s";
                if (i < bw_measurements.size() - 1) std::cout << ", ";
            }
            std::cout << std::endl;
            std::cout << "      LAT (last " << config_.point_reps << "): ";
            for (size_t i = 0; i < lat_measurements.size(); i++) {
                std::cout << std::fixed << std::setprecision(1) << lat_measurements[i] << " ns";
                if (i < lat_measurements.size() - 1) std::cout << ", ";
            }
            std::cout << std::endl;
        }
        
        bw_outliers = outlier_detector_->find_outliers(bw_measurements, 10.0, "GB/s");
        lat_outliers = outlier_detector_->find_outliers(lat_measurements, 10.0, "ns");
        
        reps_to_rerun.clear();
        for (int idx : bw_outliers) {
            reps_to_rerun.insert(idx);
        }
        for (int idx : lat_outliers) {
            reps_to_rerun.insert(idx);
        }
        
        if (reps_to_rerun.empty()) {
            if (config_.verbosity >= 2) {
                std::cout << "    Measurements now consistent" << std::endl;
                
                if (!frequency_written_to_plotter_) {
                    write_frequency_to_plotter(max_cpu_freq_ghz_);
                    frequency_written_to_plotter_ = true;
                }

                std::cout << std::endl;
            }
            break;
        } else if (config_.verbosity >= 2) {
            std::cout << "    Still have outliers in reps: ";
            for (int rep : reps_to_rerun) {
                std::cout << (rep + 1) << " ";
            }
            std::cout << std::endl;
        }
    }
    
    std::vector<double> final_bw = outlier_detector_->load_bandwidth(ratio_pct, pause, scaling_factor);
    std::vector<double> final_lat = outlier_detector_->load_latency(ratio_pct, pause);
    
    if (final_bw.size() > static_cast<size_t>(config_.point_reps) ||
        final_lat.size() > static_cast<size_t>(config_.point_reps)) {
        outlier_detector_->trim_to_last_measurements(ratio_pct, pause, config_.point_reps);
        if (config_.verbosity >= 2) {
            std::cout << "    Files cleaned: exactly " << config_.point_reps
                      << " measurements retained" << std::endl;
        }
    }
    
    if (!reps_to_rerun.empty()) {
        if (config_.verbosity >= 2) {
            std::cout << "    ⚠ Warning: Could not eliminate all outliers after " << max_retries << " retries" << std::endl;
            std::cout << "    → Deleting files and restarting point from scratch..." << std::endl;
        }
        
        std::remove(bw_file.c_str());
        std::remove(lat_file.c_str());
        
        return true;
    }
    
    return false;
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
        
        while (std::getline(current_file, line)) {
            content += line + "\n";
            if (line.find("CPU_FREQ=") == 0) {
                cpu_freq_exists = true;
                break;
            }
        }
        current_file.close();
        
        if (!cpu_freq_exists) {
            if (!content.empty() && content.back() == '\n') {
                content.pop_back();
            }
            
            content += "\nCPU_FREQ=" + std::to_string(cpu_freq_ghz) + "\n";
            
            std::ofstream out_file(plotter_file);
            out_file << content;
            out_file.close();
        }
        
        flock(fd, LOCK_UN);
        close(fd);
    };
    
    update_plotter(config_.output_root + "/multisequential/plotter.txt");
    
}


