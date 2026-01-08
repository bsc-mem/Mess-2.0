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

#include "measurement/bw_measurers/PerfBandwidthMeasurer.h"
#include "utils.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <thread>
#include <deque>
#include <iomanip>
#include <chrono>
#include <map>
#include <sys/select.h>
#include <sys/time.h>

bool PerfBandwidthMeasurer::sample_bandwidth(long long& cas_rd, long long& cas_wr, double& elapsed, const std::vector<int>& mem_nodes) const {
    int interval = static_cast<int>(sampling_interval_ms_);
    
    std::string membind_arg;
    if (!mem_nodes.empty()) {
        for (size_t i = 0; i < mem_nodes.size(); ++i) {
            membind_arg += std::to_string(mem_nodes[i]);
            if (i < mem_nodes.size() - 1) membind_arg += ",";
        }
    } else {
        membind_arg = "0";
    }

    BandwidthCounterSelection selection = counter_selection_;
    
    if (!selection.is_valid()) {
        int src_cpu = 0; 
        selection = PerformanceCounterStrategy::discoverBandwidthCounters(src_cpu, mem_nodes);
    }
    
    if (!selection.is_valid()) {
        std::cerr << "Error: No suitable bandwidth counters available." << std::endl;
        if (!selection.failure_reason.empty()) {
            std::cerr << "Reason: " << selection.failure_reason << std::endl;
        }
        return false;
    }

    std::string events_str = selection.get_all_events_string();

    std::string cmd = std::string("cd ") + storage_->bandwidth_dir() +
                      " && stdbuf -oL -eL numactl -C 0 --membind=" + membind_arg +
                      " perf stat -I " + std::to_string(interval) +
                      " -a --per-socket -x, -e " + events_str + " 2>&1";

    FILE* pipe = popen(cmd.c_str(), "r");
    if (!pipe) {
        return false;
    }

    setbuf(pipe, NULL);

    char buffer[4096];
    long long current_sample_rd = 0, current_sample_wr = 0, current_sample_combined = 0;
    long long sum_rd = 0, sum_wr = 0, sum_combined = 0;
    double last_timestamp = -1.0;
    double accumulated_time = 0.0;
    int samples_read = 0;
    int total_samples_seen = 0;
    const int samples_to_read = 2;
    const int warmup_samples = selection.extra_counters.empty() ? 0 : 1;
    bool using_combined = selection.cas.has_combined_counter && !selection.cas.has_read_write;

    while (samples_read < samples_to_read && fgets(buffer, sizeof(buffer), pipe)) {
        if (buffer[0] == '#' || strstr(buffer, "time") || strlen(buffer) < 5) {
            continue;
        }

        double timestamp;
        char socket[16];
        int core_count;
        long long value;
        char event_name[256];

        if (sscanf(buffer, "%lf,%15[^,],%d,%lld,,%255[^,]", &timestamp, socket, &core_count, &value, event_name) == 5) {
            if (strcmp(socket, "S1") == 0) {
                continue;
            }

            if (last_timestamp >= 0 && timestamp != last_timestamp) {
                double duration = timestamp - last_timestamp;
                if (duration < 0.000001) duration = interval / 1000.0;

                total_samples_seen++;
                bool sample_valid = false;
                if (using_combined) {
                    sample_valid = (current_sample_combined > 0);
                } else {
                    sample_valid = (current_sample_rd > 0 && current_sample_wr > 0);
                }
                if (sample_valid && total_samples_seen > warmup_samples) {
                    sum_rd += current_sample_rd;
                    sum_wr += current_sample_wr;
                    sum_combined += current_sample_combined;
                    accumulated_time += duration;
                    samples_read++;
                }
                current_sample_rd = 0;
                current_sample_wr = 0;
                current_sample_combined = 0;
                if (samples_read >= samples_to_read) {
                    break;
                }
            }

            bool is_read = false;
            bool is_write = false;
            bool is_combined = false;

            bool is_extra_counter = false;
            for (const auto& extra : selection.extra_counters) {
                if (strstr(event_name, extra.c_str())) {
                    is_extra_counter = true;
                    break;
                }
            }

            if (!is_extra_counter) {
                if (selection.type == CounterType::CAS_COUNT || selection.type == CounterType::NVIDIA_GRACE) {
                    for (const auto& e : selection.cas.combined_events) {
                        if (strstr(event_name, e.c_str())) { is_combined = true; break; }
                    }
                    
                    if (!is_combined) {
                        for (const auto& e : selection.cas.read_events) {
                            if (strstr(event_name, e.c_str())) { is_read = true; break; }
                        }
                        if (!is_read) {
                            std::string evt_lower = event_name;
                            std::transform(evt_lower.begin(), evt_lower.end(), evt_lower.begin(), ::tolower);
                            if (evt_lower.find("cas_count") != std::string::npos && 
                               (evt_lower.find("rd") != std::string::npos || evt_lower.find("read") != std::string::npos)) {
                                is_read = true;
                            }
                            if (evt_lower.find("nvidia_scf_pmu") != std::string::npos && 
                                (evt_lower.find("cmem_rd_data") != std::string::npos ||
                                 evt_lower.find("remote_socket_rd_data") != std::string::npos)) {
                                is_read = true;
                            }
                        }
                        
                        for (const auto& e : selection.cas.write_events) {
                            if (strstr(event_name, e.c_str())) { is_write = true; break; }
                        }
                        if (!is_write) {
                            std::string evt_lower = event_name;
                            std::transform(evt_lower.begin(), evt_lower.end(), evt_lower.begin(), ::tolower);
                            if (evt_lower.find("cas_count") != std::string::npos && 
                               (evt_lower.find("wr") != std::string::npos || evt_lower.find("write") != std::string::npos)) {
                                is_write = true;
                            }
                            if (evt_lower.find("nvidia_scf_pmu") != std::string::npos && 
                                (evt_lower.find("cmem_wr_total_bytes") != std::string::npos ||
                                 evt_lower.find("remote_socket_wr_total_bytes") != std::string::npos)) {
                                is_write = true;
                            }
                        }
                    }
                } else if (selection.type == CounterType::UPI_FLITS) {
                    for (const auto& e : selection.upi.rxl_data_events) {
                        if (strstr(event_name, e.c_str())) { is_read = true; break; }
                    }
                    for (const auto& e : selection.upi.txl_data_events) {
                        if (strstr(event_name, e.c_str())) { is_write = true; break; }
                    }
                }

                if (is_combined) {
                    current_sample_combined += value;
                } else if (is_read) {
                    current_sample_rd += value;
                } else if (is_write) {
                    current_sample_wr += value;
                }
            }

            last_timestamp = timestamp;
        }
    }

    bool sample_valid = false;
    if (using_combined) {
        sample_valid = (current_sample_combined > 0);
    } else {
        sample_valid = (current_sample_rd > 0 && current_sample_wr > 0);
    }
    if (samples_read < samples_to_read && sample_valid) {
        sum_rd += current_sample_rd;
        sum_wr += current_sample_wr;
        sum_combined += current_sample_combined;
        samples_read++;
    }

    pclose(pipe);

    bool success = false;
    if (using_combined) {
        success = (samples_read > 0 && sum_combined > 0);
    } else {
        success = (samples_read > 0 && sum_rd > 0 && sum_wr > 0);
    }

    if (success) {
        if (using_combined) {
            cas_rd = sum_combined;
            cas_wr = 0;
        } else {
            cas_rd = sum_rd;
            cas_wr = sum_wr;
        }
        elapsed = accumulated_time;
        return true;
    }

    return false;
}

#include "measurement/BandwidthStabilizer.h"

bool PerfBandwidthMeasurer::wait_for_stabilization(int& samples_taken, long long& last_cas_rd, long long& last_cas_wr, double& last_elapsed, int pause, int ratio, bool fast_resume, std::function<void()> on_sample) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    int interval = static_cast<int>(sampling_interval_ms_);

    std::vector<int> mem_nodes = numa_resolver_ ? numa_resolver_() : std::vector<int>{0};
    std::string membind_arg;
    if (!mem_nodes.empty()) {
        for (size_t i = 0; i < mem_nodes.size(); ++i) {
            membind_arg += std::to_string(mem_nodes[i]);
            if (i < mem_nodes.size() - 1) membind_arg += ",";
        }
    } else {
        membind_arg = "0";
    }

    BandwidthCounterSelection selection = counter_selection_;
    
    if (!selection.is_valid()) {
        int src_cpu = 0; 
        selection = PerformanceCounterStrategy::discoverBandwidthCounters(src_cpu, mem_nodes);
    }

    if (!selection.is_valid()) {
        return false;
    }
    std::string events_str = selection.get_all_events_string();

    std::string cmd = std::string("cd ") + storage_->bandwidth_dir() +
                      " && stdbuf -oL -eL numactl -C 0 --membind=" + membind_arg +
                      " perf stat -I " + std::to_string(interval) +
                      " -a --per-socket -x, -e " + events_str + " 2>&1";

    int relaunch_attempts = 0;

    auto relaunch_current_traffic_gen = [&]() -> bool {
        int traffic_gen_cores = sys_info_.sockets[0].core_count - 1;
        if (config_.traffic_gen_cores > 0 && config_.traffic_gen_cores <= sys_info_.sockets[0].core_count - 1) {
            traffic_gen_cores = config_.traffic_gen_cores;
        }
        if (traffic_gen_cores <= 0 || traffic_gen_cores > 1024) {
            std::cerr << "ERROR: Invalid traffic_gen_cores in relaunch: " << traffic_gen_cores << std::endl;
            return false;
        }
        return this->relaunch_traffic_gen(ratio, pause, traffic_gen_cores);
    };

    if (!traffic_gen_manager_->is_traffic_gen_running(traffic_gen_manager_->active_traffic_gen_pid())) {
        if (config_.verbosity >= 2) {
            std::cout << "    TrafficGen not running, relaunching..." << std::endl;
        }
        if (!relaunch_current_traffic_gen()) {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        relaunch_attempts++;
    }

    FILE* pipe = popen(cmd.c_str(), "r");
    if (!pipe) {
        if (config_.verbosity >= 2) {
            std::cout << "    Warning: perf stat failed, falling back to configured wait time" << std::endl;
        }
        return false;
    }

    setbuf(pipe, NULL);
    int pipe_fd = fileno(pipe);

    BandwidthStabilizer stabilizer(fast_resume ? 5 : 7);
    
    int total_samples = 0;
    const int OVERALL_TIMEOUT_SECONDS = 60;
    int max_samples = static_cast<int>((OVERALL_TIMEOUT_SECONDS * 1000.0) / sampling_interval_ms_);
    
    char buffer[4096];
    bool success = false;
    
    long long sample_cas_rd = 0, sample_cas_wr = 0;
    double last_timestamp = -1.0;

    long long final_cas_rd = 0, final_cas_wr = 0;
    double final_elapsed = sampling_interval_ms_ / 1000.0;

    std::map<std::string, long long> sample_extra_counters;
    std::map<std::string, long long> aggregate_extra_counters;
    extra_perf_values_.clear();

    auto loop_start = std::chrono::steady_clock::now();
    const int READ_TIMEOUT_SECONDS = 10;
    int consecutive_timeouts = 0;

    while (total_samples < max_samples) {
        auto elapsed_total = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - loop_start).count();
        
        if (elapsed_total > OVERALL_TIMEOUT_SECONDS) {
            std::cerr << "\n    ERROR: Perf stabilization timed out after " << OVERALL_TIMEOUT_SECONDS << " seconds." << std::endl;
            if (config_.verbosity >= 3) {
                std::cerr << "    Possible causes:\n"
                          << "      - perf stat command hung\n"
                          << "      - Performance counters unavailable\n"
                          << "      - System overloaded\n"
                          << "    Command: " << cmd << std::endl;
            }
            pclose(pipe);
            traffic_gen_manager_->kill_all_traffic_gen();
            return false;
        }

        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(pipe_fd, &read_fds);
        
        struct timeval timeout;
        timeout.tv_sec = READ_TIMEOUT_SECONDS;
        timeout.tv_usec = 0;
        
        int ready = select(pipe_fd + 1, &read_fds, NULL, NULL, &timeout);
        
        if (ready < 0) {
            std::cerr << "    ERROR: select() failed on perf pipe" << std::endl;
            pclose(pipe);
            return false;
        }
        
        if (ready == 0) {
            consecutive_timeouts++;
            if (config_.verbosity >= 2) {
                std::cout << "    Warning: No perf output for " << READ_TIMEOUT_SECONDS << " seconds (attempt " 
                          << consecutive_timeouts << "/3)" << std::endl;
            }
            
            if (consecutive_timeouts >= 3) {
                std::cerr << "\n    ERROR: Perf not producing output." << std::endl;
                if (config_.verbosity >= 3) {
                    std::cerr << "    Possible causes:\n"
                              << "      - perf_event_paranoid too restrictive\n"
                              << "      - Uncore counters require root access\n"
                              << "      - perf stat command syntax error\n"
                              << "    Command: " << cmd << std::endl;
                }
                pclose(pipe);
                traffic_gen_manager_->kill_all_traffic_gen();
                return false;
            }
            continue;
        }
        
        consecutive_timeouts = 0;
        
        if (!fgets(buffer, sizeof(buffer), pipe)) {
            break;
        }
        if (buffer[0] == '#' || strstr(buffer, "time") || strlen(buffer) < 5) {
            continue;
        }

        double timestamp;
        char socket[16];
        int core_count;
        long long value;
        char event_name[256];

        if (sscanf(buffer, "%lf,%15[^,],%d,%lld,,%255[^,]", &timestamp, socket, &core_count, &value, event_name) == 5) {
            if (strcmp(socket, "S1") == 0) {
                continue;
            }

            if (last_timestamp >= 0 && timestamp != last_timestamp) {
                double duration = timestamp - last_timestamp;
                double target_duration = sampling_interval_ms_ / 1000.0;
                double normalization_factor = 1.0;
                
                if (duration > 0.000001) {
                    normalization_factor = target_duration / duration;
                }

                if (sample_cas_rd + sample_cas_wr > 0) {
                    long long norm_rd = static_cast<long long>(sample_cas_rd * normalization_factor);
                    long long norm_wr = static_cast<long long>(sample_cas_wr * normalization_factor);
                    long long total_cas = norm_rd + norm_wr;
                    
                    total_samples++;

                    if (on_sample) {
                        on_sample();
                    }

                    if (total_samples % 20 == 0 &&
                        !traffic_gen_manager_->is_traffic_gen_running(traffic_gen_manager_->active_traffic_gen_pid())) {
                        if (relaunch_attempts < 2) {
                            if (config_.verbosity >= 2) {
                                std::cout << "    TrafficGen died during stabilization, relaunching..." << std::endl;
                            }
                            if (relaunch_current_traffic_gen()) {
                                relaunch_attempts++;
                                stabilizer.reset();
                                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                                continue;
                            } else {
                                pclose(pipe);
                                return false;
                            }
                        } else {
                            pclose(pipe);
                            return false;
                        }
                    }

                    if (total_samples > max_samples) {
                        if (config_.verbosity >= 1) {
                            std::cout << "\n    ⚠ WARNING: BW did not stabilize after " << OVERALL_TIMEOUT_SECONDS << "s" << std::endl;
                            std::cout << "    Killing TrafficGen and triggering retry..." << std::endl;
                        }
                        pclose(pipe);
                        traffic_gen_manager_->kill_all_traffic_gen();
                        std::this_thread::sleep_for(std::chrono::milliseconds(200));

                        samples_taken = total_samples;
                        return false;
                    }

                    final_cas_rd = norm_rd;
                    final_cas_wr = norm_wr;
                    final_elapsed = target_duration;

                    stabilizer.add_sample(norm_rd, norm_wr);

                    for (const auto& kv : sample_extra_counters) {
                        aggregate_extra_counters[kv.first] += static_cast<long long>(kv.second * normalization_factor);
                    }

                    if (config_.verbosity >= 3) {
                        double read_ratio = 0.0;
                        if (total_cas > 0) {
                            read_ratio = static_cast<double>(norm_rd) / static_cast<double>(total_cas);
                        }
                        
                        std::cout << "      [Sample " << total_samples << "] ";
                        if (selection.type == CounterType::CAS_COUNT || selection.type == CounterType::NVIDIA_GRACE) {
                             std::cout << "CAS: " << total_cas;
                        } else if (selection.type == CounterType::UPI_FLITS) {
                             std::cout << "UPI FLITS: " << total_cas;
                        }
                        stabilizer.print_status(read_ratio);
                    }

                    if (stabilizer.is_stable()) {
                        success = true;
                        break;
                    }

                    if (total_samples > 3 && total_cas == 0) {
                        if (config_.verbosity >= 2) {
                            std::cout << "    Warning: Zero CAS counts detected, TrafficGen may have crashed" << std::endl;
                        }
                        break;
                    }
                }
                sample_cas_rd = 0;
                sample_cas_wr = 0;
                for (auto& kv : sample_extra_counters) {
                    kv.second = 0;
                }
            }

            bool is_read = false;
            bool is_write = false;
            bool is_combined = false;

            if (selection.type == CounterType::CAS_COUNT || selection.type == CounterType::NVIDIA_GRACE) {
                for (const auto& e : selection.cas.combined_events) {
                    if (strstr(event_name, e.c_str())) { is_combined = true; break; }
                }
                
                if (!is_combined) {
                    for (const auto& e : selection.cas.read_events) {
                        if (strstr(event_name, e.c_str())) { is_read = true; break; }
                    }
                    if (!is_read) {
                        std::string evt_lower = event_name;
                        std::transform(evt_lower.begin(), evt_lower.end(), evt_lower.begin(), ::tolower);
                        if (evt_lower.find("cas_count") != std::string::npos && 
                           (evt_lower.find("rd") != std::string::npos || evt_lower.find("read") != std::string::npos)) {
                            is_read = true;
                        }
                        if (evt_lower.find("nvidia_scf_pmu") != std::string::npos && 
                            (evt_lower.find("cmem_rd_data") != std::string::npos ||
                             evt_lower.find("remote_socket_rd_data") != std::string::npos)) {
                            is_read = true;
                        }
                    }
                    
                    for (const auto& e : selection.cas.write_events) {
                        if (strstr(event_name, e.c_str())) { is_write = true; break; }
                    }
                    if (!is_write) {
                        std::string evt_lower = event_name;
                        std::transform(evt_lower.begin(), evt_lower.end(), evt_lower.begin(), ::tolower);
                        if (evt_lower.find("cas_count") != std::string::npos && 
                           (evt_lower.find("wr") != std::string::npos || evt_lower.find("write") != std::string::npos)) {
                            is_write = true;
                        }
                        if (evt_lower.find("nvidia_scf_pmu") != std::string::npos && 
                            (evt_lower.find("cmem_wr_total_bytes") != std::string::npos ||
                             evt_lower.find("remote_socket_wr_total_bytes") != std::string::npos)) {
                            is_write = true;
                        }
                    }
                }
            } else if (selection.type == CounterType::UPI_FLITS) {
                for (const auto& e : selection.upi.rxl_data_events) {
                    if (strstr(event_name, e.c_str())) { is_read = true; break; }
                }
                for (const auto& e : selection.upi.txl_data_events) {
                    if (strstr(event_name, e.c_str())) { is_write = true; break; }
                }
            }

            bool is_extra_counter = false;
            for (const auto& extra : selection.extra_counters) {
                if (strstr(event_name, extra.c_str())) {
                    sample_extra_counters[extra] += value;
                    is_extra_counter = true;
                    break;
                }
            }

            if (!is_extra_counter) {
                if (is_combined) {
                    sample_cas_rd += value;
                } else if (is_read) {
                    sample_cas_rd += value;
                } else if (is_write) {
                    sample_cas_wr += value;
                }
            }

            last_timestamp = timestamp;
        }
    }

    pclose(pipe);

    if (success) {
        long long aggregate_cas_rd = 0;
        long long aggregate_cas_wr = 0;
        const auto& samples = stabilizer.get_samples();
        for (const auto& sample : samples) {
            aggregate_cas_rd += sample.cas_rd;
            aggregate_cas_wr += sample.cas_wr;
        }
        last_cas_rd = aggregate_cas_rd;
        last_cas_wr = aggregate_cas_wr;
        last_elapsed = samples.size() * (sampling_interval_ms_ / 1000.0);

        for (const auto& kv : aggregate_extra_counters) {
            extra_perf_values_[kv.first] = kv.second;
        }

        if (config_.verbosity >= 3) {
             std::cout << "      [BW AGGREGATE] Using " << samples.size()
                       << " stable samples (" << last_elapsed << "s total)" << std::endl;
        }
        
        samples_taken = total_samples;
        return true;
    } else {
        last_cas_rd = final_cas_rd;
        last_cas_wr = final_cas_wr;
        last_elapsed = final_elapsed;
        samples_taken = total_samples;
        return false;
    }
}

#include "architecture/ArchitectureRegistry.h"
#include "architecture/PerformanceCounterStrategy.h"

bool PerfBandwidthMeasurer::monitor_command(const std::string& command, 
                                            std::function<void(double timestamp, double bw_gbps, long long raw_rd, long long raw_wr)> callback, 
                                            bool summary_mode) {
    
    std::vector<int> mem_nodes = numa_resolver_ ? numa_resolver_() : std::vector<int>{0};
    
    BandwidthCounterSelection selection = counter_selection_;
    if (!selection.is_valid()) {
        int src_cpu = 0; 
        selection = PerformanceCounterStrategy::discoverBandwidthCounters(src_cpu, mem_nodes);
    }
    
    if (!selection.is_valid()) {
        std::cerr << "Error: No suitable bandwidth counters available." << std::endl;
        return false;
    }

    std::string events_str = selection.get_all_events_string();

    std::stringstream perf_cmd;
    perf_cmd << "perf stat -a --per-socket -x, ";
    
    if (!summary_mode) {
        int interval_ms = static_cast<int>(sampling_interval_ms_);
        perf_cmd << "-I " << interval_ms << " ";
    }
    
    perf_cmd << "-e " << events_str << " ";
    perf_cmd << command;
    perf_cmd << " 2>&1";

    auto start_time = std::chrono::steady_clock::now();
    FILE* pipe = popen(perf_cmd.str().c_str(), "r");
    if (!pipe) {
        return false;
    }

    char buffer[4096];
    
    double current_timestamp = -1.0;
    long long current_rd = 0;
    long long current_wr = 0;
    bool has_data = false;

    int cache_line_size = 64;
    if (sys_info_.sockets[0].cache_count > 0) {
        cache_line_size = sys_info_.sockets[0].caches[0].line_size_bytes;
    }

#include "system_detection.h"

    double scaling_factor = 1.0;
    if (selection.type == CounterType::UPI_FLITS) {
        SystemDetector detector;
        detector.detect();
        auto caps = detector.get_capabilities();
        auto arch = ArchitectureRegistry::instance().getArchitecture(caps);
        if (arch) {
            scaling_factor = arch->getUpiScalingFactor(caps);
        } else {
            scaling_factor = 1.0 / 9.0;
        }
    }

    auto process_aggregation = [&](double timestamp, double duration_s) {
        double bw = 0.0;
        if (selection.type == CounterType::NVIDIA_GRACE) {
             double bytes_rd = static_cast<double>(current_rd) * 32.0;
             double bytes_wr = static_cast<double>(current_wr);
             bw = (bytes_rd + bytes_wr) / (duration_s * 1e9);
        } else {
             bw = (current_rd + current_wr) * cache_line_size * scaling_factor / (duration_s * 1e9);
        }
        
        if (callback) {
            callback(timestamp, bw, current_rd, current_wr);
        }
    };

    while (fgets(buffer, sizeof(buffer), pipe)) {
        if (buffer[0] == '#' || strstr(buffer, "time") || strlen(buffer) < 5) {
            continue;
        }

        float timestamp = 0.0f;
        char socket[16] = "";
        int core_count = 0;
        long long value = 0;
        char event_name[256] = "";

        if (!summary_mode) {
            if (sscanf(buffer, "%f,%15[^,],%d,%lld,,%255[^,]", &timestamp, socket, &core_count, &value, event_name) != 5) {
                continue;
            }
            if (strcmp(socket, "S1") == 0) {
                continue;
            }
        } else {
            if (sscanf(buffer, "%15[^,],%d,%lld,,%255[^,]", socket, &core_count, &value, event_name) != 4) {
                continue;
            }
            if (strcmp(socket, "S1") == 0) {
                continue;
            }
            timestamp = 0.0f;
        }

        if (!summary_mode) {
            if (std::abs(static_cast<double>(timestamp) - current_timestamp) > 0.0001) {
                if (has_data) {
                    double duration = sampling_interval_ms_ / 1000.0;
                    process_aggregation(current_timestamp, duration);
                }
                current_timestamp = static_cast<double>(timestamp);
                current_rd = 0;
                current_wr = 0;
                has_data = true;
            }
        } else {
            has_data = true;
        }

        bool is_read = false;
        bool is_write = false;

        std::string evt_lower = event_name;
        std::transform(evt_lower.begin(), evt_lower.end(), evt_lower.begin(), ::tolower);

        if (selection.type == CounterType::CAS_COUNT) {
            for (const auto& e : selection.cas.read_events) {
                if (evt_lower.find(e) != std::string::npos) { is_read = true; break; }
            }
            if (!is_read) {
                if (evt_lower.find("cas_count") != std::string::npos && 
                   (evt_lower.find("rd") != std::string::npos || evt_lower.find("read") != std::string::npos)) {
                    is_read = true;
                }
            }
            
            for (const auto& e : selection.cas.write_events) {
                if (evt_lower.find(e) != std::string::npos) { is_write = true; break; }
            }
            if (!is_write) {
                if (evt_lower.find("cas_count") != std::string::npos && 
                   (evt_lower.find("wr") != std::string::npos || evt_lower.find("write") != std::string::npos)) {
                    is_write = true;
                }
            }
        } else if (selection.type == CounterType::UPI_FLITS) {
            for (const auto& e : selection.upi.rxl_data_events) {
                if (evt_lower.find(e) != std::string::npos) { is_read = true; break; }
            }
            for (const auto& e : selection.upi.txl_data_events) {
                if (evt_lower.find(e) != std::string::npos) { is_write = true; break; }
            }
        } else if (selection.type == CounterType::NVIDIA_GRACE) {
            if (evt_lower.find("cmem_rd_data") != std::string::npos ||
                evt_lower.find("remote_socket_rd_data") != std::string::npos) is_read = true;
            else if (evt_lower.find("cmem_wr_total_bytes") != std::string::npos ||
                     evt_lower.find("remote_socket_wr_total_bytes") != std::string::npos) is_write = true;
        }

        if (is_read) current_rd += value;
        else if (is_write) current_wr += value;
    }

    auto end_time = std::chrono::steady_clock::now();
    double elapsed_seconds = std::chrono::duration<double>(end_time - start_time).count();

    if (has_data) {
        if (!summary_mode) {
             double duration = sampling_interval_ms_ / 1000.0;
             process_aggregation(current_timestamp, duration);
        } else {
             process_aggregation(0.0, elapsed_seconds);
        }
    }
    
    pclose(pipe);
    return true;
}
