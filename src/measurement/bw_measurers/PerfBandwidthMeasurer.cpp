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
#include "measurement/BandwidthStabilizer.h"
#include "architecture/ArchitectureRegistry.h"
#include "architecture/BandwidthCounterStrategy.h"
#include "system_detection.h"
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

namespace {
std::string make_membind_arg(const std::vector<int>& mem_nodes) {
    if (mem_nodes.empty()) {
        return "0";
    }
    std::string membind_arg;
    membind_arg.reserve(mem_nodes.size() * 4);
    for (size_t i = 0; i < mem_nodes.size(); ++i) {
        if (i > 0) membind_arg += ",";
        membind_arg += std::to_string(mem_nodes[i]);
    }
    return membind_arg;
}
}

bool PerfBandwidthMeasurer::sample_bandwidth(long long& cas_rd, long long& cas_wr, double& elapsed, const std::vector<int>& mem_nodes) const {
    return sample_with_popen(cas_rd, cas_wr, elapsed, mem_nodes);
}

bool PerfBandwidthMeasurer::sample_with_popen(long long& cas_rd, long long& cas_wr, double& elapsed, const std::vector<int>& mem_nodes) const {
    const bool verbose4 = config_.verbosity >= 4;

    int interval = static_cast<int>(sampling_interval_ms_);
    
    const std::string membind_arg = make_membind_arg(mem_nodes);

    const BandwidthCounterSelection& selection = counter_selection_;

    if (!selection.is_valid()) {
        std::cerr << "ERROR: Counter selection not initialized. "
                  << "BandwidthCounterStrategy must be initialized first." << std::endl;
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

    if(verbose4){std::cout << "[SAMPLING CMD] " << cmd << '\n';}

    FILE* pipe = popen(cmd.c_str(), "r");
    if (!pipe) {
        return false;
    }

    setbuf(pipe, NULL);

    char buffer[4096];
    long long current_sample_rd = 0, current_sample_wr = 0, current_sample_combined = 0;
    std::map<std::string, long long> current_extra;
    std::map<std::string, long long> sum_extra;
    long long sum_rd = 0, sum_wr = 0, sum_combined = 0;
    double last_timestamp = -1.0;
    double accumulated_time = 0.0;
    int samples_read = 0;
    int total_samples_seen = 0;
    const int samples_to_read = 2;
    const int warmup_samples = 1;
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
                    for (const auto& kv : current_extra) {
                        sum_extra[kv.first] += kv.second;
                    }    
                    accumulated_time += duration;
                    samples_read++;
                }
                current_sample_rd = 0;
                current_sample_wr = 0;
                current_sample_combined = 0;
                EventClassification::reset_extra_values(current_extra);
                if (samples_read >= samples_to_read) {
                    break;
                }
            }

            EventClassification ec = EventClassification::classify(event_name, selection);

            if (verbose4) {
                std::cout << "      [SAMPPLING EVENT] " << event_name << " -> rd:" << ec.is_read 
                          << " wr:" << ec.is_write << " comb:" << ec.is_combined << " extra: " << ec.is_extra
                          << " val:" << value << '\n';
            }

            if (ec.is_combined) {
                current_sample_combined += value;
            } else if (ec.is_read) {
                current_sample_rd += value;
            } else if (ec.is_write) {
                current_sample_wr += value;
            } else if (ec.is_extra) {
                EventClassification::accumulate_extra(current_extra, ec.extra_key, value);
            }
            last_timestamp = timestamp;
        }
    }

    bool sample_valid = using_combined 
        ? (current_sample_combined > 0) 
        : (current_sample_rd > 0 && current_sample_wr > 0);
    if (samples_read < samples_to_read && sample_valid) {
        sum_rd += current_sample_rd;
        sum_wr += current_sample_wr;
        sum_combined += current_sample_combined;
        for (const auto& kv : current_extra) {
            sum_extra[kv.first] += kv.second;
        }
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
        for (const auto& kv : sum_extra) {
            extra_perf_values_[kv.first] = kv.second;
        }
        return true;
    }

    return false;
}

bool PerfBandwidthMeasurer::wait_for_stabilization(int& samples_taken, long long& last_cas_rd, long long& last_cas_wr, double& last_elapsed, int pause, int ratio, bool fast_resume, std::function<void()> on_sample) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    const bool verbose2 = config_.verbosity >= 2;
    const bool verbose3 = config_.verbosity >= 3;
    const bool verbose4 = config_.verbosity >= 4;

    int interval = static_cast<int>(sampling_interval_ms_);
    std::vector<int> mem_nodes = numa_resolver_ ? numa_resolver_() : std::vector<int>{0};

    auto relaunch_current_traffic_gen = [&]() -> bool {
        if (!this->relaunch_traffic_gen(ratio, pause, get_traffic_gen_cores())) {
            return false;
        }
        traffic_gen_manager_->wait_for_traffic_gen_ready(120);
        return true;
    };

    int relaunch_attempts = 0;
    int traffic_gen_pid = traffic_gen_manager_->active_traffic_gen_pid();
    if (!traffic_gen_manager_->is_traffic_gen_running(traffic_gen_pid)) {
        if (verbose2) {
            std::cout << "    TrafficGen not running, relaunching..." << '\n';
        }
        if (!relaunch_current_traffic_gen()) {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        relaunch_attempts++;
        traffic_gen_pid = traffic_gen_manager_->active_traffic_gen_pid();
    }

    bool is_remote = (counter_selection_.type == CounterType::UPI_FLITS);
    int expected_cores = get_traffic_gen_cores();
    int socket_cores = (sys_info_.socket_count > 0) ? sys_info_.sockets[0].core_count : 0;
    
    double theoretical_peak_gb_s = TheoreticalPeakCalculator::calculate_achievable_peak(
        caps_, expected_cores, socket_cores, is_remote);

    if (theoretical_peak_gb_s <= 0) {
        theoretical_peak_gb_s = 300.0;
    }

    if (verbose3) {
        double full_peak = TheoreticalPeakCalculator::calculate_from_capabilities(caps_, is_remote);
        std::cout << "    [Stabilizer] Theoretical peak: " << std::fixed << std::setprecision(1) 
                  << full_peak << " GB/s, Achievable (" << expected_cores << " cores): "
                  << theoretical_peak_gb_s << " GB/s, Noise floor: " 
                  << TheoreticalPeakCalculator::get_noise_floor(theoretical_peak_gb_s) << " GB/s" << '\n';
    }
    TrafficGenHealthChecker health_checker;
    health_checker.is_pid_alive = [this](int pid) -> bool {
        return traffic_gen_manager_->is_traffic_gen_running(pid);
    };
    health_checker.count_running_instances = []() -> int {
        return TrafficGenHealthChecker::count_taskset_traffic_gen();
    };
    health_checker.expected_instance_count = expected_cores;

    ensure_scaling_factor_cached();
    int cache_line_size = cached_cache_line_size_;
    double scaling_factor = cached_scaling_factor_;

    auto calculate_bw_gb_s = [&](long long cas_rd, long long cas_wr, double elapsed_s) -> double {
        if (elapsed_s <= 0) return 0.0;
        if (counter_selection_.type == CounterType::NVIDIA_GRACE) {
            double bytes_rd = static_cast<double>(cas_rd) * 32.0;
            double bytes_wr = static_cast<double>(cas_wr);
            return (bytes_rd + bytes_wr) / (elapsed_s * 1e9);
        }
        if (counter_selection_.type == CounterType::UPI_FLITS) {
            return static_cast<double>(cas_rd + cas_wr) * cache_line_size * scaling_factor / (elapsed_s * 1e9);
        }
        return static_cast<double>(cas_rd + cas_wr) * cache_line_size * scaling_factor / (elapsed_s * 1e9);
    };

    const std::string membind_arg = make_membind_arg(mem_nodes);

    const BandwidthCounterSelection& selection = counter_selection_;
    if (!selection.is_valid()) {
        std::cerr << "ERROR: Counter selection not initialized. "
                  << "BandwidthCounterStrategy must be initialized first." << std::endl;
        return false;
    }
    std::string events_str = selection.get_bw_events_string();

    std::string cmd = std::string("cd ") + storage_->bandwidth_dir() +
                      " && stdbuf -oL -eL numactl -C 0 --membind=" + membind_arg +
                      " perf stat -I " + std::to_string(interval) +
                      " -a --per-socket -x, -e " + events_str + " 2>&1";

    FILE* pipe = popen(cmd.c_str(), "r");
    if (!pipe) {
        if (verbose2) {
            std::cout << "    Warning: perf stat failed, falling back to configured wait time" << '\n';
        }
        return false;
    }

    setbuf(pipe, NULL);
    int pipe_fd = fileno(pipe);

    BandwidthStabilizer stabilizer(
        theoretical_peak_gb_s, pause, health_checker,
        fast_resume ? 5 : 7, 0.05, config_.verbosity);
    
    int total_samples = 0;
    const int OVERALL_TIMEOUT_SECONDS = 60;
    int max_samples = static_cast<int>((OVERALL_TIMEOUT_SECONDS * 1000.0) / sampling_interval_ms_);
    
    char buffer[4096];
    bool success = false;
    
    long long sample_cas_rd = 0, sample_cas_wr = 0;
    double last_timestamp = -1.0;

    long long final_cas_rd = 0, final_cas_wr = 0;
    double final_elapsed = sampling_interval_ms_ / 1000.0;

    auto loop_start = std::chrono::steady_clock::now();
    const int READ_TIMEOUT_SECONDS = 10;
    int consecutive_timeouts = 0;

    while (total_samples < max_samples) {
        auto elapsed_total = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - loop_start).count();
        
        if (elapsed_total > OVERALL_TIMEOUT_SECONDS) {
            std::cerr << "\n    ERROR: Perf stabilization timed out after " << OVERALL_TIMEOUT_SECONDS << " seconds." << '\n';
            if (verbose3) {
                std::cerr << "    Possible causes:\n"
                          << "      - perf stat command hung\n"
                          << "      - Performance counters unavailable\n"
                          << "      - System overloaded\n"
                          << "    Command: " << cmd << '\n';
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
            std::cerr << "    ERROR: select() failed on perf pipe" << '\n';
            pclose(pipe);
            return false;
        }
        
        if (ready == 0) {
            consecutive_timeouts++;
            if (verbose2) {
                std::cout << "    Warning: No perf output for " << READ_TIMEOUT_SECONDS << " seconds (attempt " 
                          << consecutive_timeouts << "/3)" << '\n';
            }
            
            if (consecutive_timeouts >= 3) {
                std::cerr << "\n    ERROR: Perf not producing output." << '\n';
                if (verbose3) {
                    std::cerr << "    Possible causes:\n"
                              << "      - perf_event_paranoid too restrictive\n"
                              << "      - Uncore counters require root access\n"
                              << "      - perf stat command syntax error\n"
                              << "    Command: " << cmd << '\n';
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

        int parsed = sscanf(buffer, "%lf,%15[^,],%d,%lld,,%255[^,]", &timestamp, socket, &core_count, &value, event_name);
        
        if (verbose4) {
            std::cout << "      [PERF RAW] parsed=" << parsed << " line: " << buffer;
        }
        
        if (parsed == 5) {
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
                    long long total_cas_reg = sample_cas_rd + sample_cas_wr;
                    
                    total_samples++;

                    if (on_sample) {
                        on_sample();
                    }

                    if (total_samples % 20 == 0 &&
                        !traffic_gen_manager_->is_traffic_gen_running(traffic_gen_manager_->active_traffic_gen_pid())) {
                        if (relaunch_attempts < 2) {
                            if (verbose2) {
                                std::cout << "    TrafficGen died during stabilization, relaunching..." << '\n';
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
                            std::cout << "\n    ⚠ WARNING: BW did not stabilize after " << OVERALL_TIMEOUT_SECONDS << "s" << '\n';
                            std::cout << "    Killing TrafficGen and triggering retry..." << '\n';
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

                    double bw_gb_s = calculate_bw_gb_s(norm_rd, norm_wr, target_duration);
                    StabilizationResult result = stabilizer.add_sample(norm_rd, norm_wr, bw_gb_s);

                    if (result == StabilizationResult::ZOMBIE_DETECTED) {
                        if (verbose2) {
                            std::cout << "    [ZOMBIE] TrafficGen died, aborting stabilization" << '\n';
                        }
                        pclose(pipe);
                        samples_taken = total_samples;
                        return false;
                    }

                    if (verbose3) {
                        double read_ratio = 0.0;
                        if (selection.type == CounterType::NVIDIA_GRACE) {
                            double read_bytes = static_cast<double>(norm_rd) * 32.0;
                            double write_bytes = static_cast<double>(norm_wr);
                            double total_bytes = read_bytes + write_bytes;
                            if (total_bytes > 0) {
                                read_ratio = read_bytes / total_bytes;
                            }
                        } else {
                            if (total_cas_reg > 0) {
                                read_ratio = static_cast<double>(sample_cas_rd) / static_cast<double>(total_cas_reg);
                            }
                        }
                        
                        std::string counter_label = "CAS";
                        if (selection.type == CounterType::UPI_FLITS) counter_label = "UPI";
                        else if (selection.type == CounterType::NVIDIA_GRACE) counter_label = "GRACE";
                        
                        bool has_distinct_rw = selection.cas.has_read_write;
                        std::cout << "      [Sample " << total_samples << "] " << counter_label << ": " << total_cas;
                        stabilizer.print_status(read_ratio, bw_gb_s, has_distinct_rw);
                    }

                    if (stabilizer.is_stable()) {
                        success = true;
                        break;
                    }

                    if (total_samples > 3 && total_cas == 0) {
                        if (!health_checker.is_healthy()) {
                            if (config_.verbosity >= 2) {
                                std::cout << "    [ZOMBIE] Zero CAS + unhealthy traffic gen detected" << '\n';
                            }
                            pclose(pipe);
                            samples_taken = total_samples;
                            return false;
                        }
                    }
                }
                sample_cas_rd = 0;
                sample_cas_wr = 0;
            }

            EventClassification ec = EventClassification::classify(event_name, selection);

            if (verbose4) {
                std::cout << "      [EVENT] " << event_name << " -> rd:" << ec.is_read 
                          << " wr:" << ec.is_write << " comb:" << ec.is_combined 
                          << " val:" << value << '\n';
            }

            if (ec.is_combined || ec.is_read) {
                sample_cas_rd += value;
            } else if (ec.is_write) {
                sample_cas_wr += value;
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

        if (verbose3) {
             std::cout << "      [BW AGGREGATE] Using " << samples.size()
                       << " stable samples (" << last_elapsed << "s total)" << '\n';
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

bool PerfBandwidthMeasurer::monitor_command(const std::string& command, 
                                            MonitorCallback callback,
                                            bool summary_mode) {
    
    std::vector<int> mem_nodes = numa_resolver_ ? numa_resolver_() : std::vector<int>{0};

    const BandwidthCounterSelection& selection = counter_selection_;
    if (!selection.is_valid()) {
        std::cerr << "ERROR: Counter selection not initialized. "
                  << "BandwidthCounterStrategy must be initialized first." << std::endl;
        return false;
    }

    std::string events_str = selection.get_all_events_string();

    std::string trimmed_command = command;
    const auto first_non_ws = trimmed_command.find_first_not_of(" \t\r\n");
    if (first_non_ws == std::string::npos) {
        trimmed_command.clear();
    } else {
        trimmed_command.erase(0, first_non_ws);
    }
    const auto last_non_ws = trimmed_command.find_last_not_of(" \t\r\n");
    if (!trimmed_command.empty() && last_non_ws != std::string::npos) {
        trimmed_command.erase(last_non_ws + 1);
    }
    const bool pid_attach_mode =
        (trimmed_command.rfind("-p ", 0) == 0) ||
        (trimmed_command.rfind("--pid ", 0) == 0);

    std::stringstream perf_cmd;
    perf_cmd << "perf stat -x, ";
    if (!pid_attach_mode) {
        perf_cmd << "-a --per-socket ";
    }
    
    if (!summary_mode) {
        int interval_ms = static_cast<int>(sampling_interval_ms_);
        perf_cmd << "-I " << interval_ms << " ";
    }
    
    perf_cmd << "-e " << events_str << " ";
    perf_cmd << trimmed_command;
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
    std::map<std::string, long long> current_extra;
    std::map<std::string, long long> total_extra;
    bool has_data = false;

    ensure_scaling_factor_cached();

    auto process_aggregation = [&](double timestamp, double duration_s) {
        double bw = calculate_bandwidth_gbps(current_rd, current_wr, duration_s,
                                             selection.type, cached_cache_line_size_, cached_scaling_factor_);
        if (callback) {
            callback(timestamp, bw, current_rd, current_wr, current_extra);
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
                EventClassification::reset_extra_values(current_extra);
                has_data = true;
            }
        } else {
            has_data = true;
        }

        EventClassification ec = EventClassification::classify(event_name, selection);

        if (ec.is_extra) {
            EventClassification::accumulate_extra(current_extra, ec.extra_key, value);
            EventClassification::accumulate_extra(total_extra, ec.extra_key, value);
        } else if (ec.is_read || ec.is_combined) {
            current_rd += value;
        } else if (ec.is_write) {
            current_wr += value;
        }
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

    for (const auto& kv : total_extra) {
        extra_perf_values_[kv.first] = kv.second;
    }

    return true;
}
