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

#include "measurement/bw_measurers/LikwidBandwidthMeasurer.h"
#include "architecture/BandwidthCounterStrategy.h"
#include "architecture/ArchitectureRegistry.h"
#include "system_detection.h"
#include "utils.h"
#include "measurement/BandwidthStabilizer.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <thread>
#include <deque>
#include <regex>
#include <set>
#include <iomanip>
#include <chrono>
#include <sys/select.h>
#include <sys/time.h>

std::string LikwidBandwidthMeasurer::find_likwid_binary() const {
    if (!cached_likwid_binary_.empty()) {
        return cached_likwid_binary_;
    }

    if (const char* env_p = std::getenv("LIKWID_PERFCTR_PATH")) {
        cached_likwid_binary_ = std::string(env_p);
        return cached_likwid_binary_;
    }
    if (run_command_success("which likwid-perfctr > /dev/null 2>&1")) {
        cached_likwid_binary_ = "likwid-perfctr";
        return cached_likwid_binary_;
    }
    return "";
}

std::vector<LikwidBandwidthMeasurer::MemoryChannel> LikwidBandwidthMeasurer::parse_memory_counters(const std::string& output, const std::string& memType) const {
    std::set<int> channelIndices;
    std::regex counterRegex;
    
    if (memType == "HBM") {
        counterRegex = std::regex(R"(HBM(\d+)C\d+)");
    } else {
        counterRegex = std::regex(R"(MBOX(\d+)C\d+)");
    }
    
    std::sregex_iterator iter(output.begin(), output.end(), counterRegex);
    std::sregex_iterator end;
    
    while (iter != end) {
        int channelIdx = std::stoi((*iter)[1].str());
        channelIndices.insert(channelIdx);
        ++iter;
    }
    
    std::vector<MemoryChannel> channels;
    for (int idx : channelIndices) {
        MemoryChannel channel;
        channel.readCounter = memType + std::to_string(idx) + "C0";
        channel.writeCounter = memType + std::to_string(idx) + "C1";
        channels.push_back(channel);
    }
    
    return channels;
}

std::vector<LikwidBandwidthMeasurer::UpiChannel> LikwidBandwidthMeasurer::parse_upi_counters(const std::string& output) const {
    std::set<int> channelIndices;
    std::regex counterRegex(R"(UPI(\d+)C\d+)");
    
    std::sregex_iterator iter(output.begin(), output.end(), counterRegex);
    std::sregex_iterator end;
    
    while (iter != end) {
        int channelIdx = std::stoi((*iter)[1].str());
        channelIndices.insert(channelIdx);
        ++iter;
    }
    
    std::vector<UpiChannel> channels;
    for (int idx : channelIndices) {
        UpiChannel channel;
        channel.readCounter = "UPI" + std::to_string(idx) + "C0";
        channel.writeCounter = "UPI" + std::to_string(idx) + "C1";
        channels.push_back(channel);
    }
    
    return channels;
}

std::string LikwidBandwidthMeasurer::build_event_string(const std::string& likwidCmd, const std::string& memType, CounterType type) const {
    if (event_string_cache_.count(type)) {
        return event_string_cache_[type];
    }

    std::string command = "\"" + likwidCmd + "\" -e 2>&1";
    FILE* pipe = popen(command.c_str(), "r");
    if (!pipe) return "";
    
    std::string output;
    char buffer[4096];
    while (fgets(buffer, sizeof(buffer), pipe)) {
        output += buffer;
    }
    pclose_success(pipe);

    std::stringstream eventString;
    
    if (type == CounterType::UPI_FLITS) {
        auto channels = parse_upi_counters(output);
        if (channels.empty()) return "";
    
        for (const auto& channel : channels) {
            eventString << "RXL_FLITS_ALL_DATA:" << channel.readCounter << ",";
            eventString << "TXL_FLITS_ALL_DATA:" << channel.writeCounter << ",";
        }
    } else if (memType == "NVIDIA_GRACE" || type == CounterType::NVIDIA_GRACE) {
        eventString << "nvidia_scf_pmu_0/cmem_rd_data/,nvidia_scf_pmu_0/cmem_wr_total_bytes/";
        std::string res = eventString.str();
        event_string_cache_[type] = res;
        return res;
    } else if (memType == "AMD_DF") {
        std::set<int> dfIndices;
        std::regex dfRegex(R"(DF(\d+)C\d+)");
        std::sregex_iterator iter(output.begin(), output.end(), dfRegex);
        std::sregex_iterator end;
        while (iter != end) {
            int idx = std::stoi((*iter)[1].str());
            dfIndices.insert(idx);
            ++iter;
        }
        
        if (dfIndices.empty()) {
            return "";
        }
        
        for (int idx : dfIndices) {
            eventString << "DATA_FROM_LOCAL_DRAM:DF" << idx << "C0,";
        }
    } else {
        auto channels = parse_memory_counters(output, memType);
        if (channels.empty()) return "";

        bool has_instr = output.find("INSTR_RETIRED_ANY") != std::string::npos;
        bool has_core_clk = output.find("CPU_CLK_UNHALTED_CORE") != std::string::npos;
        bool has_ref_clk = output.find("CPU_CLK_UNHALTED_REF") != std::string::npos;

        if (has_instr) eventString << "INSTR_RETIRED_ANY:FIXC0,";
        if (has_core_clk) eventString << "CPU_CLK_UNHALTED_CORE:FIXC1,";
        if (has_ref_clk) eventString << "CPU_CLK_UNHALTED_REF:FIXC2,";

        for (const auto& channel : channels) {
            eventString << "CAS_COUNT_RD:" << channel.readCounter << ",";
            eventString << "CAS_COUNT_WR:" << channel.writeCounter << ",";
        }
    }
    
    std::string result = eventString.str();
    if (!result.empty() && result.back() == ',') {
        result.pop_back();
    }
    
    event_string_cache_[type] = result;
    return result;
}

std::pair<std::string, CounterType> LikwidBandwidthMeasurer::determine_memory_type() const {
    std::string memType = "MBOX";
    CounterType type = CounterType::CAS_COUNT;
    
    std::string tech(sys_info_.mem_technology);
    std::string vendor(sys_info_.cpu_vendor);
    std::string model(sys_info_.cpu_model);
    std::string arch(sys_info_.arch);
    
    if (tech == "HBM" || tech.find("HBM") != std::string::npos) {
        memType = "HBM";
    } else if (vendor.find("AMD") != std::string::npos) {
        memType = "AMD_DF";
    } else if (arch.find("aarch64") != std::string::npos || arch.find("arm64") != std::string::npos) {
        std::string model_upper = model;
        std::transform(model_upper.begin(), model_upper.end(), model_upper.begin(), ::toupper);
        if (model_upper.find("NEOVERSE-V2") != std::string::npos || model_upper.find("NEOVERSE V2") != std::string::npos) {
            memType = "NVIDIA_GRACE";
            type = CounterType::NVIDIA_GRACE;
        }
    }
    
    return {memType, type};
}

void LikwidBandwidthMeasurer::parse_likwid_header(const std::string& line, CounterType type,
                                                   std::vector<int>& rdIndices, std::vector<int>& wrIndices) const {
    rdIndices.clear();
    wrIndices.clear();
    
    std::stringstream ss(line);
    std::string token;
    int colIndex = 0;
    
    while (std::getline(ss, token, '|')) {
        if (type == CounterType::UPI_FLITS) {
            if (token.find("RXL_FLITS_ALL_DATA") != std::string::npos) {
                rdIndices.push_back(colIndex);
            } else if (token.find("TXL_FLITS_ALL_DATA") != std::string::npos) {
                wrIndices.push_back(colIndex);
            }
        } else if (type == CounterType::NVIDIA_GRACE) {
            std::string token_lower = token;
            std::transform(token_lower.begin(), token_lower.end(), token_lower.begin(), ::tolower);
            if (token_lower.find("cmem_rd_data") != std::string::npos || 
                token_lower.find("cmem_rd_bytes") != std::string::npos ||
                token_lower.find("rd_data") != std::string::npos) {
                rdIndices.push_back(colIndex);
            } else if (token_lower.find("cmem_wr_data") != std::string::npos ||
                       token_lower.find("cmem_wr_total_bytes") != std::string::npos ||
                       token_lower.find("cmem_wr_bytes") != std::string::npos ||
                       token_lower.find("wr_total_bytes") != std::string::npos) {
                wrIndices.push_back(colIndex);
            }
        } else {
            if (token.find("CAS_COUNT_RD") != std::string::npos) {
                rdIndices.push_back(colIndex);
            } else if (token.find("CAS_COUNT_WR") != std::string::npos) {
                wrIndices.push_back(colIndex);
            } else if (token.find("DATA_FROM_LOCAL_DRAM") != std::string::npos) {
                rdIndices.push_back(colIndex);
            }
        }
        colIndex++;
    }
}

bool LikwidBandwidthMeasurer::sample_bandwidth(long long& cas_rd, long long& cas_wr, double& elapsed, const std::vector<int>& /*mem_nodes*/) const {
    std::string likwidCmd = find_likwid_binary();
    if (likwidCmd.empty()) {
        std::cerr << "Error: likwid-perfctr not found." << std::endl;
        return false;
    }

    CounterType type = counter_selection_.type;
    auto [memType, baseType] = determine_memory_type();

    std::string eventString = build_event_string(likwidCmd, memType, type);
    if (eventString.empty()) {
        std::cerr << "Error: Failed to build likwid event string for " << (type == CounterType::UPI_FLITS ? "UPI" : memType) << std::endl;
        return false;
    }

    std::string intervalStr = std::to_string(static_cast<int>(sampling_interval_ms_)) + "ms";
    double sleep_sec = sampling_interval_ms_ / 1000.0;
    std::ostringstream sleep_ss;
    sleep_ss << std::fixed << std::setprecision(3) << sleep_sec;
    std::string cmd = "\"" + likwidCmd + "\" -f -c 0 -O -g " + eventString + " -t " + intervalStr + " sleep " + sleep_ss.str() + " 2>&1";

    FILE* pipe = popen(cmd.c_str(), "r");
    if (!pipe) return false;

    char buffer[4096];
    std::vector<std::string> lines;
    while (fgets(buffer, sizeof(buffer), pipe)) {
        lines.emplace_back(buffer);
    }
    pclose_success(pipe);

    long long total_rd = 0;
    long long total_wr = 0;
    bool found_data = false;
    std::vector<int> rdIndices;
    std::vector<int> wrIndices;
    bool headerParsed = false;

    for (const auto& line : lines) {
        if (line.find("# GID|") != std::string::npos) {
            parse_likwid_header(line, type, rdIndices, wrIndices);
            headerParsed = true;
        } else if (headerParsed && !line.empty() && std::isdigit(line[0])) {
            std::stringstream ss(line);
            std::string token;
            std::vector<std::string> values;
            while (std::getline(ss, token, ',')) {
                values.push_back(token);
            }
            
            for (int idx : rdIndices) {
                if (idx < static_cast<int>(values.size())) total_rd += std::stod(values[idx]);
            }
            for (int idx : wrIndices) {
                if (idx < static_cast<int>(values.size())) total_wr += std::stod(values[idx]);
            }
            found_data = true;
        }
    }

    if (found_data) {
        cas_rd = total_rd;
        cas_wr = total_wr;
        elapsed = sleep_sec;
        return true;
    }

    return false;
}

bool LikwidBandwidthMeasurer::wait_for_stabilization(int& samples_taken, long long& last_cas_rd, long long& last_cas_wr, double& last_elapsed, int pause, int ratio, bool fast_resume, std::function<void()> on_sample) {
    std::string likwidCmd = find_likwid_binary();
    if (likwidCmd.empty()) return false;

    std::vector<int> mem_nodes = numa_resolver_ ? numa_resolver_() : std::vector<int>{0};

    CounterType type = counter_selection_.type;
    auto [memType, baseType] = determine_memory_type();

    std::string eventString = build_event_string(likwidCmd, memType, type);
    if (eventString.empty()) return false;

    std::string intervalStr = std::to_string(static_cast<int>(sampling_interval_ms_)) + "ms";
    std::string cmd = "\"" + likwidCmd + "\" -f -c 0 -O -g " + eventString + " -t " + intervalStr + " sleep 3600 2>&1";

    int relaunch_attempts = 0;
    auto relaunch_current_traffic_gen = [&]() -> bool {
        if (!this->relaunch_traffic_gen(ratio, pause, get_traffic_gen_cores())) {
            return false;
        }
        traffic_gen_manager_->wait_for_traffic_gen_ready(120);
        return true;
    };

    if (!traffic_gen_manager_->is_traffic_gen_running(traffic_gen_manager_->active_traffic_gen_pid())) {
        if (!relaunch_current_traffic_gen()) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        relaunch_attempts++;
    }

    bool is_remote = (type == CounterType::UPI_FLITS);
    int expected_cores = get_traffic_gen_cores();
    int socket_cores = (sys_info_.socket_count > 0) ? sys_info_.sockets[0].core_count : 0;
    
    double theoretical_peak_gb_s = TheoreticalPeakCalculator::calculate_achievable_peak(
        caps_, expected_cores, socket_cores, is_remote);
    if (theoretical_peak_gb_s <= 0) theoretical_peak_gb_s = 300.0;
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
        if (type == CounterType::NVIDIA_GRACE) {
            double bytes_rd = static_cast<double>(cas_rd) * 32.0;
            double bytes_wr = static_cast<double>(cas_wr);
            return (bytes_rd + bytes_wr) / (elapsed_s * 1e9);
        } else {
            return static_cast<double>(cas_rd + cas_wr) * cache_line_size * scaling_factor / (elapsed_s * 1e9);
        }
    };

    FILE* pipe = popen(cmd.c_str(), "r");
    if (!pipe) return false;

    setbuf(pipe, NULL);
    int pipe_fd = fileno(pipe);

    size_t window_size = fast_resume ? 5 : (type == CounterType::UPI_FLITS ? 10 : 7);
    BandwidthStabilizer stabilizer(
        theoretical_peak_gb_s, pause, health_checker,
        window_size, 0.05, config_.verbosity);
    
    int total_samples = 0;
    int max_samples = static_cast<int>(200 * sampling_interval_ms_);

    char buffer[4096];
    bool success = false;
    
    std::vector<int> rdIndices;
    std::vector<int> wrIndices;
    bool headerParsed = false;

    auto loop_start = std::chrono::steady_clock::now();
    const int OVERALL_TIMEOUT_SECONDS = 60;
    const int READ_TIMEOUT_SECONDS = 10;
    int consecutive_timeouts = 0;

    while (total_samples < max_samples) {
        auto elapsed_total = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - loop_start).count();
        
        if (elapsed_total > OVERALL_TIMEOUT_SECONDS) {
            std::cerr << "\n    ERROR: Likwid stabilization timed out after " << OVERALL_TIMEOUT_SECONDS << " seconds." << '\n';
            if (config_.verbosity >= 3) {
                std::cerr << "    Possible causes:\n"
                          << "      - likwid-perfctr hung\n"
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
            std::cerr << "    ERROR: select() failed on likwid pipe" << '\n';
            pclose(pipe);
            return false;
        }
        
        if (ready == 0) {
            consecutive_timeouts++;
            if (config_.verbosity >= 2) {
                std::cout << "    Warning: No likwid output for " << READ_TIMEOUT_SECONDS << " seconds (attempt " 
                          << consecutive_timeouts << "/3)" << '\n';
            }
            
            if (consecutive_timeouts >= 3) {
                std::cerr << "\n    ERROR: Likwid not producing output." << '\n';
                if (config_.verbosity >= 3) {
                    std::cerr << "    Possible causes:\n"
                              << "      - likwid-perfctr requires root/setuid\n"
                              << "      - Counters not available for this architecture\n"
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
        
        std::string line(buffer);
        if (line.empty()) continue;

        if (line.find("# GID|") != std::string::npos) {
            parse_likwid_header(line, type, rdIndices, wrIndices);
            headerParsed = true;
            continue;
        }

        if (headerParsed && !line.empty() && std::isdigit(line[0])) {
            std::stringstream ss(line);
            std::string token;
            std::vector<std::string> values;
            while (std::getline(ss, token, ',')) {
                values.push_back(token);
            }

            long long sample_cas_rd = 0;
            long long sample_cas_wr = 0;

            for (int idx : rdIndices) {
                if (idx < static_cast<int>(values.size())) sample_cas_rd += std::stod(values[idx]);
            }
            for (int idx : wrIndices) {
                if (idx < static_cast<int>(values.size())) sample_cas_wr += std::stod(values[idx]);
            }

            if (sample_cas_rd > 1e15 || sample_cas_wr > 1e15) {
                if (config_.verbosity >= 3) {
                    std::cout << "      [Sample Ignored] Garbage values detected: RD=" << sample_cas_rd << ", WR=" << sample_cas_wr << '\n';
                }
                continue;
            }

            total_samples++;
            if (on_sample) on_sample();

            if (total_samples % 20 == 0 && !traffic_gen_manager_->is_traffic_gen_running(traffic_gen_manager_->active_traffic_gen_pid())) {
                if (relaunch_attempts < 2 && relaunch_current_traffic_gen()) {
                    relaunch_attempts++;
                    stabilizer.reset();
                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
                    continue;
                } else {
                    pclose_success(pipe);
                    return false;
                }
            }

            if (total_samples > 500) {
                pclose_success(pipe);
                traffic_gen_manager_->kill_all_traffic_gen();
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                samples_taken = total_samples;
                return false;
            }

            double sample_elapsed = sampling_interval_ms_ / 1000.0;
            double bw_gb_s = calculate_bw_gb_s(sample_cas_rd, sample_cas_wr, sample_elapsed);
            StabilizationResult result = stabilizer.add_sample(sample_cas_rd, sample_cas_wr, bw_gb_s);

            if (result == StabilizationResult::ZOMBIE_DETECTED) {
                if (config_.verbosity >= 2) {
                    std::cout << "    [ZOMBIE] TrafficGen died, aborting stabilization" << '\n';
                }
                pclose_success(pipe);
                samples_taken = total_samples;
                return false;
            }

            if (config_.verbosity >= 3) {
                    double read_ratio = 0.0;
                    long long total_cas = sample_cas_rd + sample_cas_wr;
                    if (type == CounterType::NVIDIA_GRACE) {
                        double read_bytes = static_cast<double>(sample_cas_rd) * 32.0;
                        double write_bytes = static_cast<double>(sample_cas_wr);
                        double total_bytes = read_bytes + write_bytes;
                        if (total_bytes > 0) {
                            read_ratio = read_bytes / total_bytes;
                        }
                    } else {
                        if (total_cas > 0) {
                            read_ratio = static_cast<double>(sample_cas_rd) / static_cast<double>(total_cas);
                        }
                    }
                    bool has_distinct_rw = counter_selection_.cas.has_read_write;
                    std::cout << "      [Sample " << total_samples << "] LIKWID " << (type == CounterType::UPI_FLITS ? "UPI FLITS" : "CAS") << ": " << total_cas
                              << " (RD: " << sample_cas_rd << ", WR: " << sample_cas_wr << ")";
                    stabilizer.print_status(read_ratio, -1.0, has_distinct_rw);
            }

            if (stabilizer.is_stable()) {
                success = true;
                break;
            }
        }
    }

    pclose_success(pipe);

    if (success) {
        long long aggregate_cas_rd = 0;
        long long aggregate_cas_wr = 0;
        const auto& samples = stabilizer.get_samples();
        for (const auto& sample : samples) {
            aggregate_cas_rd += sample.cas_rd;
            aggregate_cas_wr += sample.cas_wr;
        }
        
        if (type == CounterType::UPI_FLITS && (aggregate_cas_rd + aggregate_cas_wr) == 0) {
            std::cerr << "\n\033[1;31mERROR: Zero bandwidth detected on UPI counters!\033[0m" << '\n';
            std::cerr << "       The selected UPI counters are reporting 0 traffic." << '\n';
            std::cerr << "       This usually means either:" << '\n';
            std::cerr << "       1. The machine does not support these specific UPI counters." << '\n';
            std::cerr << "       2. Traffic is not flowing through the monitored UPI links." << '\n';
            std::cerr << "       3. Likwid is not configured correctly for this architecture." << '\n';
            std::cerr << "       Aborting benchmark to prevent invalid results." << '\n';
            pclose_success(pipe);
            return false;
        }

        last_cas_rd = aggregate_cas_rd;
        last_cas_wr = aggregate_cas_wr;
        last_elapsed = samples.size() * (sampling_interval_ms_ / 1000.0);
        
        samples_taken = total_samples;
        return true;
    }

    samples_taken = total_samples;
    return false;
}

bool LikwidBandwidthMeasurer::monitor_command(const std::string& command, 
                                              MonitorCallback callback,
                                              bool summary_mode) {
    std::string likwidCmd = find_likwid_binary();
    if (likwidCmd.empty()) {
        std::cerr << "Error: likwid-perfctr not found." << std::endl;
        return false;
    }

    std::vector<int> mem_nodes = numa_resolver_ ? numa_resolver_() : std::vector<int>{0};

    CounterType type = counter_selection_.type;
    auto [memType, baseType] = determine_memory_type();

    std::string eventString = build_event_string(likwidCmd, memType, type);
    if (eventString.empty()) {
        std::cerr << "Error: Failed to build likwid event string." << std::endl;
        return false;
    }

    int interval_ms = summary_mode ? 1000 : static_cast<int>(sampling_interval_ms_);
    std::string intervalStr = std::to_string(interval_ms) + "ms";

    std::string cmd = "\"" + likwidCmd + "\" -f -c 0 -O -g " + eventString + " -t " + intervalStr + " " + command + " 2>&1";

    auto start_time = std::chrono::steady_clock::now();
    FILE* pipe = popen(cmd.c_str(), "r");
    if (!pipe) return false;

    char buffer[4096];
    
    std::vector<int> rdIndices;
    std::vector<int> wrIndices;
    bool headerParsed = false;
    
    double current_timestamp = 0.0;
    
    long long total_rd = 0;
    long long total_wr = 0;
    double max_timestamp = 0.0;

    ensure_scaling_factor_cached();

    auto process_sample = [&](double timestamp, long long rd, long long wr) {
        double duration_s = interval_ms / 1000.0;
        double bw = calculate_bandwidth_gbps(rd, wr, duration_s, type, cached_cache_line_size_, cached_scaling_factor_);
        static const MonitorSampleExtras kEmptyExtras;
        if (callback) {
            callback(timestamp, bw, rd, wr, kEmptyExtras);
        }
    };

    int sample_count = 0;

    while (fgets(buffer, sizeof(buffer), pipe)) {
        std::string line(buffer);
        if (line.empty()) continue;

        if (line.find("# GID|") != std::string::npos) {
            parse_likwid_header(line, type, rdIndices, wrIndices);
            headerParsed = true;
            continue;
        }

        if (headerParsed && !line.empty() && std::isdigit(line[0])) {
            std::stringstream ss(line);
            std::string token;
            std::vector<std::string> values;
            while (std::getline(ss, token, ',')) {
                values.push_back(token);
            }

            long long sample_rd = 0;
            long long sample_wr = 0;

            for (int idx : rdIndices) {
                if (idx < static_cast<int>(values.size())) sample_rd += std::stod(values[idx]);
            }
            for (int idx : wrIndices) {
                if (idx < static_cast<int>(values.size())) sample_wr += std::stod(values[idx]);
            }
            
            sample_count++;
            current_timestamp = sample_count * (interval_ms / 1000.0);
            
            if (!summary_mode) {
                process_sample(current_timestamp, sample_rd, sample_wr);
            } else {
                total_rd += sample_rd;
                total_wr += sample_wr;
                max_timestamp = current_timestamp;
            }
        }
    }

    pclose(pipe);
    
    if (summary_mode) {
        auto end_time = std::chrono::steady_clock::now();
        double elapsed_seconds = std::chrono::duration<double>(end_time - start_time).count();
        
        double duration = (max_timestamp > 0) ? max_timestamp : elapsed_seconds;
        
        double bw = calculate_bandwidth_gbps(total_rd, total_wr, duration, type, cached_cache_line_size_, cached_scaling_factor_);
        static const MonitorSampleExtras kEmptyExtras;
        if (callback) {
            callback(duration, bw, total_rd, total_wr, kEmptyExtras);
        }
    }

    return true;
}
