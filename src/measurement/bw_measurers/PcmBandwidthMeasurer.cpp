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

#include "measurement/bw_measurers/PcmBandwidthMeasurer.h"
#include "measurement/BandwidthStabilizer.h"
#include "measurement/TheoreticalPeakCalculator.h"
#include "Utils.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <unistd.h>
#include <sys/wait.h>

namespace {

std::string trim_copy(const std::string& value) {
    const size_t first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return "";
    }
    const size_t last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

std::string shell_quote(const std::string& value) {
    std::string quoted;
    quoted.reserve(value.size() + 2);
    quoted.push_back('\'');
    for (char c : value) {
        if (c == '\'') {
            quoted += "'\\''";
        } else {
            quoted.push_back(c);
        }
    }
    quoted.push_back('\'');
    return quoted;
}

struct PcmCxlSample {
    double read_mbps = 0.0;
    double write_mbps = 0.0;
};

std::vector<std::string> split_csv_fields(const std::string& line) {
    std::vector<std::string> fields;
    std::istringstream ss(line);
    std::string field;
    while (std::getline(ss, field, ',')) {
        const size_t start = field.find_first_not_of(" \t\r\n");
        const size_t end = field.find_last_not_of(" \t\r\n");
        if (start == std::string::npos) {
            fields.push_back("");
        } else {
            fields.push_back(field.substr(start, end - start + 1));
        }
    }
    return fields;
}

struct PcmCsvColumnMap {
    int cxl_read_col = -1;
    std::vector<int> cxl_write_cols;
    int total_columns = 0;
};

bool find_cxl_csv_columns(const std::string& header_line, PcmCsvColumnMap& map, int verbosity) {
    auto fields = split_csv_fields(header_line);
    map.total_columns = static_cast<int>(fields.size());
    map.cxl_write_cols.clear();
    map.cxl_read_col = -1;

    bool past_first_socket_cxl = false;

    for (int i = 0; i < map.total_columns; i++) {
        const std::string& f = fields[i];

        if (f == "CXLRead") {
            map.cxl_read_col = i;
        }

        if (!past_first_socket_cxl &&
            f.find("CXL.mem_P") != std::string::npos &&
            f.find("Write") != std::string::npos) {
            map.cxl_write_cols.push_back(i);
        }

        if (!map.cxl_write_cols.empty() && f == "Ch0Read") {
            past_first_socket_cxl = true;
        }
    }

    if (verbosity >= 4) {
        std::cout << "      [PCM CSV] CXLRead col: " << map.cxl_read_col
                  << ", write cols: " << map.cxl_write_cols.size()
                  << ", total: " << map.total_columns << std::endl;
    }

    return map.cxl_read_col >= 0;
}

bool parse_pcm_cxl_samples(FILE* pipe,
                           int verbosity,
                           std::vector<PcmCxlSample>& samples_out,
                           size_t max_samples,
                           PcmCsvColumnMap& col_map,
                           bool& header_found) {
    setbuf(pipe, nullptr);

    char buffer[16384];
    int sample_count = 0;

    while (fgets(buffer, sizeof(buffer), pipe)) {
        std::string line(buffer);
        if (line.empty()) {
            continue;
        }

        if (line.find("Error") != std::string::npos ||
            line.find("Warning") != std::string::npos ||
            line.find("unsupported") != std::string::npos) {
            std::cerr << "[PCM_MSG] " << line;
        }

        if (verbosity >= 5) {
            std::cout << "[PCM RAW] " << line;
        }

        if (!header_found) {
            if (line.find("Date,Time,") != std::string::npos || line.find("CXLRead") != std::string::npos) {
                header_found = find_cxl_csv_columns(line, col_map, verbosity);
                if (!header_found && verbosity >= 2) {
                    std::cerr << "[PCM CSV] Header found but no CXLRead column" << std::endl;
                }
            }
            continue;
        }

        if (line.size() < 10 || line[4] != '-') {
            continue;
        }

        auto fields = split_csv_fields(line);
        if (static_cast<int>(fields.size()) < col_map.total_columns) {
            continue;
        }

        double read_mbps = 0.0;
        try {
            read_mbps = std::stod(fields[col_map.cxl_read_col]);
        } catch (...) {
            continue;
        }

        double write_mbps = 0.0;
        for (int col : col_map.cxl_write_cols) {
            try {
                write_mbps += std::stod(fields[col]);
            } catch (...) {}
        }

        samples_out.push_back(PcmCxlSample{read_mbps, write_mbps});
        sample_count++;

        if (verbosity >= 4) {
            std::cout << "      [PCM CSV SAMPLE " << sample_count << "] "
                      << "read=" << std::fixed << std::setprecision(2) << read_mbps << " MB/s"
                      << " write=" << write_mbps << " MB/s" << std::endl;
        }

        if (max_samples > 0 && samples_out.size() >= max_samples) {
            break;
        }
    }

    return sample_count > 0;
}

pid_t spawn_shell_command(const std::string& command) {
    pid_t pid = fork();
    if (pid < 0) {
        return -1;
    }
    if (pid == 0) {
        execl("/bin/sh", "sh", "-lc", command.c_str(), static_cast<char*>(nullptr));
        _exit(127);
    }
    return pid;
}

bool is_pid_attach_command(const std::string& command) {
    return command.rfind("-p ", 0) == 0 || command.rfind("--pid ", 0) == 0;
}

std::string find_executable_in_path(const std::string& binary) {
    const char* path_env = std::getenv("PATH");
    if (!path_env) {
        return "";
    }

    std::stringstream ss(path_env);
    std::string dir;
    while (std::getline(ss, dir, ':')) {
        if (dir.empty()) {
            continue;
        }
        std::filesystem::path candidate = std::filesystem::path(dir) / binary;
        if (access(candidate.c_str(), X_OK) == 0) {
            return candidate.string();
        }
    }

    return "";
}

} // namespace

std::string PcmBandwidthMeasurer::find_pcm_binary() const {
    if (!cached_pcm_binary_.empty()) {
        return cached_pcm_binary_;
    }

    if (const char* env_p = std::getenv("PCM_PATH")) {
        std::filesystem::path configured(env_p);
        if (std::filesystem::is_directory(configured)) {
            configured /= "pcm-memory";
        }
        if (access(configured.c_str(), X_OK) == 0) {
            cached_pcm_binary_ = configured.string();
            return cached_pcm_binary_;
        }
    }

    const std::string in_path = find_executable_in_path("pcm-memory");
    if (!in_path.empty()) {
        cached_pcm_binary_ = in_path;
        return cached_pcm_binary_;
    }

    const char* common_candidates[] = {
        "/usr/sbin/pcm-memory",
        "/usr/bin/pcm-memory",
        "/opt/pcm/pcm-memory"
    };
    for (const char* candidate : common_candidates) {
        if (access(candidate, X_OK) == 0) {
            cached_pcm_binary_ = candidate;
            return cached_pcm_binary_;
        }
    }

    return "";
}

bool PcmBandwidthMeasurer::sample_bandwidth(long long& cas_rd,
                                            long long& cas_wr,
                                            double& elapsed,
                                            const std::vector<int>& mem_nodes) const {
    (void)mem_nodes;
    extra_perf_values_.clear();

    const std::string pcm_binary = find_pcm_binary();
    if (pcm_binary.empty()) {
        if (config_.verbosity >= 2) {
            std::cerr << "PCM sampling failed: pcm-memory not found. "
                      << "Set PCM_PATH or ensure pcm-memory is in PATH." << std::endl;
        }
        return false;
    }

    const double pcm_interval_s = 1.0;
    const double requested_window_s = std::max(1.0, sampling_interval_ms_ / 1000.0);
    const double timeout_s = std::max(5.0, requested_window_s + 3.0);

    std::ostringstream cmd;
    cmd << "cd " << shell_quote(storage_->bandwidth_dir())
        << " && stdbuf -oL -eL timeout -k 1s " << std::fixed << std::setprecision(2) << timeout_s << "s "
        << shell_quote(pcm_binary)
        << " -csv " << std::fixed << std::setprecision(1) << pcm_interval_s << " 2>/dev/null";

    FILE* pipe = popen(cmd.str().c_str(), "r");
    if (!pipe) {
        return false;
    }

    std::vector<PcmCxlSample> raw_samples;
    PcmCsvColumnMap col_map;
    bool header_found = false;
    const bool parsed = parse_pcm_cxl_samples(pipe, config_.verbosity, raw_samples, 0, col_map, header_found);
    pclose(pipe);

    if (!parsed || raw_samples.empty()) {
        if (config_.verbosity >= 3) {
            std::cerr << "PCM sampling failed: unable to parse CXL throughput from pcm-memory output." << std::endl;
        }
        return false;
    }

    double avg_read_mbps = 0.0;
    double avg_write_mbps = 0.0;
    for (const auto& s : raw_samples) {
        avg_read_mbps += s.read_mbps;
        avg_write_mbps += s.write_mbps;
    }
    avg_read_mbps /= static_cast<double>(raw_samples.size());
    avg_write_mbps /= static_cast<double>(raw_samples.size());
    extra_perf_values_["pcm_cxl_read_mbps_x100"] = static_cast<long long>(std::llround(avg_read_mbps * 100.0));
    extra_perf_values_["pcm_cxl_write_mbps_x100"] = static_cast<long long>(std::llround(avg_write_mbps * 100.0));
    extra_perf_values_["pcm_cxl_total_mbps_x100"] = static_cast<long long>(std::llround((avg_read_mbps + avg_write_mbps) * 100.0));

    const int line_size = std::max(1, cached_cache_line_size_);
    const double effective_elapsed_s = requested_window_s;
    const double read_bytes = avg_read_mbps * 1e6 * effective_elapsed_s;
    const double write_bytes = avg_write_mbps * 1e6 * effective_elapsed_s;

    cas_rd = static_cast<long long>(std::llround(read_bytes / static_cast<double>(line_size)));
    cas_wr = static_cast<long long>(std::llround(write_bytes / static_cast<double>(line_size)));
    elapsed = effective_elapsed_s;

    return (cas_rd + cas_wr) > 0;
}

bool PcmBandwidthMeasurer::wait_for_stabilization(int& samples_taken,
                                                  long long& last_cas_rd,
                                                  long long& last_cas_wr,
                                                  double& last_elapsed,
                                                  int pause,
                                                  int ratio_pct,
                                                  bool fast_resume,
                                                  std::function<void()> on_sample_callback) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    if (!traffic_gen_manager_ || !storage_) {
        return false;
    }

    std::vector<int> mem_nodes = numa_resolver_ ? numa_resolver_() : std::vector<int>{0};
    (void)mem_nodes;

    auto relaunch_current_traffic_gen = [&]() -> bool {
        if (!relaunch_traffic_gen(ratio_pct, pause, get_traffic_gen_cores())) {
            return false;
        }
        traffic_gen_manager_->wait_for_traffic_gen_ready(120);
        return true;
    };

    int relaunch_attempts = 0;
    int traffic_gen_pid = traffic_gen_manager_->active_traffic_gen_pid();
    if (!traffic_gen_manager_->is_traffic_gen_running(traffic_gen_pid)) {
        if (!relaunch_current_traffic_gen()) {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        relaunch_attempts++;
    }

    const bool is_remote = false;
    const int expected_cores = get_traffic_gen_cores();
    const int socket_cores = (sys_info_.socket_count > 0) ? sys_info_.sockets[0].core_count : 0;
    double theoretical_peak_gb_s = TheoreticalPeakCalculator::calculate_achievable_peak(
        caps_, expected_cores, socket_cores, is_remote);
    if (theoretical_peak_gb_s <= 0.0) {
        theoretical_peak_gb_s = 300.0;
    }
    // PCM-CXL samples are already direct throughput and can be much lower than DRAM CAS peak.
    // CXL bandwidth typically 5-60 GB/s; use a low floor to avoid discarding valid samples.
    theoretical_peak_gb_s = std::max(1.0, theoretical_peak_gb_s * 0.05);

    TrafficGenHealthChecker health_checker;
    health_checker.is_pid_alive = [this](int pid) -> bool {
        return traffic_gen_manager_->is_traffic_gen_running(pid);
    };
    health_checker.count_running_instances = [this]() -> int {
        if (!traffic_gen_manager_) {
            return TrafficGenHealthChecker::count_taskset_traffic_gen();
        }
        return static_cast<int>(traffic_gen_manager_->traffic_gen_worker_pids().size());
    };
    health_checker.expected_instance_count = expected_cores;

    ensure_scaling_factor_cached();

    const size_t stabilization_window = fast_resume ? 5 : 7;
    BandwidthStabilizer stabilizer(
        theoretical_peak_gb_s, pause, health_checker,
        stabilization_window, 0.05, config_.verbosity);

    const int max_samples = 20;
    int sample_failures = 0;
    bool success = false;
    double successful_sample_elapsed_sum = 0.0;
    int successful_sample_count = 0;
    const int line_size = std::max(1, cached_cache_line_size_);
    const double pcm_interval_s = 1.0;
    const double stream_timeout_s = std::max(120.0, (max_samples * pcm_interval_s) + 30.0);

    const std::string pcm_binary = find_pcm_binary();
    if (pcm_binary.empty()) {
        if (config_.verbosity >= 2) {
            std::cerr << "PCM sampling failed: pcm-memory not found. "
                      << "Set PCM_PATH or ensure pcm-memory is in PATH." << std::endl;
        }
        return false;
    }

    FILE* pcm_pipe = nullptr;
    auto close_pcm_stream = [&]() {
        if (pcm_pipe) {
            pclose(pcm_pipe);
            pcm_pipe = nullptr;
        }
    };

    auto open_pcm_stream = [&]() -> bool {
        close_pcm_stream();
        std::ostringstream cmd;
        cmd << "cd " << shell_quote(storage_->bandwidth_dir())
            << " && stdbuf -oL -eL timeout -k 1s " << std::fixed << std::setprecision(2)
            << stream_timeout_s << "s " << shell_quote(pcm_binary)
            << " -csv " << std::fixed << std::setprecision(1) << pcm_interval_s << " 2>/dev/null";

        pcm_pipe = popen(cmd.str().c_str(), "r");
        return pcm_pipe != nullptr;
    };

    std::deque<PcmCxlSample> recent_window_samples;
    extra_perf_values_.clear();
    int invalid_samples = 0;
    PcmCsvColumnMap col_map;
    bool header_found = false;

    if (!open_pcm_stream()) {
        return false;
    }

    for (int total_samples = 0; total_samples < max_samples && !success; ) {
        if (!traffic_gen_manager_->is_traffic_gen_running(traffic_gen_manager_->active_traffic_gen_pid())) {
            if (relaunch_attempts >= 2 || !relaunch_current_traffic_gen()) {
                close_pcm_stream();
                samples_taken = total_samples;
                return false;
            }
            relaunch_attempts++;
            stabilizer.reset();
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }

        std::vector<PcmCxlSample> window_samples;
        if (!pcm_pipe || !parse_pcm_cxl_samples(pcm_pipe, config_.verbosity, window_samples, 1, col_map, header_found) || window_samples.empty()) {
            sample_failures++;
            if (config_.verbosity >= 3) {
                std::cout << "      [PCM] sample failure " << sample_failures
                          << "/3 (pcm-memory produced no parsable CXL sample)" << std::endl;
            }
            if (sample_failures >= 3) {
                close_pcm_stream();
                samples_taken = total_samples;
                return false;
            }
            if (!open_pcm_stream()) {
                close_pcm_stream();
                samples_taken = total_samples;
                return false;
            }
            continue;
        }

        sample_failures = 0;
        for (const auto& raw_sample : window_samples) {
            if (total_samples >= max_samples) {
                break;
            }

            const double sampled_total_bw_gb_s = (raw_sample.read_mbps + raw_sample.write_mbps) / 1000.0;
            if (!std::isfinite(sampled_total_bw_gb_s) ||
                sampled_total_bw_gb_s <= 0.0 ||
                sampled_total_bw_gb_s > 5000.0 ||
                sampled_total_bw_gb_s > (theoretical_peak_gb_s * 20.0)) {
                invalid_samples++;
                if (config_.verbosity >= 2) {
                    std::cout << "      [PCM WARN] dropping outlier sample: "
                              << sampled_total_bw_gb_s << " GB/s" << std::endl;
                }
                if (invalid_samples >= 5) {
                    close_pcm_stream();
                    samples_taken = total_samples;
                    return false;
                }
                continue;
            }
            invalid_samples = 0;

            const double read_bytes = raw_sample.read_mbps * 1e6 * pcm_interval_s;
            const double write_bytes = raw_sample.write_mbps * 1e6 * pcm_interval_s;
            const long long sample_cas_rd = static_cast<long long>(std::llround(
                read_bytes / static_cast<double>(line_size)));
            const long long sample_cas_wr = static_cast<long long>(std::llround(
                write_bytes / static_cast<double>(line_size)));
            const double sample_elapsed = pcm_interval_s;

            total_samples++;
            successful_sample_elapsed_sum += sample_elapsed;
            successful_sample_count++;
            recent_window_samples.push_back(raw_sample);
            if (recent_window_samples.size() > stabilization_window) {
                recent_window_samples.pop_front();
            }

            if (on_sample_callback) {
                on_sample_callback();
            }

            const double bw_gb_s = calculate_bandwidth_gbps(
                sample_cas_rd, sample_cas_wr, sample_elapsed,
                counter_selection_.type, cached_cache_line_size_, cached_scaling_factor_);

            const StabilizationResult result = stabilizer.add_sample(sample_cas_rd, sample_cas_wr, bw_gb_s);
            if (result == StabilizationResult::ZOMBIE_DETECTED) {
                close_pcm_stream();
                samples_taken = total_samples;
                return false;
            }

            if (config_.verbosity >= 3) {
                const double read_bw_gb_s =
                    (static_cast<double>(sample_cas_rd) * static_cast<double>(line_size)) /
                    (sample_elapsed * 1e9);
                const double write_bw_gb_s =
                    (static_cast<double>(sample_cas_wr) * static_cast<double>(line_size)) /
                    (sample_elapsed * 1e9);
                const double total_bw_gb_s = read_bw_gb_s + write_bw_gb_s;
                const double read_ratio = (total_bw_gb_s > 0.0)
                    ? (read_bw_gb_s / total_bw_gb_s)
                    : 0.0;

                const auto& stable_samples = stabilizer.get_samples();
                const double mean_bw = stabilizer.get_mean_bw();
                const size_t window_count = stable_samples.size();

                std::cout << "      [Sample " << total_samples << "] "
                          << "PCM BW: " << std::fixed << std::setprecision(2) << total_bw_gb_s << " GB/s"
                          << " | ReadBW: " << read_bw_gb_s << " GB/s"
                          << " | WriteBW: " << write_bw_gb_s << " GB/s";

                if (window_count < stabilization_window) {
                    std::cout << " | Filling window (" << window_count << "/" << stabilization_window << ")"
                              << " | Mean: " << mean_bw << " GB/s"
                              << " | RD%: " << (read_ratio * 100.0) << "%";
                } else {
                    double variance = 0.0;
                    for (const auto& s : stable_samples) {
                        const double d = s.bw_gb_s - mean_bw;
                        variance += d * d;
                    }
                    variance /= static_cast<double>(window_count);
                    const double cv = (mean_bw > 0.0) ? (std::sqrt(variance) / mean_bw) : 0.0;
                    const bool in_active_mode = mean_bw > stabilizer.get_noise_floor();

                    std::cout << " | Mean: " << mean_bw << " GB/s"
                              << " | CV: " << (cv * 100.0) << "%"
                              << " | " << (in_active_mode ? "ACTIVE" : "SILENT")
                              << " | RD%: " << (read_ratio * 100.0) << "%"
                              << (stabilizer.is_stable() ? " STABLE" : " ✗ unstable");
                }

                std::cout << '\n';
            }

            if (stabilizer.is_stable()) {
                success = true;
                samples_taken = total_samples;
                break;
            }
        }
    }

    if (!success) {
        close_pcm_stream();
        return false;
    }

    close_pcm_stream();

    long long aggregate_cas_rd = 0;
    long long aggregate_cas_wr = 0;
    for (const auto& sample : stabilizer.get_samples()) {
        aggregate_cas_rd += sample.cas_rd;
        aggregate_cas_wr += sample.cas_wr;
    }

    last_cas_rd = aggregate_cas_rd;
    last_cas_wr = aggregate_cas_wr;
    const double average_sample_elapsed =
        (successful_sample_count > 0)
            ? (successful_sample_elapsed_sum / static_cast<double>(successful_sample_count))
            : std::max(1.0, sampling_interval_ms_ / 1000.0);
    last_elapsed = stabilizer.get_samples().size() * average_sample_elapsed;

    if (!recent_window_samples.empty()) {
        double read_sum_mbps = 0.0;
        double write_sum_mbps = 0.0;
        for (const auto& s : recent_window_samples) {
            read_sum_mbps += s.read_mbps;
            write_sum_mbps += s.write_mbps;
        }
        const double denom = static_cast<double>(recent_window_samples.size());
        const double avg_read_mbps = read_sum_mbps / denom;
        const double avg_write_mbps = write_sum_mbps / denom;
        extra_perf_values_["pcm_cxl_read_mbps_x100"] = static_cast<long long>(std::llround(avg_read_mbps * 100.0));
        extra_perf_values_["pcm_cxl_write_mbps_x100"] = static_cast<long long>(std::llround(avg_write_mbps * 100.0));
        extra_perf_values_["pcm_cxl_total_mbps_x100"] = static_cast<long long>(std::llround((avg_read_mbps + avg_write_mbps) * 100.0));
    }

    return true;
}

bool PcmBandwidthMeasurer::monitor_command(const std::string& command,
                                           MonitorCallback callback,
                                           bool summary_mode) {
    std::string trimmed = trim_copy(command);
    if (trimmed.empty()) {
        std::cerr << "Error: No command specified for PCM monitoring." << std::endl;
        return false;
    }
    if (is_pid_attach_command(trimmed)) {
        std::cerr << "Error: PCM backend does not support PID attach mode in MessProfiler." << std::endl;
        return false;
    }

    if (find_pcm_binary().empty()) {
        std::cerr << "Error: pcm-memory not found. Set PCM_PATH or install Intel PCM." << std::endl;
        return false;
    }

    std::vector<int> mem_nodes = numa_resolver_ ? numa_resolver_() : std::vector<int>{0};

    pid_t child_pid = spawn_shell_command(trimmed);
    if (child_pid < 0) {
        std::cerr << "Error: Failed to launch command for PCM monitoring." << std::endl;
        return false;
    }

    double timestamp = 0.0;
    bool saw_sample = false;
    int status = 0;

    while (true) {
        pid_t wait_result = waitpid(child_pid, &status, WNOHANG);
        if (wait_result == child_pid) {
            break;
        }
        if (wait_result < 0) {
            status = 1;
            break;
        }

        if (summary_mode) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        long long cas_rd = 0;
        long long cas_wr = 0;
        double elapsed = 0.0;
        if (!sample_bandwidth(cas_rd, cas_wr, elapsed, mem_nodes)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(150));
            continue;
        }

        saw_sample = true;
        timestamp += elapsed;
        const double bw_gbps = calculate_bandwidth_gbps(
            cas_rd, cas_wr, elapsed, counter_selection_.type, cached_cache_line_size_, cached_scaling_factor_);

        if (callback) {
            callback(timestamp, bw_gbps, cas_rd, cas_wr, extra_perf_values_);
        }
    }

    if (summary_mode) {
        long long cas_rd = 0;
        long long cas_wr = 0;
        double elapsed = 0.0;
        if (sample_bandwidth(cas_rd, cas_wr, elapsed, mem_nodes)) {
            saw_sample = true;
            const double bw_gbps = calculate_bandwidth_gbps(
                cas_rd, cas_wr, elapsed, counter_selection_.type, cached_cache_line_size_, cached_scaling_factor_);
            if (callback) {
                callback(elapsed, bw_gbps, cas_rd, cas_wr, extra_perf_values_);
            }
        }
    }

    return WIFEXITED(status) && WEXITSTATUS(status) == 0 && (summary_mode || saw_sample);
}
