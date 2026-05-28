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
#include "measurement/instruction_samplers/ArmSpeSampler.h"
#include "Utils.h"
#include "utils/SubprocessCapture.h"
#include <chrono>
#include <iostream>
#include <sstream>
#include <fstream>
#include <filesystem>
#include <unistd.h>
#include <fcntl.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <signal.h>
#include <cerrno>
#include <cstring>
#include <algorithm>
#include <numeric>
#include <cstdlib>
#include <dirent.h>
#include <sys/resource.h>

namespace {
    std::string to_lower_copy(const std::string& s) {
        std::string out = s;
        std::transform(out.begin(), out.end(), out.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        return out;
    }

    std::string shell_quote(const std::string &s) {
        std::string out = "'";
        for (char c : s) {
            if (c == '\'') out += "'\\''";
            else out += c;
        }
        out += "'";
        return out;
    }

    std::string choose_temp_dir() {
        const char *candidates[] = {"/dev/shm", "/tmp"};
        for (const char *d : candidates) {
            if (std::filesystem::exists(d) && access(d, W_OK) == 0) return d;
        }
        return "/tmp";
    }

    bool is_hex_char(char c) {
        return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
    }

    uint64_t extract_spe_weight(const char* line) {
        const char* p = line;
        const char* hex_end = nullptr;
        while (*p) {
            if (is_hex_char(*p)) {
                int len = 0;
                bool has_alpha = false;
                while (is_hex_char(*p)) {
                    if (*p >= 'a') has_alpha = true;
                    ++p; ++len;
                }
                if (len >= 4 && has_alpha) {
                    hex_end = p;
                    break;
                }
                continue;
            }
            ++p;
        }
        if (!hex_end) return 0;

        while (*p == ' ' || *p == '\t') ++p;
        if (*p < '0' || *p > '9') return 0;

        uint64_t val = 0;
        const char* num_start = p;
        while (*p >= '0' && *p <= '9') {
            val = val * 10 + (*p - '0');
            ++p;
        }
        if (p == num_start) return 0;

        while (*p == ' ' || *p == '\t') ++p;
        if (!is_hex_char(*p)) return 0;

        return val;
    }

    std::string join_pid_list(const std::vector<pid_t>& pids) {
        std::string spec;
        for (pid_t pid : pids) {
            if (pid <= 0) {
                continue;
            }
            if (!spec.empty()) {
                spec += ",";
            }
            spec += std::to_string(pid);
        }
        return spec;
    }

    long online_cpu_count() {
        long n = sysconf(_SC_NPROCESSORS_ONLN);
        return n > 0 ? n : 1;
    }

    void try_raise_nofile(uint64_t needed, int verbosity) {
        struct rlimit rl;
        if (getrlimit(RLIMIT_NOFILE, &rl) != 0) return;
        if (rl.rlim_cur >= needed) return;

        rlim_t target = (rl.rlim_max == RLIM_INFINITY)
                            ? static_cast<rlim_t>(needed)
                            : std::min<rlim_t>(rl.rlim_max, static_cast<rlim_t>(needed));
        if (target <= rl.rlim_cur) {
            if (verbosity >= 1) {
                std::cerr << "      ARM-SPE: warning — RLIMIT_NOFILE cur=" << rl.rlim_cur
                          << " max=" << rl.rlim_max << " < needed=" << needed
                          << " (cannot raise; perf may fail with EMFILE)\n";
            }
            return;
        }
        rlim_t old_cur = rl.rlim_cur;
        rl.rlim_cur = target;
        if (setrlimit(RLIMIT_NOFILE, &rl) == 0 && verbosity >= 2) {
            std::cout << "      ARM-SPE: raised RLIMIT_NOFILE from " << old_cur
                      << " to " << target << " (needed ~" << needed << ")\n";
        }
    }
}

ArmSpeSampler::ArmSpeSampler(uint64_t min_latency_threshold, int sample_period, int verbosity)
    : min_latency_threshold_(min_latency_threshold),
      sample_period_(sample_period),
      verbosity_(verbosity) {}

ArmSpeSampler::~ArmSpeSampler() {
    stop_requested_ = true;
    if (sampling_thread_.joinable()) {
        sampling_thread_.join();
    }
    if (!current_perf_data_path_.empty()) unlink(current_perf_data_path_.c_str());
    if (!current_perf_log_path_.empty()) unlink(current_perf_log_path_.c_str());
}

std::string ArmSpeSampler::spe_pmu_name() {
    std::string base = "/sys/bus/event_source/devices/";
    DIR *dir = opendir(base.c_str());
    if (dir) {
        struct dirent *entry;
        while ((entry = readdir(dir)) != nullptr) {
            std::string name = entry->d_name;
            std::string lower = to_lower_copy(name);
            if (lower.find("arm_spe") != std::string::npos || lower == "spe" ||
                lower.rfind("spe_", 0) == 0 || lower.find("_spe") != std::string::npos) {
                closedir(dir);
                return name;
            }
        }
        closedir(dir);
    }

    subprocess::Options opts;
    opts.timeout          = std::chrono::seconds(10);
    opts.max_output_bytes = 262144;
    const std::string output = subprocess::run_shell("perf list 2>/dev/null", opts);

    std::istringstream stream(output);
    std::string line;
    while (std::getline(stream, line)) {
        std::string lower = to_lower_copy(line);
        if (lower.find("arm_spe") == std::string::npos &&
            lower.find("spe") == std::string::npos) {
            continue;
        }

        size_t pos = lower.find("arm_spe");
        if (pos == std::string::npos) {
            pos = lower.find("spe");
        }
        if (pos == std::string::npos) {
            continue;
        }

        size_t start = pos;
        while (start > 0 && (std::isalnum(static_cast<unsigned char>(line[start - 1])) || line[start - 1] == '_')) {
            --start;
        }
        size_t end = pos;
        while (end < line.size() && (std::isalnum(static_cast<unsigned char>(line[end])) || line[end] == '_')) {
            ++end;
        }
        std::string token = line.substr(start, end - start);
        if (!token.empty()) {
            return token;
        }
    }
    return "";
}

bool ArmSpeSampler::detect_spe_pmu() {
    const std::string pmu = spe_pmu_name();
    return !pmu.empty() && probe_spe_record(pmu);
}

std::string ArmSpeSampler::availability_diagnostic() {
    const std::string pmu = spe_pmu_name();
    if (pmu.empty()) {
        std::vector<std::string> devices;
        DIR *dir = opendir("/sys/bus/event_source/devices");
        if (dir) {
            struct dirent *entry;
            while ((entry = readdir(dir)) != nullptr) {
                std::string name = entry->d_name;
                if (name == "." || name == "..") {
                    continue;
                }
                devices.push_back(name);
            }
            closedir(dir);
        }
        std::sort(devices.begin(), devices.end());

        std::string detail = "no SPE PMU exposed by /sys/bus/event_source/devices";
        if (!devices.empty()) {
            detail += " (found: ";
            for (size_t i = 0; i < devices.size(); ++i) {
                if (i > 0) {
                    detail += ", ";
                }
                detail += devices[i];
            }
            detail += ")";
        }
        return detail;
    }

    if (!probe_spe_record(pmu)) {
        return "SPE PMU '" + pmu + "' is present, but perf record probe failed";
    }

    return "ARM SPE available via PMU '" + pmu + "'";
}

bool ArmSpeSampler::is_available() const {
    return detect_spe_pmu();
}

bool ArmSpeSampler::start_async(const std::vector<int>& cores, const std::vector<pid_t>& pids) {
    if (is_running_) return false;
    if (cores.empty() && pids.empty()) return false;

    selected_pid_spec_.clear();
    selected_cpu_spec_.clear();

    const char* prefer_pids_env = std::getenv("MESS_SPE_PREFER_PIDS");
    const bool prefer_pids = prefer_pids_env && prefer_pids_env[0] && prefer_pids_env[0] != '0';

    if (!cores.empty()) {
        static constexpr size_t kMaxSpeCores = 8;
        std::vector<int> sampled_cores;
        if (cores.size() <= kMaxSpeCores) {
            sampled_cores = cores;
        } else {
            size_t step = cores.size() / kMaxSpeCores;
            for (size_t i = 0; i < kMaxSpeCores && i * step < cores.size(); ++i) {
                sampled_cores.push_back(cores[i * step]);
            }
        }

        std::string spec;
        int start = sampled_cores.front();
        int prev = sampled_cores.front();
        auto flush_range = [&](int s, int e) {
            if (!spec.empty()) spec += ",";
            if (s == e) spec += std::to_string(s);
            else spec += std::to_string(s) + "-" + std::to_string(e);
        };
        for (size_t i = 1; i < sampled_cores.size(); ++i) {
            if (sampled_cores[i] == prev + 1) {
                prev = sampled_cores[i];
                continue;
            }
            flush_range(start, prev);
            start = prev = sampled_cores[i];
        }
        flush_range(start, prev);
        selected_cpu_spec_ = spec;
    }

    if (selected_cpu_spec_.empty() || prefer_pids) {
        static constexpr size_t kMaxSpePids = 8;
        std::vector<pid_t> sampled_pids;
        if (pids.size() <= kMaxSpePids) {
            sampled_pids = pids;
        } else {
            size_t step = pids.size() / kMaxSpePids;
            for (size_t i = 0; i < kMaxSpePids && i * step < pids.size(); ++i) {
                sampled_pids.push_back(pids[i * step]);
            }
        }
        selected_pid_spec_ = join_pid_list(sampled_pids);

        if (!selected_pid_spec_.empty()) {
            long ncpu = online_cpu_count();
            uint64_t fd_estimate =
                static_cast<uint64_t>(sampled_pids.size()) * static_cast<uint64_t>(ncpu) + 256;
            try_raise_nofile(fd_estimate, verbosity_);
            if (verbosity_ >= 2 && pids.size() > sampled_pids.size()) {
                std::cout << "      ARM-SPE: subsampled " << pids.size() << " pids -> "
                          << sampled_pids.size() << " (cap=" << kMaxSpePids
                          << ", est. " << fd_estimate << " fds for " << ncpu << " online cpus)\n";
            }
        }

        if (prefer_pids && !selected_cpu_spec_.empty()) {
            selected_cpu_spec_.clear();
        }
    }

    std::string temp_dir = choose_temp_dir();
    current_perf_data_path_ = temp_dir + "/arm_spe_sampler_" + std::to_string(getpid()) + ".data";
    current_perf_log_path_ = temp_dir + "/arm_spe_sampler_" + std::to_string(getpid()) + ".log";
    unlink(current_perf_data_path_.c_str());
    unlink(current_perf_log_path_.c_str());

    is_running_ = true;
    stop_requested_ = false;

    if (verbosity_ >= 2) {
        std::cout << "      ARM-SPE: launching perf record "
                  << (selected_pid_spec_.empty() ? ("on cores " + selected_cpu_spec_) : ("for pids " + selected_pid_spec_))
                  << " (min_lat=" << min_latency_threshold_ << ", period=" << sample_period_ << ")\n";
    }

    sampling_thread_ = std::thread(&ArmSpeSampler::sampling_thread_func, this);
    return true;
}

void ArmSpeSampler::sampling_thread_func() {
    std::string pmu = spe_pmu_name();
    if (pmu.empty()) {
        is_running_ = false;
        return;
    }

    const char* no_filter_env = std::getenv("MESS_SPE_NO_FILTER");
    const bool no_filter = no_filter_env && no_filter_env[0] && no_filter_env[0] != '0';
    std::string event_spec;
    if (no_filter) {
        event_spec = pmu + "/load_filter=1,min_latency="
                     + std::to_string(min_latency_threshold_) + "/";
    } else {
        event_spec = pmu + "/load_filter=1,jitter=1,event_filter=2,min_latency="
                     + std::to_string(min_latency_threshold_) + "/";
    }
    if (verbosity_ >= 2) {
        std::cout << "      ARM-SPE: event spec = " << event_spec
                  << (no_filter ? "  [MESS_SPE_NO_FILTER=1]" : "") << "\n";
    }
    std::string period_str = std::to_string(sample_period_);

    const char* force_cores_env = std::getenv("MESS_SPE_FORCE_CORES");
    const bool force_cores = force_cores_env && force_cores_env[0] && force_cores_env[0] != '0';
    if (force_cores && !selected_cpu_spec_.empty()) {
        if (verbosity_ >= 2) {
            std::cout << "      ARM-SPE: MESS_SPE_FORCE_CORES=1 -> using -C "
                      << selected_cpu_spec_ << " -a (ignoring pid spec)\n";
        }
        selected_pid_spec_.clear();
    }

    pid_t perf_pid = fork();
    if (perf_pid < 0) {
        is_running_ = false;
        return;
    }

    if (perf_pid == 0) {
        int log_fd = open(current_perf_log_path_.c_str(),
                          O_WRONLY | O_CREAT | O_TRUNC, 0644);
        if (log_fd >= 0) {
            dup2(log_fd, STDOUT_FILENO);
            dup2(log_fd, STDERR_FILENO);
            close(log_fd);
        }
        if (!selected_pid_spec_.empty()) {
            execlp("perf", "perf", "record",
                   "-e", event_spec.c_str(),
                   "-c", period_str.c_str(),
                   "-p", selected_pid_spec_.c_str(),
                   "-o", current_perf_data_path_.c_str(),
                   nullptr);
        } else {
            execlp("perf", "perf", "record",
                   "-e", event_spec.c_str(),
                   "-c", period_str.c_str(),
                   "-C", selected_cpu_spec_.c_str(),
                   "-a",
                   "-o", current_perf_data_path_.c_str(),
                   nullptr);
        }
        _exit(127);
    }

    perf_child_pid_ = perf_pid;

    while (!stop_requested_) {
        int status;
        pid_t w = waitpid(perf_pid, &status, WNOHANG);
        if (w == perf_pid) {
            if (verbosity_ >= 2) {
                std::ifstream log(current_perf_log_path_);
                if (log) {
                    std::string line;
                    while (std::getline(log, line)) {
                        std::cerr << "        ARM-SPE: " << line << "\n";
                    }
                }
            }
            perf_child_pid_ = -1;
            is_running_ = false;
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    kill(perf_pid, SIGINT);

    int status;
    bool reaped = false;
    for (int i = 0; i < 500; ++i) {
        pid_t w = waitpid(perf_pid, &status, WNOHANG);
        if (w == perf_pid) { reaped = true; break; }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (!reaped) {
        kill(perf_pid, SIGKILL);
        waitpid(perf_pid, &status, 0);
    }

    perf_child_pid_ = -1;
    is_running_ = false;
}

bool ArmSpeSampler::probe_spe_record(const std::string& pmu_name) {
    if (pmu_name.empty()) {
        return false;
    }

    const std::string temp_dir = choose_temp_dir();
    const std::string data_path = temp_dir + "/arm_spe_probe_" + std::to_string(getpid()) + ".data";
    const std::string log_path = temp_dir + "/arm_spe_probe_" + std::to_string(getpid()) + ".log";
    unlink(data_path.c_str());
    unlink(log_path.c_str());

    const std::string event_spec = pmu_name + "/load_filter=1,min_latency=1/";
    const std::string cmd = "timeout 2 perf record -e " + shell_quote(event_spec) +
                            " -c 4096 -p " + std::to_string(getpid()) +
                            " -o " + shell_quote(data_path) +
                            " -- sleep 0.01 >" + shell_quote(log_path) + " 2>&1";
    const int rc = std::system(cmd.c_str());

    struct stat st;
    const bool ok = (rc == 0 || rc == 124 * 256 || rc == 124) &&
                    stat(data_path.c_str(), &st) == 0;

    unlink(data_path.c_str());
    unlink(log_path.c_str());
    return ok;
}

bool ArmSpeSampler::parse_perf_script_output(const std::string& perf_data_path,
                                              std::vector<uint64_t>& latencies) {
    std::string quoted = shell_quote(perf_data_path);

    struct stat st;
    if (stat(perf_data_path.c_str(), &st) != 0 || st.st_size == 0) return false;

    static constexpr uint64_t kMaxLines = 200000;

    std::string cmd = "timeout 30 perf script -i " + quoted + " -F event,ip,addr,weight 2>&1";

    FILE *fp = popen(cmd.c_str(), "r");
    if (!fp) return false;

    uint64_t total_lines = 0;
    uint64_t memory_lines = 0;
    uint64_t below_threshold = 0;
    std::string first_line;
    char line[4096];
    while (fgets(line, sizeof(line), fp) && total_lines < kMaxLines) {
        ++total_lines;
        if (first_line.empty()) first_line = line;

        const char* mem = strstr(line, "memory:");
        if (!mem) continue;
        ++memory_lines;

        uint64_t lat = extract_spe_weight(mem + 7);
        if (lat >= min_latency_threshold_) {
            latencies.push_back(lat);
        } else {
            ++below_threshold;
        }
    }
    pclose(fp);

    if (verbosity_ >= 1) {
        std::cout << "    ARM-SPE parsed " << total_lines << " lines, "
                  << memory_lines << " memory:, "
                  << below_threshold << " below min_lat=" << min_latency_threshold_
                  << " => " << latencies.size() << " kept samples\n";
        if (verbosity_ >= 3 && !first_line.empty()) {
            std::cout << "    ARM-SPE first perf-script line: " << first_line;
        }
    }

    return !latencies.empty();
}

InstructionSamplerStats ArmSpeSampler::stop_and_process(double cpu_freq_ghz) {
    if (verbosity_ >= 2) {
        std::cout << "    BW stable — collecting samples for 50 ms more..." << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    if (verbosity_ >= 2) {
        std::cout << "    Stopping sampler and processing results..." << std::endl;
    }
    stop_requested_ = true;
    if (sampling_thread_.joinable()) {
        sampling_thread_.join();
    }
    is_running_ = false;
    stop_requested_ = false;

    InstructionSamplerStats stats{};
    std::vector<uint64_t> latencies;

    parse_perf_script_output(current_perf_data_path_, latencies);

    if (latencies.empty()) {
        if (verbosity_ >= 1) {
            std::cerr << "      [ARM-SPE] no latency samples found after parsing\n";
            struct stat st;
            if (stat(current_perf_data_path_.c_str(), &st) == 0) {
                std::cerr << "      [ARM-SPE] perf.data: " << current_perf_data_path_
                          << " (" << st.st_size << " bytes)\n";
            } else {
                std::cerr << "      [ARM-SPE] perf.data missing at "
                          << current_perf_data_path_ << "\n";
            }
            std::ifstream log(current_perf_log_path_);
            if (log) {
                std::cerr << "      [ARM-SPE] --- perf record log ---\n";
                std::string line;
                while (std::getline(log, line)) {
                    std::cerr << "      [ARM-SPE] " << line << "\n";
                }
                std::cerr << "      [ARM-SPE] --- end log ---\n";
            } else {
                std::cerr << "      [ARM-SPE] no perf log at "
                          << current_perf_log_path_ << "\n";
            }
            std::cerr << "      [ARM-SPE] hint: keep files for inspection by setting "
                      << "MESS_SPE_KEEP=1; retry without filters with MESS_SPE_NO_FILTER=1; "
                      << "force core-attach with MESS_SPE_FORCE_CORES=1\n";
        }
        const char* keep = std::getenv("MESS_SPE_KEEP");
        if (!(keep && keep[0] && keep[0] != '0')) {
            unlink(current_perf_data_path_.c_str());
            unlink(current_perf_log_path_.c_str());
        }
        return stats;
    }

    const char* keep = std::getenv("MESS_SPE_KEEP");
    if (!(keep && keep[0] && keep[0] != '0')) {
        unlink(current_perf_data_path_.c_str());
        unlink(current_perf_log_path_.c_str());
    }

    std::sort(latencies.begin(), latencies.end());
    stats.samples = latencies.size();
    stats.raw_latencies = std::move(latencies);

    double sum = std::accumulate(stats.raw_latencies.begin(), stats.raw_latencies.end(), 0.0);
    stats.mean_cycles = sum / stats.samples;
    stats.min_cycles = stats.raw_latencies.front();
    stats.max_cycles = stats.raw_latencies.back();

    auto pct = [&](double p) -> double {
        size_t idx = static_cast<size_t>(p * (stats.samples - 1));
        return stats.raw_latencies[idx];
    };

    stats.median_cycles = pct(0.50);
    stats.p90_cycles = pct(0.90);
    stats.p95_cycles = pct(0.95);
    stats.p99_cycles = pct(0.99);
    stats.p99_9_cycles = pct(0.999);

    if (cpu_freq_ghz > 0.0) {
        stats.mean_ns = stats.mean_cycles / cpu_freq_ghz;
        stats.min_ns = stats.min_cycles / cpu_freq_ghz;
        stats.median_ns = stats.median_cycles / cpu_freq_ghz;
        stats.p90_ns = stats.p90_cycles / cpu_freq_ghz;
        stats.p95_ns = stats.p95_cycles / cpu_freq_ghz;
        stats.p99_ns = stats.p99_cycles / cpu_freq_ghz;
        stats.p99_9_ns = stats.p99_9_cycles / cpu_freq_ghz;
        stats.max_ns = stats.max_cycles / cpu_freq_ghz;
    }

    return stats;
}

void ArmSpeSampler::cleanup() {
    stop_requested_ = true;
    if (sampling_thread_.joinable()) {
        sampling_thread_.join();
    }
    is_running_ = false;
    stop_requested_ = false;

    if (!current_perf_data_path_.empty()) {
        unlink(current_perf_data_path_.c_str());
        current_perf_data_path_.clear();
    }
    if (!current_perf_log_path_.empty()) {
        unlink(current_perf_log_path_.c_str());
        current_perf_log_path_.clear();
    }
    perf_child_pid_ = -1;
}

