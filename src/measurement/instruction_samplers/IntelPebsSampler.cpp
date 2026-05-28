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
#include "measurement/instruction_samplers/IntelPebsSampler.h"
#include "Utils.h"
#include <iostream>
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

namespace {
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

    std::string read_sysfs(const std::string &path) {
        std::ifstream f(path);
        if (!f) return "";
        std::string val;
        f >> val;
        return val;
    }

    uint64_t parse_hex(const std::string &s, const std::string &key) {
        auto pos = s.find(key + "=");
        if (pos == std::string::npos) return 0;
        return std::stoull(s.substr(pos + key.size() + 1), nullptr, 0);
    }
}

IntelPebsSampler::IntelPebsSampler(uint64_t ldlat_threshold, int sample_freq, int verbosity)
    : ldlat_threshold_(ldlat_threshold), sample_freq_(sample_freq), verbosity_(verbosity) {}

IntelPebsSampler::~IntelPebsSampler() {
    stop_requested_ = true;
    if (sampling_thread_.joinable()) {
        sampling_thread_.join();
    }
    if (!current_perf_data_path_.empty()) unlink(current_perf_data_path_.c_str());
    if (!current_perf_log_path_.empty()) unlink(current_perf_log_path_.c_str());
}

std::vector<PmuConfig> IntelPebsSampler::discover_pmu_candidates() const {
    std::vector<PmuConfig> candidates;

    for (const char *dev : {"cpu_core", "cpu"}) {
        std::string base = std::string("/sys/bus/event_source/devices/") + dev;
        if (!std::filesystem::exists(base)) continue;

        std::string type_str = read_sysfs(base + "/type");
        if (type_str.empty()) continue;
        uint32_t type = std::stoul(type_str);

        std::string aux_ev = read_sysfs(base + "/events/mem-loads-aux");
        if (!aux_ev.empty()) {
            uint64_t event = parse_hex(aux_ev, "event");
            uint64_t umask = parse_hex(aux_ev, "umask");
            uint64_t cfg   = event | (umask << 8);
            candidates.push_back({type, cfg, 0, true, "mem-loads-aux (SPR+)"});
        }

        std::string ev = read_sysfs(base + "/events/mem-loads");
        if (!ev.empty()) {
            uint64_t event = parse_hex(ev, "event");
            uint64_t umask = parse_hex(ev, "umask");
            uint64_t cfg   = event | (umask << 8);
            candidates.push_back({type, cfg, ldlat_threshold_, false, "mem-loads (classic)"});
            if (ldlat_threshold_ > 0) {
                candidates.push_back({type, cfg, 0, false, "mem-loads (no ldlat)"});
            }
        }
        break; 
    }
    return candidates;
}

bool IntelPebsSampler::is_available() const {
    std::string vendor = read_sysfs("/sys/devices/system/cpu/cpu0/cpufreq/vendor");
    if (vendor.empty()) {
        std::ifstream cpuinfo("/proc/cpuinfo");
        std::string line;
        while (std::getline(cpuinfo, line)) {
            if (line.find("vendor_id") != std::string::npos) {
                if (line.find("GenuineIntel") != std::string::npos) {
                    vendor = "GenuineIntel";
                }
                break;
            }
        }
    }
    
    if (vendor != "GenuineIntel") {
        return false;
    }

    auto candidates = discover_pmu_candidates();
    return !candidates.empty();
}

bool IntelPebsSampler::start_async(const std::vector<int>& cores, const std::vector<pid_t>& /* pids */) {
    if (is_running_) {
        return false;
    }
    
    if (cores.empty()) {
        return false;
    }

    std::string spec;
    int start = cores.front();
    int prev = cores.front();
    auto flush_range = [&](int s, int e) {
        if (!spec.empty()) spec += ",";
        if (s == e) spec += std::to_string(s);
        else spec += std::to_string(s) + "-" + std::to_string(e);
    };
    for (size_t i = 1; i < cores.size(); ++i) {
        if (cores[i] == prev + 1) {
            prev = cores[i];
            continue;
        }
        flush_range(start, prev);
        start = prev = cores[i];
    }
    flush_range(start, prev);
    selected_cpu_spec_ = spec;

    std::string temp_dir = choose_temp_dir();
    current_perf_data_path_ = temp_dir + "/mem_sampler_perf_" + std::to_string(getpid()) + ".data";
    current_perf_log_path_ = temp_dir + "/mem_sampler_perf_" + std::to_string(getpid()) + ".log";
    unlink(current_perf_data_path_.c_str());
    unlink(current_perf_log_path_.c_str());

    is_running_ = true;
    stop_requested_ = false;

    if (verbosity_ >= 2) {
        std::cout << "      PEBS: start_async: launching perf mem record on cores " << selected_cpu_spec_
                  << " (ldlat=" << ldlat_threshold_ << ", freq=" << sample_freq_ << ")\n";
    }

    sampling_thread_ = std::thread(&IntelPebsSampler::sampling_thread_func, this);

    return true;
}

void IntelPebsSampler::sampling_thread_func() {
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
        std::string ldlat_str = std::to_string(ldlat_threshold_);
        std::string freq_str = std::to_string(sample_freq_);
        execlp("perf", "perf", "mem", "record",
               "-t", "load",
               "--ldlat", ldlat_str.c_str(),
               "-F", freq_str.c_str(),
               "-C", selected_cpu_spec_.c_str(),
               "-o", current_perf_data_path_.c_str(),
               nullptr);
        _exit(127);
    }

    perf_child_pid_ = perf_pid;

    while (!stop_requested_) {
        int status;
        pid_t w = waitpid(perf_pid, &status, WNOHANG);
        if (w == perf_pid) {
            if (verbosity_ >= 1) {
                std::ifstream log(current_perf_log_path_);
                if (log) {
                    std::string line;
                    while (std::getline(log, line)) {
                        std::cerr << "        " << line << "\n";
                    }
                }
            }
            perf_child_pid_ = -1;
            is_running_ = false;
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    kill(perf_pid, SIGINT);
    
    int status;
    bool reaped = false;
    for (int i = 0; i < 100; ++i) {
        pid_t w = waitpid(perf_pid, &status, WNOHANG);
        if (w == perf_pid) { reaped = true; break; }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    if (!reaped) {
        kill(perf_pid, SIGKILL);
        waitpid(perf_pid, &status, 0);
    }
    perf_child_pid_ = -1;

    is_running_ = false;
}

bool IntelPebsSampler::parse_perf_script_data(
    const std::string &perf_data_path,
    const std::vector<pid_t> &pids,
    std::vector<uint64_t> &ram_latencies) {
    
    std::string cmd = "perf script -i " + shell_quote(perf_data_path) +
                      " -F pid,weight,data_src 2>/dev/null";

    FILE *fp = popen(cmd.c_str(), "r");
    if (!fp) {
        return false;
    }

    uint64_t total_lines = 0, parsed_ok = 0, pid_filtered = 0;
    uint64_t not_ram = 0, tlb_filtered = 0, zero_weight = 0;

    char line[4096];
    while (fgets(line, sizeof(line), fp)) {
        ++total_lines;
        // Skip leading whitespace
        char *p = line;
        while (*p == ' ' || *p == '\t') ++p;
        if (*p == '\0' || *p == '\n') continue;

        // Token 1: PID (decimal)
        char *end1;
        unsigned long long t1 = strtoull(p, &end1, 10);
        if (end1 == p || (*end1 != ' ' && *end1 != '\t')) continue;

        // Token 2: data_src (hex)  — perf outputs: PID  DATA_SRC  WEIGHT
        char *p2 = end1;
        while (*p2 == ' ' || *p2 == '\t') ++p2;
        char *end2;
        unsigned long long t2 = strtoull(p2, &end2, 16);
        if (end2 == p2 || (*end2 != ' ' && *end2 != '\t' && *end2 != '\n' && *end2 != '\0'))
            continue;

        // Token 3 (weight): last decimal number on line
        // Scan backwards from end of line
        char *back = line + strlen(line) - 1;
        while (back > end2 && (*back == '\n' || *back == '\r' || *back == ' ' || *back == '\t'))
            --back;
        if (back <= end2 || !isdigit((unsigned char)*back)) continue;
        char *wend = back + 1;
        while (back > end2 && isdigit((unsigned char)*(back - 1))) --back;
        char saved = *wend;
        *wend = '\0';
        unsigned long long weight = strtoull(back, nullptr, 10);
        *wend = saved;

        ++parsed_ok;

        uint64_t pid = t1;
        uint64_t data_src = t2;

        if (!pids.empty() && std::find(pids.begin(), pids.end(), static_cast<pid_t>(pid)) == pids.end()) {
            ++pid_filtered;
            continue;
        }

        uint64_t lvl_num = (data_src >> 33) & 0xF;
        uint64_t mem_lvl = (data_src >> 5) & 0x3FFF;
        bool hit = mem_lvl & 0x0002;
        bool is_ram = false;

        if (lvl_num && lvl_num != 0x0f) {
            if (lvl_num == 0x0d) is_ram = hit; 
        } else {
            if (mem_lvl & 0x0080) is_ram = hit;
            if (mem_lvl & 0x0100) is_ram = hit;
            if (mem_lvl & 0x0200) is_ram = hit;
        }

        if (!is_ram) { ++not_ram; continue; }

        uint64_t mem_dtlb = (data_src >> 26) & 0x7F;
        bool tlb_hit_l1_l2 = (mem_dtlb & 0x02) && (mem_dtlb & 0x18);
        if (!tlb_hit_l1_l2) { ++tlb_filtered; continue; }

        if (weight == 0) { ++zero_weight; continue; }

        ram_latencies.push_back(weight);
    }
    pclose(fp);
    if (verbosity_ >= 2) {
        std::cout << "    PEBS parsed " << total_lines << " lines, "
                  << parsed_ok << " parsed, "
                  << pid_filtered << " pid-filtered, "
                  << not_ram << " non-RAM, "
                  << tlb_filtered << " TLB-filtered, "
                  << zero_weight << " zero-wt => "
                  << ram_latencies.size() << " RAM-hit samples" << std::endl;
    }
    return true;
}

bool IntelPebsSampler::parse_perf_report_data(
    const std::string &perf_data_path,
    const std::vector<pid_t> &pids,
    std::vector<uint64_t> &ram_latencies) {

    std::string cmd = "perf report --mem-mode --stdio -t , --show-nr-samples "
                      "-F local_weight,pid,mem -i " + shell_quote(perf_data_path) +
                      " 2>/dev/null";

    FILE *fp = popen(cmd.c_str(), "r");
    if (!fp) {
        return false;
    }

    uint64_t total_lines = 0, parsed_ok = 0, pid_filtered = 0;
    uint64_t not_ram = 0, zero_weight = 0;

    char line[4096];
    while (fgets(line, sizeof(line), fp)) {
        ++total_lines;

        char *p = line;
        while (*p == ' ' || *p == '\t') ++p;
        if (*p == '\0' || *p == '\n' || *p == '\r' || *p == '#') continue;

        // Field 1: local weight (decimal)
        char *end1;
        unsigned long long weight = strtoull(p, &end1, 10);
        if (end1 == p) continue;
        while (*end1 == ' ' || *end1 == '\t') ++end1;
        if (*end1 != ',') continue;
        ++end1;

        // Field 2: "<pid>:<command>"
        while (*end1 == ' ' || *end1 == '\t') ++end1;
        char *end2;
        unsigned long long pid = strtoull(end1, &end2, 10);
        if (end2 == end1 || *end2 != ':') continue;

        // Field 3: memory access description, after the next comma
        char *mem_desc = strchr(end2, ',');
        if (!mem_desc) continue;
        ++mem_desc;
        while (*mem_desc == ' ' || *mem_desc == '\t') ++mem_desc;

        ++parsed_ok;

        if (!pids.empty() &&
            std::find(pids.begin(), pids.end(), static_cast<pid_t>(pid)) == pids.end()) {
            ++pid_filtered;
            continue;
        }

        if (!strstr(mem_desc, "RAM hit")) { ++not_ram; continue; }

        if (weight == 0) { ++zero_weight; continue; }

        ram_latencies.push_back(weight);
    }
    pclose(fp);

    if (verbosity_ >= 2) {
        std::cout << "    PEBS (report) parsed " << total_lines << " lines, "
                  << parsed_ok << " parsed, "
                  << pid_filtered << " pid-filtered, "
                  << not_ram << " non-RAM, "
                  << zero_weight << " zero-wt => "
                  << ram_latencies.size() << " RAM-hit samples" << std::endl;
    }
    return true;
}

void IntelPebsSampler::collect_samples(
    const std::string &perf_data_path,
    const std::vector<pid_t> &pids,
    std::vector<uint64_t> &ram_latencies) {

    parse_perf_script_data(perf_data_path, pids, ram_latencies);
    if (!ram_latencies.empty()) return;

    if (verbosity_ >= 2) {
        std::cerr << "      [PEBS] `perf script` returned no samples — "
                     "falling back to `perf report`" << std::endl;
    }
    parse_perf_report_data(perf_data_path, pids, ram_latencies);
}

InstructionSamplerStats IntelPebsSampler::stop_and_process(double cpu_freq_ghz) {
    if (verbosity_ >= 2) {
        std::cout << "    BW stable — collecting samples for 500 ms more..." << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
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
    
    std::vector<pid_t> all_pids; // We don't filter by PID right now because traffic gen has many children, perf -C is used
    collect_samples(current_perf_data_path_, all_pids, latencies);
    
    unlink(current_perf_data_path_.c_str());
    unlink(current_perf_log_path_.c_str());

    constexpr size_t min_samples = 50;
    int retry_ms = 1000;
    const int max_retries = 3;

    for (int retry = 0; retry < max_retries && latencies.size() < min_samples; ++retry) {
        if (verbosity_ >= 1) {
            if (latencies.empty()) {
                std::cerr << "      [PEBS] no RAM-hit latencies found — retrying for "
                          << retry_ms << " ms (write-heavy point?)" << std::endl;
            } else {
                std::cerr << "      [PEBS] only " << latencies.size()
                          << " RAM-hit samples (need " << min_samples
                          << ") — retrying for " << retry_ms << " ms" << std::endl;
            }
        }

        latencies.clear();

        std::string temp_dir = choose_temp_dir();
        current_perf_data_path_ = temp_dir + "/mem_sampler_perf_" + std::to_string(getpid()) + ".data";
        current_perf_log_path_  = temp_dir + "/mem_sampler_perf_" + std::to_string(getpid()) + ".log";
        unlink(current_perf_data_path_.c_str());
        unlink(current_perf_log_path_.c_str());

        is_running_ = true;
        stop_requested_ = false;
        sampling_thread_ = std::thread(&IntelPebsSampler::sampling_thread_func, this);

        std::this_thread::sleep_for(std::chrono::milliseconds(retry_ms));

        stop_requested_ = true;
        if (sampling_thread_.joinable()) {
            sampling_thread_.join();
        }
        is_running_ = false;
        stop_requested_ = false;

        collect_samples(current_perf_data_path_, all_pids, latencies);
        unlink(current_perf_data_path_.c_str());
        unlink(current_perf_log_path_.c_str());

        retry_ms *= 2;
    }

    if (latencies.empty()) {
        if (verbosity_ >= 1) {
            std::cerr << "      [PEBS] no RAM-hit latencies after retries" << std::endl;
        }
        return stats;
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

    {
        size_t q1_idx = static_cast<size_t>(0.25 * (stats.samples - 1));
        size_t q3_idx = static_cast<size_t>(0.75 * (stats.samples - 1));
        double iqr_sum = std::accumulate(
            stats.raw_latencies.begin() + q1_idx,
            stats.raw_latencies.begin() + q3_idx + 1,
            0.0);
        stats.iqr_mean_cycles = iqr_sum / (q3_idx - q1_idx + 1);
    }

    if (cpu_freq_ghz > 0.0) {
        stats.mean_ns = stats.mean_cycles / cpu_freq_ghz;
        stats.iqr_mean_ns = stats.iqr_mean_cycles / cpu_freq_ghz;
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

void IntelPebsSampler::cleanup() {
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

