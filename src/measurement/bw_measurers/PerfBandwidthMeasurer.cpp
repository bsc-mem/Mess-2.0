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
#include "SystemDetection.h"
#include "Utils.h"
#include "utils/SubprocessCapture.h"
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
#include <cerrno>
#include <sys/select.h>
#include <sys/time.h>
#include <signal.h>
#include <unistd.h>
#include <sys/wait.h>

namespace {
std::string trim_copy(const std::string& value) {
    const size_t begin = value.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) {
        return "";
    }
    const size_t end = value.find_last_not_of(" \t\r\n");
    return value.substr(begin, end - begin + 1);
}

std::string to_lower_ascii(std::string value) {
    for (char& ch : value) {
        ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    }
    return value;
}

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

int get_perf_bind_cpu() {
    const auto& cache = SystemToolsCache::instance();
    return cache.initialized ? cache.ptr_chase_cpu : find_ptr_chase_cpu().first;
}

bool parse_long_long_token(const std::string& token, long long& value) {
    const std::string trimmed = trim_copy(token);
    if (trimmed.empty()) {
        return false;
    }

    try {
        size_t consumed = 0;
        value = std::stoll(trimmed, &consumed, 10);
        return consumed == trimmed.size();
    } catch (...) {
        return false;
    }
}

bool parse_double_token(const std::string& token, double& value) {
    const std::string trimmed = trim_copy(token);
    if (trimmed.empty()) {
        return false;
    }

    try {
        size_t consumed = 0;
        value = std::stod(trimmed, &consumed);
        return consumed == trimmed.size();
    } catch (...) {
        return false;
    }
}

bool token_looks_like_event_name(const std::string& token) {
    const std::string trimmed = trim_copy(token);
    if (trimmed.empty()) {
        return false;
    }

    // Accept canonical PMU-like names and raw encodings (e.g. r17 / r1a2).
    bool has_name_separator = false;
    for (unsigned char ch : trimmed) {
        if (ch == '/' || ch == '_' || ch == '.' || ch == ':' || ch == '-') {
            has_name_separator = true;
            break;
        }
    }
    if (has_name_separator) {
        return true;
    }

    if ((trimmed.size() >= 2) && (trimmed[0] == 'r' || trimmed[0] == 'R')) {
        for (size_t i = 1; i < trimmed.size(); ++i) {
            if (!std::isxdigit(static_cast<unsigned char>(trimmed[i]))) {
                return false;
            }
        }
        return true;
    }

    return trimmed == "cycles" || trimmed == "instructions";
}

bool token_looks_like_scope_name(const std::string& token) {
    const std::string trimmed = trim_copy(token);
    if (trimmed.empty()) {
        return false;
    }

    if (trimmed.size() >= 2 && (trimmed[0] == 'S' || trimmed[0] == 's')) {
        for (size_t i = 1; i < trimmed.size(); ++i) {
            if (!std::isdigit(static_cast<unsigned char>(trimmed[i]))) {
                return false;
            }
        }
        return true;
    }

    if (trimmed.size() >= 4 &&
        (trimmed.rfind("CPU", 0) == 0 || trimmed.rfind("cpu", 0) == 0)) {
        for (size_t i = 3; i < trimmed.size(); ++i) {
            if (!std::isdigit(static_cast<unsigned char>(trimmed[i]))) {
                return false;
            }
        }
        return true;
    }

    return false;
}

bool token_looks_like_metric_unit(const std::string& token) {
    const std::string trimmed = trim_copy(token);
    if (trimmed.empty()) {
        return false;
    }

    const std::string unit = to_lower_ascii(trimmed);
    return unit == "b" || unit == "kb" || unit == "mb" || unit == "gb" ||
           unit == "tb" || unit == "kib" || unit == "mib" || unit == "gib" ||
           unit == "tib" || unit == "%" || unit == "hz" || unit == "khz" ||
           unit == "mhz" || unit == "ghz";
}

struct PerfCsvRow {
    double timestamp = 0.0;
    std::string scope;
    long long value = 0;
    double metric_value = 0.0;
    std::string metric_unit;
    bool has_metric_value = false;
    std::string event_name;
    bool has_timestamp = false;
};

double metric_unit_to_bytes_multiplier(const std::string& unit_raw) {
    std::string unit = to_lower_ascii(trim_copy(unit_raw));
    if (unit.empty()) return 0.0;

    const size_t slash = unit.find('/');
    if (slash != std::string::npos) {
        unit = unit.substr(0, slash);
    }

    if (unit == "b") return 1.0;
    if (unit == "kb") return 1e3;
    if (unit == "mb") return 1e6;
    if (unit == "gb") return 1e9;
    if (unit == "tb") return 1e12;
    if (unit == "kib") return 1024.0;
    if (unit == "mib") return 1024.0 * 1024.0;
    if (unit == "gib") return 1024.0 * 1024.0 * 1024.0;
    if (unit == "tib") return 1024.0 * 1024.0 * 1024.0 * 1024.0;
    return 0.0;
}

long long normalize_perf_row_value(const PerfCsvRow& row, int cache_line_size) {
    if (!row.has_metric_value) {
        return row.value;
    }

    const double bytes_multiplier = metric_unit_to_bytes_multiplier(row.metric_unit);
    if (bytes_multiplier > 0.0) {
        const double bytes = row.metric_value * bytes_multiplier;
        if (cache_line_size > 0) {
            return static_cast<long long>(std::llround(bytes / static_cast<double>(cache_line_size)));
        }
        return static_cast<long long>(std::llround(bytes));
    }

    return static_cast<long long>(std::llround(row.metric_value));
}

bool parse_perf_stat_csv_row(const std::string& line, bool expect_timestamp, PerfCsvRow& row) {
    std::vector<std::string> fields;
    std::stringstream ss(line);
    std::string field;
    while (std::getline(ss, field, ',')) {
        fields.push_back(trim_copy(field));
    }

    if (fields.empty()) {
        return false;
    }

    row = PerfCsvRow{};

    size_t index = 0;
    if (expect_timestamp) {
        if (!parse_double_token(fields[index], row.timestamp)) {
            return false;
        }
        row.has_timestamp = true;
        ++index;
    }
    long long parsed_value = 0;

    auto assign_raw = [&](size_t scope_index, size_t value_index, size_t event_index) -> bool {
        if (value_index >= fields.size() || event_index >= fields.size()) {
            return false;
        }
        if (!parse_long_long_token(fields[value_index], parsed_value)) {
            return false;
        }
        if (!token_looks_like_event_name(fields[event_index])) {
            return false;
        }
        row.scope = (scope_index < fields.size()) ? fields[scope_index] : "";
        row.value = parsed_value;
        row.has_metric_value = false;
        row.metric_value = 0.0;
        row.metric_unit.clear();
        row.event_name = fields[event_index];
        return true;
    };

    auto assign_metric = [&](size_t scope_index, size_t metric_value_index, size_t unit_index, size_t event_index) -> bool {
        if (metric_value_index >= fields.size() || event_index >= fields.size()) {
            return false;
        }
        if (unit_index < fields.size() && !fields[unit_index].empty() &&
            !token_looks_like_metric_unit(fields[unit_index])) {
            return false;
        }
        if (!token_looks_like_event_name(fields[event_index])) {
            return false;
        }
        double metric_value = 0.0;
        if (!parse_double_token(fields[metric_value_index], metric_value)) {
            return false;
        }
        row.scope = (scope_index < fields.size()) ? fields[scope_index] : "";
        row.value = static_cast<long long>(std::llround(metric_value));
        row.metric_value = metric_value;
        row.metric_unit = (unit_index < fields.size()) ? fields[unit_index] : "";
        row.has_metric_value = true;
        row.event_name = fields[event_index];
        return true;
    };

    const bool has_scope = (index < fields.size()) && token_looks_like_scope_name(fields[index]);

    // Most specific first: metric-output rows with explicit raw/runtime fields after event.
    // Example A:
    //   time,S0,1,1491.27,MiB,uncore_imc_0/cas_count_read/,51018789,...
    if (has_scope && assign_metric(index, index + 2, index + 3, index + 4)) {
        // Ambiguous shape guard: if the parsed "event" token is actually a unit token,
        // retry with the 7-column scoped metric layout below.
        if (token_looks_like_metric_unit(row.event_name)) {
            row = PerfCsvRow{};
        } else {
            return true;
        }
    }
    // Example B:
    //   time,S0,1,1491.27,MiB,uncore_imc_0/cas_count_read/,51018789,...
    if (has_scope && assign_metric(index, index + 3, index + 4, index + 5)) {
        return true;
    }
    // Example C:
    //   time,S0,1491.27,MiB,uncore_imc_0/cas_count_read/,51018789,...
    if (has_scope && assign_metric(index, index + 1, index + 2, index + 3)) {
        return true;
    }

    // Old raw-count CSV rows without unit/metric column.
    // Example:
    //   time,S0,1,51018789,,uncore_imc_0/cas_count_read/,...
    if (has_scope && assign_raw(index, index + 2, index + 4)) {
        return true;
    }
    // Example:
    //   time,S0,51018789,,uncore_imc_0/cas_count_read/,...
    if (has_scope && assign_raw(index, index + 1, index + 3)) {
        return true;
    }

    // PID/system-wide non-scoped rows.
    // Example:
    //   time,1491.27,MiB,uncore_imc/cas_count_read/,51018789,...
    if (!has_scope && assign_metric(fields.size(), index, index + 1, index + 2)) {
        return true;
    }
    // Example:
    //   time,51018789,,uncore_imc/cas_count_read/,...
    if (!has_scope && assign_raw(fields.size(), index, index + 2)) {
        return true;
    }

    return false;
}

bool use_per_socket_aggregation(const BandwidthCounterSelection& selection) {
    if (selection.type != CounterType::CAS_COUNT && selection.type != CounterType::NVIDIA_GRACE) {
        return true;
    }
    return !selection.cas.uses_read_subtract_formula;
}

std::pair<std::string, std::string> detect_unc_m_aggregate_events() {
    static std::pair<std::string, std::string> cached;
    static bool initialized = false;
    if (initialized) {
        return cached;
    }
    initialized = true;

    // perf list can be hundreds of KiB on big-iron — cap accordingly.
    subprocess::Options opts;
    opts.timeout          = std::chrono::seconds(10);
    opts.max_output_bytes = 262144;
    const std::string output = subprocess::run_shell("perf list 2>/dev/null", opts);

    std::istringstream stream(output);
    std::string line;
    while (std::getline(stream, line)) {
        std::istringstream ls(line);
        std::string token;
        ls >> token;
        if (token.empty()) {
            continue;
        }
        if (to_lower_ascii(token) == "unc_m_cas_count.rd") {
            cached.first = "unc_m_cas_count.rd";
        } else if (to_lower_ascii(token) == "unc_m_cas_count.wr") {
            cached.second = "unc_m_cas_count.wr";
        }
    }

    return cached;
}

std::string build_preferred_perf_events(const BandwidthCounterSelection& selection) {
    std::string events = selection.get_all_events_string();

    if (selection.type != CounterType::CAS_COUNT || selection.cas.uses_read_subtract_formula) {
        return events;
    }

    const auto agg = detect_unc_m_aggregate_events();
    if (agg.first.empty() || agg.second.empty()) {
        return events;
    }

    std::string preferred = agg.first + "," + agg.second;
    for (const auto& extra : selection.extra_counters) {
        if (!extra.empty()) {
            preferred += ",";
            preferred += extra;
        }
    }
    return preferred;
}

bool use_pid_attach_mode(const BandwidthCounterSelection& selection) {
    return selection.cas.uses_read_subtract_formula;
}

std::string join_pid_list(const std::vector<pid_t>& pids) {
    std::string joined;
    for (size_t i = 0; i < pids.size(); ++i) {
        if (i > 0) {
            joined += ",";
        }
        joined += std::to_string(pids[i]);
    }
    return joined;
}

bool is_pid_alive(pid_t pid) {
    if (pid <= 0) {
        return false;
    }
    if (kill(pid, 0) == 0) {
        return true;
    }
    return errno == EPERM;
}

std::string build_perf_scope_args(const BandwidthCounterSelection& selection,
                                  const TrafficGenProcessManager* traffic_gen_manager) {
    if (!use_pid_attach_mode(selection)) {
        return use_per_socket_aggregation(selection) ? "-a --per-socket" : "-a";
    }

    if (!traffic_gen_manager) {
        return "";
    }

    const std::vector<pid_t> worker_pids = traffic_gen_manager->traffic_gen_worker_pids();
    if (worker_pids.empty()) {
        return "";
    }

    return "-p " + join_pid_list(worker_pids);
}

void apply_bandwidth_formula(const BandwidthCounterSelection& selection,
                             long long raw_rd,
                             long long raw_wr,
                             const std::map<std::string, long long>& extras,
                             long long& effective_rd,
                             long long& effective_wr) {
    effective_rd = raw_rd;
    effective_wr = raw_wr;

    if (!selection.cas.uses_read_subtract_formula) {
        return;
    }

    long long subtract = 0;
    for (const auto& event : selection.cas.read_subtract_events) {
        auto it = extras.find(event);
        if (it != extras.end()) {
            subtract += it->second;
        }
    }

    effective_rd = std::max(0LL, raw_rd - subtract);
}
}

bool PerfBandwidthMeasurer::sample_bandwidth(long long& cas_rd, long long& cas_wr, double& elapsed, const std::vector<int>& mem_nodes) const {
    return sample_with_popen(cas_rd, cas_wr, elapsed, mem_nodes);
}

bool PerfBandwidthMeasurer::sample_with_popen(long long& cas_rd, long long& cas_wr, double& elapsed, const std::vector<int>& mem_nodes) const {
    const bool verbose4 = config_.verbosity >= 4;
    extra_perf_values_.clear();
    ensure_scaling_factor_cached();
    const int cache_line_size = std::max(1, cached_cache_line_size_);

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

    std::string events_str = build_preferred_perf_events(selection);
    const int perf_bind_cpu = get_perf_bind_cpu();
    const bool attach_mode = use_pid_attach_mode(selection);
    const std::string scope_args = build_perf_scope_args(selection, traffic_gen_manager_);
    if (scope_args.empty()) {
        if (attach_mode) {
            std::cerr << "ERROR: No TrafficGen worker PIDs available for A64FX perf sampling." << std::endl;
        }
        return false;
    }

    ensure_a64fx_pmu_available();

    std::string cmd = std::string("cd ") + storage_->bandwidth_dir() + " && echo \"__PID__$$\" && exec stdbuf -oL -eL ";
    if (!attach_mode) {
        (void)perf_bind_cpu;
        (void)membind_arg;
        // For system-wide perf stat (-a/--per-socket), avoid numactl wrapping.
        // Some Emerald Rapids/CXL setups can hang with:
        //   numactl ... perf stat -a --per-socket ...
    }
    cmd += "perf stat -I " + std::to_string(interval) +
           " -x, " + scope_args + " -e " + events_str + " 2>&1";

    if(verbose4){std::cout << "[SAMPLING CMD] " << cmd << '\n';}

    FILE* pipe = popen(cmd.c_str(), "r");
    if (!pipe) {
        return false;
    }

    setbuf(pipe, NULL);

    pid_t perf_stat_pid = -1;
    char pid_buf[256];
    if (fgets(pid_buf, sizeof(pid_buf), pipe)) {
        const char* marker = strstr(pid_buf, "__PID__");
        if (marker) {
            perf_stat_pid = static_cast<pid_t>(atol(marker + 7));
        }
    }

    if (verbose4) {
        std::cout << "      [PERF DEBUG] attach_mode=" << (attach_mode ? "yes" : "no")
                  << ", scope=\"" << scope_args << "\""
                  << ", perf_pid=" << perf_stat_pid << '\n';
        std::cout << "      [PERF DEBUG] command: " << cmd << '\n';
    }

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

        PerfCsvRow row;
        if (parse_perf_stat_csv_row(buffer, true, row)) {
            if (row.scope == "S1") {
                continue;
            }
            double timestamp = row.timestamp;
            long long value = normalize_perf_row_value(row, cache_line_size);
            const std::string& event_name = row.event_name;

            if (last_timestamp >= 0 && timestamp != last_timestamp) {
                long long effective_rd = 0;
                long long effective_wr = 0;
                apply_bandwidth_formula(selection, current_sample_rd, current_sample_wr, current_extra, effective_rd, effective_wr);
                double duration = timestamp - last_timestamp;
                if (duration < 0.000001) duration = interval / 1000.0;

                total_samples_seen++;
                bool sample_valid = false;
                if (using_combined) {
                    sample_valid = (current_sample_combined > 0);
                } else {
                    sample_valid = (effective_rd + effective_wr > 0);
                }
                if (sample_valid && total_samples_seen > warmup_samples) {
                    sum_rd += effective_rd;
                    sum_wr += effective_wr;
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

    long long effective_rd = 0;
    long long effective_wr = 0;
    apply_bandwidth_formula(selection, current_sample_rd, current_sample_wr, current_extra, effective_rd, effective_wr);
    bool sample_valid = using_combined 
        ? (current_sample_combined > 0) 
        : (effective_rd + effective_wr > 0);
    if (samples_read < samples_to_read && sample_valid) {
        sum_rd += effective_rd;
        sum_wr += effective_wr;
        sum_combined += current_sample_combined;
        for (const auto& kv : current_extra) {
            sum_extra[kv.first] += kv.second;
        }
        samples_read++;
    }

    if (perf_stat_pid > 0) {
        kill(perf_stat_pid, SIGTERM);
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
    extra_perf_values_.clear();
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
        std::cout << "      [Stabilizer] Theoretical peak: " << std::fixed << std::setprecision(1) 
                  << full_peak << " GB/s, Achievable (" << expected_cores << " cores): "
                  << theoretical_peak_gb_s << " GB/s, Noise floor: " 
                  << TheoreticalPeakCalculator::get_noise_floor(theoretical_peak_gb_s) << " GB/s" << '\n';
    }
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
    std::string events_str = build_preferred_perf_events(selection);
    const int perf_bind_cpu = get_perf_bind_cpu();
    const bool attach_mode = use_pid_attach_mode(selection);
    const std::string scope_args = build_perf_scope_args(selection, traffic_gen_manager_);
    if (scope_args.empty()) {
        if (attach_mode) {
            std::cerr << "ERROR: No TrafficGen worker PIDs available for A64FX perf sampling." << std::endl;
        }
        return false;
    }

    ensure_a64fx_pmu_available();

    std::string cmd = std::string("cd ") + storage_->bandwidth_dir() + " && echo \"__PID__$$\" && exec stdbuf -oL -eL ";
    if (!attach_mode) {
        (void)perf_bind_cpu;
        (void)membind_arg;
        // For system-wide perf stat (-a/--per-socket), avoid numactl wrapping.
        // Some Emerald Rapids/CXL setups can hang with:
        //   numactl ... perf stat -a --per-socket ...
    }
    cmd += "perf stat -I " + std::to_string(interval) +
           " -x, " + scope_args + " -e " + events_str + " 2>&1";

    FILE* pipe = popen(cmd.c_str(), "r");
    if (!pipe) {
        if (verbose2) {
            std::cout << "    Warning: perf stat failed, falling back to configured wait time" << '\n';
        }
        return false;
    }

    setbuf(pipe, NULL);
    int pipe_fd = fileno(pipe);

    pid_t perf_stat_pid = -1;
    char pid_buf[256];
    if (fgets(pid_buf, sizeof(pid_buf), pipe)) {
        const char* marker = strstr(pid_buf, "__PID__");
        if (marker) {
            perf_stat_pid = static_cast<pid_t>(atol(marker + 7));
        }
    }

    auto kill_and_pclose = [&]() {
        if (perf_stat_pid > 0) {
            kill(perf_stat_pid, SIGTERM);
        }
        pclose(pipe);
        pipe = nullptr;
    };

    BandwidthStabilizer stabilizer(
        theoretical_peak_gb_s, pause, health_checker,
        fast_resume ? 5 : 7, 0.05, config_.verbosity);
    
    int total_samples = 0;
    const int OVERALL_TIMEOUT_SECONDS = 60;
    int max_samples = static_cast<int>((OVERALL_TIMEOUT_SECONDS * 1000.0) / sampling_interval_ms_);
    
    char buffer[4096];
    bool success = false;
    
    long long sample_cas_rd = 0, sample_cas_wr = 0;
    std::map<std::string, long long> sample_extra;
    std::map<std::string, long long> aggregated_extra;
    std::map<std::string, long long> final_extra;
    double last_timestamp = -1.0;

    long long final_cas_rd = 0, final_cas_wr = 0;
    double final_elapsed = sampling_interval_ms_ / 1000.0;

    auto loop_start = std::chrono::steady_clock::now();
    const int READ_TIMEOUT_SECONDS = 10;
    int consecutive_timeouts = 0;
    long long last_heartbeat_s = -1;
    int parse_failures = 0;

    while (total_samples < max_samples) {
        auto elapsed_total = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - loop_start).count();
        if (verbose4 && (last_heartbeat_s < 0 || elapsed_total - last_heartbeat_s >= 5)) {
            last_heartbeat_s = elapsed_total;
            std::cout << "      [PERF DEBUG] heartbeat t=" << elapsed_total
                      << "s, samples=" << total_samples
                      << ", timeouts=" << consecutive_timeouts
                      << ", perf_alive=" << (is_pid_alive(perf_stat_pid) ? "yes" : "no")
                      << ", tg_alive="
                      << (traffic_gen_manager_->is_traffic_gen_running(traffic_gen_manager_->active_traffic_gen_pid()) ? "yes" : "no")
                      << '\n';
        }
        
        if (elapsed_total > OVERALL_TIMEOUT_SECONDS) {
            std::cerr << "\n    ERROR: Perf stabilization timed out after " << OVERALL_TIMEOUT_SECONDS << " seconds." << '\n';
            if (verbose3) {
                std::cerr << "    Possible causes:\n"
                          << "      - perf stat command hung\n"
                          << "      - Performance counters unavailable\n"
                          << "      - System overloaded\n"
                          << "    Command: " << cmd << '\n';
            }
            kill_and_pclose();
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
            kill_and_pclose();
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
                kill_and_pclose();
                traffic_gen_manager_->kill_all_traffic_gen();
                return false;
            }
            continue;
        }
        
        consecutive_timeouts = 0;
        
        if (!fgets(buffer, sizeof(buffer), pipe)) {
            if (verbose4) {
                std::cout << "      [PERF DEBUG] fgets returned EOF/NULL from perf pipe" << '\n';
            }
            break;
        }
        if (buffer[0] == '#' || strstr(buffer, "time") || strlen(buffer) < 5) {
            if (verbose4) {
                std::string line(buffer);
                if (line.find("not supported") != std::string::npos ||
                    line.find("No permission") != std::string::npos ||
                    line.find("failed") != std::string::npos ||
                    line.find("Error") != std::string::npos) {
                    std::cout << "      [PERF DEBUG] diagnostic line: " << line;
                }
            }
            continue;
        }

        PerfCsvRow row;
        const bool parsed = parse_perf_stat_csv_row(buffer, true, row);
        
        if (verbose4) {
            std::cout << "      [PERF RAW] parsed=" << parsed << " line: " << buffer;
        }
        if (!parsed && verbose4 && parse_failures < 20) {
            ++parse_failures;
            std::cout << "      [PERF DEBUG] parse-fail[" << parse_failures << "]: " << buffer;
        }
        
        if (parsed) {
            if (row.scope == "S1") {
                continue;
            }
            double timestamp = row.timestamp;
            long long value = normalize_perf_row_value(row, std::max(1, cached_cache_line_size_));
            const std::string& event_name = row.event_name;

            if (last_timestamp >= 0 && timestamp != last_timestamp) {
                long long effective_rd = 0;
                long long effective_wr = 0;
                apply_bandwidth_formula(selection, sample_cas_rd, sample_cas_wr, sample_extra, effective_rd, effective_wr);
                double duration = timestamp - last_timestamp;
                double target_duration = sampling_interval_ms_ / 1000.0;
                double normalization_factor = 1.0;
                
                if (duration > 0.000001) {
                    normalization_factor = target_duration / duration;
                }

                if (effective_rd + effective_wr > 0) {
                    long long norm_rd = static_cast<long long>(effective_rd * normalization_factor);
                    long long norm_wr = static_cast<long long>(effective_wr * normalization_factor);
                    long long total_cas = norm_rd + norm_wr;
                    long long total_cas_reg = effective_rd + effective_wr;
                    
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
                                kill_and_pclose();
                                return false;
                            }
                        } else {
                            kill_and_pclose();
                            return false;
                        }
                    }

                    if (total_samples > max_samples) {
                        if (config_.verbosity >= 1) {
                            std::cout << "\n    ⚠ WARNING: BW did not stabilize after " << OVERALL_TIMEOUT_SECONDS << "s" << '\n';
                            std::cout << "    Killing TrafficGen and triggering retry..." << '\n';
                        }
                        kill_and_pclose();
                        traffic_gen_manager_->kill_all_traffic_gen();
                        std::this_thread::sleep_for(std::chrono::milliseconds(200));

                        samples_taken = total_samples;
                        return false;
                    }

                    final_cas_rd = norm_rd;
                    final_cas_wr = norm_wr;
                    final_elapsed = target_duration;
                    final_extra = sample_extra;
                    for (const auto& kv : sample_extra) {
                        aggregated_extra[kv.first] += kv.second;
                    }

                    double bw_gb_s = calculate_bw_gb_s(norm_rd, norm_wr, target_duration);
                    StabilizationResult result = stabilizer.add_sample(norm_rd, norm_wr, bw_gb_s);

                    if (result == StabilizationResult::ZOMBIE_DETECTED) {
                        if (verbose2) {
                            std::cout << "    [ZOMBIE] TrafficGen died, aborting stabilization" << '\n';
                        }
                        kill_and_pclose();
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
                                read_ratio = static_cast<double>(effective_rd) / static_cast<double>(total_cas_reg);
                            }
                        }
                        
                        std::string counter_label = "CAS";
                        if (selection.type == CounterType::UPI_FLITS) counter_label = "UPI";
                        else if (selection.type == CounterType::NVIDIA_GRACE) counter_label = "GRACE";
                        else if (selection.cas.uses_read_subtract_formula) counter_label = "A64FX";
                        
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
                            kill_and_pclose();
                            samples_taken = total_samples;
                            return false;
                        }
                    }
                }
                sample_cas_rd = 0;
                sample_cas_wr = 0;
                EventClassification::reset_extra_values(sample_extra);
            }

            EventClassification ec = EventClassification::classify(event_name, selection);

            if (verbose4) {
                std::cout << "      [EVENT] " << event_name << " -> rd:" << ec.is_read 
                          << " wr:" << ec.is_write << " comb:" << ec.is_combined 
                          << " val:" << value << '\n';
            }

            if (ec.is_extra) {
                EventClassification::accumulate_extra(sample_extra, ec.extra_key, value);
            } else if (ec.is_combined || ec.is_read) {
                sample_cas_rd += value;
            } else if (ec.is_write) {
                sample_cas_wr += value;
            }

            last_timestamp = timestamp;
        }
    }

    kill_and_pclose();

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
        extra_perf_values_ = aggregated_extra;
        
        samples_taken = total_samples;
        return true;
    } else {
        last_cas_rd = final_cas_rd;
        last_cas_wr = final_cas_wr;
        last_elapsed = final_elapsed;
        extra_perf_values_ = final_extra;
        samples_taken = total_samples;
        return false;
    }
}

bool PerfBandwidthMeasurer::monitor_command(const std::string& command, 
                                            MonitorCallback callback,
                                            bool summary_mode) {
    extra_perf_values_.clear();
    
    std::vector<int> mem_nodes = numa_resolver_ ? numa_resolver_() : std::vector<int>{0};

    const BandwidthCounterSelection& selection = counter_selection_;
    if (!selection.is_valid()) {
        std::cerr << "ERROR: Counter selection not initialized. "
                  << "BandwidthCounterStrategy must be initialized first." << std::endl;
        return false;
    }

    std::string events_str = build_preferred_perf_events(selection);

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
        perf_cmd << "-a ";
        if (use_per_socket_aggregation(selection)) {
            perf_cmd << "--per-socket ";
        }
    }
    
    if (!summary_mode) {
        int interval_ms = static_cast<int>(sampling_interval_ms_);
        perf_cmd << "-I " << interval_ms << " ";
    }
    
    perf_cmd << "-e " << events_str << " ";
    perf_cmd << trimmed_command;
    perf_cmd << " 2>&1";

    ensure_a64fx_pmu_available();

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
        long long effective_rd = 0;
        long long effective_wr = 0;
        apply_bandwidth_formula(selection, current_rd, current_wr, current_extra, effective_rd, effective_wr);
        double bw = calculate_bandwidth_gbps(effective_rd, effective_wr, duration_s,
                                             selection.type, cached_cache_line_size_, cached_scaling_factor_);
        if (callback) {
            callback(timestamp, bw, effective_rd, effective_wr, current_extra);
        }
    };

    while (fgets(buffer, sizeof(buffer), pipe)) {
        if (buffer[0] == '#' || strstr(buffer, "time") || strlen(buffer) < 5) {
            continue;
        }

        PerfCsvRow row;

        if (!summary_mode) {
            if (!parse_perf_stat_csv_row(buffer, true, row)) {
                continue;
            }
            if (row.scope == "S1") {
                continue;
            }
        } else {
            if (!parse_perf_stat_csv_row(buffer, false, row)) {
                continue;
            }
            if (row.scope == "S1") {
                continue;
            }
            row.timestamp = 0.0;
        }

        if (!summary_mode) {
            float timestamp = static_cast<float>(row.timestamp);
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

        EventClassification ec = EventClassification::classify(row.event_name, selection);
        const long long row_value = normalize_perf_row_value(row, std::max(1, cached_cache_line_size_));

        if (ec.is_extra) {
            EventClassification::accumulate_extra(current_extra, ec.extra_key, row_value);
            EventClassification::accumulate_extra(total_extra, ec.extra_key, row_value);
        } else if (ec.is_read || ec.is_combined) {
            current_rd += row_value;
        } else if (ec.is_write) {
            current_wr += row_value;
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
