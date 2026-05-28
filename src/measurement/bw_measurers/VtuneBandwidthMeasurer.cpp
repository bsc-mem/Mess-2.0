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

#include "measurement/bw_measurers/VtuneBandwidthMeasurer.h"
#include "measurement/BandwidthStabilizer.h"
#include "measurement/TheoreticalPeakCalculator.h"
#include "Utils.h"
#include "utils/SubprocessCapture.h"
#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <map>
#include <regex>
#include <sstream>
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

std::string format_seconds(double seconds) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3) << seconds;
    return oss.str();
}

std::string choose_vtune_result_root() {
    const char* candidates[] = {"/dev/shm", "/tmp"};
    for (const char* candidate : candidates) {
        std::error_code ec;
        const std::filesystem::path path(candidate);
        if (std::filesystem::exists(path, ec) &&
            std::filesystem::is_directory(path, ec) &&
            access(candidate, W_OK | X_OK) == 0) {
            return candidate;
        }
    }
    return "/tmp";
}

std::string first_meaningful_vtune_error(const std::string& output) {
    std::istringstream iss(output);
    std::string line;
    while (std::getline(iss, line)) {
        const std::string trimmed = trim_copy(line);
        if (trimmed.empty()) {
            continue;
        }
        if (trimmed.find("vtune: Error:") != std::string::npos) {
            return trimmed;
        }
    }
    return "";
}

std::string canonicalize_vtune_event_name(const std::string& event) {
    if (event.empty()) {
        return event;
    }
    if (event.find('/') != std::string::npos || event[0] == 'r') {
        return event;
    }

    std::string canonical = event;
    for (char& c : canonical) {
        if (std::islower(static_cast<unsigned char>(c))) {
            c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
        }
    }
    return canonical;
}

std::string to_lower_copy(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}

std::vector<std::string> vtune_event_aliases(const std::string& requested_event) {
    std::vector<std::string> aliases;
    const std::string canonical = canonicalize_vtune_event_name(requested_event);
    if (!canonical.empty()) {
        aliases.push_back(canonical);
    }

    const std::string lower = to_lower_copy(requested_event);
    if (lower == "instructions") {
        aliases.push_back("INST_RETIRED.ANY");
    } else if (lower == "cycles" || lower == "cpu-cycles") {
        aliases.push_back("CPU_CLK_UNHALTED.THREAD");
    } else if (lower == "ref-cycles") {
        aliases.push_back("CPU_CLK_UNHALTED.REF_TSC");
    }

    std::sort(aliases.begin(), aliases.end());
    aliases.erase(std::unique(aliases.begin(), aliases.end()), aliases.end());
    return aliases;
}

std::string regex_escape(const std::string& value) {
    std::string escaped;
    escaped.reserve(value.size() * 2);
    for (char c : value) {
        switch (c) {
            case '\\':
            case '^':
            case '$':
            case '.':
            case '|':
            case '?':
            case '*':
            case '+':
            case '(':
            case ')':
            case '[':
            case ']':
            case '{':
            case '}':
                escaped.push_back('\\');
                escaped.push_back(c);
                break;
            default:
                escaped.push_back(c);
                break;
        }
    }
    return escaped;
}

std::string build_vtune_event_config(const BandwidthCounterSelection& selection) {
    std::vector<std::string> events;

    if (selection.type == CounterType::CAS_COUNT || selection.type == CounterType::NVIDIA_GRACE) {
        events.insert(events.end(), selection.cas.read_events.begin(), selection.cas.read_events.end());
        events.insert(events.end(), selection.cas.write_events.begin(), selection.cas.write_events.end());
        events.insert(events.end(), selection.cas.combined_events.begin(), selection.cas.combined_events.end());
    } else if (selection.type == CounterType::UPI_FLITS) {
        events.insert(events.end(), selection.upi.rxl_data_events.begin(), selection.upi.rxl_data_events.end());
        events.insert(events.end(), selection.upi.txl_data_events.begin(), selection.upi.txl_data_events.end());
    }

    events.insert(events.end(), selection.extra_counters.begin(), selection.extra_counters.end());

    std::ostringstream oss;
    for (size_t i = 0; i < events.size(); ++i) {
        if (i > 0) {
            oss << ",";
        }
        const std::vector<std::string> aliases = vtune_event_aliases(events[i]);
        oss << (aliases.empty() ? canonicalize_vtune_event_name(events[i]) : aliases.front());
    }
    return oss.str();
}

long long parse_counter_value(const std::string& raw) {
    std::string digits;
    digits.reserve(raw.size());
    for (char c : raw) {
        if (std::isdigit(static_cast<unsigned char>(c))) {
            digits.push_back(c);
        }
    }
    if (digits.empty()) {
        return 0;
    }
    try {
        return std::stoll(digits);
    } catch (...) {
        return 0;
    }
}

bool is_pid_attach_command(const std::string& command) {
    return command.rfind("-p ", 0) == 0 || command.rfind("--pid ", 0) == 0;
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

struct VtuneSample {
    bool success = false;
    double elapsed_s = 0.0;
    long long cas_rd = 0;
    long long cas_wr = 0;
    std::map<std::string, long long> extra_values;
    std::string error;
    std::string vtune_command;
    std::string raw_output;
    int exit_code = -1;
    bool elapsed_found = false;
};

bool parse_vtune_counter_line(const std::string& line,
                              const BandwidthCounterSelection& selection,
                              long long& cas_rd,
                              long long& cas_wr,
                              std::map<std::string, long long>& extra_values,
                              int verbosity) {
    const std::string trimmed = trim_copy(line);
    if (trimmed.empty()) {
        return false;
    }

    size_t value_end = trimmed.size();
    while (value_end > 0 && std::isspace(static_cast<unsigned char>(trimmed[value_end - 1]))) {
        value_end--;
    }
    if (value_end == 0) {
        return false;
    }

    size_t value_start = value_end;
    while (value_start > 0) {
        char c = trimmed[value_start - 1];
        if (std::isdigit(static_cast<unsigned char>(c)) || c == ',') {
            value_start--;
            continue;
        }
        break;
    }

    if (value_start == value_end) {
        return false;
    }

    const std::string value_token = trimmed.substr(value_start, value_end - value_start);
    std::string event_name = trim_copy(trimmed.substr(0, value_start));
    if (event_name.empty()) {
        return false;
    }
    if (event_name.find("Event Type") != std::string::npos) {
        return false;
    }

    const long long value = parse_counter_value(value_token);
    EventClassification classification = EventClassification::classify(event_name, selection);

    if (verbosity >= 4) {
        std::cout << "[VTUNE DEBUG] counter-line: " << event_name
                  << " value=" << value
                  << " rd=" << classification.is_read
                  << " wr=" << classification.is_write
                  << " comb=" << classification.is_combined
                  << " extra=" << classification.is_extra
                  << std::endl;
    }

    if (classification.is_combined) {
        cas_rd += value;
    } else if (classification.is_read) {
        cas_rd += value;
    } else if (classification.is_write) {
        cas_wr += value;
    } else if (classification.is_extra) {
        EventClassification::accumulate_extra(extra_values, classification.extra_key, value);
    }

    return true;
}

int accumulate_exact_vtune_event_matches(const std::string& output,
                                         const std::vector<std::string>& events,
                                         long long& total,
                                         int verbosity,
                                         const char* label) {
    int matches = 0;
    for (const auto& raw_event : events) {
        const std::string event = canonicalize_vtune_event_name(raw_event);
        if (event.empty()) {
            continue;
        }

        const std::regex pattern(
            "(?:^|\\n)\\s*" + regex_escape(event) + R"((?:\[[^\]\n]+\])?\s+([0-9][0-9,]*))",
            std::regex::icase);

        for (std::sregex_iterator it(output.begin(), output.end(), pattern), end; it != end; ++it) {
            const long long value = parse_counter_value((*it)[1].str());
            total += value;
            matches++;

            if (verbosity >= 4) {
                std::cout << "[VTUNE DEBUG] exact-match[" << label << "]: "
                          << trim_copy((*it)[0].str())
                          << " value=" << value << std::endl;
            }
        }
    }
    return matches;
}

int accumulate_exact_vtune_extra_matches(const std::string& output,
                                         const std::vector<std::string>& events,
                                         std::map<std::string, long long>& extra_values,
                                         int verbosity) {
    int matches = 0;
    for (const auto& requested_event : events) {
        const std::vector<std::string> aliases = vtune_event_aliases(requested_event);
        if (aliases.empty()) {
            continue;
        }

        long long total = 0;
        for (const auto& alias : aliases) {
            const std::regex pattern(
                "(?:^|\\n)\\s*" + regex_escape(alias) + R"((?:\[[^\]\n]+\])?\s+([0-9][0-9,]*))",
                std::regex::icase);

            for (std::sregex_iterator it(output.begin(), output.end(), pattern), end; it != end; ++it) {
                const long long value = parse_counter_value((*it)[1].str());
                total += value;
                matches++;

                if (verbosity >= 4) {
                    std::cout << "[VTUNE DEBUG] exact-match[extra]: "
                              << trim_copy((*it)[0].str())
                              << " value=" << value
                              << " requested=" << requested_event << std::endl;
                }
            }
        }

        if (total > 0) {
            extra_values[requested_event] += total;
        }
    }
    return matches;
}

VtuneSample run_vtune_collection(const std::string& vtune_binary,
                                 const BandwidthCounterSelection& selection,
                                 double duration_seconds,
                                 const std::string& command,
                                 int verbosity) {
    VtuneSample sample;

    if (!selection.is_valid()) {
        sample.error = "Counter selection not initialized";
        return sample;
    }

    std::string tmp_template = choose_vtune_result_root() + "/mess_vtune_XXXXXX";
    std::vector<char> tmp_buffer(tmp_template.begin(), tmp_template.end());
    tmp_buffer.push_back('\0');
    char* result_dir = mkdtemp(tmp_buffer.data());
    if (!result_dir) {
        sample.error = "Failed to create temporary VTune result directory";
        return sample;
    }

    const std::filesystem::path result_path(result_dir);
    const std::string event_config = build_vtune_event_config(selection);
    std::string collect_cmd =
        shell_quote(vtune_binary) +
        " -collect-with runsa -quiet -result-dir " + shell_quote(result_path.string()) +
        " -finalization-mode=fast -discard-raw-data -no-summary" +
        " -knob enable-stack-collection=false" +
        " -knob sampling-interval=10" +
        " -knob event-config=" + shell_quote(event_config);
    if (duration_seconds >= 1.0) {
        collect_cmd += " -duration=" + format_seconds(duration_seconds);
    }
    if (!command.empty()) {
        collect_cmd += " -- " + command;
    }
    collect_cmd += " 2>&1";
    const std::string report_cmd =
        shell_quote(vtune_binary) +
        " -report summary -quiet -result-dir " + shell_quote(result_path.string()) +
        " 2>&1";
    sample.vtune_command = collect_cmd + " && " + report_cmd;

    if (verbosity >= 4) {
        std::cout << "[VTUNE DEBUG] collect-command: " << collect_cmd << std::endl;
        std::cout << "[VTUNE DEBUG] report-command: " << report_cmd << std::endl;
        std::cout << "[VTUNE DEBUG] result-dir: " << result_path << std::endl;
        std::cout << "[VTUNE DEBUG] events: " << event_config << std::endl;
    }

    FILE* pipe = popen(collect_cmd.c_str(), "r");
    if (!pipe) {
        std::filesystem::remove_all(result_path);
        sample.error = "Failed to launch VTune";
        return sample;
    }

    std::string collect_output;
    char buffer[4096];
    while (fgets(buffer, sizeof(buffer), pipe)) {
        collect_output += buffer;
    }
    int rc = pclose(pipe);
    if (rc != -1 && WIFEXITED(rc)) {
        sample.exit_code = WEXITSTATUS(rc);
    }
    const bool collect_ok = (rc != -1) && WIFEXITED(rc) && (WEXITSTATUS(rc) == 0);
    if (!collect_ok) {
        sample.raw_output = collect_output;
        std::filesystem::remove_all(result_path);
        const std::string vtune_error = first_meaningful_vtune_error(collect_output);
        sample.error = !vtune_error.empty() ? vtune_error : "VTune collect command failed";
        return sample;
    }

    pipe = popen(report_cmd.c_str(), "r");
    if (!pipe) {
        sample.raw_output = collect_output;
        std::filesystem::remove_all(result_path);
        sample.error = "Failed to launch VTune summary report";
        return sample;
    }

    std::string output;
    while (fgets(buffer, sizeof(buffer), pipe)) {
        output += buffer;
    }
    rc = pclose(pipe);
    if (rc != -1 && WIFEXITED(rc)) {
        sample.exit_code = WEXITSTATUS(rc);
    }
    const bool command_ok = (rc != -1) && WIFEXITED(rc) && (WEXITSTATUS(rc) == 0);
    sample.raw_output = collect_output;
    if (!sample.raw_output.empty() && !output.empty()) {
        sample.raw_output += "\n";
    }
    sample.raw_output += output;
    std::filesystem::remove_all(result_path);

    const std::string vtune_error = first_meaningful_vtune_error(sample.raw_output);

    std::smatch elapsed_match;
    if (std::regex_search(output, elapsed_match, std::regex(R"(Elapsed Time:\s*([0-9]+(?:\.[0-9]+)?))"))) {
        try {
            sample.elapsed_s = std::stod(elapsed_match[1].str());
            sample.elapsed_found = true;
        } catch (...) {
            sample.elapsed_s = 0.0;
        }
    }

    if (verbosity >= 4) {
        std::cout << "[VTUNE DEBUG] exit-code: " << sample.exit_code
                  << " elapsed-found: " << (sample.elapsed_found ? "yes" : "no")
                  << " elapsed: " << sample.elapsed_s << std::endl;
    }

    std::istringstream iss(output);
    std::string line;
    const bool using_combined = selection.cas.has_combined_counter && !selection.cas.has_read_write;
    int matched_counter_lines = 0;

    if (selection.type == CounterType::CAS_COUNT || selection.type == CounterType::NVIDIA_GRACE) {
        matched_counter_lines += accumulate_exact_vtune_event_matches(
            output, selection.cas.read_events, sample.cas_rd, verbosity, "rd");
        matched_counter_lines += accumulate_exact_vtune_event_matches(
            output, selection.cas.write_events, sample.cas_wr, verbosity, "wr");
        matched_counter_lines += accumulate_exact_vtune_event_matches(
            output, selection.cas.combined_events, sample.cas_rd, verbosity, "combined");
    } else if (selection.type == CounterType::UPI_FLITS) {
        matched_counter_lines += accumulate_exact_vtune_event_matches(
            output, selection.upi.rxl_data_events, sample.cas_rd, verbosity, "rxl");
        matched_counter_lines += accumulate_exact_vtune_event_matches(
            output, selection.upi.txl_data_events, sample.cas_wr, verbosity, "txl");
    }
    matched_counter_lines += accumulate_exact_vtune_extra_matches(
        output, selection.extra_counters, sample.extra_values, verbosity);

    bool in_uncore_summary = false;
    int uncore_debug_lines = 0;

    if (matched_counter_lines == 0) {
        while (std::getline(iss, line)) {
            const std::string trimmed = trim_copy(line);
            if (trimmed == "Uncore Event summary") {
                in_uncore_summary = true;
                continue;
            }

            if (in_uncore_summary) {
                if (trimmed.empty()) {
                    if (matched_counter_lines > 0) {
                        break;
                    }
                    continue;
                }
                if (trimmed.find("Uncore Event Type") != std::string::npos ||
                    trimmed.find("--------") == 0) {
                    continue;
                }

                if (verbosity >= 4 && uncore_debug_lines < 20) {
                    std::cout << "[VTUNE DEBUG] uncore-raw-line: " << trimmed << std::endl;
                    uncore_debug_lines++;
                }

                if (parse_vtune_counter_line(line, selection, sample.cas_rd, sample.cas_wr, sample.extra_values, verbosity)) {
                    matched_counter_lines++;
                }
            }
        }

        sample.cas_rd = 0;
        sample.cas_wr = 0;
        sample.extra_values.clear();
        iss.clear();
        iss.str(output);
        while (std::getline(iss, line)) {
            if (parse_vtune_counter_line(line, selection, sample.cas_rd, sample.cas_wr, sample.extra_values, verbosity)) {
                matched_counter_lines++;
            }
        }
    }

    if (sample.elapsed_s <= 0.0) {
        sample.error = !vtune_error.empty() ? vtune_error : "VTune did not report elapsed time";
        return sample;
    }

    if (using_combined) {
        sample.success = sample.cas_rd > 0;
    } else {
        sample.success = sample.cas_rd > 0 && sample.cas_wr > 0;
    }

    if (!sample.success && sample.error.empty()) {
        sample.error = !vtune_error.empty()
            ? vtune_error
            : (command_ok ? "VTune did not report usable counter values" : "VTune command failed");
    }

    return sample;
}

std::string make_sleep_command(double seconds, int helper_cpu) {
    const std::string sleep_seconds = format_seconds(seconds);
    if (helper_cpu >= 0 && run_command_success("command -v taskset >/dev/null 2>&1")) {
        return "taskset -c " + std::to_string(helper_cpu) + " sleep " + sleep_seconds;
    }
    return "sleep " + sleep_seconds;
}

int local_socket_helper_cpu(const system_info& sys_info) {
    if (sys_info.total_physical_cores > 0 && sys_info.socket_count > 0) {
        const int physicals_per_socket = std::max(1, sys_info.total_physical_cores / sys_info.socket_count);
        return std::max(0, physicals_per_socket - 1);
    }

    if (sys_info.total_logical_cores <= 0) {
        return 0;
    }

    const int sockets = std::max(1, sys_info.socket_count);
    const int logicals_per_socket = std::max(1, sys_info.total_logical_cores / sockets);
    return std::max(0, logicals_per_socket - 1);
}

double effective_vtune_window_seconds(double requested_ms) {
    return std::max(0.1, requested_ms / 1000.0);
}

}

std::string VtuneBandwidthMeasurer::find_vtune_binary() const {
    // command -v is near-instant; the timeout is just insurance against a
    // pathological login shell hanging /bin/sh.
    const std::string output = subprocess::run_shell("command -v vtune 2>/dev/null");
    if (output.empty()) return "";

    // Mirror the legacy fgets() behavior: take only the first line.
    const size_t nl = output.find('\n');
    return trim_copy(nl == std::string::npos ? output : output.substr(0, nl));
}

bool VtuneBandwidthMeasurer::sample_bandwidth(long long& cas_rd, long long& cas_wr, double& elapsed, const std::vector<int>& mem_nodes) const {
    (void)mem_nodes;
    const std::string vtune_binary = find_vtune_binary();
    if (vtune_binary.empty()) {
        std::cerr << "Error: vtune not found in PATH." << std::endl;
        return false;
    }

    extra_perf_values_.clear();

    const double sample_seconds = effective_vtune_window_seconds(sampling_interval_ms_);
    const int helper_cpu = local_socket_helper_cpu(sys_info_);
    const std::string helper_command = make_sleep_command(sample_seconds, helper_cpu);

    VtuneSample sample = run_vtune_collection(vtune_binary, counter_selection_, 0.0, helper_command, config_.verbosity);

    if (!sample.success) {
        if (config_.verbosity >= 2 && !sample.error.empty()) {
            std::cerr << "VTune sampling failed: " << sample.error
                      << " (exit-code=" << sample.exit_code << ")" << std::endl;
        }
        return false;
    }

    cas_rd = sample.cas_rd;
    cas_wr = sample.cas_wr;
    elapsed = sample.elapsed_s;
    extra_perf_values_ = sample.extra_values;
    return true;
}

bool VtuneBandwidthMeasurer::wait_for_stabilization(int& samples_taken, long long& last_cas_rd, long long& last_cas_wr, double& last_elapsed, int pause, int ratio_pct, bool fast_resume, std::function<void()> on_sample_callback) {
    (void)fast_resume;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    if (!traffic_gen_manager_ || !storage_) {
        return false;
    }

    std::vector<int> mem_nodes = numa_resolver_ ? numa_resolver_() : std::vector<int>{0};

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

    const bool is_remote = (counter_selection_.type == CounterType::UPI_FLITS);
    const int expected_cores = get_traffic_gen_cores();
    const int socket_cores = (sys_info_.socket_count > 0) ? sys_info_.sockets[0].core_count : 0;

    double theoretical_peak_gb_s = TheoreticalPeakCalculator::calculate_achievable_peak(
        caps_, expected_cores, socket_cores, is_remote);
    if (theoretical_peak_gb_s <= 0.0) {
        theoretical_peak_gb_s = 300.0;
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

    const size_t stabilization_window = 3;
    BandwidthStabilizer stabilizer(
        theoretical_peak_gb_s, pause, health_checker,
        stabilization_window, 0.05, config_.verbosity);

    const double sample_window_seconds = effective_vtune_window_seconds(sampling_interval_ms_);
    const int max_samples = std::max(3, static_cast<int>(60.0 / sample_window_seconds));
    int sample_failures = 0;
    bool success = false;
    double successful_sample_elapsed_sum = 0.0;
    int successful_sample_count = 0;

    for (int total_samples = 0; total_samples < max_samples; ) {
        if (!traffic_gen_manager_->is_traffic_gen_running(traffic_gen_manager_->active_traffic_gen_pid())) {
            if (relaunch_attempts >= 2 || !relaunch_current_traffic_gen()) {
                samples_taken = total_samples;
                return false;
            }
            relaunch_attempts++;
            stabilizer.reset();
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }

        long long sample_cas_rd = 0;
        long long sample_cas_wr = 0;
        double sample_elapsed = 0.0;
        if (!sample_bandwidth(sample_cas_rd, sample_cas_wr, sample_elapsed, mem_nodes)) {
            sample_failures++;
            if (sample_failures >= 3) {
                samples_taken = total_samples;
                return false;
            }
            continue;
        }

        sample_failures = 0;
        total_samples++;
        successful_sample_elapsed_sum += sample_elapsed;
        successful_sample_count++;
        if (on_sample_callback) {
            on_sample_callback();
        }

        const double bw_gb_s = calculate_bandwidth_gbps(
            sample_cas_rd, sample_cas_wr, sample_elapsed,
            counter_selection_.type, cached_cache_line_size_, cached_scaling_factor_);

        const StabilizationResult result = stabilizer.add_sample(sample_cas_rd, sample_cas_wr, bw_gb_s);
        if (result == StabilizationResult::ZOMBIE_DETECTED) {
            samples_taken = total_samples;
            return false;
        }

        if (config_.verbosity >= 3) {
            double read_ratio = 0.0;
            const long long total_cas = sample_cas_rd + sample_cas_wr;
            if (counter_selection_.type == CounterType::NVIDIA_GRACE) {
                const double read_bytes = static_cast<double>(sample_cas_rd) * 32.0;
                const double write_bytes = static_cast<double>(sample_cas_wr);
                const double total_bytes = read_bytes + write_bytes;
                if (total_bytes > 0.0) {
                    read_ratio = read_bytes / total_bytes;
                }
            } else if (total_cas > 0) {
                read_ratio = static_cast<double>(sample_cas_rd) / static_cast<double>(total_cas);
            }
            std::cout << "      [VTune Sample " << total_samples << "] ";
            stabilizer.print_status(read_ratio, bw_gb_s, counter_selection_.cas.has_read_write);
        }

        if (stabilizer.is_stable()) {
            success = true;
            samples_taken = total_samples;
            break;
        }
    }

    if (!success) {
        return false;
    }

    long long aggregate_cas_rd = 0;
    long long aggregate_cas_wr = 0;
    for (const auto& sample : stabilizer.get_samples()) {
        aggregate_cas_rd += sample.cas_rd;
        aggregate_cas_wr += sample.cas_wr;
    }

    last_cas_rd = aggregate_cas_rd;
    last_cas_wr = aggregate_cas_wr;
    const double average_sample_elapsed =
        (successful_sample_count > 0) ? (successful_sample_elapsed_sum / successful_sample_count)
                                      : effective_vtune_window_seconds(sampling_interval_ms_);
    last_elapsed = stabilizer.get_samples().size() * average_sample_elapsed;
    return true;
}

bool VtuneBandwidthMeasurer::monitor_command(const std::string& command,
                                             MonitorCallback callback,
                                             bool summary_mode) {
    const std::string vtune_binary = find_vtune_binary();
    if (vtune_binary.empty()) {
        std::cerr << "Error: vtune not found in PATH." << std::endl;
        return false;
    }

    const std::string trimmed_command = trim_copy(command);
    if (trimmed_command.empty()) {
        std::cerr << "Error: No command specified for VTune monitoring." << std::endl;
        return false;
    }
    if (is_pid_attach_command(trimmed_command)) {
        std::cerr << "Error: VTune backend does not support PID attach mode in MessProfiler." << std::endl;
        return false;
    }

    extra_perf_values_.clear();
    ensure_scaling_factor_cached();

    if (!summary_mode) {
        const double window_seconds = effective_vtune_window_seconds(sampling_interval_ms_);
        pid_t child_pid = spawn_shell_command(trimmed_command);
        if (child_pid < 0) {
            std::cerr << "Error: Failed to launch command for VTune monitoring." << std::endl;
            return false;
        }

        double timestamp = 0.0;
        int status = 0;
        bool saw_sample = false;

        while (true) {
            const int helper_cpu = local_socket_helper_cpu(sys_info_);
            const std::string helper_command = make_sleep_command(window_seconds, helper_cpu);
            VtuneSample sample = run_vtune_collection(vtune_binary, counter_selection_, 0.0, helper_command, config_.verbosity);
            if (sample.success) {
                saw_sample = true;
                timestamp += sample.elapsed_s;
                const double bw_gb_s = calculate_bandwidth_gbps(
                    sample.cas_rd, sample.cas_wr, sample.elapsed_s,
                    counter_selection_.type, cached_cache_line_size_, cached_scaling_factor_);
                for (const auto& kv : sample.extra_values) {
                    extra_perf_values_[kv.first] += kv.second;
                }
                if (callback) {
                    callback(timestamp, bw_gb_s, sample.cas_rd, sample.cas_wr, sample.extra_values);
                }
            }

            pid_t wait_result = waitpid(child_pid, &status, WNOHANG);
            if (wait_result == child_pid) {
                break;
            }
            if (wait_result < 0) {
                break;
            }
        }

        waitpid(child_pid, &status, 0);
        return saw_sample;
    }

    VtuneSample sample = run_vtune_collection(vtune_binary, counter_selection_, 0.0, trimmed_command, config_.verbosity);
    if (!sample.success) {
        if (!sample.error.empty()) {
            std::cerr << "Error: " << sample.error << " (exit-code=" << sample.exit_code << ")" << std::endl;
        }
        return false;
    }

    extra_perf_values_ = sample.extra_values;

    const double bw_gb_s = calculate_bandwidth_gbps(
        sample.cas_rd, sample.cas_wr, sample.elapsed_s,
        counter_selection_.type, cached_cache_line_size_, cached_scaling_factor_);

    if (callback) {
        callback(sample.elapsed_s, bw_gb_s, sample.cas_rd, sample.cas_wr, sample.extra_values);
    }
    return true;
}
