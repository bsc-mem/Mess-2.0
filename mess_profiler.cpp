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

#include "system_detection.h"
#include "architecture/ArchitectureRegistry.h"
#include "measurement.h"
#include "benchmark_config.h"
#include "profiler/profiler_config.h"
#include "profiler/process_binding.h"
#include "utils.h"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace {

enum class ParseStatus {
    OK,
    HELP,
    ERROR
};

struct ParseResult {
    ParseStatus status = ParseStatus::OK;
    ProfilerConfig config;
    std::string error;
};

struct ProfetResolution {
    std::vector<std::string> selected;
    std::vector<std::string> warnings;
};

struct SummarySample {
    bool valid = false;
    double duration_s = 0.0;
    double bw_gbps = 0.0;
    long long read_bytes = 0;
    long long write_bytes = 0;
};

std::string to_lower_copy(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value;
}

bool parse_positive_pid(const std::string& value, pid_t& out) {
    try {
        long parsed = std::stol(value);
        if (parsed <= 0) {
            return false;
        }
        out = static_cast<pid_t>(parsed);
        return true;
    } catch (...) {
        return false;
    }
}

bool parse_backend(const std::string& value, ProfilerBackend& backend) {
    const std::string lower = to_lower_copy(value);
    if (lower == "auto") {
        backend = ProfilerBackend::AUTO;
        return true;
    }
    if (lower == "perf-event" || lower == "direct") {
        backend = ProfilerBackend::PERF_EVENT;
        return true;
    }
    if (lower == "perf-stat" || lower == "perf") {
        backend = ProfilerBackend::PERF_STAT;
        return true;
    }
    if (lower == "likwid") {
        backend = ProfilerBackend::LIKWID;
        return true;
    }
    return false;
}

MeasurerType backend_to_measurer(ProfilerBackend backend) {
    if (backend == ProfilerBackend::LIKWID) {
        return MeasurerType::LIKWID;
    }
    return MeasurerType::PERF;
}

std::string backend_label(ProfilerBackend backend) {
    switch (backend) {
        case ProfilerBackend::AUTO: return "auto";
        case ProfilerBackend::PERF_EVENT: return "perf-event";
        case ProfilerBackend::PERF_STAT: return "perf-stat";
        case ProfilerBackend::LIKWID: return "likwid";
    }
    return "unknown";
}

std::string counter_type_label(CounterType type) {
    switch (type) {
        case CounterType::CAS_COUNT: return "CAS_COUNT";
        case CounterType::UPI_FLITS: return "UPI_FLITS";
        case CounterType::NVIDIA_GRACE: return "NVIDIA_GRACE";
        default: return "UNKNOWN";
    }
}

bool parse_args_value_required(int arg_idx, int argc, const char* option, ParseResult& result) {
    if (arg_idx + 1 >= argc) {
        result.status = ParseStatus::ERROR;
        result.error = std::string("Error: ") + option + " requires an argument.";
        return false;
    }
    return true;
}

ParseResult parse_args(int argc, char* argv[]) {
    ParseResult result;
    ProfilerConfig& config = result.config;

    for (int arg_idx = 1; arg_idx < argc;) {
        const std::string arg = argv[arg_idx];

        if (arg == "-s" || arg == "--interval") {
            if (!parse_args_value_required(arg_idx, argc, arg.c_str(), result)) return result;
            config.sampling_interval_str = argv[arg_idx + 1];
            arg_idx += 2;
            continue;
        }

        if (arg == "-o" || arg == "--output" || arg == "-f") {
            if (!parse_args_value_required(arg_idx, argc, arg.c_str(), result)) return result;
            config.output_file = argv[arg_idx + 1];
            arg_idx += 2;
            continue;
        }

        if (arg == "-a" || arg == "--system-wide") {
            config.system_wide = true;
            ++arg_idx;
            continue;
        }

        if (arg == "-p" || arg == "--pid") {
            if (!parse_args_value_required(arg_idx, argc, arg.c_str(), result)) return result;
            if (!parse_positive_pid(argv[arg_idx + 1], config.target_pid)) {
                result.status = ParseStatus::ERROR;
                result.error = std::string("Error: invalid PID '") + argv[arg_idx + 1] + "'.";
                return result;
            }
            arg_idx += 2;
            continue;
        }

        if (arg == "-C" || arg == "--cpu") {
            if (!parse_args_value_required(arg_idx, argc, arg.c_str(), result)) return result;
            try {
                config.cpu_list = parse_cpu_list_arg(argv[arg_idx + 1]);
            } catch (...) {
                result.status = ParseStatus::ERROR;
                result.error = std::string("Error: invalid CPU list '") + argv[arg_idx + 1] + "'.";
                return result;
            }
            config.auto_detect_binding = false;
            arg_idx += 2;
            continue;
        }

        if (arg == "-N" || arg == "--nodes") {
            if (!parse_args_value_required(arg_idx, argc, arg.c_str(), result)) return result;
            try {
                config.mem_nodes = parse_node_list_arg(argv[arg_idx + 1]);
            } catch (...) {
                result.status = ParseStatus::ERROR;
                result.error = std::string("Error: invalid node list '") + argv[arg_idx + 1] + "'.";
                return result;
            }
            config.auto_detect_binding = false;
            arg_idx += 2;
            continue;
        }

        if (arg == "--no-inherit") {
            config.auto_detect_binding = false;
            ++arg_idx;
            continue;
        }

        if (arg == "-v" || arg == "--verbose") {
            config.verbose = true;
            ++config.verbosity;
            ++arg_idx;
            continue;
        }

        if (arg == "--csv") {
            config.csv_output = true;
            ++arg_idx;
            continue;
        }

        if (arg == "--human") {
            config.csv_output = false;
            ++arg_idx;
            continue;
        }

        if (arg == "--dry" || arg == "--dry-run") {
            config.dry_run = true;
            ++arg_idx;
            continue;
        }

        if (arg == "--profet") {
            config.profet_mode = true;
            ++arg_idx;
            continue;
        }

        if (arg == "--backend") {
            if (!parse_args_value_required(arg_idx, argc, arg.c_str(), result)) return result;
            if (!parse_backend(argv[arg_idx + 1], config.backend)) {
                result.status = ParseStatus::ERROR;
                result.error = std::string("Error: Unknown backend: ") + argv[arg_idx + 1];
                return result;
            }
            arg_idx += 2;
            continue;
        }

        if (arg == "-h" || arg == "--help") {
            result.status = ParseStatus::HELP;
            return result;
        }

        if (arg == "--") {
            for (int i = arg_idx + 1; i < argc; ++i) {
                config.command_args.push_back(argv[i]);
            }
            return result;
        }

        if (!arg.empty() && arg[0] == '-') {
            result.status = ParseStatus::ERROR;
            result.error = std::string("Error: Unknown option: ") + arg;
            return result;
        }

        for (int i = arg_idx; i < argc; ++i) {
            config.command_args.push_back(argv[i]);
        }
        return result;
    }

    return result;
}

bool validate_config(const ProfilerConfig& config, std::string& error) {
    if (!config.sampling_interval_str.empty()) {
        int interval_ms = 0;
        try {
            interval_ms = parse_time_to_ms(config.sampling_interval_str);
        } catch (...) {
            error = "Error: invalid interval '" + config.sampling_interval_str + "'.";
            return false;
        }
        if (interval_ms <= 0) {
            error = "Error: interval must be greater than 0.";
            return false;
        }
    }

    if (config.target_pid > 0 && !config.command_args.empty()) {
        error = "Error: use either --pid <pid> or a command, not both.";
        return false;
    }

    if (config.backend == ProfilerBackend::LIKWID && config.target_pid > 0) {
        error = "Error: --backend likwid does not support --pid attach mode.";
        return false;
    }

    if (config.command_args.empty() && !config.dry_run && config.target_pid < 0) {
        error = "Error: No command specified. Use -p <pid> to attach or provide a command.";
        return false;
    }

    return true;
}

std::string build_monitor_command(const ProfilerConfig& config) {
    if (config.target_pid > 0) {
        return "-p " + std::to_string(config.target_pid);
    }
    return config.get_command_string();
}

void append_unique(std::vector<std::string>& dst, const std::string& value) {
    if (value.empty()) {
        return;
    }
    if (std::find(dst.begin(), dst.end(), value) == dst.end()) {
        dst.push_back(value);
    }
}

std::string find_first_supported_counter(const std::vector<std::string>& candidates) {
    struct PerfListTokenCache {
        bool loaded = false;
        std::set<std::string> raw_tokens;
        std::set<std::string> normalized_tokens;
    };

    auto normalize_token = [](const std::string& token) {
        std::string normalized;
        normalized.reserve(token.size());
        for (char c : token) {
            if (std::isalnum(static_cast<unsigned char>(c))) {
                normalized.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
            }
        }
        return normalized;
    };

    auto add_perf_list_token = [&](PerfListTokenCache& cache, std::string token) {
        if (token.empty()) {
            return;
        }
        token = to_lower_copy(token);
        cache.raw_tokens.insert(token);
        std::string normalized = normalize_token(token);
        if (!normalized.empty()) {
            cache.normalized_tokens.insert(normalized);
        }
    };

    auto perf_list_supports = [&](const std::string& candidate) {
        static PerfListTokenCache cache;
        if (!cache.loaded) {
            cache.loaded = true;

            FILE* pipe = popen("perf list 2>/dev/null", "r");
            if (pipe) {
                std::string buffer;
                buffer.resize(4096);
                std::string token;
                token.reserve(128);

                for (;;) {
                    size_t read_bytes = std::fread(buffer.data(), 1, buffer.size(), pipe);
                    if (read_bytes == 0) {
                        break;
                    }
                    for (size_t i = 0; i < read_bytes; ++i) {
                        const unsigned char ch = static_cast<unsigned char>(buffer[i]);
                        const bool token_char =
                            std::isalnum(ch) || ch == '_' || ch == '-' || ch == '.' ||
                            ch == ':' || ch == '/' || ch == '=';
                        if (token_char) {
                            token.push_back(static_cast<char>(ch));
                        } else if (!token.empty()) {
                            add_perf_list_token(cache, token);
                            token.clear();
                        }
                    }
                }

                if (!token.empty()) {
                    add_perf_list_token(cache, token);
                }
                pclose(pipe);
            }
        }

        const std::string candidate_lower = to_lower_copy(candidate);
        if (cache.raw_tokens.find(candidate_lower) != cache.raw_tokens.end()) {
            return true;
        }

        const std::string candidate_normalized = normalize_token(candidate);
        if (!candidate_normalized.empty() &&
            cache.normalized_tokens.find(candidate_normalized) != cache.normalized_tokens.end()) {
            return true;
        }

        return false;
    };

    for (const auto& candidate : candidates) {
        if (perf_list_supports(candidate)) {
            return candidate;
        }
        ResolvedPerfEvent resolved = BandwidthCounterStrategy::resolveEvent(candidate);
        if (resolved.valid) {
            return candidate;
        }
    }
    return "";
}

ProfetResolution resolve_profet_counters() {
    ProfetResolution resolution;

    struct CounterFamily {
        std::string label;
        std::vector<std::string> aliases;
        std::string fallback;
    };

    const std::vector<CounterFamily> families = {
        {"cycles", {"cycles", "cpu-cycles", "cpu_clk_unhalted.thread", "cpu_unhalted.thread"}, "cycles"},
        {"instructions", {"instructions", "inst_retired.any", "inst_retired.any_p"}, "instructions"},
        {"longest_lat_cache_miss",
         {"longest_lat_cache.miss", "longest_lat_cache_miss", "llc_cache_miss",
          "llc-load-misses", "cache-misses", "mem_load_retired.l3_miss"},
         "llc-load-misses"}
    };

    for (const auto& family : families) {
        std::string selected = find_first_supported_counter(family.aliases);
        if (selected.empty()) {
            selected = family.fallback;
            resolution.warnings.push_back(
                "profet: no direct alias detected for '" + family.label +
                "', falling back to '" + selected + "'.");
        }
        append_unique(resolution.selected, selected);
    }

    return resolution;
}

int detect_cache_line_size(const system_info& sys_info) {
    if (sys_info.socket_count > 0) {
        for (int s = 0; s < sys_info.socket_count; ++s) {
            for (int c = 0; c < sys_info.sockets[s].cache_count; ++c) {
                if (sys_info.sockets[s].caches[c].line_size_bytes > 0) {
                    return static_cast<int>(sys_info.sockets[s].caches[c].line_size_bytes);
                }
            }
        }
    }
    return 64;
}

void print_profet_totals(std::ostream& os,
                         const std::vector<std::string>& profet_counters,
                         const std::map<std::string, long long>& totals) {
    if (profet_counters.empty()) {
        return;
    }
    os << "  Profet Counters Totals:\n";
    for (const auto& counter : profet_counters) {
        long long value = 0;
        auto it = totals.find(counter);
        if (it != totals.end()) {
            value = it->second;
        }
        os << "    - " << counter << ": " << value << '\n';
    }
}

}  // namespace

int main(int argc, char* argv[]) {
    if (argc < 2) {
        print_profiler_help(argv[0]);
        return 1;
    }

    ParseResult parse = parse_args(argc, argv);
    if (parse.status == ParseStatus::HELP) {
        print_profiler_help(argv[0]);
        return 0;
    }
    if (parse.status == ParseStatus::ERROR) {
        std::cerr << parse.error << '\n';
        print_profiler_help(argv[0]);
        return 1;
    }

    ProfilerConfig prof_config = parse.config;
    std::string validation_error;
    if (!validate_config(prof_config, validation_error)) {
        std::cerr << validation_error << '\n';
        return 1;
    }

    const std::string startup_target = build_monitor_command(prof_config);
    std::cerr << "mess-profiler: Starting profiler...\n";
    std::cerr << "  Target: " << startup_target << '\n';
    std::cerr << "  Backend (requested): " << backend_label(prof_config.backend) << '\n';
    std::cerr << "  Interval: " << (prof_config.has_sampling() ? prof_config.sampling_interval_str : "summary mode") << '\n';
    std::cerr << "  Resolving counters and system topology...\n";
    std::cerr << std::flush;

    system_info sys_info;
    if (system_info_detect(&sys_info) != 0) {
        std::cerr << "Error: Failed to detect system information.\n";
        return 1;
    }

    SystemDetector detector;
    detector.detect();
    CPUCapabilities caps = detector.get_capabilities();

    auto arch = ArchitectureRegistry::instance().getArchitecture(caps);
    if (!arch && prof_config.verbose) {
        std::cerr << "Warning: Unknown architecture, using generic fallbacks.\n";
    }

    ProcessBindingDetector binding_detector;
    ProcessBinding binding = (prof_config.target_pid > 0)
                           ? binding_detector.detect_for_pid(prof_config.target_pid)
                           : binding_detector.detect();

    int representative_cpu = 0;
    if (!prof_config.cpu_list.empty()) {
        representative_cpu = *prof_config.cpu_list.begin();
    } else if (prof_config.auto_detect_binding && binding.is_cpu_bound) {
        representative_cpu = binding.get_representative_cpu();
    }

    std::vector<int> target_mem_nodes;
    if (!prof_config.mem_nodes.empty()) {
        target_mem_nodes.assign(prof_config.mem_nodes.begin(), prof_config.mem_nodes.end());
    } else if (prof_config.auto_detect_binding && binding.is_mem_bound) {
        target_mem_nodes = binding.get_target_mem_nodes();
    } else if (prof_config.system_wide) {
        for (int i = 0; i < sys_info.numa_node_count; ++i) {
            target_mem_nodes.push_back(sys_info.numa_nodes[i].id);
        }
    } else {
        target_mem_nodes = {get_numa_node_of_cpu(representative_cpu)};
    }

    if (target_mem_nodes.empty()) {
        target_mem_nodes.push_back(0);
    }

    if (prof_config.profet_mode) {
        ProfetResolution profet = resolve_profet_counters();
        prof_config.profet_counters = profet.selected;
        for (const auto& warning : profet.warnings) {
            std::cerr << "Warning: " << warning << '\n';
        }
    }

    auto& strategy = BandwidthCounterStrategy::instance();
    strategy.set_measurer_type(backend_to_measurer(prof_config.backend));
    strategy.set_extra_counters(prof_config.profet_counters);
    strategy.set_memory_type(caps.memory_type);
    strategy.initialize(representative_cpu, target_mem_nodes, caps);

    BandwidthCounterSelection selection = strategy.get_selection();
    selection.extra_counters = prof_config.profet_counters;

    const int cache_line_size = detect_cache_line_size(sys_info);

    if (prof_config.dry_run) {
        std::cout << "=== Mess Profiler - Dry Run ===\n\n";

        std::cout << "System Information:\n";
        std::cout << "  Architecture: " << (arch ? arch->getName() : "Unknown") << '\n';
        std::cout << "  CPU Model: " << sys_info.cpu_model << '\n';
        std::cout << "  Cache Line Size: " << cache_line_size << " bytes\n";
        std::cout << "  NUMA Nodes: " << sys_info.numa_node_count << "\n\n";

        std::cout << "Process Binding Detection:\n";
        std::cout << "  " << binding.describe() << "\n\n";

        std::cout << "Profiling Configuration:\n";
        std::cout << "  Backend (requested): " << backend_label(prof_config.backend) << '\n';
        std::cout << "  Backend (resolved): " << strategy.get_measurer_name() << '\n';
        std::cout << "  Target CPUs: ";
        if (!prof_config.cpu_list.empty()) {
            std::cout << format_cpu_set(prof_config.cpu_list) << " (explicit)\n";
        } else if (binding.is_cpu_bound) {
            std::cout << format_cpu_set(binding.allowed_cpus) << " (inherited)\n";
        } else {
            std::cout << "all\n";
        }

        std::cout << "  Target Memory Nodes: ";
        std::set<int> nodes_set(target_mem_nodes.begin(), target_mem_nodes.end());
        std::cout << format_node_set(nodes_set);
        if (!prof_config.mem_nodes.empty()) {
            std::cout << " (explicit)\n";
        } else if (binding.is_mem_bound) {
            std::cout << " (inherited from numactl/membind)\n";
        } else {
            std::cout << " (auto-detected)\n";
        }

        std::cout << "  Representative CPU: " << representative_cpu << '\n';
        std::cout << "  Remote Access: " << (binding.has_remote_access ? "yes" : "no") << "\n\n";

        std::cout << "Counter Selection:\n";
        std::cout << "  Type: " << counter_type_label(selection.type) << '\n';
        std::cout << "  Events: " << selection.get_all_events_string() << '\n';
        std::cout << "  Units: " << get_counter_unit_description(selection.type, cache_line_size) << '\n';
        std::cout << "  Available: " << (selection.is_valid() ? "yes" : "no") << '\n';
        if (!selection.is_valid()) {
            std::cout << "  Failure: " << selection.failure_reason << '\n';
        }
        if (!prof_config.profet_counters.empty()) {
            std::cout << "  Profet Extras: ";
            for (size_t i = 0; i < prof_config.profet_counters.size(); ++i) {
                if (i > 0) std::cout << ", ";
                std::cout << prof_config.profet_counters[i];
            }
            std::cout << '\n';
        }

        std::cout << "\nOutput Headers:\n";
        std::cout << "  Read Column: " << get_raw_read_header(selection.type, cache_line_size) << '\n';
        std::cout << "  Write Column: " << get_raw_write_header(selection.type, cache_line_size) << '\n';

        std::string full_command = build_monitor_command(prof_config);
        if (!full_command.empty()) {
            std::cout << "\nMonitor Target: " << full_command << '\n';
        }

        return 0;
    }

    BenchmarkConfig bench_config;
    bench_config.verbosity = prof_config.verbosity;
    bench_config.measurer = measurer_type_to_string(strategy.get_resolved_measurer_type());

    auto measurer = create_bandwidth_measurer(
        bench_config,
        sys_info,
        caps,
        nullptr,
        nullptr,
        [&]() { return target_mem_nodes; },
        ExecutionMode::MULTISEQUENTIAL
    );

    if (!measurer) {
        std::cerr << "Error: Failed to create bandwidth measurer.\n";
        return 1;
    }
    measurer->set_counter_selection(selection);

    std::ostream* out_stream = &std::cout;
    std::ofstream file_stream;
    if (!prof_config.output_file.empty()) {
        file_stream.open(prof_config.output_file);
        if (!file_stream.is_open()) {
            std::cerr << "Error: Could not open output file " << prof_config.output_file << '\n';
            return 1;
        }
        out_stream = &file_stream;
    }

    const bool summary_mode = !prof_config.has_sampling();
    const std::string raw_rd_header = get_raw_read_header(selection.type, cache_line_size);
    const std::string raw_wr_header = get_raw_write_header(selection.type, cache_line_size);
    const std::string monitor_target = startup_target;

    std::cerr << "  Backend (resolved): " << strategy.get_measurer_name() << '\n';
    std::cerr << "  Counter: " << counter_type_label(selection.type) << '\n';
    if (!prof_config.output_file.empty()) {
        std::cerr << "  Output: " << prof_config.output_file << '\n';
    }
    if (!prof_config.profet_counters.empty()) {
        std::cerr << "  Profet: ";
        for (size_t i = 0; i < prof_config.profet_counters.size(); ++i) {
            if (i > 0) std::cerr << ", ";
            std::cerr << prof_config.profet_counters[i];
        }
        std::cerr << '\n';
    }
    if (prof_config.verbose) {
        std::cerr << "  Events: " << selection.get_all_events_string() << '\n';
        if (binding.is_cpu_bound || binding.is_mem_bound) {
            std::cerr << "  Binding: " << binding.describe() << '\n';
        }
    }

    if (!summary_mode && prof_config.csv_output) {
        *out_stream << "Timestamp(s),Bandwidth(GB/s)," << raw_rd_header << "," << raw_wr_header;
        for (const auto& counter : prof_config.profet_counters) {
            *out_stream << ',' << counter;
        }
        *out_stream << '\n';
        measurer->set_sampling_interval_ms(prof_config.sampling_interval_ms());
    } else if (summary_mode && !prof_config.csv_output) {
        *out_stream << "Summary:\n";
    }

    SummarySample summary;
    std::size_t streamed_rows = 0;

    auto callback = [&](double timestamp,
                        double bw_gbps,
                        long long raw_rd,
                        long long raw_wr,
                        const BandwidthMeasurer::MonitorSampleExtras& sample_extras) {
        long long read_bytes = raw_rd;
        long long write_bytes = raw_wr;

        if (selection.type == CounterType::CAS_COUNT || selection.type == CounterType::UPI_FLITS) {
            read_bytes = raw_rd * cache_line_size;
            write_bytes = raw_wr * cache_line_size;
        } else if (selection.type == CounterType::NVIDIA_GRACE) {
            read_bytes = raw_rd * 32;
            write_bytes = raw_wr;
        }

        if (summary_mode) {
            summary.valid = true;
            summary.duration_s = timestamp;
            summary.bw_gbps = bw_gbps;
            summary.read_bytes = read_bytes;
            summary.write_bytes = write_bytes;
            return;
        }

        if (prof_config.csv_output) {
            *out_stream << std::fixed << std::setprecision(4) << timestamp << ','
                        << bw_gbps << ',' << read_bytes << ',' << write_bytes;
            for (const auto& counter : prof_config.profet_counters) {
                long long value = 0;
                auto it = sample_extras.find(counter);
                if (it != sample_extras.end()) {
                    value = it->second;
                }
                *out_stream << ',' << value;
            }
            *out_stream << '\n';
        } else {
            *out_stream << std::fixed << std::setprecision(4)
                        << '[' << timestamp << "s] " << std::setprecision(2) << bw_gbps << " GB/s"
                        << " (rd=" << read_bytes << " B, wr=" << write_bytes << " B)";
            if (!prof_config.profet_counters.empty()) {
                *out_stream << " [";
                for (size_t i = 0; i < prof_config.profet_counters.size(); ++i) {
                    if (i > 0) {
                        *out_stream << ", ";
                    }
                    const std::string& counter = prof_config.profet_counters[i];
                    long long value = 0;
                    auto it = sample_extras.find(counter);
                    if (it != sample_extras.end()) {
                        value = it->second;
                    }
                    *out_stream << counter << '=' << value;
                }
                *out_stream << ']';
            }
            *out_stream << '\n';
        }

        ++streamed_rows;
        if ((streamed_rows % 16) == 0) {
            out_stream->flush();
        }
    };

    bool success = measurer->monitor_command(monitor_target, callback, summary_mode);
    if (!success) {
        std::cerr << "Error: Failed to monitor target.\n";
        return 1;
    }

    const std::map<std::string, long long>& extra_totals = measurer->get_extra_perf_values();

    if (summary_mode) {
        if (!summary.valid) {
            std::cerr << "Error: No summary sample was collected.\n";
            return 1;
        }

        if (prof_config.csv_output) {
            *out_stream << "duration_s,bandwidth_gbps," << raw_rd_header << "," << raw_wr_header;
            for (const auto& counter : prof_config.profet_counters) {
                *out_stream << ',' << counter;
            }
            *out_stream << '\n';

            *out_stream << std::fixed << std::setprecision(2)
                        << summary.duration_s << ','
                        << summary.bw_gbps << ','
                        << summary.read_bytes << ','
                        << summary.write_bytes;
            for (const auto& counter : prof_config.profet_counters) {
                long long value = 0;
                auto it = extra_totals.find(counter);
                if (it != extra_totals.end()) {
                    value = it->second;
                }
                *out_stream << ',' << value;
            }
            *out_stream << '\n';
        } else {
            *out_stream << "  Duration: " << std::fixed << std::setprecision(2) << summary.duration_s << " s\n";
            *out_stream << "  Average Bandwidth: " << std::fixed << std::setprecision(2) << summary.bw_gbps << " GB/s\n";
            *out_stream << "  Total " << raw_rd_header << ": " << summary.read_bytes << '\n';
            *out_stream << "  Total " << raw_wr_header << ": " << summary.write_bytes << '\n';
            print_profet_totals(*out_stream, prof_config.profet_counters, extra_totals);
        }
    }

    out_stream->flush();
    return 0;
}
