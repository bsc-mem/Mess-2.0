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
#include "profiler/perf_event_profiler.h"
#include "utils.h"
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <algorithm>
#include <chrono>
#include <cstring>

ProfilerConfig parse_args(int argc, char* argv[]) {
    ProfilerConfig config;
    
    int arg_idx = 1;
    while (arg_idx < argc) {
        std::string arg = argv[arg_idx];
        
        if (arg == "-s" || arg == "--interval") {
            if (arg_idx + 1 < argc) {
                config.sampling_interval_str = argv[arg_idx + 1];
                arg_idx += 2;
            } else {
                std::cerr << "Error: " << arg << " requires an argument.\n";
                exit(1);
            }
        } else if (arg == "-o" || arg == "--output" || arg == "-f") {
            if (arg_idx + 1 < argc) {
                config.output_file = argv[arg_idx + 1];
                arg_idx += 2;
            } else {
                std::cerr << "Error: " << arg << " requires an argument.\n";
                exit(1);
            }
        } else if (arg == "-a" || arg == "--system-wide") {
            config.system_wide = true;
            arg_idx++;
        } else if (arg == "-p" || arg == "--pid") {
            if (arg_idx + 1 < argc) {
                config.target_pid = std::stoi(argv[arg_idx + 1]);
                arg_idx += 2;
            } else {
                std::cerr << "Error: " << arg << " requires a PID.\n";
                exit(1);
            }
        } else if (arg == "-C" || arg == "--cpu") {
            if (arg_idx + 1 < argc) {
                config.cpu_list = parse_cpu_list_arg(argv[arg_idx + 1]);
                config.auto_detect_binding = false;
                arg_idx += 2;
            } else {
                std::cerr << "Error: " << arg << " requires a CPU list.\n";
                exit(1);
            }
        } else if (arg == "-N" || arg == "--nodes") {
            if (arg_idx + 1 < argc) {
                config.mem_nodes = parse_node_list_arg(argv[arg_idx + 1]);
                config.auto_detect_binding = false;
                arg_idx += 2;
            } else {
                std::cerr << "Error: " << arg << " requires a node list.\n";
                exit(1);
            }
        } else if (arg == "--no-inherit") {
            config.auto_detect_binding = false;
            arg_idx++;
        } else if (arg == "-v" || arg == "--verbose") {
            config.verbose = true;
            config.verbosity++;
            arg_idx++;
        } else if (arg == "--csv") {
            config.csv_output = true;
            arg_idx++;
        } else if (arg == "--human") {
            config.csv_output = false;
            arg_idx++;
        } else if (arg == "--dry" || arg == "--dry-run") {
            config.dry_run = true;
            arg_idx++;
        } else if (arg == "--backend") {
            if (arg_idx + 1 < argc) {
                std::string backend_str = argv[arg_idx + 1];
                if (backend_str == "auto") {
                    config.backend = ProfilerBackend::AUTO;
                } else if (backend_str == "perf-event" || backend_str == "direct") {
                    config.backend = ProfilerBackend::PERF_EVENT;
                } else if (backend_str == "perf-stat" || backend_str == "perf") {
                    config.backend = ProfilerBackend::PERF_STAT;
                } else if (backend_str == "likwid") {
                    config.backend = ProfilerBackend::LIKWID;
                } else {
                    std::cerr << "Error: Unknown backend: " << backend_str << "\n";
                    exit(1);
                }
                arg_idx += 2;
            } else {
                std::cerr << "Error: --backend requires an argument.\n";
                exit(1);
            }
        } else if (arg == "-h" || arg == "--help") {
            print_profiler_help(argv[0]);
            exit(0);
        } else if (arg == "--") {
            arg_idx++;
            for (int i = arg_idx; i < argc; ++i) {
                config.command_args.push_back(argv[i]);
            }
            break;
        } else if (arg[0] == '-') {
            std::cerr << "Error: Unknown option: " << arg << "\n";
            print_profiler_help(argv[0]);
            exit(1);
        } else {
            for (int i = arg_idx; i < argc; ++i) {
                config.command_args.push_back(argv[i]);
            }
            break;
        }
    }
    
    return config;
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        print_profiler_help(argv[0]);
        return 1;
    }

    ProfilerConfig prof_config = parse_args(argc, argv);

    if (prof_config.command_args.empty() && !prof_config.dry_run && prof_config.target_pid < 0) {
        std::cerr << "Error: No command specified. Use -p <pid> to attach or provide a command.\n";
        return 1;
    }

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

    int cache_line_size = 64;
    if (sys_info.sockets[0].cache_count > 0) {
        cache_line_size = sys_info.sockets[0].caches[0].line_size_bytes;
    }

    ProcessBindingDetector binding_detector;
    ProcessBinding binding = binding_detector.detect();
    
    std::vector<int> target_mem_nodes;
    int representative_cpu = 0;
    
    if (!prof_config.cpu_list.empty()) {
        representative_cpu = *prof_config.cpu_list.begin();
    } else if (prof_config.auto_detect_binding && binding.is_cpu_bound) {
        representative_cpu = binding.get_representative_cpu();
    }
    
    int local_node_of_rep_cpu = get_numa_node_of_cpu(representative_cpu);
    
    if (!prof_config.mem_nodes.empty()) {
        target_mem_nodes = std::vector<int>(prof_config.mem_nodes.begin(), prof_config.mem_nodes.end());
    } else if (prof_config.auto_detect_binding && binding.is_mem_bound) {
        target_mem_nodes = binding.get_target_mem_nodes();
    } else if (prof_config.system_wide) {
        for (int i = 0; i < sys_info.numa_node_count; ++i) {
            target_mem_nodes.push_back(sys_info.numa_nodes[i].id);
        }
    } else {
        target_mem_nodes = {local_node_of_rep_cpu};
    }

    BandwidthCounterSelection selection = PerformanceCounterStrategy::discoverBandwidthCounters(
        representative_cpu, target_mem_nodes);

    if (prof_config.dry_run) {
        std::cout << "=== Mess Profiler - Dry Run ===\n\n";
        
        std::cout << "System Information:\n";
        std::cout << "  Architecture: " << (arch ? arch->getName() : "Unknown") << "\n";
        std::cout << "  CPU Model: " << sys_info.cpu_model << "\n";
        std::cout << "  Cache Line Size: " << cache_line_size << " bytes\n";
        std::cout << "  NUMA Nodes: " << sys_info.numa_node_count << "\n\n";
        
        std::cout << "Process Binding Detection:\n";
        std::cout << "  " << binding.describe() << "\n\n";
        
        std::cout << "Profiling Configuration:\n";
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
        
        std::cout << "  Representative CPU: " << representative_cpu << "\n";
        std::cout << "  Remote Access: " << (binding.has_remote_access ? "yes" : "no") << "\n\n";
        
        std::cout << "Counter Selection:\n";
        std::cout << "  Type: ";
        switch (selection.type) {
            case CounterType::CAS_COUNT: std::cout << "CAS_COUNT\n"; break;
            case CounterType::UPI_FLITS: std::cout << "UPI_FLITS\n"; break;
            case CounterType::NVIDIA_GRACE: std::cout << "NVIDIA_GRACE\n"; break;
            default: std::cout << "UNKNOWN\n"; break;
        }
        std::cout << "  Events: " << selection.get_all_events_string() << "\n";
        std::cout << "  Units: " << get_counter_unit_description(selection.type, cache_line_size) << "\n";
        std::cout << "  Available: " << (selection.is_valid() ? "yes" : "no") << "\n";
        if (!selection.is_valid()) {
            std::cout << "  Failure: " << selection.failure_reason << "\n";
        }
        
        std::cout << "\nOutput Headers:\n";
        std::cout << "  Read Column: " << get_raw_read_header(selection.type, cache_line_size) << "\n";
        std::cout << "  Write Column: " << get_raw_write_header(selection.type, cache_line_size) << "\n";
        
        if (!prof_config.command_args.empty()) {
            std::cout << "\nCommand: " << prof_config.get_command_string() << "\n";
        }
        
        return 0;
    }

    BenchmarkConfig bench_config;
    bench_config.verbosity = prof_config.verbosity;
    
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
            std::cerr << "Error: Could not open output file " << prof_config.output_file << "\n";
            return 1;
        }
        out_stream = &file_stream;
    }

    bool summary_mode = !prof_config.has_sampling();
    
    std::string raw_rd_header = get_raw_read_header(selection.type, cache_line_size);
    std::string raw_wr_header = get_raw_write_header(selection.type, cache_line_size);

    std::string full_command = prof_config.get_command_string();
    
    std::string backend_name = "perf_event";
    if (prof_config.backend == ProfilerBackend::LIKWID) {
        backend_name = "likwid";
    } else if (prof_config.backend == ProfilerBackend::PERF_STAT) {
        backend_name = "perf stat";
    }

    std::cerr << "mess-profiler: Starting profiler...\n";
    std::cerr << "  Command: " << full_command << "\n";
    std::cerr << "  Backend: " << backend_name << "\n";
    std::cerr << "  Interval: " << (summary_mode ? "summary mode" : prof_config.sampling_interval_str) << "\n";
    std::cerr << "  Counter: " << (selection.type == CounterType::CAS_COUNT ? "CAS_COUNT" : 
                                   selection.type == CounterType::UPI_FLITS ? "UPI_FLITS" : 
                                   selection.type == CounterType::NVIDIA_GRACE ? "NVIDIA_GRACE" : "UNKNOWN") << "\n";
    if (!prof_config.output_file.empty()) {
        std::cerr << "  Output: " << prof_config.output_file << "\n";
    }
    std::cerr << std::flush;

    if (prof_config.verbose) {
        std::cerr << "  Events: " << selection.get_all_events_string() << "\n";
        if (binding.is_cpu_bound || binding.is_mem_bound) {
            std::cerr << "  Binding: " << binding.describe() << "\n";
        }
    }

    if (!summary_mode && prof_config.csv_output) {
        *out_stream << "Timestamp(s),Bandwidth(GB/s)," << raw_rd_header << "," << raw_wr_header << "\n";
        out_stream->flush();
        measurer->set_sampling_interval_ms(prof_config.sampling_interval_ms());
    } else if (summary_mode) {
        if (!prof_config.csv_output) {
            *out_stream << "Summary:\n";
        }
    }
    
    auto callback = [&](double timestamp, double bw_gbps, long long raw_rd, long long raw_wr) {
        long long read_bytes = raw_rd;
        long long write_bytes = raw_wr;
        
        if (selection.type == CounterType::CAS_COUNT) {
            read_bytes = raw_rd * cache_line_size;
            write_bytes = raw_wr * cache_line_size;
        } else if (selection.type == CounterType::UPI_FLITS) {
            read_bytes = raw_rd * cache_line_size;
            write_bytes = raw_wr * cache_line_size;
        } else if (selection.type == CounterType::NVIDIA_GRACE) {
            read_bytes = raw_rd * 32;
            write_bytes = raw_wr;
        }
        
        if (!summary_mode) {
            if (prof_config.csv_output) {
                *out_stream << std::fixed << std::setprecision(4) << timestamp << "," 
                           << bw_gbps << "," << read_bytes << "," << write_bytes << "\n";
            } else {
                *out_stream << std::fixed << std::setprecision(4) 
                           << "[" << timestamp << "s] " << std::setprecision(2) << bw_gbps << " GB/s"
                           << " (rd=" << read_bytes << " B, wr=" << write_bytes << " B)\n";
            }
            out_stream->flush();
        } else {
            if (prof_config.csv_output) {
                *out_stream << "duration_s,bandwidth_gbps," << raw_rd_header << "," << raw_wr_header << "\n";
                *out_stream << std::fixed << std::setprecision(2) << timestamp << "," 
                           << bw_gbps << "," << read_bytes << "," << write_bytes << "\n";
            } else {
                *out_stream << "  Duration: " << std::fixed << std::setprecision(2) << timestamp << " s\n";
                *out_stream << "  Average Bandwidth: " << std::fixed << std::setprecision(2) << bw_gbps << " GB/s\n";
                *out_stream << "  Total " << raw_rd_header << ": " << read_bytes << "\n";
                *out_stream << "  Total " << raw_wr_header << ": " << write_bytes << "\n";
            }
        }
    };

    bool success = false;
    
#ifdef __linux__
    if (prof_config.backend == ProfilerBackend::AUTO || prof_config.backend == ProfilerBackend::PERF_EVENT) {
        PerfEventProfiler perf_profiler(sys_info, selection);
        if (perf_profiler.is_available()) {
            int interval_ms = summary_mode ? 100 : static_cast<int>(prof_config.sampling_interval_ms());
            success = perf_profiler.monitor_command(full_command, callback, interval_ms, summary_mode);
            if (!success && prof_config.verbose) {
                std::cerr << "Note: perf_event failed (" << perf_profiler.get_error() << "), using perf stat.\n";
            }
        }
    }
#endif

    if (!success) {
        success = measurer->monitor_command(full_command, callback, summary_mode);
    }
    
    if (!success) {
        std::cerr << "Error: Failed to monitor command.\n";
        return 1;
    }

    return 0;
}