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
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <algorithm>
#include <chrono>

int parse_time_to_ms(const std::string& time_str) {
    std::string num_part;
    std::string unit_part;
    size_t i = 0;
    while (i < time_str.size() && (isdigit(time_str[i]) || time_str[i] == '.')) {
        num_part += time_str[i];
        i++;
    }
    while (i < time_str.size()) {
        unit_part += time_str[i];
        i++;
    }

    double val = std::stod(num_part);
    if (unit_part == "s" || unit_part == "sec") {
        return static_cast<int>(val * 1000);
    } else if (unit_part == "ms") {
        return static_cast<int>(val);
    }
    return static_cast<int>(val);
}

void print_help(const char* prog_name) {
    std::cout << "Usage: " << prog_name << " [options] <command>\n"
              << "Options:\n"
              << "  -s <time>   Sampling interval (e.g., 1s, 100ms). If not set, prints summary.\n"
              << "  -f <file>   Output file. Default is stdout.\n"
              << "  --dry       Dry run: print detected counters and command, then exit.\n"
              << "  -h, --help  Show this help.\n";
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        print_help(argv[0]);
        return 1;
    }

    std::string sampling_interval_str;
    std::string output_file;
    std::vector<std::string> command_args;
    bool dry_run = false;
    
    int arg_idx = 1;
    while (arg_idx < argc) {
        std::string arg = argv[arg_idx];
        if (arg == "-s") {
            if (arg_idx + 1 < argc) {
                sampling_interval_str = argv[arg_idx + 1];
                arg_idx += 2;
            } else {
                std::cerr << "Error: -s requires an argument.\n";
                return 1;
            }
        } else if (arg == "-f") {
            if (arg_idx + 1 < argc) {
                output_file = argv[arg_idx + 1];
                arg_idx += 2;
            } else {
                std::cerr << "Error: -f requires an argument.\n";
                return 1;
            }
        } else if (arg == "--dry") {
            dry_run = true;
            arg_idx++;
        } else if (arg == "-h" || arg == "--help") {
            print_help(argv[0]);
            return 0;
        } else {
            for (int i = arg_idx; i < argc; ++i) {
                command_args.push_back(argv[i]);
            }
            break;
        }
    }

    if (command_args.empty() && !dry_run) {
        std::cerr << "Error: No command specified.\n";
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
    if (!arch) {
        std::cerr << "Warning: Unknown architecture, using generic fallbacks.\n";
    }

    BenchmarkConfig config;
    config.verbosity = 0;
    
    std::vector<int> all_mem_nodes;
    for (int i = 0; i < sys_info.numa_node_count; ++i) {
        all_mem_nodes.push_back(sys_info.numa_nodes[i].id);
    }
    
    auto measurer = create_bandwidth_measurer(
        config, 
        sys_info, 
        caps, 
        nullptr, 
        nullptr, 
        [&]() { return all_mem_nodes; },
        ExecutionMode::MULTISEQUENTIAL
    );

    if (!measurer) {
        std::cerr << "Error: Failed to create bandwidth measurer.\n";
        return 1;
    }

    if (dry_run) {
        std::cout << "Detected Architecture: " << (arch ? arch->getName() : "Unknown") << "\n";
        std::cout << "Detected CPU Model: " << sys_info.cpu_model << "\n";
        std::cout << "Measurer Type: " << (caps.memory_type.find("HBM") != std::string::npos ? "Likwid" : "Perf") << "\n";
        
        if (caps.memory_type.find("HBM") != std::string::npos) {
             std::cout << "Detected Counters: Likwid HBM Counters (CAS_COUNT_RD/WR for all channels)\n";
             std::cout << "Counter Type: CAS_COUNT (HBM)\n";
        } else {
            BandwidthCounterSelection selection = PerformanceCounterStrategy::discoverBandwidthCounters(0, all_mem_nodes);
            std::cout << "Detected Counters: " << selection.get_all_events_string() << "\n";
            std::cout << "Counter Type: ";
            if (selection.type == CounterType::CAS_COUNT) std::cout << "CAS_COUNT\n";
            else if (selection.type == CounterType::UPI_FLITS) std::cout << "UPI_FLITS\n";
            else if (selection.type == CounterType::NVIDIA_GRACE) std::cout << "NVIDIA_GRACE\n";
            else std::cout << "UNKNOWN\n";
        }

        return 0;
    }

    std::ostream* out_stream = &std::cout;
    std::ofstream file_stream;
    if (!output_file.empty()) {
        file_stream.open(output_file);
        if (!file_stream.is_open()) {
            std::cerr << "Error: Could not open output file " << output_file << "\n";
            return 1;
        }
        out_stream = &file_stream;
    }

    if (!sampling_interval_str.empty()) {
        *out_stream << "Timestamp(s),Bandwidth(GB/s),RawRead,RawWrite\n";
        measurer->set_sampling_interval_ms(parse_time_to_ms(sampling_interval_str));
    } else {
        *out_stream << "Summary:\n";
    }

    std::string full_command;
    for (const auto& arg : command_args) {
        full_command += arg + " ";
    }

    bool summary_mode = sampling_interval_str.empty();
    
    auto callback = [&](double timestamp, double bw_gbps, long long raw_rd, long long raw_wr) {
        if (!summary_mode) {
            *out_stream << std::fixed << std::setprecision(4) << timestamp << "," << bw_gbps << "," << raw_rd << "," << raw_wr << "\n";
        } else {
            *out_stream << "  Average Bandwidth: " << std::fixed << std::setprecision(2) << bw_gbps << " GB/s\n";
            *out_stream << "  Total Reads: " << raw_rd << " (raw)\n";
            *out_stream << "  Total Writes: " << raw_wr << " (raw)\n";
            *out_stream << "  Duration: " << std::fixed << std::setprecision(2) << timestamp << " s\n";
        }
    };

    if (!measurer->monitor_command(full_command, callback, summary_mode)) {
        std::cerr << "Error: Failed to monitor command.\n";
        return 1;
    }

    return 0;
}