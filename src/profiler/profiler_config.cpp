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

#include "profiler/profiler_config.h"
#include <sstream>
#include <iostream>
#include <cctype>

double ProfilerConfig::sampling_interval_ms() const {
    return static_cast<double>(parse_time_to_ms(sampling_interval_str));
}

std::string ProfilerConfig::get_command_string() const {
    std::string result;
    for (const auto& arg : command_args) {
        if (!result.empty()) result += " ";
        if (arg.find(' ') != std::string::npos) {
            result += "\"" + arg + "\"";
        } else {
            result += arg;
        }
    }
    return result;
}

int parse_time_to_ms(const std::string& time_str) {
    if (time_str.empty()) return 0;
    
    std::string num_part;
    std::string unit_part;
    size_t i = 0;
    while (i < time_str.size() && (std::isdigit(time_str[i]) || time_str[i] == '.')) {
        num_part += time_str[i];
        i++;
    }
    while (i < time_str.size()) {
        unit_part += std::tolower(time_str[i]);
        i++;
    }

    double val = std::stod(num_part);
    if (unit_part == "s" || unit_part == "sec") {
        return static_cast<int>(val * 1000);
    } else if (unit_part == "ms") {
        return static_cast<int>(val);
    } else if (unit_part == "us" || unit_part == "µs") {
        return static_cast<int>(val / 1000);
    }
    return static_cast<int>(val);
}

std::set<int> parse_cpu_list_arg(const std::string& arg) {
    std::set<int> cpus;
    std::stringstream ss(arg);
    std::string token;
    
    while (std::getline(ss, token, ',')) {
        size_t dash = token.find('-');
        if (dash != std::string::npos) {
            int start = std::stoi(token.substr(0, dash));
            int end = std::stoi(token.substr(dash + 1));
            for (int i = start; i <= end; ++i) {
                cpus.insert(i);
            }
        } else {
            cpus.insert(std::stoi(token));
        }
    }
    return cpus;
}

std::set<int> parse_node_list_arg(const std::string& arg) {
    return parse_cpu_list_arg(arg);
}

std::string get_counter_unit_description(CounterType type, int cache_line_size) {
    switch (type) {
        case CounterType::CAS_COUNT:
            return "CAS_COUNT (cache line accesses, " + std::to_string(cache_line_size) + " bytes each)";
        case CounterType::UPI_FLITS:
            return "UPI_FLITS (flit counts, scaled by architecture factor)";
        case CounterType::NVIDIA_GRACE:
            return "NVIDIA_GRACE (read: 32-byte beats, write: bytes)";
        default:
            return "UNKNOWN";
    }
}

std::string get_raw_read_header(CounterType type, int cache_line_size) {
    (void)cache_line_size;
    (void)type;
    return "ReadBytes";
}

std::string get_raw_write_header(CounterType type, int cache_line_size) {
    (void)cache_line_size;
    (void)type;
    return "WriteBytes";
}

void print_profiler_help(const char* prog_name) {
    std::cout << "Mess Profiler - Memory Bandwidth Profiler\n"
              << "Usage: " << prog_name << " [options] [--] <command> [args...]\n"
              << "\n"
              << "Measurement Options:\n"
              << "  -s, --interval <time>   Sampling interval (e.g., 100ms, 1s). Default: summary mode.\n"
              << "  -o, --output <file>     Output file. Default: stdout.\n"
              << "\n"
              << "Targeting Options:\n"
              << "  -a, --system-wide       System-wide profiling (all CPUs/sockets).\n"
              << "  -p, --pid <pid>         Profile existing process by PID.\n"
              << "  -C, --cpu <list>        Profile only specified CPUs (e.g., 0-7,16-23).\n"
              << "  -N, --nodes <list>      Monitor memory traffic to specified NUMA nodes.\n"
              << "  --no-inherit            Don't inherit binding from parent (numactl/taskset).\n"
              << "\n"
              << "Output Options:\n"
              << "  -v, --verbose           Verbose output (show counter details).\n"
              << "  --csv                   CSV output (default).\n"
              << "  --human                 Human-readable output.\n"
              << "\n"
              << "Backend Options:\n"
              << "  --backend <name>        Select backend: auto, perf-event, perf-stat, likwid.\n"
              << "                          Default: auto (uses perf_event_open, falls back to perf stat).\n"
              << "\n"
              << "Other Options:\n"
              << "  --dry, --dry-run        Dry run: show detected counters and exit.\n"
              << "  --profet                Add profet counters (cycles,instructions,longest-lat-cache-miss alias auto-detected).\n"
              << "  -h, --help              Show this help.\n"
              << "\n"
              << "Examples:\n"
              << "  " << prog_name << " -s 100ms ./my_app             # Profile with 100ms sampling\n"
              << "  " << prog_name << " -s 50ms -o bw.csv ./my_app    # Save to file\n"
              << "  " << prog_name << " -a -s 1s sleep 10             # System-wide, 1s intervals\n"
              << "  " << prog_name << " -p 12345 -s 100ms             # Attach to running process\n"
              << "  numactl -m 0 -C 0-7 " << prog_name << " -s 100ms ./app  # Inherits numactl binding\n"
              << "  " << prog_name << " -C 0-7 -N 0 -s 100ms ./app    # Explicit CPU/node binding\n"
              << "\n"
              << "Counter Units:\n"
              << "  CAS_COUNT:    Raw values = cache line accesses (typically 64B each)\n"
              << "  UPI_FLITS:    Raw values = flit counts (cross-socket traffic)\n"
              << "  NVIDIA_GRACE: Read = 32-byte beats (cmem_rd_data*32), Write = bytes (cmem_wr_total_bytes)\n";
}
