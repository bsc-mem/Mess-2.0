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

#ifndef PROFILER_CONFIG_H
#define PROFILER_CONFIG_H

#include <string>
#include <vector>
#include <set>
#include "architecture/PerformanceCounterStrategy.h"

enum class ProfilerBackend {
    AUTO,
    PERF_EVENT,
    PERF_STAT,
    LIKWID
};

struct ProfilerConfig {
    std::string sampling_interval_str;
    std::string output_file;
    std::vector<std::string> command_args;
    
    bool dry_run = false;
    bool system_wide = false;
    bool verbose = false;
    bool show_raw_units = true;
    bool csv_output = true;
    
    pid_t target_pid = -1;
    
    std::set<int> cpu_list;
    std::set<int> mem_nodes;
    
    bool auto_detect_binding = true;
    
    int verbosity = 0;
    
    ProfilerBackend backend = ProfilerBackend::AUTO;
    
    double sampling_interval_ms() const;
    bool has_sampling() const { return !sampling_interval_str.empty(); }
    std::string get_command_string() const;
};

int parse_time_to_ms(const std::string& time_str);

std::set<int> parse_cpu_list_arg(const std::string& arg);
std::set<int> parse_node_list_arg(const std::string& arg);

std::string get_counter_unit_description(CounterType type, int cache_line_size);
std::string get_raw_read_header(CounterType type, int cache_line_size);
std::string get_raw_write_header(CounterType type, int cache_line_size);

void print_profiler_help(const char* prog_name);

#endif
