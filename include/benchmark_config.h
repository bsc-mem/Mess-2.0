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

#ifndef BENCHMARK_CONFIG_H
#define BENCHMARK_CONFIG_H

#include <vector>
#include <string>
#include <memory>
#include <cstdint>
#include <map>

#include "kernel_types.h"

struct BenchmarkResult {
    double ratio_pct;           // Requested read/write ratio percentage
    double measured_ratio_pct;  // Actual measured read/write ratio percentage
    int pause;                  // Pause value for stress kernel
    int repetition;             // Repetition number
    ExecutionMode mode;         // Execution mode
    double bandwidth_mbps;      // Measured bandwidth in MB/s
    double latency_ns;          // Measured latency in nanoseconds
    int repetitions;            // Number of measurement repetitions
    double iteration_time_s;    // Time taken for this iteration in seconds
    
    double cycles;
    double instructions;
    double accesses;
    double tlb1miss;
    double tlb2miss;
    double duration_s;
    bool using_hugepages;
    long long bw_cas_rd;
    long long bw_cas_wr;
    double bw_elapsed;
    int traffic_gen_samples;
    
    // Dynamic TLB event names - set from architecture strategy
    std::string tlb1_event_name;
    std::string tlb2_event_name;
    
    std::map<std::string, long long> extra_perf_values;
};

struct BenchmarkConfig {
    KernelConfig kernel;

    bool show_version = false;
    bool show_help = false;

    std::vector<double> ratios_pct;
    std::vector<int> pauses;
    std::vector<int> memory_bind_nodes; // List of NUMA nodes to bind memory to

    int num_threads = 0;
    int point_reps = 3;

    std::string output_file;
    int verbosity = 1;

    bool profile_output = false;
    bool dry_run = false;
    bool generate_multiseq = true;
    bool persistent_traffic_gen = false;

    std::string measurer = "auto";  // auto, perf, likwid, pcm

    int traffic_gen_cores = 0;
    std::vector<std::string> traffic_gen_explicit_cores;
    std::string output_root = "measuring";

    std::vector<std::string> add_counters;

    std::string get_bind_name() const;
    void print_summary(std::ostream& os, bool dry_run = false) const;
};

#endif
