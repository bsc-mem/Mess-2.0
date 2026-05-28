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

#include "KernelTypes.h"

/**
 * @file BenchmarkConfig.h
 * @brief Benchmark result records and user-facing runtime configuration.
 */

/**
 * @brief Stores one measured bandwidth-latency point together with raw metadata.
 */
struct BenchmarkResult {
    double ratio_pct;
    double measured_ratio_pct;
    int pause;
    int repetition;
    ExecutionMode mode;
    double bandwidth_mbps;
    double latency_ns;
    int repetitions;
    double iteration_time_s;
    
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
    
    // Dynamic TLB event names selected by the active counter strategy.
    std::string tlb1_event_name;
    std::string tlb2_event_name;
    
    std::map<std::string, long long> extra_perf_values;

    long long sampler_samples = 0;
    double sampler_mean_ns = 0.0;
    double sampler_iqr_mean_ns = 0.0;
    double sampler_min_ns = 0.0;
    double sampler_median_ns = 0.0;
    double sampler_p90_ns = 0.0;
    double sampler_p95_ns = 0.0;
    double sampler_p99_ns = 0.0;
    double sampler_p99_9_ns = 0.0;
    double sampler_max_ns = 0.0;
    
    double sampler_mean_cycles = 0.0;
    double sampler_iqr_mean_cycles = 0.0;
    double sampler_min_cycles = 0.0;
    double sampler_median_cycles = 0.0;
    double sampler_p90_cycles = 0.0;
    double sampler_p95_cycles = 0.0;
    double sampler_p99_cycles = 0.0;
    double sampler_p99_9_cycles = 0.0;
    double sampler_max_cycles = 0.0;

    std::vector<uint64_t> sampler_raw_latencies;

};

/**
 * @brief Captures the full benchmark configuration after CLI parsing and auto-detection.
 */
/**
 * @brief Global configuration object containing all parameters required to run the benchmark.
 * 
 * This struct aggregates the settings from the command line (`CLIParser`) and holds the state 
 * for the generated memory kernels (`KernelConfig`), traffic ratios, pause lengths, core bindings, 
 * and measurement modes (latency vs bandwidth, execution mode). It acts as the single source 
 * of truth passed down to managers, executors, and output formatters.
 */
struct BenchmarkConfig {
    KernelConfig kernel;

    bool show_version = false;
    bool show_help = false;

    std::vector<double> ratios_pct;
    std::vector<int> pauses;
    std::string adaptive_tier = "standard";
    int adaptive_point_count = 0;
    std::vector<int> memory_bind_nodes; // List of NUMA nodes to bind memory to

    int num_threads = 0;
    int point_reps = 3;

    std::string output_file;
    int verbosity = 1;

    bool profile_output = false;
    bool dry_run = false;
    bool generate_multiseq = true;
    bool instruction_sampling = false;
    bool persistent_traffic_gen = false;

    std::string measurer = "auto";  // auto, perf, likwid, vtune, pcm

    int traffic_gen_cores = 0;
    std::vector<std::string> traffic_gen_explicit_cores;
    std::string output_root = "measuring";

    std::vector<std::string> add_counters;


    /**
     * @brief Returns a short label that describes the memory-binding mode.
     * @return Human-readable binding name used by logs and plotter metadata.
     */
    std::string get_bind_name() const;

    /**
     * @brief Prints a resolved configuration summary.
     * @param os Output stream that receives the summary.
     * @param dry_run When true, emits the dry-run oriented presentation.
     */
    void print_summary(std::ostream& os, bool dry_run = false) const;
};

#endif
