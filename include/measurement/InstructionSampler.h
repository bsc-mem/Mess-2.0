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

#ifndef INSTRUCTION_SAMPLER_H
#define INSTRUCTION_SAMPLER_H

#include <vector>
#include <cstdint>
#include <sys/types.h>

/**
 * @file InstructionSampler.h
 * @brief Abstract interface for instruction-level latency sampling backends.
 */

/**
 * @brief Summary statistics returned by an instruction-sampling backend.
 */
struct InstructionSamplerStats {
    long long samples = 0;
    
    double mean_cycles = 0.0;
    double iqr_mean_cycles = 0.0;
    double min_cycles = 0.0;
    double median_cycles = 0.0;
    double p90_cycles = 0.0;
    double p95_cycles = 0.0;
    double p99_cycles = 0.0;
    double p99_9_cycles = 0.0;
    double max_cycles = 0.0;

    double mean_ns = 0.0;
    double iqr_mean_ns = 0.0;
    double min_ns = 0.0;
    double median_ns = 0.0;
    double p90_ns = 0.0;
    double p95_ns = 0.0;
    double p99_ns = 0.0;
    double p99_9_ns = 0.0;
    double max_ns = 0.0;

    std::vector<uint64_t> raw_latencies;
};

/**
 * @brief Base class for hardware-backed instruction latency samplers.
 * 
 * Instead of relying on a software-based pointer chasing loop, modern architectures
 * support precise hardware instruction sampling (e.g., Intel PEBS, ARM SPE) which captures
 * the exact latency of individual memory instructions.
 * 
 */
class InstructionSampler {
public:
    virtual ~InstructionSampler() = default;

    virtual bool is_available() const = 0;

    virtual bool start_async(const std::vector<int>& cores, const std::vector<pid_t>& pids) = 0;

    virtual InstructionSamplerStats stop_and_process(double cpu_freq_ghz) = 0;

    virtual void cleanup() {}
};

#endif

