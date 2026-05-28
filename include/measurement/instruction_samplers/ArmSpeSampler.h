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

#ifndef ARM_SPE_SAMPLER_H
#define ARM_SPE_SAMPLER_H

#include "../InstructionSampler.h"
#include <string>
#include <thread>
#include <atomic>
#include <vector>
#include <cstdint>

/**
 * @file ArmSpeSampler.h
 * @brief ARM SPE instruction-sampling backend.
 */

/**
 * @brief Collects load-to-use latency samples using ARM SPE through `perf`.
 */
class ArmSpeSampler : public InstructionSampler {
public:
    ArmSpeSampler(uint64_t min_latency_threshold = 50, int sample_period = 4096, int verbosity = 0);
    ~ArmSpeSampler() override;

    static std::string availability_diagnostic();
    bool is_available() const override;
    bool start_async(const std::vector<int>& cores, const std::vector<pid_t>& pids) override;
    InstructionSamplerStats stop_and_process(double cpu_freq_ghz) override;
    void cleanup() override;

private:
    uint64_t min_latency_threshold_;
    int sample_period_;
    int verbosity_;

    std::atomic<bool> is_running_{false};
    std::atomic<bool> stop_requested_{false};
    std::thread sampling_thread_;
    std::string current_perf_data_path_;
    std::string current_perf_log_path_;
    pid_t perf_child_pid_ = -1;
    std::string selected_cpu_spec_;
    std::string selected_pid_spec_;

    void sampling_thread_func();

    bool parse_perf_script_output(const std::string& perf_data_path,
                                  std::vector<uint64_t>& latencies);

    static bool probe_spe_record(const std::string& pmu_name);
    static bool detect_spe_pmu();
    static std::string spe_pmu_name();
};

#endif

