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

#ifndef RESULTS_PROCESSOR_H
#define RESULTS_PROCESSOR_H

#include "benchmark_config.h"
#include "benchmark_executor.h"
#include <vector>
#include <string>
#include <memory>
#include <ostream>

class ResultsProcessor {
private:
    const BenchmarkConfig& config_;
    std::vector<BenchmarkResult> results_;
    double total_runtime_;
    std::vector<double> iteration_times_;
    const BenchmarkExecutor* executor_;

    bool write_csv_output(std::ostream& os) const;
    bool write_json_output(std::ostream& os) const;
    void print_timing_statistics() const;
    bool save_profile_files();

    std::string detect_available_bw_tool() const;
    std::string detect_available_lat_tool() const;

public:
    ResultsProcessor(const BenchmarkConfig& config, const BenchmarkExecutor* executor = nullptr);
    ~ResultsProcessor() = default;

    bool process(const std::vector<BenchmarkResult>& results, double total_runtime = 0.0, const std::vector<double>& iteration_times = {});
};

#endif
