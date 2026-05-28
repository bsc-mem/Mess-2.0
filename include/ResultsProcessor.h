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

#include "BenchmarkConfig.h"
#include "BenchmarkExecutor.h"
#include <vector>
#include <string>
#include <memory>
#include <ostream>

/**
 * @file ResultsProcessor.h
 * @brief Post-processing helpers for benchmark results and profile output.
 */

/**
 * @brief Formats benchmark results for console, JSON, CSV, and profile-oriented outputs.
 */
/**
 * @brief Processes, aggregates, and formats benchmark measurements for output.
 * 
 * After the `BenchmarkExecutor` finishes all traffic iterations, this class takes 
 * the raw `BenchmarkResult` structs and formats them into the final output representations.
 * It handles the calculation of derived metrics (like Read/Write split percentages), 
 * terminal-friendly table printing, CSV output generation, and JSON exporting.
 * It also prints timing statistics indicating the total wall-clock duration of the runs.
 */
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

public:
    /** @brief Creates a processor bound to the active benchmark configuration. */
    ResultsProcessor(const BenchmarkConfig& config, const BenchmarkExecutor* executor = nullptr);
    ~ResultsProcessor() = default;

    /**
     * @brief Processes a completed benchmark run.
     * @param results Benchmark points to process.
     * @param total_runtime Aggregate runtime in seconds.
     * @param iteration_times Per-iteration runtime values in seconds.
     * @return `true` on success.
     */
    bool process(const std::vector<BenchmarkResult>& results, double total_runtime = 0.0, const std::vector<double>& iteration_times = {});
};

#endif
