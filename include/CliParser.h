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

#ifndef CLI_PARSER_H
#define CLI_PARSER_H

#include "BenchmarkConfig.h"
#include <string>
#include <vector>

/**
 * @file CliParser.h
 * @brief Command-line parser for the `mess` benchmark binary.
 */

/**
 * @brief Parses Mess CLI arguments into a @ref BenchmarkConfig instance.
 */
class CLIParser {
private:
    /**
     * @brief Describes one command-line option for help generation and parsing.
     */
    struct OptionSpec {
        std::string short_opt;   /**< Short option form, including leading dash. */
        std::string long_opt;    /**< Long option form, including leading dashes. */
        bool has_arg;            /**< Whether the option expects a following argument. */
        std::string metavar;     /**< Placeholder name shown in help output. */
        std::string description; /**< Human-readable help text. */
    };

    std::vector<OptionSpec> specs_;

    bool parse_bind(const std::string& s, std::vector<int>& out);
    bool expand_core_list(const std::string& s, std::vector<std::string>& out);
    bool parse_ratios(const std::string& s, std::vector<double>& out);
    bool parse_pauses(const std::string& s, std::vector<int>& out);
    void print_usage(const char* argv0);

public:
    /** @brief Creates the parser and registers supported options. */
    CLIParser();
    ~CLIParser() = default;

    /**
     * @brief Parses the provided argv vector into a benchmark configuration.
     * @param argc Argument count.
     * @param argv Argument vector.
     * @param config Configuration object updated in place.
     * @return `true` on successful parsing.
     */
    bool parse(int argc, char** argv, BenchmarkConfig& config);

    /**
     * @brief Prints the full help text.
     * @param argv0 Program name used in usage examples.
     */
    void print_help(const char* argv0);
    
    /**
     * @brief Prints the resolved run configuration together with environment checks.
     * @param config Parsed benchmark configuration.
     * @param perf_accessible Whether performance counters are currently usable.
     * @param paranoid_level Current `perf_event_paranoid` level.
     * @param tlb_ok Whether the TLB calibration step succeeded.
     * @param tlb_latency_ns Measured TLB-hit latency in nanoseconds.
     */
    void display_configuration(const BenchmarkConfig& config, 
                             bool perf_accessible, 
                             int paranoid_level, 
                             bool tlb_ok, 
                             double tlb_latency_ns) const;
};

#endif
