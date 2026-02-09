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

#include "benchmark_config.h"
#include <string>
#include <vector>

class CLIParser {
private:
    struct OptionSpec {
        std::string short_opt;
        std::string long_opt;
        bool has_arg;
        std::string metavar;
        std::string description;
    };

    std::vector<OptionSpec> specs_;

    bool parse_bind(const std::string& s, std::vector<int>& out);
    bool parse_ratios(const std::string& s, std::vector<double>& out);
    bool parse_pauses(const std::string& s, std::vector<int>& out);
    void print_usage(const char* argv0);

public:
    CLIParser();
    ~CLIParser() = default;

    bool parse(int argc, char** argv, BenchmarkConfig& config);
    void print_help(const char* argv0);
    
    void display_configuration(const BenchmarkConfig& config, 
                             bool perf_accessible, 
                             int paranoid_level, 
                             bool tlb_ok, 
                             double tlb_latency_ns) const;
};

#endif
