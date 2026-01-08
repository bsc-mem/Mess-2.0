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

#include "benchmark_config.h"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <sstream>



std::string BenchmarkConfig::get_size_name() const {
    switch (size) {
        case SizeTier::LITE: return "lite";
        case SizeTier::MEDIUM: return "medium";
        case SizeTier::FULL: return "full";
        default: return "unset";
    }
}

std::string BenchmarkConfig::get_bind_name() const {
    if (memory_bind_nodes.empty()) {
        return "local";
    }
    
    std::stringstream ss;
    for (size_t i = 0; i < memory_bind_nodes.size(); ++i) {
        ss << memory_bind_nodes[i];
        if (i < memory_bind_nodes.size() - 1) {
            ss << ",";
        }
    }
    return ss.str();
}

void BenchmarkConfig::print_summary(std::ostream& os, bool dry_run) const {
    if (dry_run) {
        return;
    }
    
    os << "Benchmark Configuration:" << std::endl;
    os << "  Size: " << get_size_name() << std::endl;
    os << "  Memory Bind: " << get_bind_name() << std::endl;
    os << "  Threads: " << (num_threads > 0 ? std::to_string(num_threads) : "auto") << std::endl;

    os << "  Ratios: ";
    if (ratios_pct.empty()) {
        os << "(none)";
    } else {
        for (size_t i = 0; i < ratios_pct.size(); ++i) {
            if (i > 0) os << ", ";
            os << ratios_pct[i] << "%";
        }
    }
    os << std::endl;

    os << "  Pause values: ";
    if (pauses.empty()) {
        os << "(none)";
    } else {
        for (size_t i = 0; i < pauses.size(); ++i) {
            if (i > 0) os << ", ";
            os << pauses[i];
        }
    }
    os << std::endl;

    os << "  Repetitions:" << std::endl;
    os << "    Points: " << point_reps << std::endl;

    os << "  Output:" << std::endl;
    os << "    Verbosity: " << verbosity << std::endl;
}
