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

#include "measurement.h"
#include "measurement/bw_measurers/PerfBandwidthMeasurer.h"
#include "measurement/bw_measurers/LikwidBandwidthMeasurer.h"
#include "measurement/bw_measurers/PcmBandwidthMeasurer.h"
#include "architecture/PerformanceCounterStrategy.h"
#include <iostream>

std::unique_ptr<BandwidthMeasurer> create_bandwidth_measurer(
    const BenchmarkConfig& config,
    const system_info& sys_info,
    const CPUCapabilities& caps,
    MeasurementStorage* storage,
    TrafficGenProcessManager* traffic_gen_manager,
    std::function<std::vector<int>()> numa_resolver,
    ExecutionMode mode) {

    static bool printed_measurer_type = false;

    bool is_hbm = caps.memory_type.find("HBM") != std::string::npos;
    
    bool needs_cross_socket = false;
    if (!config.memory_bind_nodes.empty()) {
        int src_cpu = 0;
        if (!config.traffic_gen_explicit_cores.empty()) {
            try { src_cpu = std::stoi(config.traffic_gen_explicit_cores[0]); } catch (...) {}
        }
        for (int node : config.memory_bind_nodes) {
            CounterType ct = PerformanceCounterStrategy::detectCounterType(src_cpu, node);
            if (ct == CounterType::UPI_FLITS) {
                needs_cross_socket = true;
                break;
            }
        }
    }
    
    if (is_hbm && needs_cross_socket && !config.force_likwid) {
        if (config.verbosity >= 1 && !printed_measurer_type) {
            std::cout << "Using Perf Bandwidth Measurer (HBM with remote/cross-socket access)" << std::endl;
            printed_measurer_type = true;
        }
        return std::make_unique<PerfBandwidthMeasurer>(config, sys_info, storage, traffic_gen_manager, numa_resolver, mode);
    }

    if (is_hbm || config.force_likwid) {
        if (config.verbosity >= 1 && !printed_measurer_type) {
            std::cout << "Using Likwid Bandwidth Measurer" << std::endl;
            printed_measurer_type = true;
        }
        
        auto measurer = std::make_unique<LikwidBandwidthMeasurer>(config, sys_info, storage, traffic_gen_manager, numa_resolver, mode);

        if (measurer->find_likwid_binary().empty()) {
            std::cerr << "\n\033[1;31m============================================================\033[0m" << std::endl;
            std::cerr << "\033[1;31m ERROR: Bandwidth Measurement Tool (LIKWID) Not Found!\033[0m" << std::endl;
            std::cerr << "\033[1;31m------------------------------------------------------------\033[0m" << std::endl;
            if (config.force_likwid) {
                std::cerr << " Mode: Forced Likwid (--likwid)" << std::endl;
            } else {
                std::cerr << " Mode: HBM Auto-Detection (" << caps.memory_type << ")" << std::endl;
            }
            std::cerr << " Requirement: 'likwid-perfctr' must be in PATH or LIKWID_PERFCTR_PATH." << std::endl;
            std::cerr << "\n Aborting execution." << std::endl;
            std::cerr << "\033[1;31m============================================================\033[0m\n" << std::endl;
            exit(1);
        }
        return measurer;
    } else if (caps.memory_type == "CXL") {
        return std::make_unique<PcmBandwidthMeasurer>(config, sys_info, storage, traffic_gen_manager, numa_resolver, mode);
    } else {
        return std::make_unique<PerfBandwidthMeasurer>(config, sys_info, storage, traffic_gen_manager, numa_resolver, mode);
    }
}
