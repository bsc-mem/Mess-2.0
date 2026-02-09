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
#include "architecture/BandwidthCounterStrategy.h"
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

    auto& strategy = BandwidthCounterStrategy::instance();
    MeasurerType measurer_type = strategy.get_resolved_measurer_type();

    std::unique_ptr<BandwidthMeasurer> measurer;

    switch (measurer_type) {
        case MeasurerType::LIKWID: {
            if (config.verbosity >= 1 && !printed_measurer_type) {
                std::cout << "Using Likwid Bandwidth Measurer" << std::endl;
                printed_measurer_type = true;
            }

            auto likwid_measurer = std::make_unique<LikwidBandwidthMeasurer>(config, sys_info, caps, storage, traffic_gen_manager, numa_resolver, mode);

            if (likwid_measurer->find_likwid_binary().empty()) {
                if (strategy.get_requested_measurer_type() == MeasurerType::LIKWID) {
                    std::cerr << "\n\033[1;31m============================================================\033[0m" << std::endl;
                    std::cerr << "\033[1;31m ERROR: Bandwidth Measurement Tool (LIKWID) Not Found!\033[0m" << std::endl;
                    std::cerr << "\033[1;31m------------------------------------------------------------\033[0m" << std::endl;
                    std::cerr << " Mode: Forced Likwid (--measurer=likwid)" << std::endl;
                    std::cerr << " Requirement: 'likwid-perfctr' must be in PATH or LIKWID_PERFCTR_PATH." << std::endl;
                    std::cerr << "\n Aborting execution." << std::endl;
                    std::cerr << "\033[1;31m============================================================\033[0m\n" << std::endl;
                    return nullptr;
                }
                std::cerr << "Warning: LIKWID backend unavailable; falling back to perf measurer." << std::endl;
                measurer = std::make_unique<PerfBandwidthMeasurer>(config, sys_info, caps, storage, traffic_gen_manager, numa_resolver, mode);
                break;
            }
            measurer = std::move(likwid_measurer);
            break;
        }

        case MeasurerType::PCM: {
            if (strategy.get_requested_measurer_type() == MeasurerType::PCM) {
                std::cerr << "ERROR: PCM measurer is still work-in-progress and currently disabled." << std::endl;
                std::cerr << "       Please use --measurer=perf or --measurer=likwid." << std::endl;
                return nullptr;
            }
            std::cerr << "Warning: PCM measurer is not enabled yet; falling back to perf measurer." << std::endl;
            measurer = std::make_unique<PerfBandwidthMeasurer>(config, sys_info, caps, storage, traffic_gen_manager, numa_resolver, mode);
            break;
        }

        case MeasurerType::PERF:
        default: {
            if (config.verbosity >= 1 && !printed_measurer_type) {
                std::cout << "Using Perf Bandwidth Measurer" << std::endl;
                printed_measurer_type = true;
            }
            measurer = std::make_unique<PerfBandwidthMeasurer>(config, sys_info, caps, storage, traffic_gen_manager, numa_resolver, mode);
            break;
        }
    }

    measurer->set_counter_selection(strategy.get_selection());

    return measurer;
}
