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
#include "architecture/ArchitectureRegistry.h"
#include "architecture/BandwidthCounterStrategy.h"
#include "utils.h"

BandwidthMeasurer::BandwidthMeasurer(const BenchmarkConfig& config, const system_info& sys_info, const CPUCapabilities& caps, MeasurementStorage* storage, TrafficGenProcessManager* traffic_gen_manager, std::function<std::vector<int>()> numa_resolver, ExecutionMode mode)
    : config_(config),
      sys_info_(sys_info),
      caps_(caps),
      storage_(storage),
      traffic_gen_manager_(traffic_gen_manager),
      numa_resolver_(std::move(numa_resolver)),
      mode_(mode),
      sampling_interval_ms_(50.0) {
    if (sys_info_.socket_count > 0 && sys_info_.sockets[0].cache_count > 0) {
        cached_cache_line_size_ = sys_info_.sockets[0].caches[0].line_size_bytes;
    }
}

double BandwidthMeasurer::calculate_bandwidth_gbps(long long cas_rd, long long cas_wr, double elapsed_s,
                                                    CounterType type, int cache_line_size, double scaling_factor) {
    if (elapsed_s <= 0) return 0.0;

    if (type == CounterType::NVIDIA_GRACE) {
        double bytes_rd = static_cast<double>(cas_rd) * 32.0;
        double bytes_wr = static_cast<double>(cas_wr);
        return (bytes_rd + bytes_wr) / (elapsed_s * 1e9);
    }

    return (cas_rd + cas_wr) * cache_line_size * scaling_factor / (elapsed_s * 1e9);
}

void BandwidthMeasurer::ensure_scaling_factor_cached() const {
    if (scaling_factor_initialized_) return;
    scaling_factor_initialized_ = true;
    
    if (counter_selection_.type == CounterType::UPI_FLITS) {
        auto arch = ArchitectureRegistry::instance().getArchitecture(caps_);
        if (arch) {
            const_cast<BandwidthMeasurer*>(this)->cached_scaling_factor_ = arch->getUpiScalingFactor(caps_);
        } else {
            const_cast<BandwidthMeasurer*>(this)->cached_scaling_factor_ = 1.0;
        }
    }
}

int BandwidthMeasurer::get_traffic_gen_cores() const {
    int cores = sys_info_.sockets[0].core_count - 1;
    if (config_.traffic_gen_cores > 0 && config_.traffic_gen_cores <= sys_info_.sockets[0].core_count - 1) {
        cores = config_.traffic_gen_cores;
    }
    return cores;
}

bool BandwidthMeasurer::relaunch_traffic_gen(int ratio, int pause, int traffic_gen_cores) {
    if (!traffic_gen_manager_) {
        return false;
    }
    std::vector<int> traffic_gen_mem_nodes = numa_resolver_ ? numa_resolver_() : std::vector<int>{0};
    std::string traffic_gen_log_file = storage_->traffic_gen_log_file_path(ratio, pause);
    return traffic_gen_manager_->relaunch_traffic_gen(ratio, pause, traffic_gen_cores, traffic_gen_mem_nodes, traffic_gen_log_file, mode_);
}
