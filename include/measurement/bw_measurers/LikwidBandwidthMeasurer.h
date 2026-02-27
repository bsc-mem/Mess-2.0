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

#ifndef LIKWID_BANDWIDTH_MEASURER_H
#define LIKWID_BANDWIDTH_MEASURER_H

#include "measurement.h"
#include <string>
#include <vector>
#include <map>

class LikwidBandwidthMeasurer : public BandwidthMeasurer {
public:
    using BandwidthMeasurer::BandwidthMeasurer;

    bool sample_bandwidth(long long& cas_rd, long long& cas_wr, double& elapsed, const std::vector<int>& mem_nodes) const override;
    bool wait_for_stabilization(int& samples_taken, long long& last_cas_rd, long long& last_cas_wr, double& last_elapsed, int pause, int ratio, bool fast_resume, std::function<void()> on_sample) override;

    bool monitor_command(const std::string& command, 
                         MonitorCallback callback,
                         bool summary_mode) override;

    std::string find_likwid_binary() const;
    std::string build_event_string(const std::string& likwidCmd, const std::string& memType, CounterType type) const;

private:
    struct MemoryChannel {
        std::string readCounter;
        std::string writeCounter;
    };

    struct UpiChannel {
        std::string readCounter;
        std::string writeCounter;
    };

    struct ResolvedExtraEvent {
        std::string eventName;
        std::string boxType;
        std::vector<std::string> resolvedStrings;
    };
    
    std::vector<MemoryChannel> parse_memory_counters(const std::string& output, const std::string& memType) const;
    std::vector<UpiChannel> parse_upi_counters(const std::string& output) const;
    std::vector<ResolvedExtraEvent> resolve_extra_counters(
        const std::string& likwidCmd,
        const std::string& bwEventString) const;
    
    void parse_likwid_header(const std::string& line, CounterType type,
                              std::vector<int>& rdIndices, std::vector<int>& wrIndices,
                              std::map<std::string, std::vector<int>>& extraIndices) const;
    std::pair<std::string, CounterType> determine_memory_type() const;

    mutable std::string cached_likwid_binary_;
    mutable std::map<CounterType, std::string> event_string_cache_;
    mutable std::vector<ResolvedExtraEvent> cached_resolved_extras_;
    mutable std::string cached_bw_event_string_;
    mutable bool extras_resolved_ = false;
};

#endif
