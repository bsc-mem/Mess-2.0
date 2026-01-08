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

#ifndef PERFORMANCE_COUNTER_STRATEGY_H
#define PERFORMANCE_COUNTER_STRATEGY_H

#include <string>
#include <vector>
#include <cstdint>
#include <cstdio>

struct CounterEvent {
    std::string name;
    uint64_t rawCode;
    bool isRaw;
    
    CounterEvent(std::string n) : name(n), rawCode(0), isRaw(false) {}
    CounterEvent(uint64_t code) : name("raw_" + std::to_string(code)), rawCode(code), isRaw(true) {}
    CounterEvent(std::string n, uint64_t code) : name(n), rawCode(code), isRaw(true) {}
};

struct UpiFlitSelection {
    bool perf_available = false;
    std::string failure_reason;

    std::vector<std::string> rxl_data_events;
    std::vector<std::string> txl_data_events;

    bool has_rxl_txl = false;
    bool requires_link_aggregation = false;

    std::string get_rxl_events_string() const;
    std::string get_txl_events_string() const;
    std::string get_all_events_string() const;
};

enum class CounterType {
    CAS_COUNT,
    UPI_FLITS,
    NVIDIA_GRACE,
    UNKNOWN
};

struct CasCounterSelection {
    bool perf_available = false;
    std::string failure_reason;
    
    std::vector<std::string> read_events;
    std::vector<std::string> write_events;
    std::vector<std::string> combined_events;
    
    bool has_read_write = false;
    bool has_combined_counter = false;
    bool requires_channel_aggregation = false;
    
    std::string get_read_events_string() const;
    std::string get_write_events_string() const;
    std::string get_all_events_string() const;
};

struct BandwidthCounterSelection {
    bool perf_available = false;
    std::string failure_reason;
    
    CounterType type = CounterType::CAS_COUNT;

    CasCounterSelection cas;
    UpiFlitSelection upi;

    std::vector<std::string> extra_counters;

    std::string get_all_events_string() const;
    bool is_valid() const;
};

class PerformanceCounterStrategy {
public:
    virtual ~PerformanceCounterStrategy() = default;

    virtual CasCounterSelection detectCasCounters() = 0;

    virtual void getTlbMissCounters(uint64_t& tlb1_raw, uint64_t& tlb2_raw, bool& use_tlb1, bool& use_tlb2) = 0;
    
    static std::string formatTlbEventString(uint64_t raw_code) {
        char buf[16];
        snprintf(buf, sizeof(buf), "r%x", static_cast<unsigned int>(raw_code));
        return std::string(buf);
    }
    
    static CasCounterSelection discoverFromPerf();
    
    static CasCounterSelection discoverFromPerf(
        const std::vector<std::string>& preferred_read,
        const std::vector<std::string>& preferred_write,
        const std::vector<std::string>& preferred_combined
    );
    
    static UpiFlitSelection discoverUpiFlitsFromPerf();
    
    static BandwidthCounterSelection discoverBandwidthCounters(int src_cpu, int target_mem_node, bool remote_explicitly_requested);
    static BandwidthCounterSelection discoverBandwidthCounters(int src_cpu, const std::vector<int>& target_mem_nodes, bool likwid_enabled = false);
    static CounterType detectCounterType(int src_cpu, int target_mem_node);

protected:

private:
    static std::vector<std::string> extractCasEventsFromPerf();
    static std::vector<std::string> extractUpiFlitEventsFromPerf();
    
    static int getPhysicalPackageId(int cpu);
    static int getNodeSocketId(int node_id);
};

#endif
