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

#ifndef BANDWIDTH_COUNTER_STRATEGY_H
#define BANDWIDTH_COUNTER_STRATEGY_H

#include <string>
#include <vector>
#include <map>
#include <cstdint>
#include <cstdio>

struct CPUCapabilities;

struct ResolvedPerfEvent {
    std::string name;
    uint32_t type = 0;
    uint64_t config = 0;
    int cpu = -1;  // -1 means system-wide (for CPU PMUs), >= 0 for uncore PMUs
    bool valid = false;

    ResolvedPerfEvent() = default;
    ResolvedPerfEvent(const std::string& n) : name(n) {}
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

enum class MeasurerType {
    AUTO,
    PERF,
    LIKWID,
    PCM
};

std::string measurer_type_to_string(MeasurerType type);
MeasurerType string_to_measurer_type(const std::string& str);

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

    std::string get_popen_events_string() const;
};

struct BandwidthCounterSelection {
    bool perf_available = false;
    std::string failure_reason;

    CounterType type = CounterType::CAS_COUNT;

    CasCounterSelection cas;
    UpiFlitSelection upi;

    std::vector<std::string> extra_counters;

    std::string get_all_events_string() const;
    std::string get_bw_events_string() const;
    std::string get_popen_events_string() const;
    bool is_valid() const;
};

struct EventClassification {
    bool is_read = false;
    bool is_write = false;
    bool is_combined = false;
    bool is_extra = false;
    std::string extra_key;
    
    static EventClassification classify(const std::string& event_name, const BandwidthCounterSelection& selection);
    
    static void accumulate_extra(std::map<std::string, long long>& target, 
                                 const std::string& extra_key, long long value);
    static void reset_extra_values(std::map<std::string, long long>& target);
};

class BandwidthCounterStrategy {
public:
    static BandwidthCounterStrategy& instance();

    void set_measurer_type(MeasurerType type);
    void set_extra_counters(const std::vector<std::string>& counters);
    void set_memory_type(const std::string& mem_type);

    void initialize(int src_cpu, const std::vector<int>& target_mem_nodes, const CPUCapabilities& caps);
    bool is_initialized() const { return initialized_; }

    const BandwidthCounterSelection& get_selection() const { return selection_; }

    bool is_hbm() const { return is_hbm_; }
    bool needs_upi() const { return selection_.type == CounterType::UPI_FLITS; }

    MeasurerType get_resolved_measurer_type() const { return resolved_measurer_type_; }
    MeasurerType get_requested_measurer_type() const { return requested_measurer_type_; }
    std::string get_tool_name() const;
    std::string get_measurer_name() const;
    void get_tlb_counters(uint64_t& tlb1_raw, uint64_t& tlb2_raw, bool& use_tlb1, bool& use_tlb2) const;
    void print_counter_info(std::ostream& out) const;

    static std::string formatTlbEventString(uint64_t raw_code) {
        char buf[16];
        snprintf(buf, sizeof(buf), "r%x", static_cast<unsigned int>(raw_code));
        return std::string(buf);
    }

    static CounterType detectCounterType(int src_cpu, int target_mem_node);
    static ResolvedPerfEvent resolveEvent(const std::string& event_name);

    static BandwidthCounterSelection discoverBandwidthCounters(BandwidthCounterStrategy* strategy, int src_cpu, const std::vector<int>& target_mem_nodes, MeasurerType measurer_type = MeasurerType::AUTO);

protected:
    BandwidthCounterStrategy() = default;
    
    void discover_counters(int src_cpu, const std::vector<int>& target_mem_nodes);
    
    static int getPhysicalPackageId(int cpu);
    static int getNodeSocketId(int node_id);
    static int getNodeId(int cpu);
    static int getNodeDistance(int src_node, int dst_node);
    
    static bool queryEventViaPerf(const std::string& event_name, uint32_t& type, uint64_t& config);
    static bool readPmuType(const std::string& pmu_path, uint32_t& type);
    static bool readPmuCpumask(const std::string& pmu_path, int& cpu);
    
    static std::vector<std::string> extractCasEventsFromPerf();
    static std::vector<std::string> extractUpiFlitEventsFromPerf();
    static std::string lookupEventEncoding(const std::string& event_name);
    
    static CasCounterSelection discoverFromPerf();
    static CasCounterSelection discoverFromPerf(const std::vector<std::string>& preferred_read, const std::vector<std::string>& preferred_write, const std::vector<std::string>& preferred_combined);
    static UpiFlitSelection discoverUpiFlitsFromPerf();

public:
    virtual ~BandwidthCounterStrategy() = default;
    
    virtual CasCounterSelection detectCasCounters() = 0;
    virtual void getTlbMissCounters(uint64_t& tlb1_raw, uint64_t& tlb2_raw, bool& use_tlb1, bool& use_tlb2) = 0;

private:
    BandwidthCounterStrategy(const BandwidthCounterStrategy&) = delete;
    BandwidthCounterStrategy& operator=(const BandwidthCounterStrategy&) = delete;

    void resolve_measurer_type();

    bool initialized_ = false;
    bool is_hbm_ = false;
    int cached_src_cpu_ = 0;
    std::vector<int> target_nodes_;
    std::string cached_memory_type_;
    std::vector<std::string> extra_counters_;
    MeasurerType requested_measurer_type_ = MeasurerType::AUTO;
    MeasurerType resolved_measurer_type_ = MeasurerType::PERF;
    BandwidthCounterSelection selection_;

    uint64_t cached_tlb1_raw_ = 0;
    uint64_t cached_tlb2_raw_ = 0;
    bool cached_use_tlb1_ = false;
    bool cached_use_tlb2_ = false;
};

#endif
