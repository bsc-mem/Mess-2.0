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

#ifndef PERF_EVENT_PROFILER_H
#define PERF_EVENT_PROFILER_H

#include "architecture/PerformanceCounterStrategy.h"
#include "system_info.h"
#include <functional>
#include <string>
#include <vector>
#include <atomic>

#ifdef __linux__
#include <linux/perf_event.h>
#include <sys/ioctl.h>
#include <sys/syscall.h>
#include <unistd.h>
#endif

struct PerfEventGroup {
    int leader_fd = -1;
    std::vector<int> member_fds;
    std::vector<std::string> event_names;
    
    void close_all();
    bool is_valid() const { return leader_fd >= 0; }
};

class PerfEventProfiler {
public:
    PerfEventProfiler(const system_info& sys_info, const BandwidthCounterSelection& selection);
    ~PerfEventProfiler();
    
    PerfEventProfiler(const PerfEventProfiler&) = delete;
    PerfEventProfiler& operator=(const PerfEventProfiler&) = delete;
    
    bool initialize(const std::vector<int>& cpus = {});
    
    bool start();
    bool stop();
    bool reset();
    
    struct Sample {
        double timestamp;
        double bandwidth_gbps;
        long long read_count;
        long long write_count;
    };
    
    bool read_counters(Sample& sample);
    
    bool monitor_command(
        const std::string& command,
        std::function<void(double timestamp, double bw_gbps, long long raw_rd, long long raw_wr)> callback,
        int interval_ms,
        bool summary_mode
    );
    
    bool monitor_pid(
        pid_t pid,
        std::function<void(double timestamp, double bw_gbps, long long raw_rd, long long raw_wr)> callback,
        int interval_ms,
        std::atomic<bool>& stop_flag
    );
    
    std::string get_error() const { return last_error_; }
    bool is_available() const;
    
private:
    const system_info& sys_info_;
    BandwidthCounterSelection selection_;
    
    std::vector<PerfEventGroup> event_groups_;
    std::vector<int> target_cpus_;
    
    int cache_line_size_ = 64;
    double scaling_factor_ = 1.0;
    size_t num_read_events_ = 0;
    
    std::string last_error_;
    bool initialized_ = false;
    
#ifdef __linux__
    static long perf_event_open(struct perf_event_attr* hw_event, pid_t pid, int cpu, int group_fd, unsigned long flags);
    bool parse_event_config(const std::string& event_name, uint32_t& type, uint64_t& config);
    int open_event(uint32_t type, uint64_t config, int cpu, int group_fd);
#endif
    
    bool setup_uncore_events(int cpu);
    void cleanup();
};

#endif
