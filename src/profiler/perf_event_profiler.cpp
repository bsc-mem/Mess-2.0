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

#include "profiler/perf_event_profiler.h"
#include "architecture/ArchitectureRegistry.h"
#include "system_detection.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstring>
#include <chrono>
#include <thread>
#include <signal.h>
#include <sys/wait.h>

#ifdef __linux__
#include <linux/perf_event.h>
#include <sys/ioctl.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <dirent.h>
#endif

void PerfEventGroup::close_all() {
#ifdef __linux__
    for (int fd : member_fds) {
        if (fd >= 0) close(fd);
    }
    if (leader_fd >= 0) close(leader_fd);
    leader_fd = -1;
    member_fds.clear();
    event_names.clear();
#endif
}

PerfEventProfiler::PerfEventProfiler(const system_info& sys_info, const BandwidthCounterSelection& selection)
    : sys_info_(sys_info), selection_(selection) {
    
    if (sys_info_.sockets[0].cache_count > 0) {
        cache_line_size_ = sys_info_.sockets[0].caches[0].line_size_bytes;
    }
    
    if (selection_.type == CounterType::UPI_FLITS) {
        SystemDetector detector;
        detector.detect();
        auto caps = detector.get_capabilities();
        auto arch = ArchitectureRegistry::instance().getArchitecture(caps);
        if (arch) {
            scaling_factor_ = arch->getUpiScalingFactor(caps);
        } else {
            scaling_factor_ = 1.0 / 9.0;
        }
    }
}

PerfEventProfiler::~PerfEventProfiler() {
    cleanup();
}

void PerfEventProfiler::cleanup() {
    for (auto& group : event_groups_) {
        group.close_all();
    }
    event_groups_.clear();
    initialized_ = false;
}

bool PerfEventProfiler::is_available() const {
#ifdef __linux__
    return access("/proc/sys/kernel/perf_event_paranoid", F_OK) == 0;
#else
    return false;
#endif
}

#ifdef __linux__
long PerfEventProfiler::perf_event_open(struct perf_event_attr* hw_event, pid_t pid, int cpu, int group_fd, unsigned long flags) {
    return syscall(__NR_perf_event_open, hw_event, pid, cpu, group_fd, flags);
}

bool PerfEventProfiler::parse_event_config(const std::string& event_name, uint32_t& type, uint64_t& config) {
    std::string sysfs_base = "/sys/bus/event_source/devices/";
    std::string pmu_name;
    std::string event_spec;
    
    size_t slash_pos = event_name.find('/');
    if (slash_pos != std::string::npos) {
        pmu_name = event_name.substr(0, slash_pos);
        event_spec = event_name.substr(slash_pos + 1);
        if (!event_spec.empty() && event_spec.back() == '/') {
            event_spec.pop_back();
        }
    } else {
        std::string search_event = event_name;
        size_t dot_pos = search_event.find('.');
        if (dot_pos != std::string::npos) {
            search_event.replace(dot_pos, 1, "_");
        }
        
        DIR* devices_dir = opendir(sysfs_base.c_str());
        if (!devices_dir) {
            last_error_ = "Cannot open sysfs devices directory";
            return false;
        }
        
        bool found = false;
        struct dirent* entry;
        while ((entry = readdir(devices_dir)) != nullptr) {
            if (entry->d_name[0] == '.') continue;
            
            std::string pmu_path = sysfs_base + entry->d_name;
            std::string events_path = pmu_path + "/events";
            
            DIR* events_dir = opendir(events_path.c_str());
            if (!events_dir) continue;
            
            struct dirent* event_entry;
            while ((event_entry = readdir(events_dir)) != nullptr) {
                std::string ev_name = event_entry->d_name;
                if (ev_name == search_event || ev_name == event_name) {
                    pmu_name = entry->d_name;
                    event_spec = ev_name;
                    found = true;
                    break;
                }
            }
            closedir(events_dir);
            if (found) break;
        }
        closedir(devices_dir);
        
        if (!found) {
            last_error_ = "Event not found in sysfs: " + event_name;
            return false;
        }
    }
    
    std::string type_path = sysfs_base + pmu_name + "/type";
    std::ifstream type_file(type_path);
    if (!type_file.is_open()) {
        last_error_ = "Cannot open PMU type file: " + type_path;
        return false;
    }
    type_file >> type;
    type_file.close();
    
    std::string event_path = sysfs_base + pmu_name + "/events/" + event_spec;
    std::ifstream event_file(event_path);
    if (event_file.is_open()) {
        std::string config_str;
        std::getline(event_file, config_str);
        event_file.close();
        
        size_t eq_pos = config_str.find('=');
        if (eq_pos != std::string::npos) {
            std::string value_str = config_str.substr(eq_pos + 1);
            if (value_str.substr(0, 2) == "0x") {
                config = std::stoull(value_str, nullptr, 16);
            } else {
                config = std::stoull(value_str);
            }
            return true;
        }
    }
    
    config = 0;
    return true;
}

int PerfEventProfiler::open_event(uint32_t type, uint64_t config, int cpu, int group_fd) {
    struct perf_event_attr pe;
    memset(&pe, 0, sizeof(pe));
    
    pe.type = type;
    pe.size = sizeof(pe);
    pe.config = config;
    pe.disabled = (group_fd == -1) ? 1 : 0;
    pe.exclude_kernel = 0;
    pe.exclude_hv = 0;
    pe.read_format = PERF_FORMAT_GROUP | PERF_FORMAT_TOTAL_TIME_ENABLED | PERF_FORMAT_TOTAL_TIME_RUNNING;
    
    int fd = perf_event_open(&pe, -1, cpu, group_fd, 0);
    if (fd < 0) {
        last_error_ = "perf_event_open failed: " + std::string(strerror(errno));
    }
    return fd;
}
#endif

bool PerfEventProfiler::setup_uncore_events(int cpu) {
#ifdef __linux__
    if (!selection_.is_valid()) {
        last_error_ = "Invalid counter selection";
        return false;
    }
    
    PerfEventGroup group;
    
    std::vector<std::string> read_events;
    std::vector<std::string> write_events;
    
    if (selection_.type == CounterType::UPI_FLITS) {
        read_events = selection_.upi.rxl_data_events;
        write_events = selection_.upi.txl_data_events;
    } else {
        read_events = selection_.cas.read_events;
        write_events = selection_.cas.write_events;
    }
    
    for (const auto& event : read_events) {
        uint32_t type;
        uint64_t config;
        
        if (!parse_event_config(event, type, config)) {
            return false;
        }
        
        int fd;
        if (group.leader_fd < 0) {
            fd = open_event(type, config, cpu, -1);
            if (fd < 0) return false;
            group.leader_fd = fd;
        } else {
            fd = open_event(type, config, cpu, group.leader_fd);
            if (fd < 0) return false;
            group.member_fds.push_back(fd);
        }
        group.event_names.push_back(event);
    }
    
    for (const auto& event : write_events) {
        uint32_t type;
        uint64_t config;
        
        if (!parse_event_config(event, type, config)) {
            return false;
        }
        
        int fd;
        if (group.leader_fd < 0) {
            fd = open_event(type, config, cpu, -1);
            if (fd < 0) return false;
            group.leader_fd = fd;
        } else {
            fd = open_event(type, config, cpu, group.leader_fd);
            if (fd < 0) return false;
            group.member_fds.push_back(fd);
        }
        group.event_names.push_back(event);
    }
    
    num_read_events_ = read_events.size();
    
    if (group.is_valid()) {
        event_groups_.push_back(std::move(group));
        return true;
    }
    
    last_error_ = "No events configured";
    return false;
#else
    (void)cpu;
    last_error_ = "perf_event not available on this platform";
    return false;
#endif
}

bool PerfEventProfiler::initialize(const std::vector<int>& cpus) {
#ifdef __linux__
    cleanup();
    
    if (cpus.empty()) {
        target_cpus_ = {0};
    } else {
        target_cpus_ = cpus;
    }
    
    for (int cpu : target_cpus_) {
        if (!setup_uncore_events(cpu)) {
            cleanup();
            return false;
        }
    }
    
    initialized_ = true;
    return true;
#else
    (void)cpus;
    last_error_ = "perf_event not available on this platform";
    return false;
#endif
}

bool PerfEventProfiler::start() {
#ifdef __linux__
    if (!initialized_) {
        last_error_ = "Profiler not initialized";
        return false;
    }
    
    for (auto& group : event_groups_) {
        if (ioctl(group.leader_fd, PERF_EVENT_IOC_ENABLE, PERF_IOC_FLAG_GROUP) < 0) {
            last_error_ = "Failed to enable event group: " + std::string(strerror(errno));
            return false;
        }
    }
    return true;
#else
    last_error_ = "perf_event not available on this platform";
    return false;
#endif
}

bool PerfEventProfiler::stop() {
#ifdef __linux__
    if (!initialized_) return false;
    
    for (auto& group : event_groups_) {
        ioctl(group.leader_fd, PERF_EVENT_IOC_DISABLE, PERF_IOC_FLAG_GROUP);
    }
    return true;
#else
    return false;
#endif
}

bool PerfEventProfiler::reset() {
#ifdef __linux__
    if (!initialized_) return false;
    
    for (auto& group : event_groups_) {
        ioctl(group.leader_fd, PERF_EVENT_IOC_RESET, PERF_IOC_FLAG_GROUP);
    }
    return true;
#else
    return false;
#endif
}

bool PerfEventProfiler::read_counters(Sample& sample) {
#ifdef __linux__
    if (!initialized_ || event_groups_.empty()) {
        last_error_ = "Profiler not initialized";
        return false;
    }
    
    long long total_rd = 0;
    long long total_wr = 0;
    
    for (auto& group : event_groups_) {
        size_t num_events = 1 + group.member_fds.size();
        size_t buffer_size = sizeof(uint64_t) * (3 + num_events);
        std::vector<uint64_t> data(3 + num_events);
        
        ssize_t ret = read(group.leader_fd, data.data(), buffer_size);
        if (ret < 0) {
            last_error_ = "Failed to read counters: " + std::string(strerror(errno));
            return false;
        }
        
        size_t rd_events = num_read_events_;
        for (size_t i = 0; i < num_events && i < group.event_names.size(); ++i) {
            uint64_t value = data[3 + i];
            if (i < rd_events) {
                total_rd += value;
            } else {
                total_wr += value;
            }
        }
    }
    
    sample.read_count = total_rd;
    sample.write_count = total_wr;
    
    return true;
#else
    (void)sample;
    last_error_ = "perf_event not available on this platform";
    return false;
#endif
}

bool PerfEventProfiler::monitor_command(
    const std::string& command,
    std::function<void(double timestamp, double bw_gbps, long long raw_rd, long long raw_wr)> callback,
    int interval_ms,
    bool summary_mode) {
    
#ifdef __linux__
    pid_t pid = fork();
    if (pid < 0) {
        last_error_ = "fork() failed: " + std::string(strerror(errno));
        return false;
    }
    
    if (pid == 0) {
        execl("/bin/sh", "sh", "-c", command.c_str(), nullptr);
        _exit(127);
    }
    
    if (!initialize()) {
        kill(pid, SIGTERM);
        waitpid(pid, nullptr, 0);
        return false;
    }
    
    reset();
    start();
    
    auto start_time = std::chrono::steady_clock::now();
    
    long long total_rd = 0;
    long long total_wr = 0;
    
    long long prev_rd = 0;
    long long prev_wr = 0;
    
    Sample sample;
    int sample_count = 0;
    
    while (true) {
        int status;
        pid_t result = waitpid(pid, &status, WNOHANG);
        
        if (result == pid) {
            break;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
        
        if (read_counters(sample)) {
            sample_count++;
            double timestamp = sample_count * (interval_ms / 1000.0);
            
            long long delta_rd = sample.read_count - prev_rd;
            long long delta_wr = sample.write_count - prev_wr;
            prev_rd = sample.read_count;
            prev_wr = sample.write_count;
            
            double duration_s = interval_ms / 1000.0;
            double bw = 0.0;
            
            if (selection_.type == CounterType::NVIDIA_GRACE) {
                double bytes_rd = static_cast<double>(delta_rd) * 32.0;
                double bytes_wr = static_cast<double>(delta_wr);
                bw = (bytes_rd + bytes_wr) / (duration_s * 1e9);
            } else {
                bw = (delta_rd + delta_wr) * cache_line_size_ * scaling_factor_ / (duration_s * 1e9);
            }
            
            if (!summary_mode && callback) {
                callback(timestamp, bw, delta_rd, delta_wr);
            }
            
            total_rd += delta_rd;
            total_wr += delta_wr;
        }
    }
    
    stop();
    
    auto end_time = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(end_time - start_time).count();
    
    if (summary_mode && callback) {
        double bw = 0.0;
        if (selection_.type == CounterType::NVIDIA_GRACE) {
            double bytes_rd = static_cast<double>(total_rd) * 32.0;
            double bytes_wr = static_cast<double>(total_wr);
            bw = (bytes_rd + bytes_wr) / (elapsed * 1e9);
        } else {
            bw = (total_rd + total_wr) * cache_line_size_ * scaling_factor_ / (elapsed * 1e9);
        }
        callback(elapsed, bw, total_rd, total_wr);
    }
    
    cleanup();
    return true;
#else
    (void)command; (void)callback; (void)interval_ms; (void)summary_mode;
    last_error_ = "perf_event not available on this platform";
    return false;
#endif
}

bool PerfEventProfiler::monitor_pid(
    pid_t pid,
    std::function<void(double timestamp, double bw_gbps, long long raw_rd, long long raw_wr)> callback,
    int interval_ms,
    std::atomic<bool>& stop_flag) {
    
#ifdef __linux__
    if (!initialize()) {
        return false;
    }
    
    reset();
    start();
    
    long long prev_rd = 0;
    long long prev_wr = 0;
    
    Sample sample;
    int sample_count = 0;
    
    while (!stop_flag.load()) {
        if (kill(pid, 0) != 0) {
            break;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
        
        if (read_counters(sample)) {
            sample_count++;
            double timestamp = sample_count * (interval_ms / 1000.0);
            
            long long delta_rd = sample.read_count - prev_rd;
            long long delta_wr = sample.write_count - prev_wr;
            prev_rd = sample.read_count;
            prev_wr = sample.write_count;
            
            double duration_s = interval_ms / 1000.0;
            double bw = 0.0;
            
            if (selection_.type == CounterType::NVIDIA_GRACE) {
                double bytes_rd = static_cast<double>(delta_rd) * 32.0;
                double bytes_wr = static_cast<double>(delta_wr);
                bw = (bytes_rd + bytes_wr) / (duration_s * 1e9);
            } else {
                bw = (delta_rd + delta_wr) * cache_line_size_ * scaling_factor_ / (duration_s * 1e9);
            }
            
            if (callback) {
                callback(timestamp, bw, delta_rd, delta_wr);
            }
        }
    }
    
    stop();
    cleanup();
    return true;
#else
    (void)pid; (void)callback; (void)interval_ms; (void)stop_flag;
    last_error_ = "perf_event not available on this platform";
    return false;
#endif
}
