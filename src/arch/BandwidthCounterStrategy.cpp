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

#include "architecture/BandwidthCounterStrategy.h"
#include "architecture/ArchitectureRegistry.h"
#include "system_detection.h"
#include "utils.h"
#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <regex>
#include <set>
#include <sstream>
#include <map>
#include <unistd.h>
#include <dirent.h>
#include <sys/wait.h>

std::string measurer_type_to_string(MeasurerType type) {
    switch (type) {
        case MeasurerType::AUTO: return "auto";
        case MeasurerType::PERF: return "perf";
        case MeasurerType::LIKWID: return "likwid";
        case MeasurerType::PCM: return "pcm";
        default: return "unknown";
    }
}

MeasurerType string_to_measurer_type(const std::string& str) {
    std::string lower = str;
    std::transform(lower.begin(), lower.end(), lower.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (lower == "perf") return MeasurerType::PERF;
    if (lower == "likwid") return MeasurerType::LIKWID;
    if (lower == "pcm") return MeasurerType::PCM;
    return MeasurerType::AUTO;
}

namespace {

std::string trim_event_token(const std::string& str) {
    size_t first = str.find_first_not_of(" \t\r\n");
    if (std::string::npos == first) {
        return "";
    }
    size_t last = str.find_last_not_of(" \t\r\n");
    return str.substr(first, (last - first + 1));
}

std::string to_lower_copy(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}

bool contains_ci(const std::string& haystack, const std::string& needle) {
    if (needle.size() > haystack.size()) return false;
    return std::search(haystack.begin(), haystack.end(), needle.begin(), needle.end(),
        [](unsigned char a, unsigned char b) { return std::tolower(a) == std::tolower(b); }
    ) != haystack.end();
}

std::string join_events(const std::vector<std::string>& events, const std::string& sep = ",") {
    std::string result;
    for (size_t i = 0; i < events.size(); ++i) {
        result += events[i];
        if (i < events.size() - 1) result += sep;
    }
    return result;
}

std::vector<std::string> find_pmu_instances(const std::string& base_pmu_name) {
    std::vector<std::string> instances;
    DIR* dir = opendir("/sys/bus/event_source/devices");
    if (!dir) return instances;
    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        std::string name = entry->d_name;
        if (name.find(base_pmu_name + "_") != 0) continue;
        std::string suffix = name.substr(base_pmu_name.length() + 1);
        bool all_digits = !suffix.empty();
        for (char c : suffix) {
            if (!std::isdigit(static_cast<unsigned char>(c))) { all_digits = false; break; }
        }
        if (all_digits) instances.push_back(name);
    }
    closedir(dir);
    std::sort(instances.begin(), instances.end());
    return instances;
}

std::vector<std::string> scan_pmu_event_dir(const std::string& pmu_name,
                                             std::function<bool(const std::string&)> filter = nullptr) {
    std::vector<std::string> events;
    std::string events_path = "/sys/bus/event_source/devices/" + pmu_name + "/events";
    DIR* events_dir = opendir(events_path.c_str());
    if (!events_dir) return events;
    struct dirent* event_entry;
    while ((event_entry = readdir(events_dir)) != nullptr) {
        std::string event_name = event_entry->d_name;
        if (event_name == "." || event_name == "..") continue;
        if (!filter || filter(event_name)) {
            events.push_back(pmu_name + "/" + event_name + "/");
        }
    }
    closedir(events_dir);
    return events;
}

std::string run_perf_capture(const std::vector<std::string>& args) {
    int pipefd[2];
    if (pipe(pipefd) < 0) return "";

    pid_t pid = fork();
    if (pid < 0) {
        close(pipefd[0]);
        close(pipefd[1]);
        return "";
    }

    if (pid == 0) {
        close(pipefd[0]);
        dup2(pipefd[1], STDOUT_FILENO);
        dup2(pipefd[1], STDERR_FILENO);
        close(pipefd[1]);
        std::vector<const char*> argv;
        argv.push_back("perf");
        for (const auto& a : args) argv.push_back(a.c_str());
        argv.push_back(nullptr);
        execvp("perf", const_cast<char* const*>(argv.data()));
        _exit(127);
    }

    close(pipefd[1]);
    std::string output;
    output.reserve(8192);
    char buffer[1024];
    ssize_t n;
    while ((n = read(pipefd[0], buffer, sizeof(buffer) - 1)) > 0) {
        buffer[n] = '\0';
        output += buffer;
        if (output.size() > 65536) break;
    }
    close(pipefd[0]);

    int status;
    waitpid(pid, &status, 0);
    return output;
}

std::string strip_pmu_instance_suffix(const std::string& name) {
    size_t last_underscore = name.rfind('_');
    if (last_underscore != std::string::npos && last_underscore < name.length() - 1) {
        bool all_digits = true;
        for (size_t i = last_underscore + 1; i < name.length(); ++i) {
            if (!std::isdigit(name[i])) { all_digits = false; break; }
        }
        if (all_digits) return name.substr(0, last_underscore);
    }
    return name;
}

int getNodeId(int cpu) {
    static std::map<int, int> cache;
    if (cache.count(cpu)) return cache[cpu];

    std::string cpu_path = "/sys/devices/system/cpu/cpu" + std::to_string(cpu);
    DIR* dir = opendir(cpu_path.c_str());
    if (!dir) return -1;
    
    struct dirent* entry;
    int node = -1;
    while ((entry = readdir(dir)) != nullptr) {
        if (strncmp(entry->d_name, "node", 4) == 0 && std::isdigit(entry->d_name[4])) {
            try {
                node = std::stoi(&entry->d_name[4]);
            } catch (...) {}
            break;
        }
    }
    closedir(dir);
    if (node != -1) cache[cpu] = node;
    return node;
}

int getNodeDistance(int src_node, int dst_node) {
    if (src_node == dst_node) return 10;
    
    static std::map<std::pair<int, int>, int> cache;
    if (cache.count({src_node, dst_node})) return cache[{src_node, dst_node}];

    std::string path = "/sys/devices/system/node/node" + std::to_string(src_node) + "/distance";
    std::ifstream f(path);
    if (!f.is_open()) return -1;

    int distance = -1;
    int val;
    int idx = 0;
    while (f >> val) {
        if (idx == dst_node) {
            distance = val;
            break;
        }
        idx++;
    }
    
    if (distance != -1) cache[{src_node, dst_node}] = distance;
    return distance;
}

}

class BandwidthCounterStrategySingleton : public BandwidthCounterStrategy {
public:
    static BandwidthCounterStrategySingleton& instance() {
        static BandwidthCounterStrategySingleton instance;
        return instance;
    }
    
    CasCounterSelection detectCasCounters() override {
        return discoverFromPerf();
    }
    
    void getTlbMissCounters(uint64_t& tlb1_raw, uint64_t& tlb2_raw, bool& use_tlb1, bool& use_tlb2) override {
        get_tlb_counters(tlb1_raw, tlb2_raw, use_tlb1, use_tlb2);
    }

protected:
    BandwidthCounterStrategySingleton() = default;
};

BandwidthCounterStrategy& BandwidthCounterStrategy::instance() {
    return BandwidthCounterStrategySingleton::instance();
}

void BandwidthCounterStrategy::set_measurer_type(MeasurerType type) {
    requested_measurer_type_ = type;
}

void BandwidthCounterStrategy::set_extra_counters(const std::vector<std::string>& counters) {
    extra_counters_ = counters;
}

void BandwidthCounterStrategy::set_memory_type(const std::string& mem_type) {
    cached_memory_type_ = mem_type;
}

void BandwidthCounterStrategy::initialize(int src_cpu, const std::vector<int>& target_mem_nodes, const CPUCapabilities& caps) {
    if (initialized_) return;

    cached_src_cpu_ = src_cpu;
    target_nodes_ = target_mem_nodes;

    is_hbm_ = (cached_memory_type_.find("HBM") != std::string::npos);

    discover_counters(src_cpu, target_mem_nodes);

    selection_.extra_counters = extra_counters_;

    resolve_measurer_type();

    auto arch = ArchitectureRegistry::instance().getArchitecture(caps);
    if (arch) {
        auto counter_strategy = arch->createCounterStrategy(caps);
        if (counter_strategy) {
            counter_strategy->getTlbMissCounters(cached_tlb1_raw_, cached_tlb2_raw_,
                                                  cached_use_tlb1_, cached_use_tlb2_);
        }
    }

    initialized_ = true;
}

void BandwidthCounterStrategy::resolve_measurer_type() {
    if (requested_measurer_type_ != MeasurerType::AUTO) {
        resolved_measurer_type_ = requested_measurer_type_;

        bool needs_cross_socket = needs_upi();
        if (requested_measurer_type_ == MeasurerType::PERF && is_hbm_ && !needs_cross_socket) {
            std::cerr << "Warning: Using perf on HBM system. Consider --measurer=likwid for better counter support." << std::endl;
        }
        if (requested_measurer_type_ == MeasurerType::LIKWID && cached_memory_type_ == "CXL") {
            std::cerr << "Warning: Using likwid on CXL system. Consider --measurer=pcm for CXL support." << std::endl;
        }
        return;
    }

    bool needs_cross_socket = needs_upi();

    if (is_hbm_ && needs_cross_socket && selection_.perf_available) {
        resolved_measurer_type_ = MeasurerType::PERF;
    } else if (is_hbm_) {
        resolved_measurer_type_ = MeasurerType::LIKWID;
    } else if (cached_memory_type_ == "CXL") {
        resolved_measurer_type_ = MeasurerType::PERF;
        std::cerr << "Warning: CXL detected but PCM backend is not enabled yet; using perf." << std::endl;
    } else {
        resolved_measurer_type_ = MeasurerType::PERF;
    }
}

void BandwidthCounterStrategy::discover_counters(int src_cpu, const std::vector<int>& target_mem_nodes) {
    selection_ = discoverBandwidthCounters(this, src_cpu, target_mem_nodes, requested_measurer_type_);
}

std::string BandwidthCounterStrategy::get_tool_name() const {
    switch (resolved_measurer_type_) {
        case MeasurerType::LIKWID:
            return is_hbm_ ? "LIKWID (HBM Mode)" : "LIKWID";
        case MeasurerType::PCM:
            return "PCM (CXL Mode)";
        case MeasurerType::PERF:
        default:
            if (is_hbm_ && needs_upi()) {
                return "perf (HBM with remote/cross-socket access)";
            } else if (selection_.type == CounterType::UPI_FLITS) {
                return "perf (UPI/Interconnect)";
            } else {
                return "perf (Standard)";
            }
    }
}

std::string BandwidthCounterStrategy::get_measurer_name() const {
    return measurer_type_to_string(resolved_measurer_type_);
}

void BandwidthCounterStrategy::get_tlb_counters(uint64_t& tlb1_raw, uint64_t& tlb2_raw, bool& use_tlb1, bool& use_tlb2) const {
    tlb1_raw = cached_tlb1_raw_;
    tlb2_raw = cached_tlb2_raw_;
    use_tlb1 = cached_use_tlb1_;
    use_tlb2 = cached_use_tlb2_;
}

void BandwidthCounterStrategy::print_counter_info(std::ostream& out) const {
    if (!initialized_) {
        out << "BandwidthCounterStrategy not initialized" << std::endl;
        return;
    }

    out << "Bandwidth Measurement Tool: " << get_tool_name() << std::endl;

    switch (resolved_measurer_type_) {
        case MeasurerType::LIKWID:
            out << "  LIKWID Counters: CAS_COUNT_RD/WR (via likwid-perfctr)" << std::endl;
            break;

        case MeasurerType::PCM:
            out << "  PCM Counters: (CXL support via Intel PCM)" << std::endl;
            break;

        case MeasurerType::PERF:
        default:
            if (selection_.type == CounterType::UPI_FLITS) {
                out << "UPI/Interconnect Counters (Remote Bandwidth):" << std::endl;
                if (selection_.perf_available) {
                    if (selection_.upi.has_rxl_txl) {
                        out << "  RXL Events: " << selection_.upi.get_rxl_events_string() << std::endl;
                        out << "  TXL Events: " << selection_.upi.get_txl_events_string() << std::endl;
                        if (selection_.upi.requires_link_aggregation) {
                            out << "  (Link aggregation required: Yes)" << std::endl;
                        }
                    } else {
                        out << "  No suitable UPI counters found." << std::endl;
                        if (!selection_.upi.failure_reason.empty()) {
                            out << "  Reason: " << selection_.upi.failure_reason << std::endl;
                        }
                    }
                } else {
                    out << "  perf not available." << std::endl;
                }
            } else {
                out << "CAS Counters (Local Bandwidth):" << std::endl;
                if (selection_.perf_available) {
                    if (selection_.cas.has_read_write) {
                        out << "  Read Events: " << selection_.cas.get_read_events_string() << std::endl;
                        out << "  Write Events: " << selection_.cas.get_write_events_string() << std::endl;
                        if (selection_.cas.requires_channel_aggregation) {
                            out << "  (Aggregation required: Yes)" << std::endl;
                        }
                    } else if (selection_.cas.has_combined_counter) {
                        out << "  Combined Events: " << selection_.cas.get_all_events_string() << std::endl;
                    } else {
                        out << "  No suitable CAS counters found." << std::endl;
                        if (!selection_.cas.failure_reason.empty()) {
                            out << "  Reason: " << selection_.cas.failure_reason << std::endl;
                        }
                    }
                } else {
                    out << "  perf not available." << std::endl;
                }
            }
            break;
    }
}

int BandwidthCounterStrategy::getNodeId(int cpu) {
    return ::getNodeId(cpu);
}

int BandwidthCounterStrategy::getNodeDistance(int src_node, int dst_node) {
    return ::getNodeDistance(src_node, dst_node);
}

bool BandwidthCounterStrategy::readPmuType(const std::string& pmu_path, uint32_t& type) {
    std::ifstream f(pmu_path + "/type");
    if (!f.is_open()) return false;
    f >> type;
    return f.good() || f.eof();
}

bool BandwidthCounterStrategy::readPmuCpumask(const std::string& pmu_path, int& cpu) {
    std::ifstream f(pmu_path + "/cpumask");
    if (!f.is_open()) return false;

    std::string line;
    if (!std::getline(f, line)) return false;

    size_t pos = line.find_first_of(",-");
    std::string first_cpu = (pos != std::string::npos) ? line.substr(0, pos) : line;

    try {
        cpu = std::stoi(first_cpu);
        return true;
    } catch (...) {
        return false;
    }
}

bool BandwidthCounterStrategy::queryEventViaPerf(const std::string& event_name, uint32_t& type, uint64_t& config) {
    std::string safe_event;
    safe_event.reserve(event_name.size());
    for (char c : event_name) {
        bool valid = std::isalnum(static_cast<unsigned char>(c)) ||
                     c == '_' || c == '-' || c == '/' || c == '=' ||
                     c == ',' || c == ':' || c == '.';
        if (!valid) return false;
        safe_event += c;
    }
    if (safe_event.empty() || safe_event.size() > 255) return false;

    std::string output = run_perf_capture({"stat", "-vvv", "-e", safe_event, "true"});

    if (output.find("perf_event_attr") == std::string::npos) {
        return false;
    }

    bool found_type = false, found_config = false;

    size_t type_pos = output.find("\n  type");
    if (type_pos != std::string::npos) {
        size_t line_end = output.find('\n', type_pos + 1);
        if (line_end != std::string::npos) {
            std::string type_line = output.substr(type_pos, line_end - type_pos);
            size_t val_start = type_line.find_first_of("0123456789");
            if (val_start != std::string::npos) {
                size_t val_end = val_start;
                while (val_end < type_line.length() &&
                       std::isdigit(static_cast<unsigned char>(type_line[val_end]))) val_end++;
                try {
                    type = static_cast<uint32_t>(std::stoul(type_line.substr(val_start, val_end - val_start)));
                    found_type = true;
                } catch (...) {}
            }
        }
    }

    size_t config_pos = output.find("\n  config ");
    if (config_pos != std::string::npos) {
        size_t line_end = output.find('\n', config_pos + 1);
        if (line_end != std::string::npos) {
            std::string config_line = output.substr(config_pos, line_end - config_pos);
            size_t hex_pos = config_line.find("0x");
            if (hex_pos != std::string::npos) {
                size_t val_end = hex_pos + 2;
                while (val_end < config_line.length() &&
                       std::isxdigit(static_cast<unsigned char>(config_line[val_end]))) val_end++;
                try {
                    config = std::stoull(config_line.substr(hex_pos, val_end - hex_pos), nullptr, 16);
                    found_config = true;
                } catch (...) {}
            } else {
                size_t val_start = config_line.find_first_of("0123456789");
                if (val_start != std::string::npos) {
                    size_t val_end = val_start;
                    while (val_end < config_line.length() &&
                           std::isdigit(static_cast<unsigned char>(config_line[val_end]))) val_end++;
                    try {
                        config = std::stoull(config_line.substr(val_start, val_end - val_start));
                        found_config = true;
                    } catch (...) {}
                }
            }
        }
    }

    return found_type && found_config;
}

ResolvedPerfEvent BandwidthCounterStrategy::resolveEvent(const std::string& event_name) {
    ResolvedPerfEvent resolved(event_name);

    if (event_name.size() > 1 && event_name[0] == 'r') {
        try {
            resolved.config = std::stoull(event_name.substr(1), nullptr, 16);
            resolved.type = 4; 
            resolved.cpu = -1;
            resolved.valid = true;
            return resolved;
        } catch (...) {
            // Not a raw event, continue with normal resolution
        }
    }

    size_t slash = event_name.find('/');
    if (slash != std::string::npos) {
        std::string pmu_name = event_name.substr(0, slash);
        std::string pmu_path = "/sys/bus/event_source/devices/" + pmu_name;

        if (!readPmuType(pmu_path, resolved.type)) {
            if (!queryEventViaPerf(event_name, resolved.type, resolved.config)) {
                return resolved; 
            }
            resolved.cpu = 0;
            resolved.valid = true;
            return resolved;
        }

        if (!readPmuCpumask(pmu_path, resolved.cpu)) {
            resolved.cpu = 0;
        }
    }

    if (!queryEventViaPerf(event_name, resolved.type, resolved.config)) {
        return resolved;
    }

    if (slash == std::string::npos) {
        const uint32_t PERF_TYPE_HARDWARE_MAX = 4;
        if (resolved.type > PERF_TYPE_HARDWARE_MAX) {
            DIR* dir = opendir("/sys/bus/event_source/devices");
            if (dir) {
                struct dirent* entry;
                while ((entry = readdir(dir)) != nullptr) {
                    std::string pmu_name = entry->d_name;
                    if (pmu_name[0] == '.') continue;

                    std::string pmu_path = "/sys/bus/event_source/devices/" + pmu_name;
                    uint32_t pmu_type;
                    if (readPmuType(pmu_path, pmu_type) && pmu_type == resolved.type) {
                        readPmuCpumask(pmu_path, resolved.cpu);
                        break;
                    }
                }
                closedir(dir);
            }
            if (resolved.cpu < 0) resolved.cpu = 0;
        } else {
            resolved.cpu = -1;
        }
    }

    resolved.valid = true;
    return resolved;
}

EventClassification EventClassification::classify(const std::string& event_name, const BandwidthCounterSelection& selection) {
    EventClassification result;

    auto match_any = [&](const std::vector<std::string>& list) -> bool {
        for (const auto& e : list) {
            if (contains_ci(event_name, e)) return true;
        }
        return false;
    };
    
    for (const auto& extra : selection.extra_counters) {
        if (contains_ci(event_name, extra)) {
            result.is_extra = true;
            result.extra_key = extra;
            return result;
        }
    }
    
    if (selection.type == CounterType::CAS_COUNT || selection.type == CounterType::NVIDIA_GRACE) {
        if (match_any(selection.cas.combined_events)) { result.is_combined = true; return result; }
        if (match_any(selection.cas.read_events)) { result.is_read = true; return result; }

        if (!result.is_read) {
            if (contains_ci(event_name, "cas_count") && 
               (contains_ci(event_name, "rd") || contains_ci(event_name, "read"))) {
                result.is_read = true;
            }
            if (contains_ci(event_name, "nvidia_scf_pmu") && 
                (contains_ci(event_name, "cmem_rd_data") || contains_ci(event_name, "remote_socket_rd_data"))) {
                result.is_read = true;
            }
        }
        
        if (!result.is_read) {
            if (match_any(selection.cas.write_events)) { result.is_write = true; return result; }
            if (!result.is_write) {
                if (contains_ci(event_name, "cas_count") && 
                   (contains_ci(event_name, "wr") || contains_ci(event_name, "write"))) {
                    result.is_write = true;
                }
                if (contains_ci(event_name, "nvidia_scf_pmu") && 
                    (contains_ci(event_name, "cmem_wr_total_bytes") || contains_ci(event_name, "remote_socket_wr_total_bytes"))) {
                    result.is_write = true;
                }
            }
        }
    } else if (selection.type == CounterType::UPI_FLITS) {
        if (match_any(selection.upi.rxl_data_events)) { result.is_read = true; return result; }
        if (match_any(selection.upi.txl_data_events)) { result.is_write = true; return result; }
    }
    
    return result;
}

void EventClassification::accumulate_extra(std::map<std::string, long long>& target,
                                           const std::string& extra_key, long long value) {
    if (!extra_key.empty()) {
        target[extra_key] += value;
    }
}

void EventClassification::reset_extra_values(std::map<std::string, long long>& target) {
    for (auto& kv : target) {
        kv.second = 0;
    }
}

std::string CasCounterSelection::get_read_events_string() const {
    return join_events(read_events);
}

std::string CasCounterSelection::get_write_events_string() const {
    return join_events(write_events);
}

std::string CasCounterSelection::get_all_events_string() const {
    std::vector<std::string> all;
    all.insert(all.end(), read_events.begin(), read_events.end());
    all.insert(all.end(), write_events.begin(), write_events.end());
    all.insert(all.end(), combined_events.begin(), combined_events.end());
    return join_events(all);
}

std::string CasCounterSelection::get_popen_events_string() const {
    std::vector<std::string> events;

    if (has_read_write) {
        if (!read_events.empty()) {
            events.push_back(read_events.front());
        }
        if (!write_events.empty()) {
            events.push_back(write_events.front());
        }
    } else if (has_combined_counter) {
        if (!combined_events.empty()) {
            events.push_back(combined_events.front());
        }
    }

    return join_events(events);
}

CasCounterSelection BandwidthCounterStrategy::discoverFromPerf() {
    CasCounterSelection selection;
    if (!run_command_success("which perf > /dev/null 2>&1")) {
        selection.failure_reason = "perf binary not found";
        return selection;
    }

    selection.perf_available = true;
    std::vector<std::string> detected_events = extractCasEventsFromPerf();
    if (detected_events.empty()) {
        selection.failure_reason = "perf list did not report CAS counters";
        return selection;
    }

    std::vector<std::string> read_candidates;
    std::vector<std::string> write_candidates;
    std::vector<std::string> combined_candidates;

    auto is_weird_specific = [](const std::string& name) {
        return contains_ci(name, "reg") || contains_ci(name, "underfill") ||
               contains_ci(name, "pre") || contains_ci(name, "nonpre");
    };

    for (const auto& event : detected_events) {
        if (event.empty()) continue;
        if (event.size() <= 1 && event.back() == '/') continue;

        bool is_combined = contains_ci(event, "all");
        bool is_read = contains_ci(event, "cas_count") && (contains_ci(event, "rd") || contains_ci(event, "read"));
        bool is_write = contains_ci(event, "cas_count") && (contains_ci(event, "wr") || contains_ci(event, "write"));

        if (contains_ci(event, "nvidia_scf_pmu")) {
            if (contains_ci(event, "cmem_rd_data") || contains_ci(event, "remote_socket_rd_data")) {
                is_read = true;
            } else if (contains_ci(event, "cmem_wr_total_bytes") || contains_ci(event, "remote_socket_wr_total_bytes")) {
                is_write = true;
            }
        }

        if (contains_ci(event, "dram_channel_data_controller")) {
            is_combined = true;
        }
        
        if (is_combined) {
            combined_candidates.push_back(event);
        } else if (is_read) {
            read_candidates.push_back(event);
        } else if (is_write) {
            write_candidates.push_back(event);
        }
    }

    auto is_channel_specific = [](const std::string& name) {
        if (contains_ci(name, "_sch") || contains_ci(name, "_imc") ||
            contains_ci(name, "/imc") || contains_ci(name, "uncore_imc") ||
            contains_ci(name, "dram_channel_data_controller")) return true;

        static const std::regex channel_regex(R"((ch|channel|unit|imc)[0-9]+)", std::regex_constants::icase);
        if (std::regex_search(name, channel_regex)) return true;

        return false;
    };

    auto select_best_counters = [&](const std::vector<std::string>& candidates) -> std::vector<std::string> {
        if (candidates.empty()) return {};

        std::vector<std::string> aggregates;
        std::vector<std::string> channels;
        std::vector<std::string> others;

        for (const auto& c : candidates) {
            if (is_weird_specific(c)) {
                others.push_back(c);
                continue;
            }

            if (is_channel_specific(c)) {
                channels.push_back(c);
            } else {
                aggregates.push_back(c);
            }
        }

        if (!channels.empty()) {
            std::vector<std::string> specific_channels;
            for (const auto& c : channels) {
                if (c.find("_sch") != std::string::npos || 
                    std::regex_search(c, std::regex(R"((ch|channel|unit|imc)[0-9]+)", std::regex_constants::icase))) {
                    specific_channels.push_back(c);
                }
            }
            
            if (!specific_channels.empty()) {
                return specific_channels;
            }
            return channels; 
        }

        if (!aggregates.empty()) {
            std::sort(aggregates.begin(), aggregates.end(), [](const std::string& a, const std::string& b) {
                return a.length() < b.length();
            });
            return {aggregates[0]}; 
        }

        if (!others.empty()) {
             std::sort(others.begin(), others.end(), [](const std::string& a, const std::string& b) {
                return a.length() < b.length();
            });
            return {others[0]};
        }

        return {};
    };

    selection.read_events = select_best_counters(read_candidates);
    selection.write_events = select_best_counters(write_candidates);

    if (selection.read_events.empty() || selection.write_events.empty()) {
        selection.combined_events = select_best_counters(combined_candidates);
    } else {
        selection.combined_events.clear();
    }

    selection.has_read_write = !selection.read_events.empty() && !selection.write_events.empty();
    selection.has_combined_counter = !selection.combined_events.empty();

    selection.requires_channel_aggregation = (selection.read_events.size() > 1) || (selection.write_events.size() > 1);

    if (!selection.has_read_write && !selection.has_combined_counter) {
        selection.failure_reason = "No usable CAS counters detected";
    }

    return selection;
}

CasCounterSelection BandwidthCounterStrategy::discoverFromPerf(
    const std::vector<std::string>& preferred_read,
    const std::vector<std::string>& preferred_write,
    const std::vector<std::string>& preferred_combined
) {
    auto result = discoverFromPerf();
    
    if (!preferred_read.empty() || !preferred_write.empty() || !preferred_combined.empty()) {
        std::vector<std::string> found_read;
        std::vector<std::string> found_write;
        std::vector<std::string> found_combined;
        
        for (const auto& event : preferred_read) {
            std::string encoding = lookupEventEncoding(event);
            if (!encoding.empty()) {
                found_read.push_back(encoding);
            }
        }
        
        for (const auto& event : preferred_write) {
            std::string encoding = lookupEventEncoding(event);
            if (!encoding.empty()) {
                found_write.push_back(encoding);
            }
        }
        
        for (const auto& event : preferred_combined) {
            std::string encoding = lookupEventEncoding(event);
            if (!encoding.empty()) {
                found_combined.push_back(encoding);
            }
        }
        
        if (!found_read.empty() && !found_write.empty()) {
            result.read_events = found_read;
            result.write_events = found_write;
            result.has_read_write = true;
            result.failure_reason = "";
        }
        if (!found_combined.empty()) {
            result.combined_events = found_combined;
            result.has_combined_counter = true;
            result.failure_reason = "";
        }
    }
    
    return result;
}

std::string BandwidthCounterStrategy::lookupEventEncoding(const std::string& event_name) {
    std::string cmd = "perf list -j 2>&1 | grep -A10 '\"EventName\": \"" + event_name + "\"'";
    FILE* pipe = popen(cmd.c_str(), "r");
    if (!pipe) {
        return "";
    }
    
    char buffer[1024];
    std::string output;
    while (fgets(buffer, sizeof(buffer), pipe)) {
        output += buffer;
    }
    pclose(pipe);
    
    size_t enc_pos = output.find("\"Encoding\"");
    if (enc_pos == std::string::npos) {
        return "";
    }
    
    size_t colon = output.find(':', enc_pos);
    if (colon == std::string::npos) return "";
    
    size_t quote1 = output.find('"', colon);
    size_t quote2 = output.find('"', quote1 + 1);
    if (quote1 == std::string::npos || quote2 == std::string::npos) return "";
    
    return output.substr(quote1 + 1, quote2 - quote1 - 1);
}

std::vector<std::string> BandwidthCounterStrategy::extractCasEventsFromPerf() {
    static std::vector<std::string> cache;
    static bool cached = false;
    
    if (cached) return cache;

    std::vector<std::string> perf_list_events;
    
    FILE* pipe = popen("perf list 2>/dev/null", "r"); 
    if (pipe) {
        char buffer[4096];
        while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
            std::string line(buffer);
            
            std::istringstream iss(line);
            std::string token;
            iss >> token;
            
            if (token.empty()) continue;

            if (token.find("unc_m_cas_count") != std::string::npos) {
                perf_list_events.push_back(token);
            } else if (token.find("cas_count") != std::string::npos) {
                perf_list_events.push_back(token);
            } else if (token.find("nvidia_scf_pmu") != std::string::npos) {
                perf_list_events.push_back(token);
            } else if (token.find("dram_channel_data_controller") != std::string::npos) {
                perf_list_events.push_back(token);
            }
        }
        pclose_success(pipe);
    }

    std::vector<std::string> unc_m_sch_events;
    std::vector<std::string> unc_m_other_events;
    std::vector<std::string> other_perf_events;
    for (const auto& evt : perf_list_events) {
        if (evt.find("unc_m_cas_count_sch") != std::string::npos) {
            unc_m_sch_events.push_back(evt);
        } else if (evt.find("unc_m_cas_count") != std::string::npos) {
            unc_m_other_events.push_back(evt);
        } else {
            other_perf_events.push_back(evt);
        }
    }
    
    if (!unc_m_sch_events.empty()) {
        cache = unc_m_sch_events;
        cached = true;
        return unc_m_sch_events;
    }
    
    if (!unc_m_other_events.empty()) {
        cache = unc_m_other_events;
        cached = true;
        return unc_m_other_events;
    }

    auto cas_filter = [](const std::string& name) {
        return name.find("cas_count") != std::string::npos &&
               name.find(".scale") == std::string::npos &&
               name.find(".unit") == std::string::npos;
    };

    std::vector<std::string> sysfs_events;
    std::set<std::string> added_events;

    DIR* pmu_dir = opendir("/sys/bus/event_source/devices");
    if (pmu_dir) {
        struct dirent* pmu_entry;
        while ((pmu_entry = readdir(pmu_dir)) != nullptr) {
            std::string pmu_name = pmu_entry->d_name;
            if (pmu_name.find("nvidia_scf_pmu") != std::string::npos) {
                std::string base_pmu_name = strip_pmu_instance_suffix(pmu_name);
                for (const auto& evt : scan_pmu_event_dir(pmu_name)) {
                    if (added_events.insert(evt).second) sysfs_events.push_back(evt);
                    size_t slash = evt.find('/');
                    std::string base_evt = base_pmu_name + evt.substr(slash);
                    if (added_events.insert(base_evt).second) sysfs_events.push_back(base_evt);
                }
            }
            if (pmu_name == "uncore_imc") {
                for (const auto& evt : scan_pmu_event_dir(pmu_name, cas_filter)) {
                    sysfs_events.push_back(evt);
                }
            }
        }
        closedir(pmu_dir);
    }
    
    if (!sysfs_events.empty()) {
        cache = sysfs_events;
        cached = true;
        return sysfs_events;
    }

    auto imc_instances = find_pmu_instances("uncore_imc");
    for (const auto& inst : imc_instances) {
        for (const auto& evt : scan_pmu_event_dir(inst, cas_filter)) {
            sysfs_events.push_back(evt);
        }
    }

    if (!sysfs_events.empty()) {
        cache = sysfs_events;
        cached = true;
        return sysfs_events;
    }
    
    if (!other_perf_events.empty()) {
        cache = other_perf_events;
        cached = true;
        return other_perf_events;
    }
    
    cache = perf_list_events;
    cached = true;
    return perf_list_events;
}
std::string UpiFlitSelection::get_rxl_events_string() const {
    return join_events(rxl_data_events);
}

std::string UpiFlitSelection::get_txl_events_string() const {
    return join_events(txl_data_events);
}

std::string UpiFlitSelection::get_all_events_string() const {
    std::vector<std::string> all;
    all.insert(all.end(), rxl_data_events.begin(), rxl_data_events.end());
    all.insert(all.end(), txl_data_events.begin(), txl_data_events.end());
    return join_events(all);
}

std::string BandwidthCounterSelection::get_bw_events_string() const {
    if (type == CounterType::CAS_COUNT || type == CounterType::NVIDIA_GRACE) {
        return cas.get_all_events_string();
    } else if (type == CounterType::UPI_FLITS) {
        return upi.get_all_events_string();
    }
    return {};
}

std::string BandwidthCounterSelection::get_all_events_string() const {
    std::string result = get_bw_events_string();
    
    for (const auto& counter : extra_counters) {
        if (!result.empty()) result += ",";
        result += counter;
    }
    
    return result;
}

std::string BandwidthCounterSelection::get_popen_events_string() const {
    std::string result;
    if (type == CounterType::CAS_COUNT || type == CounterType::NVIDIA_GRACE) {
        result = cas.get_popen_events_string();
    } else if (type == CounterType::UPI_FLITS) {
        result = upi.get_all_events_string();
    }

    for (const auto& counter : extra_counters) {
        if (!result.empty()) result += ",";
        result += counter;
    }

    return result;
}

bool BandwidthCounterSelection::is_valid() const {
    if (!perf_available) return false;
    if (type == CounterType::CAS_COUNT || type == CounterType::NVIDIA_GRACE) {
        return cas.has_read_write || cas.has_combined_counter;
    } else if (type == CounterType::UPI_FLITS) {
        return upi.has_rxl_txl;
    }
    return false;
}

std::vector<std::string> BandwidthCounterStrategy::extractUpiFlitEventsFromPerf() {
    static std::vector<std::string> cache;
    static bool cached = false;
    
    if (cached) return cache;
    
    std::vector<std::string> events;
    FILE* pipe = popen("perf list 2>/dev/null | grep -E '(flits|amd_df)'", "r");
    if (!pipe) {
        return events;
    }

    char buffer[4096];
    while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        std::string line(buffer);
        std::string token = trim_event_token(line);
        if (token.empty()) continue;

        std::string lowered = to_lower_copy(token);
        if ((lowered.find("upi") != std::string::npos && lowered.find("flits") != std::string::npos) ||
            (lowered.find("qpi") != std::string::npos && lowered.find("flits") != std::string::npos) ||
            (lowered.find("amd_df") != std::string::npos)) {
            events.push_back(token);
        }
    }

    pclose_success(pipe);
    
    cache = events;
    cached = true;
    return events;
}

UpiFlitSelection BandwidthCounterStrategy::discoverUpiFlitsFromPerf() {
    UpiFlitSelection selection;
    if (!run_command_success("which perf > /dev/null 2>&1")) {
        selection.failure_reason = "perf binary not found";
        return selection;
    }

    selection.perf_available = true;
    std::vector<std::string> detected_events = extractUpiFlitEventsFromPerf();
    if (detected_events.empty()) {
        selection.failure_reason = "perf list did not report interconnect flit/fabric counters";
        return selection;
    }

    std::vector<std::string> rxl_candidates;
    std::vector<std::string> txl_candidates;

    for (const auto& event : detected_events) {
        if (event.empty()) continue;
        std::string lowered = to_lower_copy(event);
        
        if (event.back() == '/') continue;

        bool is_intel_upi = lowered.find("upi") != std::string::npos || lowered.find("qpi") != std::string::npos;
        bool is_amd_df = lowered.find("amd_df") != std::string::npos;
        
        if (is_intel_upi) {
            bool is_all_data = lowered.find("all_data") != std::string::npos;
            bool is_rxl = lowered.find("rxl") != std::string::npos;
            bool is_txl = lowered.find("txl") != std::string::npos;

            if (is_all_data) {
                if (is_rxl) {
                    rxl_candidates.push_back(event);
                } else if (is_txl) {
                    txl_candidates.push_back(event);
                }
            }
        } else if (is_amd_df) {
            bool is_rx = lowered.find("rx") != std::string::npos ||
                         lowered.find("read") != std::string::npos ||
                         lowered.find("in") != std::string::npos;
            bool is_tx = lowered.find("tx") != std::string::npos || 
                         lowered.find("write") != std::string::npos ||
                         lowered.find("out") != std::string::npos;
            
            if (is_rx && !is_tx) {
                rxl_candidates.push_back(event);
            } else if (is_tx && !is_rx) {
                txl_candidates.push_back(event);
            }
        }
    }

    auto is_link_specific = [](const std::string& name) {
        static const std::regex link_regex(R"((upi|qpi|df)[0-9]+)", std::regex_constants::icase);
        if (std::regex_search(name, link_regex)) return true;
        if (contains_ci(name, "/upi") || contains_ci(name, "/amd_df")) return true;
        return false;
    };

    auto is_aggregate_event = [&is_link_specific](const std::string& name) -> bool {
        return contains_ci(name, "all_data") && !is_link_specific(name);
    };

    auto select_best_counters = [&](const std::vector<std::string>& candidates) -> std::vector<std::string> {
        if (candidates.empty()) return {};

        std::vector<std::string> aggregates;
        std::vector<std::string> links;

        for (const auto& c : candidates) {
            if (is_link_specific(c)) {
                links.push_back(c);
            } else {
                aggregates.push_back(c);
            }
        }

        for (const auto& agg : aggregates) {
            if (is_aggregate_event(agg)) {
                return {agg};
            }
        }

        if (!links.empty()) {
            return links;
        }

        if (!aggregates.empty()) {
            std::sort(aggregates.begin(), aggregates.end(), [](const std::string& a, const std::string& b) {
                return a.length() < b.length();
            });
            return {aggregates[0]};
        }

        return {};
    };

    selection.rxl_data_events = select_best_counters(rxl_candidates);
    selection.txl_data_events = select_best_counters(txl_candidates);

    selection.has_rxl_txl = !selection.rxl_data_events.empty() && !selection.txl_data_events.empty();
    selection.requires_link_aggregation = (selection.rxl_data_events.size() > 1) || (selection.txl_data_events.size() > 1);

    if (!selection.has_rxl_txl) {
        selection.failure_reason = "No usable UPI flit counters detected (need both RXL and TXL all_data)";
    }

    return selection;
}

int BandwidthCounterStrategy::getPhysicalPackageId(int cpu) {
    std::string path = "/sys/devices/system/cpu/cpu" + std::to_string(cpu) + "/topology/physical_package_id";
    std::ifstream f(path);
    int pkg = -1;
    if (f >> pkg) return pkg;
    return -1;
}

int BandwidthCounterStrategy::getNodeSocketId(int node_id) {
    std::string path = "/sys/devices/system/node/node" + std::to_string(node_id) + "/cpulist";
    std::ifstream f(path);
    if (!f) return -1;

    std::string cpulist;
    if (!std::getline(f, cpulist)) return -1;
    
    cpulist.erase(0, cpulist.find_first_not_of(" \t\r\n"));
    cpulist.erase(cpulist.find_last_not_of(" \t\r\n") + 1);
    
    if (cpulist.empty()) {
        return -1;
    }

    int first_cpu = -1;
    try {
        size_t pos = cpulist.find_first_of(",-");
        if (pos != std::string::npos) {
            first_cpu = std::stoi(cpulist.substr(0, pos));
        } else {
            first_cpu = std::stoi(cpulist);
        }
    } catch (const std::exception&) {
        return -1;
    }

    if (first_cpu < 0) return -1;
    return getPhysicalPackageId(first_cpu);
}

CounterType BandwidthCounterStrategy::detectCounterType(int src_cpu, int target_mem_node) {
    int src_socket = getPhysicalPackageId(src_cpu);
    int dst_socket = getNodeSocketId(target_mem_node);

    if (src_socket != -1 && dst_socket != -1) {
        if (src_socket == dst_socket) {
            return CounterType::CAS_COUNT;
        } else {
            return CounterType::UPI_FLITS;
        }
    }

    int src_node = getNodeId(src_cpu);
    if (src_node != -1) {
        int distance = getNodeDistance(src_node, target_mem_node);
        if (distance > 15) {
            return CounterType::UPI_FLITS;
        } else if (distance != -1) {
            return CounterType::CAS_COUNT;
        }
    }

    return CounterType::UNKNOWN;
}

BandwidthCounterSelection BandwidthCounterStrategy::discoverBandwidthCounters(BandwidthCounterStrategy* strategy, int src_cpu, const std::vector<int>& target_mem_nodes, MeasurerType measurer_type) {
    BandwidthCounterSelection selection;

    bool use_upi = false;
    for (int node : target_mem_nodes) {
        if (detectCounterType(src_cpu, node) == CounterType::UPI_FLITS) {
            use_upi = true;
            break;
        }
    }

    if (use_upi) {
        selection.type = CounterType::UPI_FLITS;
    } else {
        selection.type = CounterType::CAS_COUNT;
    }

    if (measurer_type == MeasurerType::LIKWID) {
        return selection;
    }

    if (!run_command_success("which perf > /dev/null 2>&1")) {
        selection.failure_reason = "perf binary not found";
        return selection;
    }
    selection.perf_available = true;

    CasCounterSelection cas_counters;
    
    if (strategy) {
        cas_counters = strategy->detectCasCounters();
    } else {
        cas_counters = discoverFromPerf();
    }
    
    bool cas_available = cas_counters.has_read_write || cas_counters.has_combined_counter;
    
    if (use_upi) {
        UpiFlitSelection upi_counters = discoverUpiFlitsFromPerf();
        bool upi_available = upi_counters.has_rxl_txl;
        
        if (upi_available) {
            selection.upi = upi_counters;
            return selection;
        }
        
        int target_mem_node = target_mem_nodes.empty() ? 0 : target_mem_nodes[0];
        std::string target_pmu = "nvidia_scf_pmu_" + std::to_string(target_mem_node);
        
        std::string target_pmu_path = "/sys/bus/event_source/devices/" + target_pmu;
        if (access((target_pmu_path + "/type").c_str(), F_OK) == 0) {
            CasCounterSelection grace_remote;
            grace_remote.perf_available = true;
            grace_remote.read_events = {target_pmu + "/cmem_rd_data/"};
            grace_remote.write_events = {target_pmu + "/cmem_wr_total_bytes/"};
            grace_remote.has_read_write = true;
            
            selection.type = CounterType::NVIDIA_GRACE;
            selection.cas = grace_remote;
            return selection;
        }
        
        selection.failure_reason = 
            "Cross-socket traffic detected but no UPI flit or Grace remote_socket counters found.\n"
            "    - The counters needed for your selected binding are not available via perf.\n"
            "    - Try adding --likwid flag to use an alternate profiling tool.\n"
            "    - If nothing works, please contact the maintainers to add support for this machine.";
        selection.upi = upi_counters;
        return selection;
    }

    selection.type = CounterType::CAS_COUNT;
    selection.cas = cas_counters;
    
    if (cas_available) {
        bool is_nvidia = false;
        if (!cas_counters.read_events.empty() && cas_counters.read_events[0].find("nvidia_scf_pmu") != std::string::npos) is_nvidia = true;
        if (!cas_counters.write_events.empty() && cas_counters.write_events[0].find("nvidia_scf_pmu") != std::string::npos) is_nvidia = true;
        
        if (is_nvidia) {
            selection.type = CounterType::NVIDIA_GRACE;
        }
    }

    if (!cas_available) {
        selection.failure_reason = cas_counters.failure_reason;
    }

    return selection;
}
