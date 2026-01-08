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

#include "architecture/PerformanceCounterStrategy.h"
#include "utils.h"
#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <fstream>
#include <regex>
#include <set>
#include <sstream>
#include <map>


#include <dirent.h>

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

    FILE* pipe = popen("numactl -H", "r");
    if (!pipe) return -1;

    char buffer[4096];
    std::vector<int> column_nodes;
    bool header_found = false;
    int distance = -1;

    while (fgets(buffer, sizeof(buffer), pipe)) {
        std::string line(buffer);
        std::stringstream ss(line);
        std::string token;

        if (line.find("node distances:") != std::string::npos) {
            continue;
        }

        if (!header_found && line.find("node") != std::string::npos) {
            ss >> token; // "node"
            if (token != "node") continue;
            
            int node_idx;
            while (ss >> node_idx) {
                column_nodes.push_back(node_idx);
            }
            if (!column_nodes.empty()) header_found = true;
            continue;
        }

        if (header_found) {
            int row_node;
            ss >> token;
            if (token.back() == ':') token.pop_back();
            try {
                row_node = std::stoi(token);
            } catch (...) { continue; }

            if (row_node == src_node) {
                int dist_val;
                int col_idx = 0;
                while (ss >> dist_val) {
                    if (col_idx < static_cast<int>(column_nodes.size()) && column_nodes[col_idx] == dst_node) {
                        distance = dist_val;
                        break;
                    }
                    col_idx++;
                }
                break;
            }
        }
    }
    pclose_success(pipe);
    if (distance != -1) cache[{src_node, dst_node}] = distance;
    return distance;
}

}

std::string CasCounterSelection::get_read_events_string() const {
    std::string result;
    for (size_t i = 0; i < read_events.size(); ++i) {
        result += read_events[i];
        if (i < read_events.size() - 1) result += ",";
    }
    return result;
}

std::string CasCounterSelection::get_write_events_string() const {
    std::string result;
    for (size_t i = 0; i < write_events.size(); ++i) {
        result += write_events[i];
        if (i < write_events.size() - 1) result += ",";
    }
    return result;
}

std::string CasCounterSelection::get_all_events_string() const {
    std::string result;
    for (const auto& event : read_events) {
        if (!result.empty()) result += ",";
        result += event;
    }
    for (const auto& event : write_events) {
        if (!result.empty()) result += ",";
        result += event;
    }
    for (const auto& event : combined_events) {
        if (!result.empty()) result += ",";
        result += event;
    }
    return result;
}

CasCounterSelection PerformanceCounterStrategy::discoverFromPerf() {
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
        std::string lower = to_lower_copy(name);
        return lower.find("reg") != std::string::npos || 
               lower.find("underfill") != std::string::npos ||
               lower.find("pre") != std::string::npos ||
               lower.find("nonpre") != std::string::npos;
    };

    for (const auto& event : detected_events) {
        if (event.empty()) continue;
        std::string lowered = to_lower_copy(event);
        
        if (event.size() <= 1 && event.back() == '/') continue;

        bool is_combined = lowered.find("all") != std::string::npos;
        bool is_read = (lowered.find("cas_count") != std::string::npos && (lowered.find("rd") != std::string::npos || lowered.find("read") != std::string::npos));
        bool is_write = (lowered.find("cas_count") != std::string::npos && (lowered.find("wr") != std::string::npos || lowered.find("write") != std::string::npos));

        if (lowered.find("nvidia_scf_pmu") != std::string::npos) {
            if (lowered.find("cmem_rd_data") != std::string::npos) {
                is_read = true;
            } else if (lowered.find("cmem_wr_total_bytes") != std::string::npos) {
                is_write = true;
            } else if (lowered.find("remote_socket_rd_data") != std::string::npos) {
                is_read = true;
            } else if (lowered.find("remote_socket_wr_total_bytes") != std::string::npos) {
                is_write = true;
            }
        }

        if (lowered.find("dram_channel_data_controller") != std::string::npos) {
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
        std::string lower = to_lower_copy(name);
        
        if (lower.find("_sch") != std::string::npos) return true;
        if (lower.find("_imc") != std::string::npos) return true;
        if (lower.find("/imc") != std::string::npos) return true;
        if (lower.find("uncore_imc") != std::string::npos) return true;
        if (lower.find("dram_channel_data_controller") != std::string::npos) return true;

        static const std::regex channel_regex(R"((ch|channel|unit|imc)[0-9]+)", std::regex_constants::icase);
        if (std::regex_search(lower, channel_regex)) return true;

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

CasCounterSelection PerformanceCounterStrategy::discoverFromPerf(
    const std::vector<std::string>& preferred_read,
    const std::vector<std::string>& preferred_write,
    const std::vector<std::string>& preferred_combined
) {
    auto result = discoverFromPerf();
    
    if (!preferred_read.empty() || !preferred_write.empty() || !preferred_combined.empty()) {
        std::vector<std::string> all_perf_events = extractCasEventsFromPerf();
        
        auto event_exists = [&](const std::string& event) {
            return std::find(all_perf_events.begin(), all_perf_events.end(), event) != all_perf_events.end();
        };
        
        std::vector<std::string> found_read;
        std::vector<std::string> found_write;
        std::vector<std::string> found_combined;
        
        for (const auto& event : preferred_read) {
            if (event_exists(event)) {
                found_read.push_back(event);
            }
        }
        
        for (const auto& event : preferred_write) {
            if (event_exists(event)) {
                found_write.push_back(event);
            }
        }
        
        for (const auto& event : preferred_combined) {
            if (event_exists(event)) {
                found_combined.push_back(event);
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

std::vector<std::string> PerformanceCounterStrategy::extractCasEventsFromPerf() {
    static std::vector<std::string> cache;
    static bool cached = false;
    
    if (cached) return cache;

    std::vector<std::string> events;
    std::vector<std::string> perf_list_events;
    std::vector<std::string> sysfs_events;
    
    auto strip_pmu_instance_suffix = [](const std::string& name) -> std::string {
        size_t last_underscore = name.rfind('_');
        if (last_underscore != std::string::npos && last_underscore < name.length() - 1) {
            bool all_digits = true;
            for (size_t i = last_underscore + 1; i < name.length(); ++i) {
                if (!std::isdigit(name[i])) {
                    all_digits = false;
                    break;
                }
            }
            if (all_digits) {
                return name.substr(0, last_underscore);
            }
        }
        return name;
    };

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

    std::vector<std::string> unc_m_events;
    std::vector<std::string> other_perf_events;
    for (const auto& evt : perf_list_events) {
        if (evt.find("unc_m_cas_count") != std::string::npos) {
            unc_m_events.push_back(evt);
        } else {
            other_perf_events.push_back(evt);
        }
    }
    
    if (!unc_m_events.empty()) {
        cache = unc_m_events;
        cached = true;
        return unc_m_events;
    }
    
    if (!other_perf_events.empty()) {
        cache = other_perf_events;
        cached = true;
        return other_perf_events;
    }

    DIR* pmu_dir = opendir("/sys/bus/event_source/devices");
    if (pmu_dir) {
        struct dirent* pmu_entry;
        std::set<std::string> added_events;
        while ((pmu_entry = readdir(pmu_dir)) != nullptr) {
            std::string pmu_name = pmu_entry->d_name;
            if (pmu_name.find("nvidia_scf_pmu") != std::string::npos) {
                std::string base_pmu_name = strip_pmu_instance_suffix(pmu_name);
                std::string events_path = "/sys/bus/event_source/devices/" + pmu_name + "/events";
                DIR* events_dir = opendir(events_path.c_str());
                if (events_dir) {
                    struct dirent* event_entry;
                    while ((event_entry = readdir(events_dir)) != nullptr) {
                        std::string event_name = event_entry->d_name;
                        if (event_name == "." || event_name == "..") continue;
                        std::string full_event_with_suffix = pmu_name + "/" + event_name + "/";
                        if (added_events.find(full_event_with_suffix) == added_events.end()) {
                            sysfs_events.push_back(full_event_with_suffix);
                            added_events.insert(full_event_with_suffix);
                        }
                        std::string full_event_base = base_pmu_name + "/" + event_name + "/";
                        if (added_events.find(full_event_base) == added_events.end()) {
                            sysfs_events.push_back(full_event_base);
                            added_events.insert(full_event_base);
                        }
                    }
                    closedir(events_dir);
                }
            }
            if (pmu_name.find("uncore_imc") != std::string::npos) {
                std::string events_path = "/sys/bus/event_source/devices/" + pmu_name + "/events";
                DIR* events_dir = opendir(events_path.c_str());
                if (events_dir) {
                    struct dirent* event_entry;
                    while ((event_entry = readdir(events_dir)) != nullptr) {
                        std::string event_name = event_entry->d_name;
                        if (event_name == "." || event_name == "..") continue;
                        if (event_name.find("cas_count") != std::string::npos) {
                            std::string full_event = pmu_name + "/" + event_name + "/";
                            sysfs_events.push_back(full_event);
                        }
                    }
                    closedir(events_dir);
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
    
    cache = perf_list_events;
    cached = true;
    return perf_list_events;
}
std::string UpiFlitSelection::get_rxl_events_string() const {
    std::string result;
    for (size_t i = 0; i < rxl_data_events.size(); ++i) {
        result += rxl_data_events[i];
        if (i < rxl_data_events.size() - 1) result += ",";
    }
    return result;
}

std::string UpiFlitSelection::get_txl_events_string() const {
    std::string result;
    for (size_t i = 0; i < txl_data_events.size(); ++i) {
        result += txl_data_events[i];
        if (i < txl_data_events.size() - 1) result += ",";
    }
    return result;
}

std::string UpiFlitSelection::get_all_events_string() const {
    std::string result;
    for (const auto& event : rxl_data_events) {
        if (!result.empty()) result += ",";
        result += event;
    }
    for (const auto& event : txl_data_events) {
        if (!result.empty()) result += ",";
        result += event;
    }
    return result;
}

std::string BandwidthCounterSelection::get_all_events_string() const {
    std::string result;
    if (type == CounterType::CAS_COUNT || type == CounterType::NVIDIA_GRACE) {
        result = cas.get_all_events_string();
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

std::vector<std::string> PerformanceCounterStrategy::extractUpiFlitEventsFromPerf() {
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

UpiFlitSelection PerformanceCounterStrategy::discoverUpiFlitsFromPerf() {
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
        std::string lower = to_lower_copy(name);
        static const std::regex link_regex(R"((upi|qpi|df)[0-9]+)", std::regex_constants::icase);
        if (std::regex_search(lower, link_regex)) return true;
        if (lower.find("/upi") != std::string::npos) return true;
        if (lower.find("/amd_df") != std::string::npos) return true;
        return false;
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

        if (!aggregates.empty()) {
            std::sort(aggregates.begin(), aggregates.end(), [](const std::string& a, const std::string& b) {
                return a.length() < b.length();
            });
            return {aggregates[0]};
        }

        if (!links.empty()) {
            return links;
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

int PerformanceCounterStrategy::getPhysicalPackageId(int cpu) {
    std::string path = "/sys/devices/system/cpu/cpu" + std::to_string(cpu) + "/topology/physical_package_id";
    std::ifstream f(path);
    int pkg = -1;
    if (f >> pkg) return pkg;
    return -1;
}

int PerformanceCounterStrategy::getNodeSocketId(int node_id) {
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

CounterType PerformanceCounterStrategy::detectCounterType(int src_cpu, int target_mem_node) {
    int src_node = getNodeId(src_cpu);
    if (src_node != -1) {
        int distance = getNodeDistance(src_node, target_mem_node);
        if (distance > 15) {
            return CounterType::UPI_FLITS;
        } else if (distance != -1) {
            return CounterType::CAS_COUNT;
        }
    }

    int src_socket = getPhysicalPackageId(src_cpu);
    int dst_socket = getNodeSocketId(target_mem_node);

    if (src_socket == -1 || dst_socket == -1) {
        return CounterType::UNKNOWN;
    }

    if (src_socket == dst_socket) {
        return CounterType::CAS_COUNT;
    } else {
        return CounterType::UPI_FLITS;
    }
}

BandwidthCounterSelection PerformanceCounterStrategy::discoverBandwidthCounters(int src_cpu, const std::vector<int>& target_mem_nodes, bool likwid_enabled) {
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

    if (likwid_enabled) {
        return selection;
    }

    if (!run_command_success("which perf > /dev/null 2>&1")) {
        selection.failure_reason = "perf binary not found";
        return selection;
    }
    selection.perf_available = true;

    CasCounterSelection cas_counters = discoverFromPerf();
    bool cas_available = cas_counters.has_read_write || cas_counters.has_combined_counter;
    
    if (use_upi) {
        UpiFlitSelection upi_counters = discoverUpiFlitsFromPerf();
        bool upi_available = upi_counters.has_rxl_txl;
        
        if (upi_available) {
            selection.upi = upi_counters;
            return selection;
        }
        
        std::vector<std::string> all_events = extractCasEventsFromPerf();
        std::string remote_rd_event;
        std::string remote_wr_event;
        
        for (const auto& evt : all_events) {
            std::string lowered = to_lower_copy(evt);
            if (lowered.find("nvidia_scf_pmu") != std::string::npos) {
                if (lowered.find("remote_socket_rd_data") != std::string::npos) {
                    remote_rd_event = evt;
                } else if (lowered.find("remote_socket_wr_total_bytes") != std::string::npos) {
                    remote_wr_event = evt;
                }
            }
        }
        
        if (!remote_rd_event.empty() && !remote_wr_event.empty()) {
            CasCounterSelection grace_remote;
            grace_remote.perf_available = true;
            grace_remote.read_events = {remote_rd_event};
            grace_remote.write_events = {remote_wr_event};
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

    selection.cas = cas_counters;
    if (!cas_available) {
        selection.failure_reason = cas_counters.failure_reason;
    }

    return selection;
}
