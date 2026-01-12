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

#include "profiler/process_binding.h"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <filesystem>
#include <cstring>

#ifdef __linux__
#include <sched.h>
#include <unistd.h>
#include <sys/syscall.h>
#endif

std::string ProcessBinding::describe() const {
    std::stringstream ss;
    
    if (is_cpu_bound) {
        ss << "CPU bound to: " << format_cpu_set(allowed_cpus);
    } else {
        ss << "CPU: all";
    }
    
    ss << " | ";
    
    if (is_mem_bound) {
        ss << "Memory bound to nodes: " << format_node_set(allowed_mem_nodes);
    } else {
        ss << "Memory: all nodes";
    }
    
    if (!inferred_local_nodes.empty()) {
        ss << " | Local nodes: " << format_node_set(inferred_local_nodes);
    }
    
    if (has_remote_access) {
        ss << " | Remote access detected";
        if (!inferred_remote_nodes.empty()) {
            ss << " (nodes: " << format_node_set(inferred_remote_nodes) << ")";
        }
    }
    
    if (!binding_source.empty()) {
        ss << " [" << binding_source << "]";
    }
    
    return ss.str();
}

std::vector<int> ProcessBinding::get_target_mem_nodes() const {
    if (is_mem_bound && !allowed_mem_nodes.empty()) {
        return std::vector<int>(allowed_mem_nodes.begin(), allowed_mem_nodes.end());
    }
    if (!inferred_local_nodes.empty()) {
        return std::vector<int>(inferred_local_nodes.begin(), inferred_local_nodes.end());
    }
    return std::vector<int>{0};
}

int ProcessBinding::get_representative_cpu() const {
    if (!allowed_cpus.empty()) {
        return *allowed_cpus.begin();
    }
#ifdef __linux__
    unsigned cpu;
    if (syscall(SYS_getcpu, &cpu, nullptr, nullptr) == 0) {
        return static_cast<int>(cpu);
    }
#endif
    return 0;
}

ProcessBindingDetector::ProcessBindingDetector() {}

ProcessBinding ProcessBindingDetector::detect() {
    return detect_for_pid(getpid());
}

ProcessBinding ProcessBindingDetector::detect_for_pid(pid_t pid) {
    ProcessBinding binding;
    
    binding.allowed_cpus = get_cpu_affinity_mask_for_pid(pid);
    binding.allowed_mem_nodes = get_membind_nodes_for_pid(pid);
    
    std::set<int> all_nodes = get_all_system_nodes();
    
    if (binding.allowed_cpus.size() < 256) {
        binding.is_cpu_bound = true;
        binding.binding_source = "sched_getaffinity";
    }
    
    if (!binding.allowed_mem_nodes.empty() && binding.allowed_mem_nodes.size() < all_nodes.size()) {
        binding.is_mem_bound = true;
        if (binding.binding_source.empty()) {
            binding.binding_source = "numa_get_membind";
        } else {
            binding.binding_source += "+numa";
        }
    }
    
    binding.inferred_local_nodes = cpus_to_numa_nodes(binding.allowed_cpus);
    
    std::set<int> target_nodes = binding.is_mem_bound ? binding.allowed_mem_nodes : binding.inferred_local_nodes;
    
    for (int cpu : binding.allowed_cpus) {
        for (int node : target_nodes) {
            if (is_remote_access(cpu, node)) {
                binding.has_remote_access = true;
                binding.inferred_remote_nodes.insert(node);
            }
        }
    }
    
    return binding;
}

std::set<int> ProcessBindingDetector::get_cpu_affinity_mask() {
    return get_cpu_affinity_mask_for_pid(0);
}

std::set<int> ProcessBindingDetector::get_cpu_affinity_mask_for_pid(pid_t pid) {
    std::set<int> cpus;
    
#ifdef __linux__
    cpu_set_t mask;
    CPU_ZERO(&mask);
    
    if (sched_getaffinity(pid, sizeof(mask), &mask) == 0) {
        for (int i = 0; i < CPU_SETSIZE; ++i) {
            if (CPU_ISSET(i, &mask)) {
                cpus.insert(i);
            }
        }
    }
#endif
    
    return cpus;
}

std::set<int> ProcessBindingDetector::get_membind_nodes() {
    return get_membind_nodes_for_pid(getpid());
}

std::set<int> ProcessBindingDetector::get_membind_nodes_for_pid(pid_t pid) {
    std::set<int> nodes;
    
#ifdef __linux__
    std::string numa_maps_path = "/proc/" + std::to_string(pid) + "/numa_maps";
    if (pid == 0 || pid == getpid()) {
        numa_maps_path = "/proc/self/numa_maps";
    }
    
    std::string status_path = "/proc/" + std::to_string(pid == 0 ? getpid() : pid) + "/status";
    std::ifstream status(status_path);
    if (status) {
        std::string line;
        while (std::getline(status, line)) {
            if (line.find("Mems_allowed:") == 0) {
                std::string mask_str = line.substr(line.find(':') + 1);
                mask_str.erase(0, mask_str.find_first_not_of(" \t"));
                nodes = parse_node_mask(mask_str);
                break;
            }
        }
    }
#endif
    
    return nodes;
}

std::set<int> ProcessBindingDetector::cpus_to_numa_nodes(const std::set<int>& cpus) {
    std::set<int> nodes;
    
#ifdef __linux__
    for (int cpu : cpus) {
        std::string cpu_path = "/sys/devices/system/cpu/cpu" + std::to_string(cpu);
        if (!std::filesystem::exists(cpu_path)) continue;
        
        for (int node = 0; node < 64; ++node) {
            std::string node_path = cpu_path + "/node" + std::to_string(node);
            if (std::filesystem::exists(node_path)) {
                nodes.insert(node);
                break;
            }
        }
    }
#endif
    
    return nodes;
}

std::set<int> ProcessBindingDetector::get_all_system_nodes() {
    std::set<int> nodes;
    
#ifdef __linux__
    for (int i = 0; i < 64; ++i) {
        std::string path = "/sys/devices/system/node/node" + std::to_string(i);
        if (std::filesystem::exists(path)) {
            nodes.insert(i);
        }
    }
#endif
    
    if (nodes.empty()) {
        nodes.insert(0);
    }
    
    return nodes;
}

bool ProcessBindingDetector::is_remote_access(int cpu, int mem_node) {
#ifdef __linux__
    std::string cpu_path = "/sys/devices/system/cpu/cpu" + std::to_string(cpu);
    std::string node_link = cpu_path + "/node" + std::to_string(mem_node);
    
    if (std::filesystem::exists(node_link)) {
        return false;
    }
    
    for (int node = 0; node < 64; ++node) {
        std::string local_link = cpu_path + "/node" + std::to_string(node);
        if (std::filesystem::exists(local_link)) {
            if (node != mem_node) {
                return true;
            }
            break;
        }
    }
#endif
    return false;
}

std::set<int> ProcessBindingDetector::parse_cpu_list(const std::string& list_str) {
    std::set<int> cpus;
    std::stringstream ss(list_str);
    std::string token;
    
    while (std::getline(ss, token, ',')) {
        size_t dash = token.find('-');
        if (dash != std::string::npos) {
            int start = std::stoi(token.substr(0, dash));
            int end = std::stoi(token.substr(dash + 1));
            for (int i = start; i <= end; ++i) {
                cpus.insert(i);
            }
        } else {
            cpus.insert(std::stoi(token));
        }
    }
    
    return cpus;
}

std::set<int> ProcessBindingDetector::parse_node_mask(const std::string& mask_str) {
    std::set<int> nodes;
    
    std::string cleaned;
    for (char c : mask_str) {
        if (c != ',' && c != ' ' && c != '\t' && c != '\n') {
            cleaned += c;
        }
    }
    
    int total_bits = cleaned.size() * 4;
    int node = 0;
    
    for (int i = static_cast<int>(cleaned.size()) - 1; i >= 0; --i) {
        char c = cleaned[i];
        int val = 0;
        if (c >= '0' && c <= '9') val = c - '0';
        else if (c >= 'a' && c <= 'f') val = 10 + (c - 'a');
        else if (c >= 'A' && c <= 'F') val = 10 + (c - 'A');
        
        for (int bit = 0; bit < 4; ++bit) {
            if (val & (1 << bit)) {
                nodes.insert(node);
            }
            node++;
        }
    }
    
    return nodes;
}

std::string format_cpu_set(const std::set<int>& cpus) {
    if (cpus.empty()) return "none";
    if (cpus.size() > 16) {
        std::stringstream ss;
        auto it = cpus.begin();
        ss << *it;
        ++it;
        int count = 1;
        while (it != cpus.end() && count < 3) {
            ss << "," << *it;
            ++it;
            ++count;
        }
        ss << "...(" << cpus.size() << " total)";
        return ss.str();
    }
    
    std::vector<int> sorted(cpus.begin(), cpus.end());
    std::stringstream ss;
    
    size_t i = 0;
    while (i < sorted.size()) {
        int start = sorted[i];
        int end = start;
        
        while (i + 1 < sorted.size() && sorted[i + 1] == end + 1) {
            ++i;
            end = sorted[i];
        }
        
        if (ss.tellp() > 0) ss << ",";
        
        if (end > start + 1) {
            ss << start << "-" << end;
        } else if (end == start + 1) {
            ss << start << "," << end;
        } else {
            ss << start;
        }
        
        ++i;
    }
    
    return ss.str();
}

std::string format_node_set(const std::set<int>& nodes) {
    if (nodes.empty()) return "none";
    
    std::stringstream ss;
    bool first = true;
    for (int n : nodes) {
        if (!first) ss << ",";
        ss << n;
        first = false;
    }
    return ss.str();
}
