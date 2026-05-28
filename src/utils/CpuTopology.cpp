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

#include "utils/CpuTopology.h"

#include <cctype>
#include <cstring>
#include <fstream>
#include <map>
#include <utility>

#if defined(__linux__)
#include <dirent.h>
#include <unistd.h>
#endif

namespace cpu_topology {

namespace {

constexpr const char* kCpuRoot  = "/sys/devices/system/cpu";
constexpr const char* kNodeRoot = "/sys/devices/system/node";

void trim(std::string& s) {
    const size_t first = s.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) { s.clear(); return; }
    const size_t last = s.find_last_not_of(" \t\r\n");
    s = s.substr(first, last - first + 1);
}

int parse_first_cpu_in_list(const std::string& cpulist) {
    if (cpulist.empty()) return -1;
    const size_t pos = cpulist.find_first_of(",-");
    const std::string head = (pos != std::string::npos) ? cpulist.substr(0, pos) : cpulist;
    try {
        return std::stoi(head);
    } catch (...) {
        return -1;
    }
}

}

std::string cpu_path(int cpu) {
    return std::string(kCpuRoot) + "/cpu" + std::to_string(cpu);
}

std::string node_path(int node) {
    return std::string(kNodeRoot) + "/node" + std::to_string(node);
}

int numa_node_of(int cpu) {
#if defined(__linux__)
    static std::map<int, int> cache;
    auto it = cache.find(cpu);
    if (it != cache.end()) return it->second;

    DIR* dir = opendir(cpu_path(cpu).c_str());
    if (!dir) return -1;

    int node = -1;
    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        if (std::strncmp(entry->d_name, "node", 4) == 0 &&
            std::isdigit(static_cast<unsigned char>(entry->d_name[4]))) {
            try {
                node = std::stoi(&entry->d_name[4]);
            } catch (...) {}
            break;
        }
    }
    closedir(dir);

    if (node != -1) cache[cpu] = node;
    return node;
#else
    (void)cpu;
    return -1;
#endif
}

int physical_package_id(int cpu) {
    std::ifstream f(cpu_path(cpu) + "/topology/physical_package_id");
    int pkg = -1;
    if (f >> pkg) return pkg;
    return -1;
}

int distance(int src_node, int dst_node) {
    if (src_node == dst_node) return 10;

    static std::map<std::pair<int, int>, int> cache;
    const auto key = std::make_pair(src_node, dst_node);
    auto it = cache.find(key);
    if (it != cache.end()) return it->second;

    std::ifstream f(node_path(src_node) + "/distance");
    if (!f.is_open()) return -1;

    int result = -1;
    int val;
    int idx = 0;
    while (f >> val) {
        if (idx == dst_node) {
            result = val;
            break;
        }
        ++idx;
    }

    if (result != -1) cache[key] = result;
    return result;
}

int socket_of_node(int node) {
    std::ifstream f(node_path(node) + "/cpulist");
    if (!f) return -1;

    std::string cpulist;
    if (!std::getline(f, cpulist)) return -1;
    trim(cpulist);

    const int first_cpu = parse_first_cpu_in_list(cpulist);
    if (first_cpu < 0) return -1;
    return physical_package_id(first_cpu);
}

std::set<int> list_nodes(int max_index) {
    std::set<int> nodes;
#if defined(__linux__)
    for (int i = 0; i < max_index; ++i) {
        if (access(node_path(i).c_str(), F_OK) == 0) {
            nodes.insert(i);
        }
    }
#else
    (void)max_index;
#endif
    if (nodes.empty()) nodes.insert(0);
    return nodes;
}

std::set<int> nodes_for_cpus(const std::set<int>& cpus) {
    std::set<int> nodes;
#if defined(__linux__)
    for (int cpu : cpus) {
        const int n = numa_node_of(cpu);
        if (n >= 0) nodes.insert(n);
    }
#else
    (void)cpus;
#endif
    return nodes;
}

bool is_remote(int cpu, int mem_node) {
#if defined(__linux__)
    const std::string cpu_dir = cpu_path(cpu);

    const std::string direct = cpu_dir + "/node" + std::to_string(mem_node);
    if (access(direct.c_str(), F_OK) == 0) return false;

    for (int n = 0; n < 64; ++n) {
        const std::string link = cpu_dir + "/node" + std::to_string(n);
        if (access(link.c_str(), F_OK) == 0) {
            return n != mem_node;
        }
    }
#else
    (void)cpu;
    (void)mem_node;
#endif
    return false;
}

}
