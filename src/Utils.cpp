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

#include "Utils.h"
#include "utils/CpuTopology.h"
#include "utils/SubprocessCapture.h"
#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>
#include <unistd.h>
#include <sys/file.h>
#include <fcntl.h>
#include <sys/syscall.h>
#include <unistd.h>
#ifdef __linux__
#include <sched.h>
#endif
#include <algorithm>
#include <cmath>
#include <cctype>
#include <mutex>
#include <set>
#include <vector>

namespace {

#ifdef __linux__
std::string str_tolower(const std::string& s) {
    std::string out = s;
    std::transform(out.begin(), out.end(), out.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return out;
}
#endif

bool command_exists_in_path(const std::string& command) {
    if (command.empty()) {
        return false;
    }

    const char* path_env = std::getenv("PATH");
    if (!path_env) {
        return false;
    }

    std::stringstream path_stream(path_env);
    std::string path_entry;
    while (std::getline(path_stream, path_entry, ':')) {
        if (path_entry.empty()) {
            path_entry = ".";
        }
        std::filesystem::path candidate = std::filesystem::path(path_entry) / command;
        if (access(candidate.c_str(), X_OK) == 0) {
            return true;
        }
    }

    return false;
}

bool read_int_file(const std::filesystem::path& file_path, int& value) {
    std::ifstream file(file_path);
    if (!file.is_open()) {
        return false;
    }
    file >> value;
    return file.good() || file.eof();
}

#ifdef __linux__
std::vector<int> parse_cpu_range_list(const std::string& cpu_list) {
    std::vector<int> cpus;
    std::istringstream iss(cpu_list);
    std::string token;
    while (std::getline(iss, token, ',')) {
        if (token.empty()) {
            continue;
        }

        int lo = -1;
        int hi = -1;
        const size_t dash = token.find('-');
        try {
            if (dash == std::string::npos) {
                lo = hi = std::stoi(token);
            } else {
                lo = std::stoi(token.substr(0, dash));
                hi = std::stoi(token.substr(dash + 1));
            }
        } catch (...) {
            continue;
        }

        if (lo > hi) {
            std::swap(lo, hi);
        }
        for (int cpu = lo; cpu <= hi; ++cpu) {
            cpus.push_back(cpu);
        }
    }
    return cpus;
}
#endif

void upsert_plotter_key(std::string& content, const std::string& key, const std::string& value) {
    const std::string prefix = key + "=";
    const std::string replacement = prefix + value;

    size_t line_start = 0;
    while (line_start < content.size()) {
        size_t line_end = content.find('\n', line_start);
        const size_t compare_len = (line_end == std::string::npos) ? (content.size() - line_start)
                                                                   : (line_end - line_start);
        if (content.compare(line_start, prefix.size(), prefix) == 0) {
            content.replace(line_start, compare_len, replacement);
            return;
        }
        if (line_end == std::string::npos) {
            break;
        }
        line_start = line_end + 1;
    }

    if (!content.empty() && content.back() != '\n') {
        content += '\n';
    }
    content += replacement;
    content += '\n';
}

std::map<int, CpuTopology> get_cpu_topology_from_sysfs() {
    std::map<int, CpuTopology> cpu_topo;
    std::error_code ec;
    const std::filesystem::path cpu_root("/sys/devices/system/cpu");
    for (const auto& entry : std::filesystem::directory_iterator(cpu_root, ec)) {
        if (ec) {
            break;
        }
        if (!entry.is_directory(ec)) {
            continue;
        }

        const std::string name = entry.path().filename().string();
        if (name.rfind("cpu", 0) != 0 || name.size() <= 3) {
            continue;
        }

        bool numeric_suffix = true;
        for (size_t i = 3; i < name.size(); ++i) {
            if (!std::isdigit(static_cast<unsigned char>(name[i]))) {
                numeric_suffix = false;
                break;
            }
        }
        if (!numeric_suffix) {
            continue;
        }

        int cpu = 0;
        try {
            cpu = std::stoi(name.substr(3));
        } catch (...) {
            continue;
        }

        int socket = 0;
        read_int_file(entry.path() / "topology" / "physical_package_id", socket);
        int core_id = cpu;
        read_int_file(entry.path() / "topology" / "core_id", core_id);

        int node = -1;
        std::error_code node_ec;
        for (const auto& node_entry : std::filesystem::directory_iterator(entry.path(), node_ec)) {
            if (node_ec) {
                break;
            }
            const std::string node_name = node_entry.path().filename().string();
            if (node_name.rfind("node", 0) != 0 || node_name.size() <= 4) {
                continue;
            }

            bool node_numeric = true;
            for (size_t i = 4; i < node_name.size(); ++i) {
                if (!std::isdigit(static_cast<unsigned char>(node_name[i]))) {
                    node_numeric = false;
                    break;
                }
            }
            if (!node_numeric) {
                continue;
            }

            try {
                node = std::stoi(node_name.substr(4));
            } catch (...) {
                node = -1;
            }
            if (node >= 0) {
                break;
            }
        }

        cpu_topo[cpu] = {node >= 0 ? node : 0, socket, core_id};
    }

    return cpu_topo;
}

}

int get_numa_node_of_cpu(int cpu) {
    // Bug fix as part of unification: the legacy loop here capped at 16 nodes,
    // silently returning 0 on systems with NUMA node IDs >= 16 (large EPYC,
    // Grace SuperPOD). cpu_topology::numa_node_of uses a readdir-based scan
    // with no upper bound. Falls back to 0 to preserve the original return
    // contract for callers that treat -1 as a node id.
    const int node = cpu_topology::numa_node_of(cpu);
    return node >= 0 ? node : 0;
}

std::filesystem::path get_project_root() {
    std::error_code ec;
    std::filesystem::path exe_path = std::filesystem::canonical("/proc/self/exe", ec);
    if (ec) {
        return std::filesystem::current_path();
    }
    
    std::filesystem::path p = exe_path.parent_path(); 
    
    if (p.filename() == "bin") {
        p = p.parent_path(); 
    }
    
    if (p.filename() == "build") {
        p = p.parent_path(); 
    }
    
    return p;
}

void append_memory_config_to_plotter(const std::filesystem::path& plotter_path, const CPUCapabilities& caps, const std::string& binding_type, double tlb_ns, int cache_line_size, double upi_scaling_factor, double page_walk_lat_ns) {
    if (plotter_path.has_parent_path()) {
        std::filesystem::create_directories(plotter_path.parent_path());
    }
    
    int fd = open(plotter_path.c_str(), O_RDWR | O_CREAT, 0644);
    if (fd == -1) return;

    if (flock(fd, LOCK_EX) == -1) {
        close(fd);
        return;
    }

    std::string content;
    {
        lseek(fd, 0, SEEK_SET);
        char buffer[4096];
        ssize_t bytes_read;
        while ((bytes_read = read(fd, buffer, sizeof(buffer))) > 0) {
            content.append(buffer, bytes_read);
        }
    }

    upsert_plotter_key(content, "TLB_NS", std::to_string(tlb_ns));
    if (page_walk_lat_ns > 0.0) {
        upsert_plotter_key(content, "PAGE_WALK_LAT_NS", std::to_string(page_walk_lat_ns));
    }
    upsert_plotter_key(content, "CACHE_LINE_SIZE", std::to_string(cache_line_size));
    upsert_plotter_key(content, "MEMORY_BINDING", binding_type);

    if (binding_type == "remote") {
        if (caps.upi_freq > 0) {
            upsert_plotter_key(content, "UPI_FREQ", std::to_string(caps.upi_freq));
            upsert_plotter_key(content, "N_DATA_LANES", std::to_string(caps.n_data_lanes));
            upsert_plotter_key(content, "FLIT_BIT", std::to_string(caps.flit_bit));
            upsert_plotter_key(content, "DATA_FLIT_BIT", std::to_string(caps.data_flit_bit));
            upsert_plotter_key(content, "N_UPI_CHANNELS", std::to_string(caps.n_upi_channels));
            if (upi_scaling_factor > 0) {
                upsert_plotter_key(content, "UPI_SCALING_FACTOR", std::to_string(upi_scaling_factor));
            }
        } else if (caps.nvlink_bw_gb_s > 0) {
            upsert_plotter_key(content, "NVLINK_BW_GB_S", std::to_string(caps.nvlink_bw_gb_s));
        }
    } else {
        upsert_plotter_key(content, "MEM_FREQ", caps.memory_frequency);
        upsert_plotter_key(content, "N_CHANNELS", std::to_string(caps.memory_channels));
        upsert_plotter_key(content, "BUS_WIDTH", std::to_string(caps.bus_width));
    }

    lseek(fd, 0, SEEK_SET);
    if (ftruncate(fd, 0) == 0 && !content.empty()) {
        write(fd, content.c_str(), content.length());
    }

    flock(fd, LOCK_UN);
    close(fd);
}

std::map<int, CpuTopology> get_cpu_topology() {
    std::map<int, CpuTopology> cpu_topo = get_cpu_topology_from_sysfs();
    if (!cpu_topo.empty()) {
        return cpu_topo;
    }

    cpu_topo.clear();
    // lscpu on high-core-count systems (256+ cores) can produce >64 KiB —
    // raise the cap so the whole table is captured.
    subprocess::Options opts;
    opts.timeout          = std::chrono::seconds(10);
    opts.max_output_bytes = 262144;
    const std::string output = subprocess::run_shell(
        "lscpu -p=cpu,node,socket,core 2>/dev/null", opts);
    if (output.empty()) {
        return cpu_topo;
    }

    std::istringstream stream(output);
    std::string line;
    while (std::getline(stream, line)) {
        if (line.empty() || line[0] == '#') continue;

        size_t first_comma  = line.find(',');
        size_t second_comma = line.find(',', first_comma + 1);
        size_t third_comma  = line.find(',', second_comma + 1);

        if (first_comma != std::string::npos && second_comma != std::string::npos) {
            try {
                int cpu = std::stoi(line.substr(0, first_comma));
                int node = std::stoi(line.substr(first_comma + 1, second_comma - first_comma - 1));
                int socket = 0;
                int core_id = cpu;
                if (third_comma != std::string::npos) {
                    socket = std::stoi(line.substr(second_comma + 1, third_comma - second_comma - 1));
                    core_id = std::stoi(line.substr(third_comma + 1));
                } else {
                    socket = std::stoi(line.substr(second_comma + 1));
                }
                cpu_topo[cpu] = {node, socket, core_id};
            } catch (...) {}
        }
    }
    return cpu_topo;
}

bool numa_node_has_memory(int node) {
#ifdef __linux__
    if (node < 0) {
        return false;
    }

    std::string path = "/sys/devices/system/node/node" + std::to_string(node) + "/meminfo";
    std::ifstream f(path);
    if (!f.is_open()) {
        return false;
    }

    std::string line;
    while (std::getline(f, line)) {
        if (line.find("MemTotal") == std::string::npos) {
            continue;
        }
        const size_t pos = line.find_first_of("0123456789");
        if (pos == std::string::npos) {
            continue;
        }
        long val = 0;
        try {
            val = std::stol(line.substr(pos));
        } catch (...) {
            val = 0;
        }
        return val > 0;
    }
#else
    (void)node;
#endif
    return false;
}

std::vector<int> get_allowed_cpus() {
    std::vector<int> cpus;
#ifdef __linux__
    cpu_set_t mask;
    CPU_ZERO(&mask);
    if (sched_getaffinity(0, sizeof(mask), &mask) == 0) {
        for (int cpu = 0; cpu < CPU_SETSIZE; ++cpu) {
            if (CPU_ISSET(cpu, &mask)) {
                cpus.push_back(cpu);
            }
        }
    }

    if (cpus.empty()) {
        std::ifstream online("/sys/devices/system/cpu/online");
        if (online.is_open()) {
            std::string range_str;
            std::getline(online, range_str);
            cpus = parse_cpu_range_list(range_str);
        }
    }
#endif

    if (cpus.empty()) {
        const std::map<int, CpuTopology> cpu_topo = get_cpu_topology();
        cpus.reserve(cpu_topo.size());
        for (const auto& pair : cpu_topo) {
            cpus.push_back(pair.first);
        }
    }

    std::sort(cpus.begin(), cpus.end());
    cpus.erase(std::unique(cpus.begin(), cpus.end()), cpus.end());
    return cpus;
}

std::vector<int> select_worker_cpus(int requested_count, int reserved_cpu) {
    std::vector<int> selected;
    if (requested_count <= 0) {
        return selected;
    }

    const std::map<int, CpuTopology> cpu_topo = get_cpu_topology();

    std::set<int> socket0_cores;
    for (const auto& [cpu, topo] : cpu_topo) {
        if (topo.socket == 0) socket0_cores.insert(topo.core_id);
    }
    if (requested_count >= static_cast<int>(socket0_cores.size())) {
        reserved_cpu = -1;
    }

    if (!is_a64fx_system()) {
        std::set<long long> used_cores;
        for (const auto& [cpu, topo] : cpu_topo) {
            if (static_cast<int>(selected.size()) >= requested_count) {
                break;
            }
            if (cpu == reserved_cpu || topo.socket != 0) {
                continue;
            }

            const long long core_key =
                (static_cast<long long>(topo.socket) << 32) | static_cast<unsigned int>(topo.core_id);
            if (!used_cores.insert(core_key).second) {
                continue;
            }
            selected.push_back(cpu);
        }

        if (!selected.empty()) {
            return selected;
        }

        for (int cpu = 0; static_cast<int>(selected.size()) < requested_count; ++cpu) {
            if (cpu == reserved_cpu) {
                continue;
            }
            selected.push_back(cpu);
        }
        return selected;
    }

    std::vector<int> allowed_cpus = get_allowed_cpus();
    if (allowed_cpus.empty()) {
        for (int cpu = 0; static_cast<int>(selected.size()) < requested_count; ++cpu) {
            if (cpu == reserved_cpu) {
                continue;
            }
            selected.push_back(cpu);
        }
        return selected;
    }

    int preferred_socket = 0;
    if (cpu_topo.count(reserved_cpu)) {
        preferred_socket = cpu_topo.at(reserved_cpu).socket;
    } else if (!allowed_cpus.empty() && cpu_topo.count(allowed_cpus.front())) {
        preferred_socket = cpu_topo.at(allowed_cpus.front()).socket;
    }

    std::set<int> a64fx_skip_cpus;
    if (is_a64fx_system()) {
        std::map<int, int> first_cpu_per_memory_node;
        for (int cpu : allowed_cpus) {
            auto it = cpu_topo.find(cpu);
            if (it == cpu_topo.end() || !numa_node_has_memory(it->second.node)) {
                continue;
            }
            auto [node_it, inserted] = first_cpu_per_memory_node.emplace(it->second.node, cpu);
            if (!inserted && cpu < node_it->second) {
                node_it->second = cpu;
            }
        }
        for (const auto& kv : first_cpu_per_memory_node) {
            a64fx_skip_cpus.insert(kv.second);
        }
        a64fx_skip_cpus.erase(reserved_cpu);
    }

    std::set<int> used_cpus;
    std::set<long long> used_cores;
    auto append_candidates = [&](bool preferred_only, bool require_memory) {
        for (int cpu : allowed_cpus) {
            if (static_cast<int>(selected.size()) >= requested_count) {
                return;
            }
            if (cpu == reserved_cpu || used_cpus.count(cpu)) {
                continue;
            }
            if (a64fx_skip_cpus.count(cpu)) {
                continue;
            }

            auto it = cpu_topo.find(cpu);
            if (preferred_only) {
                if (it == cpu_topo.end()) {
                    if (!cpu_topo.empty()) {
                        continue;
                    }
                } else if (it->second.socket != preferred_socket) {
                    continue;
                }
            }

            if (require_memory && it != cpu_topo.end() && !numa_node_has_memory(it->second.node)) {
                continue;
            }

            long long core_key = static_cast<long long>(cpu);
            if (it != cpu_topo.end()) {
                core_key = (static_cast<long long>(it->second.socket) << 32) |
                           static_cast<unsigned int>(it->second.core_id);
            }
            if (!used_cores.insert(core_key).second) {
                continue;
            }

            used_cpus.insert(cpu);
            selected.push_back(cpu);
        }
    };

    append_candidates(true, true);
    append_candidates(false, true);
    append_candidates(true, false);
    append_candidates(false, false);
    return selected;
}

bool is_a64fx_system() {
#ifdef __linux__
    std::ifstream cpuinfo("/proc/cpuinfo");
    if (!cpuinfo.is_open()) {
        return false;
    }

    std::string line;
    std::string implementer;
    std::string part;
    while (std::getline(cpuinfo, line)) {
        if (line.find("A64FX") != std::string::npos) {
            return true;
        }

        std::string lower = str_tolower(line);
        if (lower.rfind("cpu implementer", 0) == 0) {
            const size_t colon = line.find(':');
            if (colon != std::string::npos) {
                implementer = str_tolower(line.substr(colon + 1));
            }
        } else if (lower.rfind("cpu part", 0) == 0) {
            const size_t colon = line.find(':');
            if (colon != std::string::npos) {
                part = str_tolower(line.substr(colon + 1));
            }
        }
    }

    auto trim = [](std::string s) {
        const size_t first = s.find_first_not_of(" \t");
        if (first == std::string::npos) {
            return std::string();
        }
        const size_t last = s.find_last_not_of(" \t");
        return s.substr(first, last - first + 1);
    };

    implementer = trim(implementer);
    part = trim(part);
    return implementer == "0x46" && part == "0x001";
#else
    return false;
#endif
}

void ensure_a64fx_pmu_available() {
#ifdef __aarch64__
    static bool checked = false;
    if (checked) return;
    if (!is_a64fx_system()) {
        checked = true;
        return;
    }
    // On Fugaku, the OS Performance Agent (xospastop) occupies
    // the PMU counters. Calling xospastop releases them for user perf.
    // This is a no-op if the agent is already stopped.
    (void)system("xospastop >/dev/null 2>&1");
    checked = true;
#endif
}

int get_default_traffic_gen_cores(const system_info& info) {
    int cores = (info.socket_count > 0) ? (info.sockets[0].core_count - 1) : 0;
    if (cores <= 0) {
        return cores;
    }

    if (!is_a64fx_system()) {
        return cores;
    }

    const auto& cache = SystemToolsCache::instance();
    const int reserved_cpu = cache.initialized ? cache.ptr_chase_cpu : find_ptr_chase_cpu().first;
    const int requested = static_cast<int>(get_allowed_cpus().size());
    const auto selected = select_worker_cpus(requested, reserved_cpu);
    return static_cast<int>(selected.size());
}

std::map<int, std::set<int>> get_socket_to_nodes_map(const std::map<int, CpuTopology>& topo) {
    std::map<int, std::set<int>> socket_nodes;
    for (const auto& pair : topo) {
        socket_nodes[pair.second.socket].insert(pair.second.node);
    }
    return socket_nodes;
}

namespace {

bool read_long_long_file(const std::filesystem::path& path, long long& value) {
    std::ifstream file(path);
    if (!file.is_open()) {
        return false;
    }
    file >> value;
    return file.good() || file.eof();
}

bool read_string_file(const std::filesystem::path& path, std::string& out) {
    std::ifstream file(path);
    if (!file.is_open()) {
        return false;
    }
    std::getline(file, out);
    // Trim trailing whitespace.
    while (!out.empty() && std::isspace(static_cast<unsigned char>(out.back()))) {
        out.pop_back();
    }
    return true;
}

bool node_has_any_cpu(int node_id) {
    std::string cpulist;
    if (!read_string_file("/sys/devices/system/node/node" + std::to_string(node_id) + "/cpulist", cpulist)) {
        return false;
    }
    for (char c : cpulist) {
        if (std::isdigit(static_cast<unsigned char>(c))) {
            return true;
        }
    }
    return false;
}

long long read_node_memory_total_bytes(int node_id) {
    const std::filesystem::path meminfo_path =
        "/sys/devices/system/node/node" + std::to_string(node_id) + "/meminfo";
    std::ifstream file(meminfo_path);
    if (!file.is_open()) {
        return 0;
    }
    std::string line;
    while (std::getline(file, line)) {
        const auto pos = line.find("MemTotal:");
        if (pos == std::string::npos) {
            continue;
        }
        // Format: "Node N MemTotal:        131067368 kB"
        std::istringstream iss(line.substr(pos + std::string("MemTotal:").size()));
        long long kb = 0;
        if (iss >> kb) {
            return kb * 1024LL;
        }
    }
    return 0;
}

std::set<int> read_dax_target_nodes() {
    // Walk /sys/bus/dax/devices/daxN.M/target_node. Each value is the NUMA node
    // ID of a DAX-backed memory range (HBM in Flat Mode, CXL, PMEM, etc.).
    std::set<int> dax_nodes;
    std::error_code ec;
    const std::filesystem::path dax_root("/sys/bus/dax/devices");
    if (!std::filesystem::exists(dax_root, ec)) {
        return dax_nodes;
    }
    for (const auto& entry : std::filesystem::directory_iterator(dax_root, ec)) {
        if (ec) {
            break;
        }
        long long target_node = -1;
        if (read_long_long_file(entry.path() / "target_node", target_node) && target_node >= 0) {
            dax_nodes.insert(static_cast<int>(target_node));
        }
    }
    return dax_nodes;
}

void read_hmat_metrics(int node_id, long long& read_bw_mb_s, long long& read_lat_ns) {
    const std::filesystem::path access_root =
        "/sys/devices/system/node/node" + std::to_string(node_id) + "/access0/initiators";
    read_long_long_file(access_root / "read_bandwidth", read_bw_mb_s);
    read_long_long_file(access_root / "read_latency", read_lat_ns);
}

std::string refine_generation(const std::string& tier, const std::string& cpu_model) {
    std::string upper = cpu_model;
    std::transform(upper.begin(), upper.end(), upper.begin(),
                   [](unsigned char c) { return static_cast<char>(std::toupper(c)); });

    if (tier == "HBM") {
        if (upper.find("A64FX") != std::string::npos) return "HBM2";
        // Intel Xeon Max (Sapphire Rapids HBM) and Nvidia Grace-Hopper use HBM2E/HBM3.
        if (upper.find("MAX") != std::string::npos) return "HBM2E";
        if (upper.find("HOPPER") != std::string::npos || upper.find("GH200") != std::string::npos) return "HBM3";
        return "HBM2E";
    }
    if (tier == "DDR") {
        // Recent Intel Xeon (Sapphire/Emerald/Granite Rapids) and EPYC 9xxx use DDR5.
        if (upper.find("MAX") != std::string::npos
            || upper.find("PLATINUM") != std::string::npos
            || upper.find("GOLD") != std::string::npos
            || upper.find("EMERALD") != std::string::npos
            || upper.find("SAPPHIRE") != std::string::npos
            || upper.find("GRANITE") != std::string::npos) {
            return "DDR5";
        }
        if (upper.find("EPYC") != std::string::npos && upper.find("9") != std::string::npos) {
            return "DDR5";
        }
        if (upper.find("EPYC") != std::string::npos) return "DDR4";
        if (upper.find("NEOVERSE") != std::string::npos || upper.find("GRACE") != std::string::npos) {
            return "LPDDR5X";
        }
        return "DDR";  // generic when we can't be sure
    }
    return tier;
}

}

std::map<int, NumaNodeMemoryInfo> detect_numa_memory_info(const std::string& cpu_model,
                                                          bool has_onpackage_hbm) {
    std::map<int, NumaNodeMemoryInfo> nodes;

    std::error_code ec;
    const std::filesystem::path node_root("/sys/devices/system/node");
    if (!std::filesystem::exists(node_root, ec)) {
        return nodes;
    }

    const std::set<int> dax_target_nodes = read_dax_target_nodes();

    for (const auto& entry : std::filesystem::directory_iterator(node_root, ec)) {
        if (ec) {
            break;
        }
        const std::string name = entry.path().filename().string();
        if (name.rfind("node", 0) != 0 || name.size() <= 4) {
            continue;
        }
        bool all_digits = true;
        for (size_t i = 4; i < name.size(); ++i) {
            if (!std::isdigit(static_cast<unsigned char>(name[i]))) {
                all_digits = false;
                break;
            }
        }
        if (!all_digits) {
            continue;
        }

        int node_id = 0;
        try {
            node_id = std::stoi(name.substr(4));
        } catch (...) {
            continue;
        }

        NumaNodeMemoryInfo info;
        info.id = node_id;
        info.mem_total_bytes = read_node_memory_total_bytes(node_id);
        info.has_cpus = node_has_any_cpu(node_id);
        info.is_dax_backed = (dax_target_nodes.count(node_id) > 0);
        read_hmat_metrics(node_id, info.read_bandwidth_mb_s, info.read_latency_ns);
        nodes[node_id] = info;
    }

    if (nodes.empty()) {
        return nodes;
    }

    long long max_bw = 0;
    long long min_bw_nonzero = 0;
    std::vector<long long> bw_samples;
    for (const auto& kv : nodes) {
        if (kv.second.read_bandwidth_mb_s > 0) {
            bw_samples.push_back(kv.second.read_bandwidth_mb_s);
            max_bw = std::max(max_bw, kv.second.read_bandwidth_mb_s);
            if (min_bw_nonzero == 0 || kv.second.read_bandwidth_mb_s < min_bw_nonzero) {
                min_bw_nonzero = kv.second.read_bandwidth_mb_s;
            }
        }
    }
    const bool have_hmat_bw = !bw_samples.empty() && max_bw > 0;

    const double hbm_ratio_threshold = 1.8;
    const long long hbm_dax_capacity_max_bytes = 64LL << 30;
    const long long hbm_absolute_bandwidth_floor_mb_s = 400000;
    const bool hbm_capable = has_onpackage_hbm;

    for (auto& kv : nodes) {
        NumaNodeMemoryInfo& info = kv.second;
        std::string tier;

        if (have_hmat_bw && info.read_bandwidth_mb_s > 0) {
            const double ratio = static_cast<double>(info.read_bandwidth_mb_s) /
                                 static_cast<double>(min_bw_nonzero);
            const bool high_latency = info.read_latency_ns > 200;
            const bool dax_hbm_shape =
                info.is_dax_backed &&
                !info.has_cpus &&
                info.mem_total_bytes > 0 &&
                info.mem_total_bytes <= hbm_dax_capacity_max_bytes;

            const bool hbm_bandwidth_shape =
                hbm_capable &&
                info.read_bandwidth_mb_s >= hbm_absolute_bandwidth_floor_mb_s &&
                info.mem_total_bytes > 0 &&
                info.mem_total_bytes <= hbm_dax_capacity_max_bytes;

            if (high_latency) {
                tier = (info.read_latency_ns > 250) ? "PMEM" : "CXL";
            } else if (ratio >= hbm_ratio_threshold || dax_hbm_shape || hbm_bandwidth_shape) {
                tier = "HBM";
            } else {
                tier = "DDR";
            }
        } else {
            const bool hbm_capacity_shape =
                hbm_capable && info.has_cpus &&
                info.mem_total_bytes > 0 &&
                info.mem_total_bytes <= hbm_dax_capacity_max_bytes;

            if (info.is_dax_backed) {

                if (!info.has_cpus && info.mem_total_bytes > 0 &&
                    info.mem_total_bytes <= (64LL << 30)) {
                    tier = "HBM";
                } else {
                    tier = "CXL";
                }
            } else if (hbm_capacity_shape) {
                tier = "HBM";
            } else if (info.has_cpus) {
                tier = "DDR";
            } else {
                tier = "UNKNOWN";
            }
        }

        info.memory_type = refine_generation(tier, cpu_model);
        if (info.memory_type.empty()) {
            info.memory_type = "UNKNOWN";
        }
    }

    return nodes;
}

bool memory_type_matches(const std::string& memory_type, const std::string& family) {
    std::string upper_type = memory_type;
    std::string upper_family = family;
    std::transform(upper_type.begin(), upper_type.end(), upper_type.begin(),
                   [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    std::transform(upper_family.begin(), upper_family.end(), upper_family.begin(),
                   [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    return upper_type.find(upper_family) != std::string::npos;
}

std::string summarize_numa_memory_type_for_nodes(const std::map<int, NumaNodeMemoryInfo>& numa_memory,
                                                 const std::vector<int>& nodes,
                                                 const std::string& fallback) {
    if (nodes.empty() || numa_memory.empty()) {
        return fallback;
    }

    bool saw_hbm = false;
    bool saw_cxl = false;
    bool saw_pmem = false;
    bool saw_ddr = false;
    std::string first_known;

    for (int node : nodes) {
        auto it = numa_memory.find(node);
        if (it == numa_memory.end()) {
            continue;
        }

        const std::string& type = it->second.memory_type;
        if (type.empty() || type == "UNKNOWN") {
            continue;
        }
        if (first_known.empty()) {
            first_known = type;
        }

        saw_hbm = saw_hbm || memory_type_matches(type, "HBM");
        saw_cxl = saw_cxl || memory_type_matches(type, "CXL");
        saw_pmem = saw_pmem || memory_type_matches(type, "PMEM");
        saw_ddr = saw_ddr || memory_type_matches(type, "DDR") || memory_type_matches(type, "LPDDR");
    }

    if (saw_cxl) return "CXL";
    if (saw_pmem) return "PMEM";
    if (saw_hbm && !saw_ddr) return first_known;
    if (saw_ddr && !saw_hbm) return first_known;
    if (saw_hbm && saw_ddr) return "Mixed-HBM-DDR";
    return fallback;
}

bool numa_nodes_target_cxl_or_pmem(const std::map<int, NumaNodeMemoryInfo>& numa_memory,
                                   const std::vector<int>& nodes) {
    for (int node : nodes) {
        auto it = numa_memory.find(node);
        if (it == numa_memory.end()) {
            continue;
        }

        const std::string& type = it->second.memory_type;
        if (memory_type_matches(type, "CXL") || memory_type_matches(type, "PMEM")) {
            return true;
        }
    }
    return false;
}


uint64_t calculate_traffic_gen_array_size(uint64_t l3_size_bytes) {
    // TODO: This might need improving to ensure a minimum size, like we used to do with the previous formula
    return (l3_size_bytes * 4) / sizeof(double);
}

std::pair<int, int> find_ptr_chase_cpu() {
    if (!is_a64fx_system()) {
        const std::map<int, CpuTopology> cpu_topo = get_cpu_topology();
        auto it = cpu_topo.find(0);
        if (it != cpu_topo.end()) {
            return {0, it->second.node};
        }

        for (const auto& [cpu, topo] : cpu_topo) {
            if (topo.socket == 0) {
                return {cpu, topo.node};
            }
        }

        return {0, get_numa_node_of_cpu(0)};
    }

    const std::vector<int> allowed_cpus = get_allowed_cpus();
    for (int preferred_cpu : {0}) {
        if (std::find(allowed_cpus.begin(), allowed_cpus.end(), preferred_cpu) != allowed_cpus.end()) {
            int node = get_numa_node_of_cpu(preferred_cpu);
            if (numa_node_has_memory(node)) {
                return {preferred_cpu, node};
            }
        }
    }

    for (int cpu : allowed_cpus) {
        int node = get_numa_node_of_cpu(cpu);
        if (numa_node_has_memory(node)) {
            return {cpu, node};
        }
    }

    if (!allowed_cpus.empty()) {
        int node = get_numa_node_of_cpu(allowed_cpus.front());
        return {allowed_cpus.front(), node};
    }

    return {0, get_numa_node_of_cpu(0)};
}

SystemToolsCache& SystemToolsCache::instance() {
    static SystemToolsCache cache;
    return cache;
}

void SystemToolsCache::init() {
    static std::mutex init_mutex;
    std::lock_guard<std::mutex> lock(init_mutex);
    if (initialized) return;

    have_taskset = command_exists_in_path("taskset");
    have_numactl = command_exists_in_path("numactl");
    have_srun = command_exists_in_path("srun");
    have_mpirun = command_exists_in_path("mpirun");
    cpu_topology = get_cpu_topology();

    auto [pc_cpu, pc_node] = find_ptr_chase_cpu();
    ptr_chase_cpu = pc_cpu;
    ptr_chase_node = pc_node;

    initialized = true;
}
