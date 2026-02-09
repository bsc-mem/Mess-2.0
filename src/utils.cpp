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

#include "utils.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <unistd.h>
#include <sys/file.h>
#include <fcntl.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <cmath>
#include <cctype>
#include <mutex>

namespace {

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

        cpu_topo[cpu] = {node >= 0 ? node : 0, socket};
    }

    return cpu_topo;
}

}

int get_current_numa_node() {
#ifdef __linux__
    unsigned cpu, node;
#ifdef SYS_getcpu
    if (syscall(SYS_getcpu, &cpu, &node, NULL) == 0) {
        return (int)node;
    }
#endif
#endif
    return 0;
}

int get_numa_node_of_cpu(int cpu) {
    (void)cpu;
#ifdef __linux__
    std::string cpu_path = "/sys/devices/system/cpu/cpu" + std::to_string(cpu);
    if (!std::filesystem::exists(cpu_path)) return 0;

    for (int node = 0; node < 16; ++node) { 
        std::string node_path = cpu_path + "/node" + std::to_string(node);
        if (std::filesystem::exists(node_path)) {
            return node;
        }
    }
#endif
    return 0;
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

void append_memory_config_to_plotter(const std::filesystem::path& plotter_path, const CPUCapabilities& caps, const std::string& binding_type, double tlb_ns, int cache_line_size, double upi_scaling_factor) {
    if (plotter_path.has_parent_path()) {
        std::filesystem::create_directories(plotter_path.parent_path());
    }
    
    int fd = open(plotter_path.c_str(), O_RDWR | O_CREAT | O_APPEND, 0644);
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

    std::stringstream ss;
    bool needs_newline = !content.empty() && content.back() != '\n';

    if (content.find("TLB_NS=") == std::string::npos) {
        if (needs_newline) { ss << "\n"; needs_newline = false; }
        ss << "TLB_NS=" << tlb_ns << "\n";
        ss << "CACHE_LINE_SIZE=" << cache_line_size << "\n";
    }

    if (content.find("MEMORY_BINDING=") == std::string::npos) {
        if (needs_newline) { ss << "\n"; needs_newline = false; }
        ss << "MEMORY_BINDING=" << binding_type << "\n";
        
        if (binding_type == "remote") {
            if (caps.upi_freq > 0) {
                ss << "UPI_FREQ=" << caps.upi_freq << "\n";
                ss << "N_DATA_LANES=" << caps.n_data_lanes << "\n";
                ss << "FLIT_BIT=" << caps.flit_bit << "\n";
                ss << "DATA_FLIT_BIT=" << caps.data_flit_bit << "\n";
                ss << "N_UPI_CHANNELS=" << caps.n_upi_channels << "\n";
                
                if (upi_scaling_factor > 0) {
                     ss << "UPI_SCALING_FACTOR=" << upi_scaling_factor << "\n";
                }
            } else if (caps.nvlink_bw_gb_s > 0) {
                ss << "NVLINK_BW_GB_S=" << caps.nvlink_bw_gb_s << "\n";
            }
        } else {
            ss << "MEM_FREQ=" << caps.memory_frequency << "\n";
            ss << "N_CHANNELS=" << caps.memory_channels << "\n";
            ss << "BUS_WIDTH=" << caps.bus_width << "\n";
        }
    }
    
    std::string data = ss.str();
    if (!data.empty()) {
        write(fd, data.c_str(), data.length());
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
    FILE* pipe = popen("lscpu -p=cpu,node,socket 2>/dev/null", "r");
    if (!pipe) {
        return cpu_topo;
    }

    char buffer[256];
    while (fgets(buffer, sizeof(buffer), pipe)) {
        std::string line(buffer);
        if (line.empty() || line[0] == '#') continue;
        
        size_t first_comma = line.find(',');
        size_t second_comma = line.find(',', first_comma + 1);
        
        if (first_comma != std::string::npos && second_comma != std::string::npos) {
            try {
                int cpu = std::stoi(line.substr(0, first_comma));
                int node = std::stoi(line.substr(first_comma + 1, second_comma - first_comma - 1));
                int socket = std::stoi(line.substr(second_comma + 1));
                cpu_topo[cpu] = {node, socket};
            } catch (...) {}
        }
    }
    pclose_success(pipe);
    return cpu_topo;
}

std::map<int, std::set<int>> get_socket_to_nodes_map(const std::map<int, CpuTopology>& topo) {
    std::map<int, std::set<int>> socket_nodes;
    for (const auto& pair : topo) {
        socket_nodes[pair.second.socket].insert(pair.second.node);
    }
    return socket_nodes;
}


uint64_t calculate_traffic_gen_array_size(uint64_t l3_size_bytes) {
    uint64_t arg1 = 5ULL * 1000 * 1000 * 1000;
    uint64_t arg2 = l3_size_bytes * 10;
    return std::min(arg1, arg2) / sizeof(double);
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
    
    initialized = true;
}
