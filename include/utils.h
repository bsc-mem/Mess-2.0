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

#ifndef UTILS_H
#define UTILS_H

#include <string>
#include <map>
#include <set>
#include <vector>
#include <cstdlib>
#include <sys/wait.h>
#include "architecture/PerformanceCounterStrategy.h"
#include "system_detection.h"
#include <filesystem>

inline bool run_command_success(const char* cmd) {
    int ret = std::system(cmd);
    if (ret == -1) {
        return false;
    }
    return WIFEXITED(ret) && WEXITSTATUS(ret) == 0;
}

inline bool run_command_success(const std::string& cmd) {
    return run_command_success(cmd.c_str());
}

inline bool pclose_success(FILE* stream) {
    if (!stream) return false;
    int ret = pclose(stream);
    if (ret == -1) return false;
    return WIFEXITED(ret) && WEXITSTATUS(ret) == 0;
}

struct CpuTopology {
    int node;
    int socket;
};

std::map<int, CpuTopology> get_cpu_topology();
std::map<int, std::set<int>> get_socket_to_nodes_map(const std::map<int, CpuTopology>& topo);
std::filesystem::path get_project_root();

struct SystemToolsCache {
    bool have_taskset;
    bool have_numactl;
    bool have_srun;
    bool have_mpirun;
    std::map<int, CpuTopology> cpu_topology;
    bool initialized;
    
    static SystemToolsCache& instance();
    void init();
    
private:
    SystemToolsCache() : have_taskset(false), have_numactl(false), have_srun(false), have_mpirun(false), initialized(false) {}
};

#ifndef USE_MPI_SPAWN
#define USE_MPI_SPAWN 0
#endif

std::string read_template(const std::string& filename);
std::string replace_template_variables(const std::string& template_content, 
                                      const std::map<std::string, std::string>& replacements);

int get_current_numa_node();
int get_numa_node_of_cpu(int cpu);
void append_memory_config_to_plotter(const std::filesystem::path& plotter_path, const CPUCapabilities& caps, const std::string& binding_type, double tlb_ns, int cache_line_size, double upi_scaling_factor = 0.0);

struct TLBMeasurement {
    double latency_ns = 0.0;
    int cache_line_size = 0;
    double nominal_cpu_freq_ghz = 0.0;
};

TLBMeasurement measure_and_set_tlb_latency(int attempt = 1);
int get_cache_line_size();

enum class MemoryType {
    DDR,
    HBM,
    CXL,
    UNKNOWN
};

class CounterDiscovery {
public:
    using CasCounterSelection = ::CasCounterSelection;

    struct CounterInfo {
        std::string name;
        std::string tool;
        bool is_available;
    };

    static const CasCounterSelection& get_perf_cas_counters();

private:
    static CasCounterSelection detect_perf_cas_counters();
    static std::vector<std::string> extract_cas_events_from_perf();
};

#endif
