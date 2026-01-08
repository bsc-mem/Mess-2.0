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

#include "process_manager.h"
#include "utils.h"

#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <thread>
#include <vector>
#include <unistd.h>
#include <set>
#include <sstream>
#include <map>
#include <algorithm>

namespace {

std::string build_core_query_command(int traffic_gen_cores) {
    return "if command -v lscpu >/dev/null 2>&1; then "
           "lscpu -p=cpu,core,socket 2>/dev/null | grep -v '^#' | "
           "awk -F, '$3==0{ if (!seen[$2]++) print $1 }' | grep -v '^0$' | head -n " +
           std::to_string(traffic_gen_cores) + "; "
           "else seq 1 " + std::to_string(traffic_gen_cores) + "; fi";
}

bool collect_core_list(int traffic_gen_cores, const std::vector<std::string>& explicit_cores, std::vector<std::string>& cores) {
    cores.clear();
    
    if (!explicit_cores.empty()) {
        cores = explicit_cores;
        return true;
    }

    if (traffic_gen_cores <= 0) {
        return false;
    }

    std::string command = build_core_query_command(traffic_gen_cores);
    FILE* pipe = popen(command.c_str(), "r");
    if (pipe) {
        char buffer[256];
        while (fgets(buffer, sizeof(buffer), pipe)) {
            std::string line(buffer);
            line.erase(line.find_last_not_of(" \t\n\r") + 1);
            line.erase(0, line.find_first_not_of(" \t\n\r"));
            if (!line.empty()) {
                cores.push_back(line);
                if (static_cast<int>(cores.size()) >= traffic_gen_cores) {
                    break;
                }
            }
        }
        pclose(pipe);
    }

    if (cores.empty()) {
        cores.reserve(traffic_gen_cores);
        for (int i = 1; i <= traffic_gen_cores; ++i) {
            cores.push_back(std::to_string(i));
        }
    }

    return !cores.empty();
}

#if USE_MPI_SPAWN
std::string join_core_list(const std::vector<std::string>& cores, const std::string& sep) {
    std::string out;
    for (size_t i = 0; i < cores.size(); ++i) {
        if (i > 0) {
            out += sep;
        }
        out += cores[i];
    }
    return out;
}
#endif


bool is_process_alive_and_not_zombie(int pid) {
    std::string stat_path = "/proc/" + std::to_string(pid) + "/stat";
    if (std::filesystem::exists(stat_path)) {
        std::ifstream stat_file(stat_path);
        if (stat_file.is_open()) {
            std::string line;
            std::getline(stat_file, line);
            size_t last_paren = line.find_last_of(')');
            if (last_paren != std::string::npos && last_paren + 2 < line.length()) {
                char state = line[last_paren + 2];
                return (state != 'Z');
            }
        }
    }
    
    std::string cmd = "ps -o state= -p " + std::to_string(pid);
    FILE* pipe = popen(cmd.c_str(), "r");
    bool alive = false;
    if (pipe) {
        char buffer[16];
        if (fgets(buffer, sizeof(buffer), pipe)) {
            std::string state(buffer);
            state.erase(state.find_last_not_of(" \n\r\t") + 1);
            state.erase(0, state.find_first_not_of(" \n\r\t"));
            
            if (!state.empty() && state.find('Z') == std::string::npos) {
                alive = true;
            }
        }
        pclose(pipe);
    }
    return alive;
}

bool any_traffic_gen_running() {
    std::set<int> pids;
    const char* cmds[] = {
        "pgrep -f '[t]raffic_gen_.*\\.x'",
        "pgrep '^traffic_gen_'"
    };
    
    for (const char* cmd : cmds) {
        FILE* pipe = popen(cmd, "r");
        if (pipe) {
            char buffer[64];
            while (fgets(buffer, sizeof(buffer), pipe)) {
                try {
                    int pid = std::stoi(buffer);
                    if (pid > 0) pids.insert(pid);
                } catch (...) {}
            }
            pclose(pipe);
        }
    }
    
    for (int pid : pids) {
        if (is_process_alive_and_not_zombie(pid)) {
            return true;
        }
    }
    return false;
}

std::string format_core_ranges(std::vector<int>& cores) {
    if (cores.empty()) return "";
    std::sort(cores.begin(), cores.end());
    std::stringstream ss;
    int range_start = cores[0];
    int range_end = cores[0];
    bool first = true;

    auto flush_range = [&](int start, int end) {
        if (!first) ss << ",";
        if (start == end) ss << start;
        else ss << start << "-" << end;
        first = false;
    };

    for (size_t i = 1; i < cores.size(); ++i) {
        if (cores[i] == range_end + 1) {
            range_end = cores[i];
        } else {
            flush_range(range_start, range_end);
            range_start = cores[i];
            range_end = cores[i];
        }
    }
    flush_range(range_start, range_end);
    return ss.str();
}

}

TrafficGenProcessManager::TrafficGenProcessManager(const BenchmarkConfig& config, const system_info& /*sys_info*/)
    : config_(config), keep_traffic_gen_on_retry_(false),
      retry_bw_cas_rd_(0), retry_bw_cas_wr_(0), retry_bw_elapsed_(0.0), retry_bw_gb_s_(0.0),
      current_traffic_gen_pid_(0), active_traffic_gen_pid_(0),
      persistent_traffic_gen_valid_(false), persistent_ratio_pct_(0), persistent_pause_(0),
      persistent_mode_(ExecutionMode::MULTISEQUENTIAL), persistent_traffic_gen_cores_(0),
      cached_traffic_gen_cores_(0) {}

bool TrafficGenProcessManager::prepare_traffic_gen(double ratio_pct,
                                          int pause,
                                          int traffic_gen_cores,
                                          const std::vector<int>& traffic_gen_mem_nodes,
                                          const std::string& traffic_gen_log_file,
                                          TrafficGenPreparation& prep,
                                          ExecutionMode mode) {
    prep = {};

    if (consume_bandwidth_retry(prep.prev_cas_rd, prep.prev_cas_wr, prep.prev_elapsed, prep.prev_bw_gb_s)) {
        prep.reused_existing = true;
        prep.pid = active_traffic_gen_pid_;
        current_traffic_gen_pid_ = active_traffic_gen_pid_;

        if (config_.verbosity >= 2) {
            std::cout << "  Re-measuring with existing TrafficGen (BW still growing)..." << std::endl;
        }
        if (config_.verbosity >= 3) {
            std::cout << "    Using existing TrafficGen process (PID: " << prep.pid << ")" << std::endl;
            std::cout << "      - Previous BW baseline: " << std::fixed << std::setprecision(2)
                      << prep.prev_bw_gb_s << " GB/s" << std::endl;
        }
        return true;
    }

    if (config_.persistent_traffic_gen && persistent_traffic_gen_valid_) {
        bool config_match = (
            std::abs(persistent_ratio_pct_ - ratio_pct) < 1e-5 &&
            persistent_pause_ == pause &&
            persistent_traffic_gen_cores_ == traffic_gen_cores &&
            persistent_mode_ == mode &&
            persistent_traffic_gen_mem_nodes_ == traffic_gen_mem_nodes
        );

        if (config_match) {
            if (active_traffic_gen_pid_ > 0 && is_traffic_gen_running(active_traffic_gen_pid_)) {
                prep.reused_existing = true;
                prep.pid = active_traffic_gen_pid_;
                current_traffic_gen_pid_ = active_traffic_gen_pid_;

                if (config_.verbosity >= 2) {
                    std::cout << "  Using persistent TrafficGen process (PID: " << prep.pid << ")..." << std::endl;
                }
                return true;
            } else {
                 if (config_.verbosity >= 2) {
                    std::cout << "  Persistent TrafficGen process died, spawning new one..." << std::endl;
                 }
                 persistent_traffic_gen_valid_ = false;
            }
        } else {
             persistent_traffic_gen_valid_ = false;
             if (active_traffic_gen_pid_ > 0) {
                 terminate_traffic_gen(active_traffic_gen_pid_);
             }
        }
    }

    if (!cleanup_zombie_traffic_gen()) {
        std::cerr << "ERROR: Failed to clean up existing TrafficGen processes" << std::endl;
        return false;
    }

    pid_t traffic_gen_pid = 0;
    bool launched = false;

    bool force_process_launch = !config_.traffic_gen_explicit_cores.empty();

#if USE_MPI_SPAWN == 0
    force_process_launch = true;
#endif

    if (force_process_launch) {
        if (mode == ExecutionMode::MULTISEQUENTIAL) {
            launched = launch_traffic_gen_multiseq(ratio_pct, pause, traffic_gen_cores, traffic_gen_mem_nodes,
                                         traffic_gen_log_file, traffic_gen_pid);
        }
    } else {
#if USE_MPI_SPAWN
        launched = launch_traffic_gen_mpi(ratio_pct, pause, traffic_gen_cores, traffic_gen_mem_nodes,
                                     traffic_gen_log_file, traffic_gen_pid, mode);
#endif
    }

    if (!launched) {
        return false;
    }

    current_traffic_gen_pid_ = traffic_gen_pid;
    active_traffic_gen_pid_ = traffic_gen_pid;
    prep.pid = traffic_gen_pid;
    prep.reused_existing = false;
    prep.reused_existing = false;

    if (config_.persistent_traffic_gen) {
        persistent_traffic_gen_valid_ = true;
        persistent_ratio_pct_ = ratio_pct;
        persistent_pause_ = pause;
        persistent_mode_ = mode;
        persistent_traffic_gen_cores_ = traffic_gen_cores;
        persistent_traffic_gen_mem_nodes_ = traffic_gen_mem_nodes;
    }

    return true;
}

bool TrafficGenProcessManager::relaunch_traffic_gen(double ratio_pct,
                                           int pause,
                                           int traffic_gen_cores,
                                           const std::vector<int>& traffic_gen_mem_nodes,
                                           const std::string& traffic_gen_log_file,
                                           ExecutionMode mode) {
    pid_t traffic_gen_pid = 0;
    bool force_process_launch = !config_.traffic_gen_explicit_cores.empty();
#if USE_MPI_SPAWN == 0
    force_process_launch = true;
#endif

    if (force_process_launch) {
        bool ret = false;
        if (mode == ExecutionMode::MULTISEQUENTIAL) {
            ret = launch_traffic_gen_multiseq(ratio_pct, pause, traffic_gen_cores, traffic_gen_mem_nodes,
                               traffic_gen_log_file, traffic_gen_pid);
        }
        if (!ret) return false;
    } else {
#if USE_MPI_SPAWN
        if (!launch_traffic_gen_mpi(ratio_pct, pause, traffic_gen_cores, traffic_gen_mem_nodes,
                               traffic_gen_log_file, traffic_gen_pid, mode)) {
            return false;
        }
#endif
    }
    current_traffic_gen_pid_ = traffic_gen_pid;
    active_traffic_gen_pid_ = traffic_gen_pid;
    return true;
}

bool TrafficGenProcessManager::is_traffic_gen_running(pid_t pid) const {
    if (pid <= 0) {
        return false;
    }
    return (::kill(pid, 0) == 0);
}

void TrafficGenProcessManager::terminate_traffic_gen(pid_t pid) {
    if (pid <= 0) {
        return;
    }
    ::kill(-pid, SIGKILL);
    ::kill(pid, SIGKILL);
    if (pid == current_traffic_gen_pid_) {
        current_traffic_gen_pid_ = 0;
    }
    if (pid == active_traffic_gen_pid_) {
        active_traffic_gen_pid_ = 0;
    }
}

void TrafficGenProcessManager::kill_all_traffic_gen() {
    if (current_traffic_gen_pid_ > 0) {
        terminate_traffic_gen(current_traffic_gen_pid_);
    }
    if (active_traffic_gen_pid_ > 0) {
        terminate_traffic_gen(active_traffic_gen_pid_);
    }
    persistent_traffic_gen_valid_ = false;
}

void TrafficGenProcessManager::stash_bandwidth_retry(pid_t traffic_gen_pid,
                                                 long long cas_rd,
                                                 long long cas_wr,
                                                 double elapsed,
                                                 double bw_gb_s) {
    keep_traffic_gen_on_retry_ = true;
    retry_bw_cas_rd_ = cas_rd;
    retry_bw_cas_wr_ = cas_wr;
    retry_bw_elapsed_ = elapsed;
    retry_bw_gb_s_ = bw_gb_s;
    active_traffic_gen_pid_ = traffic_gen_pid;
}

bool TrafficGenProcessManager::consume_bandwidth_retry(long long& cas_rd,
                                                   long long& cas_wr,
                                                   double& elapsed,
                                                   double& bw_gb_s) {
    if (!keep_traffic_gen_on_retry_ || active_traffic_gen_pid_ <= 0) {
        return false;
    }
    keep_traffic_gen_on_retry_ = false;
    cas_rd = retry_bw_cas_rd_;
    cas_wr = retry_bw_cas_wr_;
    elapsed = retry_bw_elapsed_;
    bw_gb_s = retry_bw_gb_s_;
    retry_bw_cas_rd_ = 0;
    retry_bw_cas_wr_ = 0;
    retry_bw_elapsed_ = 0.0;
    retry_bw_gb_s_ = 0.0;
    return true;
}

bool TrafficGenProcessManager::launch_traffic_gen_multiseq(double ratio_pct,
                                                 int pause,
                                                 int traffic_gen_cores,
                                                 const std::vector<int>& traffic_gen_mem_nodes,
                                                 const std::string& traffic_gen_log_file,
                                                 pid_t& traffic_gen_pid) const {
    std::string pid_filename = "/tmp/traffic_gen_pid_" + std::to_string(getpid()) + "_" +
                               std::to_string(static_cast<int>(ratio_pct)) + "_" +
                               std::to_string(pause) + ".tmp";

    std::vector<std::string> cores;
    if (!resolve_core_list(traffic_gen_cores, config_.traffic_gen_explicit_cores, cores)) {
        std::cerr << "ERROR: Failed to select core list" << std::endl;
        return false;
    }

    const auto& cache = SystemToolsCache::instance();
    bool have_taskset = cache.have_taskset;
    bool have_numactl = cache.have_numactl;

    std::string worker_executable;
    std::filesystem::path root = get_project_root();
    std::filesystem::path build_bin = root / "build/bin";
    std::filesystem::path binary_path = build_bin / "traffic_gen_multiseq.x";

    if (std::filesystem::exists(binary_path)) {
        worker_executable = binary_path.string();
    } else {
        worker_executable = "./traffic_gen_multiseq.x";
    }
    
    std::vector<std::string> worker_cmds;
    
    const std::map<int, CpuTopology>& cpu_topo = cache.cpu_topology;
    std::map<int, std::set<int>> socket_nodes_map = get_socket_to_nodes_map(cpu_topo);
    
    std::string explicit_membind_str;
    if (!traffic_gen_mem_nodes.empty()) {
        for (size_t i = 0; i < traffic_gen_mem_nodes.size(); ++i) {
            explicit_membind_str += std::to_string(traffic_gen_mem_nodes[i]);
            if (i < traffic_gen_mem_nodes.size() - 1) explicit_membind_str += ",";
        }
    }

    std::map<std::string, std::vector<int>> node_to_cores;

    for (const auto& core_str : cores) {
        int core_id = -1;
        try { core_id = std::stoi(core_str); } catch(...) {}
        
        std::string membind_arg;
        bool core_found = (core_id != -1 && cpu_topo.count(core_id));
        
        if (!explicit_membind_str.empty()) {
            membind_arg = explicit_membind_str;
        } else {
            if (core_found) {
                membind_arg = std::to_string(cpu_topo.at(core_id).node);
            } else {
                membind_arg = "0"; // Fallback
            }
        }
        
        if (core_id != -1) {
            node_to_cores[membind_arg].push_back(core_id);
        }
        
        std::string cmd;
        std::string worker_args = " -r " + std::to_string(static_cast<int>(ratio_pct)) +
                                  " -p " + std::to_string(pause) +
                                  " -w " + std::to_string(cores.size());

        if (have_numactl && have_taskset) {
            cmd = "numactl --membind=" + membind_arg + " taskset -c " + core_str + " " +
                  worker_executable + worker_args;
        } else if (have_numactl) {
            cmd = "numactl --membind=" + membind_arg + " --physcpubind=" + core_str + " " +
                  worker_executable + worker_args;
        } else if (have_taskset) {
            cmd = "taskset -c " + core_str + " " + worker_executable + worker_args;
        } else {
            cmd = worker_executable + worker_args;
        }
        worker_cmds.push_back(cmd);
    }

    std::string spawn_script;
    for (const auto& cmd : worker_cmds) {
        spawn_script += cmd + " >>" + traffic_gen_log_file + " 2>&1 & ";
    }
    spawn_script += "wait";

    std::string launcher = "set -m; ( " + spawn_script + " ) & echo $! > " + pid_filename;

    if (config_.verbosity >= 2) {
        std::cout << "    Launching TrafficGen (MultiSeq - " << cores.size() << " procs)";
        if (config_.verbosity < 3) std::cout << std::endl;
        std::cout << std::flush;
        
        if (!node_to_cores.empty()) {
             if (config_.verbosity >= 3) {
                std::cout << "\n      Memory Bindings: ";
                bool first_node = true;
                for (auto& pair : node_to_cores) {
                    if (!first_node) std::cout << ", ";
                    if (!traffic_gen_mem_nodes.empty()) {
                        std::cout << pair.first << " (Forced)";
                    } else {
                        std::cout << pair.first << " (Cores " << format_core_ranges(pair.second) << ")";
                    }
                    first_node = false;
                }
             }
        }

        if (config_.verbosity >= 3) {
            std::cout << std::endl;
            size_t preview = std::min<size_t>(worker_cmds.size(), 5);
            for (size_t i = 0; i < preview; ++i) {
                std::cout << "      cmd[" << i << "]: " << worker_cmds[i] << std::endl;
            }
            if (worker_cmds.size() > preview) {
                std::cout << "      ... (" << (worker_cmds.size() - preview)
                          << " more)" << std::endl;
            }
        }
    }

    int traffic_gen_ret = system(launcher.c_str());
    if (traffic_gen_ret != 0) {
        std::cerr << "ERROR: Failed to launch TrafficGen" << std::endl;
        return false;
    }

    if (!read_traffic_gen_pid(pid_filename, traffic_gen_pid)) {
        std::cerr << "ERROR: Could not obtain TrafficGen PID" << std::endl;
        return false;
    }
    return true;
}


#if USE_MPI_SPAWN
bool TrafficGenProcessManager::launch_traffic_gen_mpi(double ratio_pct,
                                             int pause,
                                             int traffic_gen_cores,
                                             const std::vector<int>& traffic_gen_mem_nodes,
                                             const std::string& traffic_gen_log_file,
                                             pid_t& traffic_gen_pid,
                                             ExecutionMode mode) const {
    std::string pid_filename = "/tmp/traffic_gen_pid_" + std::to_string(getpid()) + "_" +
                               std::to_string(static_cast<int>(ratio_pct)) + "_" +
                               std::to_string(pause) + ".tmp";
    std::vector<std::string> cores;
    if (!resolve_core_list(traffic_gen_cores, config_.traffic_gen_explicit_cores, cores)) {
        std::cerr << "ERROR: Failed to select core list" << std::endl;
        return false;
    }

    if (!config_.traffic_gen_explicit_cores.empty()) {

        if (config_.verbosity >= 1) {
           std::cout << "    Notice: Explicit core list with cross-socket binding is not fully supported in MPI mode yet." << std::endl;
           std::cout << "            Falling back to standard MultiSequential launcher for accurate binding." << std::endl;
        }
        return launch_traffic_gen_multiseq(ratio_pct, pause, traffic_gen_cores, traffic_gen_mem_nodes, traffic_gen_log_file, traffic_gen_pid);
    }

    std::string core_list = join_core_list(cores, ",");

    const auto& cache = SystemToolsCache::instance();
    bool have_srun = cache.have_srun;
    bool have_mpirun = cache.have_mpirun;
    bool have_numactl = cache.have_numactl;

    if (!have_srun && !have_mpirun) {
        if (config_.verbosity >= 1) {
            std::cout << "    Notice: MPI launchers not found. Falling back to taskset method." << std::endl;
        }
        return launch_traffic_gen_multiseq(ratio_pct, pause, traffic_gen_cores, traffic_gen_mem_nodes,
                                 traffic_gen_log_file, traffic_gen_pid);
    }

    bool use_srun = have_srun;

    bool use_srun = have_srun;

    std::string worker_executable;
    std::filesystem::path root = get_project_root();
    std::filesystem::path build_bin = root / "build/bin";
    std::string bin_name = "traffic_gen_multiseq.x";
    std::filesystem::path binary_path = build_bin / bin_name;

    if (std::filesystem::exists(binary_path)) {
        worker_executable = binary_path.string();
    } else {
        worker_executable = "./traffic_gen_multiseq.x";
    }
    std::string worker_args = " -r " + std::to_string(static_cast<int>(ratio_pct)) +
                              " -p " + std::to_string(pause) +
                              " -w " + std::to_string(traffic_gen_cores);
    
    std::string membind_arg = get_membind_arg(traffic_gen_mem_nodes.empty() ? 0 : traffic_gen_mem_nodes[0]);
    std::string numactl_cmd;
    if (membind_arg == "LOCALALLOC") {
        numactl_cmd = "numactl --localalloc";
    } else {
        numactl_cmd = "numactl --membind=" + membind_arg;
    }

    std::string spawn_cmd;
    if (use_srun) {
        
        std::string srun_mem_bind_option = "";
        if (membind_arg != "LOCALALLOC") {
            srun_mem_bind_option = " --mem-bind=map_mem:" + membind_arg;
        }

        if (have_numactl) {
            spawn_cmd = "srun -n " + std::to_string(traffic_gen_cores) +
                        " --cpu-bind=map_cpu:" + core_list +
                        srun_mem_bind_option + " " + numactl_cmd + " " + worker_executable +
                        worker_args + " >>" + traffic_gen_log_file + " 2>&1";
        } else {
            spawn_cmd = "srun -n " + std::to_string(traffic_gen_cores) +
                        " --cpu-bind=map_cpu:" + core_list +
                        srun_mem_bind_option + " " + worker_executable +
                        worker_args + " >>" + traffic_gen_log_file + " 2>&1";
        }
    } else {
        if (have_numactl) {
            spawn_cmd = numactl_cmd + " mpirun -np " + std::to_string(traffic_gen_cores) +
                        " --bind-to core " + worker_executable + worker_args + " >>" +
                        traffic_gen_log_file + " 2>&1";
        } else {
            spawn_cmd = "mpirun -np " + std::to_string(traffic_gen_cores) +
                        " --bind-to core " + worker_executable + worker_args + " >>" +
                        traffic_gen_log_file + " 2>&1";
        }
    }

    std::string launcher = "set -m; ( " + spawn_cmd + " ) & echo $! > " + pid_filename;

    if (config_.verbosity >= 2) {
        std::cout << "    Launching TrafficGen (MPI method)...";
        if (config_.verbosity < 3) std::cout << std::endl;
        std::cout << std::flush;
        
        std::vector<int> int_cores;
        for (const auto& c : cores) {
            try { int_cores.push_back(std::stoi(c)); } catch(...) {}
        }
        
        if (!int_cores.empty()) {
            if (config_.verbosity >= 3) {
                if (!traffic_gen_mem_nodes.empty()) {
                    std::cout << "\n      Memory Bindings: " << membind_arg << " (Forced)";
                } else {
                    std::cout << "\n      Memory Bindings: " << membind_arg << " (Cores " << format_core_ranges(int_cores) << ")";
                }
            }
        }

        if (config_.verbosity >= 3) {
            std::cout << std::endl;
            std::cout << "      - Method: " << (use_srun ? "srun (SLURM)" : "mpirun") << std::endl;
            std::cout << "      - Cores: " << core_list << std::endl;
            std::cout << "      - " << launcher << std::endl;
        }
    }

    int traffic_gen_ret = system(launcher.c_str());
    if (traffic_gen_ret != 0) {
        std::cerr << "ERROR: Failed to launch TrafficGen" << std::endl;
        return false;
    }

    if (!read_traffic_gen_pid(pid_filename, traffic_gen_pid)) {
        std::cerr << "ERROR: Could not obtain TrafficGen PID" << std::endl;
        return false;
    }

    return true;
}
#endif

bool TrafficGenProcessManager::read_traffic_gen_pid(const std::string& pid_filename,
                                           pid_t& traffic_gen_pid) const {
    FILE* pid_file = fopen(pid_filename.c_str(), "r");
    bool have_pid = false;
    if (pid_file) {
        char pid_str[32];
        if (fgets(pid_str, sizeof(pid_str), pid_file) != nullptr) {
            traffic_gen_pid = atoi(pid_str);
            if (traffic_gen_pid > 0) {
                have_pid = true;
            }
        }
        fclose(pid_file);
        unlink(pid_filename.c_str());
    }
    return have_pid;
}

bool TrafficGenProcessManager::cleanup_zombie_traffic_gen() const {
    int pre_cleanup = 0;
    if (active_traffic_gen_pid_ > 0) {
        ::kill(-active_traffic_gen_pid_, SIGKILL);
        ::kill(active_traffic_gen_pid_, SIGKILL);
    }

    int max_retries = 50;
    while (pre_cleanup < max_retries) {
        if (!any_traffic_gen_running()) {
            break;
        }

        system("pkill -9 -f 'traffic_gen_.*\\.x' 2>/dev/null");
        system("pkill -9 '^traffic_gen_' 2>/dev/null");
        
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        pre_cleanup++;
    }
    return pre_cleanup < max_retries;
}


std::string TrafficGenProcessManager::get_membind_arg(int default_node) const {
    std::string membind_arg;
    
    if (!config_.memory_bind_nodes.empty()) {
        for (size_t i = 0; i < config_.memory_bind_nodes.size(); ++i) {
            membind_arg += std::to_string(config_.memory_bind_nodes[i]);
            if (i < config_.memory_bind_nodes.size() - 1) membind_arg += ",";
        }
    } else {
        membind_arg = std::to_string(default_node);
    }
    return membind_arg;
}

bool TrafficGenProcessManager::resolve_core_list(int traffic_gen_cores, const std::vector<std::string>& explicit_cores, std::vector<std::string>& out_cores) const {
    bool can_use_cache = true;
    
    if (explicit_cores != cached_explicit_cores_) {
        can_use_cache = false;
    }
    
    if (traffic_gen_cores != cached_traffic_gen_cores_) {
        can_use_cache = false;
    }
    
    if (cached_core_list_.empty()) {
        can_use_cache = false;
    }

    if (can_use_cache) {
        out_cores = cached_core_list_;
        return true;
    }

    if (collect_core_list(traffic_gen_cores, explicit_cores, out_cores)) {
        cached_core_list_ = out_cores;
        cached_traffic_gen_cores_ = traffic_gen_cores;
        cached_explicit_cores_ = explicit_cores;
        return true;
    }

    return false;
}
