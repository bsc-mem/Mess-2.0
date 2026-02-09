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
#include "system_detection.h"

#include <chrono>
#include <csignal>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <thread>
#include <vector>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/select.h>
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


bool process_exists(pid_t pid) {
    if (pid <= 0) {
        return false;
    }
    if (::kill(pid, 0) == 0) {
        return true;
    }
    return errno == EPERM;
}

#ifdef __linux__
bool is_numeric_pid_dirname(const std::string& name) {
    if (name.empty()) {
        return false;
    }
    return std::all_of(name.begin(), name.end(), [](unsigned char c) { return std::isdigit(c); });
}

bool is_traffic_gen_name(const std::string& text) {
    if (text.find("traffic_gen_multiseq.x") != std::string::npos) {
        return true;
    }
    if (text.find("traffic_gen_rand.x") != std::string::npos) {
        return true;
    }
    return false;
}

bool pid_owned_by_current_user(const std::filesystem::path& proc_dir) {
    std::ifstream status_file(proc_dir / "status");
    if (!status_file.is_open()) {
        return false;
    }

    std::string line;
    while (std::getline(status_file, line)) {
        if (line.rfind("Uid:", 0) == 0) {
            std::istringstream iss(line.substr(4));
            uid_t real_uid = 0;
            iss >> real_uid;
            return real_uid == getuid();
        }
    }
    return false;
}
#endif

std::vector<pid_t> list_traffic_gen_pids() {
    std::vector<pid_t> pids;

#ifdef __linux__
    std::error_code ec;
    for (const auto& entry : std::filesystem::directory_iterator("/proc", ec)) {
        if (ec) {
            break;
        }
        if (!entry.is_directory(ec) || ec) {
            continue;
        }

        std::string pid_str = entry.path().filename().string();
        if (!is_numeric_pid_dirname(pid_str)) {
            continue;
        }
        if (!pid_owned_by_current_user(entry.path())) {
            continue;
        }

        pid_t pid = static_cast<pid_t>(std::stoi(pid_str));
        bool match = false;

        {
            std::ifstream comm_file(entry.path() / "comm");
            std::string comm;
            if (comm_file.is_open() && std::getline(comm_file, comm)) {
                if (comm.rfind("traffic_gen_", 0) == 0) {
                    match = true;
                }
            }
        }

        if (!match) {
            std::ifstream cmd_file(entry.path() / "cmdline", std::ios::binary);
            if (cmd_file.is_open()) {
                std::string cmd((std::istreambuf_iterator<char>(cmd_file)),
                                 std::istreambuf_iterator<char>());
                std::replace(cmd.begin(), cmd.end(), '\0', ' ');
                if (is_traffic_gen_name(cmd)) {
                    match = true;
                }
            }
        }

        if (match) {
            pids.push_back(pid);
        }
    }
#else
    const char* cmds[] = {
        "pgrep -f '[t]raffic_gen_.*\\.x'",
        "pgrep '^traffic_gen_'"
    };
    for (const char* cmd : cmds) {
        FILE* pipe = popen(cmd, "r");
        if (!pipe) {
            continue;
        }
        char buffer[64];
        while (fgets(buffer, sizeof(buffer), pipe)) {
            try {
                pid_t pid = static_cast<pid_t>(std::stol(buffer));
                if (pid > 0) {
                    pids.push_back(pid);
                }
            } catch (...) {}
        }
        pclose(pipe);
    }
#endif

    std::sort(pids.begin(), pids.end());
    pids.erase(std::unique(pids.begin(), pids.end()), pids.end());
    return pids;
}

void signal_process_tree(pid_t pid, int sig) {
    if (pid <= 0) {
        return;
    }
    ::kill(-pid, sig);
    ::kill(pid, sig);
}

bool wait_for_exit(pid_t pid, std::chrono::milliseconds timeout) {
    auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (!process_exists(pid)) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return !process_exists(pid);
}

bool stop_process_tree(pid_t pid, std::chrono::milliseconds term_timeout) {
    if (!process_exists(pid)) {
        return true;
    }

    signal_process_tree(pid, SIGTERM);
    if (wait_for_exit(pid, term_timeout)) {
        return true;
    }

    signal_process_tree(pid, SIGKILL);
    return wait_for_exit(pid, std::chrono::milliseconds(500));
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

TrafficGenProcessManager::TrafficGenProcessManager(const BenchmarkConfig& config, const system_info& sys_info)
    : config_(config), sys_info_(sys_info), keep_traffic_gen_on_retry_(false),
      retry_bw_cas_rd_(0), retry_bw_cas_wr_(0), retry_bw_elapsed_(0.0), retry_bw_gb_s_(0.0),
      current_traffic_gen_pid_(0), active_traffic_gen_pid_(0),
      persistent_traffic_gen_valid_(false), persistent_ratio_pct_(0), persistent_pause_(0),
      persistent_mode_(ExecutionMode::MULTISEQUENTIAL), persistent_traffic_gen_cores_(0),
      expected_ready_count_(0), cached_traffic_gen_cores_(0) {
#if !USE_MPI_SPAWN
    (void)sys_info_;
#endif
    generate_unique_names();
}

TrafficGenProcessManager::~TrafficGenProcessManager() {
    if (!ready_fifo_path_.empty()) {
        ::unlink(ready_fifo_path_.c_str());
    }
}

void TrafficGenProcessManager::generate_unique_names() {
    const char* existing_id = std::getenv("MESS_UNIQUE_ID");
    if (existing_id && existing_id[0] != '\0') {
        unique_id_ = existing_id;
    } else {
        const char* job_id = nullptr;

        if ((job_id = std::getenv("SLURM_JOB_ID")) && job_id[0] != '\0') {
            unique_id_ = std::string("slurm_") + job_id;
        } else if ((job_id = std::getenv("PBS_JOBID")) && job_id[0] != '\0') {
            unique_id_ = std::string("pbs_") + job_id;
        } else if ((job_id = std::getenv("LSB_JOBID")) && job_id[0] != '\0') {
            unique_id_ = std::string("lsf_") + job_id;
        } else if ((job_id = std::getenv("JOB_ID")) && job_id[0] != '\0') {
            unique_id_ = std::string("sge_") + job_id;
        } else {
            unique_id_ = std::string("pid_") + std::to_string(getpid());
        }

        setenv("MESS_UNIQUE_ID", unique_id_.c_str(), 1);
    }

    ready_fifo_path_ = "/tmp/mess_tgen_ready_" + unique_id_;
}

bool TrafficGenProcessManager::wait_for_traffic_gen_ready(int timeout_seconds) {
    int num_workers = expected_ready_count_;
    if (ready_fifo_path_.empty() || num_workers <= 0) {
        return false;
    }

    int fd = ::open(ready_fifo_path_.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd < 0) {
        if (config_.verbosity >= 2) {
            std::cerr << "    Warning: Could not open ready FIFO " << ready_fifo_path_
                      << ": " << std::strerror(errno) << std::endl;
        }
        return false;
    }

    int total_read = 0;
    char buf[1024];
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(timeout_seconds);

    while (total_read < num_workers) {
        auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now());
        if (remaining.count() <= 0) {
            if (config_.verbosity >= 1) {
                std::cerr << "    Warning: TrafficGen ready timeout (" << timeout_seconds
                          << "s). Got " << total_read << "/" << num_workers << " ready signals." << std::endl;
            }
            ::close(fd);
            return false;
        }

        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(fd, &read_fds);

        struct timeval tv;
        long ms = std::min((long)remaining.count(), 5000L);
        tv.tv_sec = ms / 1000;
        tv.tv_usec = (ms % 1000) * 1000;

        int ready = ::select(fd + 1, &read_fds, nullptr, nullptr, &tv);
        if (ready < 0) {
            if (errno == EINTR) continue;
            ::close(fd);
            return false;
        }
        if (ready == 0) {
            if (active_traffic_gen_pid_ > 0 && !is_traffic_gen_running(active_traffic_gen_pid_)) {
                if (config_.verbosity >= 2) {
                    std::cerr << "    Warning: TrafficGen died during initialization" << std::endl;
                }
                ::close(fd);
                return false;
            }
            continue;
        }

        int want = std::min(num_workers - total_read, (int)sizeof(buf));
        ssize_t n = ::read(fd, buf, want);
        if (n > 0) {
            total_read += n;
        } else if (n == 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        } else if (errno != EAGAIN && errno != EINTR) {
            ::close(fd);
            return false;
        }
    }

    ::close(fd);

    if (config_.verbosity >= 2) {
        std::cout << "    All " << num_workers << " TrafficGen workers ready (init complete)" << std::endl;
    }

    return true;
}

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
    stop_process_tree(pid, std::chrono::milliseconds(500));
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
    cleanup_zombie_traffic_gen();
    current_traffic_gen_pid_ = 0;
    active_traffic_gen_pid_ = 0;
    persistent_traffic_gen_valid_ = false;
    if (!ready_fifo_path_.empty()) {
        ::unlink(ready_fifo_path_.c_str());
    }
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
    return launch_traffic_gen_impl(ratio_pct, pause, traffic_gen_cores, traffic_gen_mem_nodes,
                                                traffic_gen_log_file, traffic_gen_pid);

}


bool TrafficGenProcessManager::launch_traffic_gen_impl(double ratio_pct,
                                                 int pause,
                                                 int traffic_gen_cores,
                                                 const std::vector<int>& traffic_gen_mem_nodes,
                                                 const std::string& traffic_gen_log_file,
                                                 pid_t& traffic_gen_pid
                                                ) const {
    std::string pid_filename = "/tmp/traffic_gen_pid_" + std::to_string(getpid()) + "_" +
                               std::to_string(static_cast<int>(ratio_pct)) + "_" +
                               std::to_string(pause) + ".tmp";

    std::vector<std::string> cores;
    if (!resolve_core_list(traffic_gen_cores, config_.traffic_gen_explicit_cores, cores)) {
        std::cerr << "ERROR: Failed to select core list" << std::endl;
        return false;
    }

    expected_ready_count_ = static_cast<int>(cores.size());

    const auto& cache = SystemToolsCache::instance();
    bool have_taskset = cache.have_taskset;
    bool have_numactl = cache.have_numactl;

    std::string bin_name = "traffic_gen_multiseq.x";
    std::string mode_label = "MultiSeq";

    std::string worker_executable;
    std::filesystem::path root = get_project_root();
    std::filesystem::path build_bin = root / "build/bin";
    std::filesystem::path binary_path = build_bin / bin_name;

    if (std::filesystem::exists(binary_path)) {
        worker_executable = binary_path.string();
    } else {
        worker_executable = "./" + bin_name;
    }
    
    std::vector<std::string> worker_cmds;
    
    const std::map<int, CpuTopology>& cpu_topo = cache.cpu_topology;
    std::map<int, std::set<int>> socket_nodes_map = get_socket_to_nodes_map(cpu_topo);
    (void)socket_nodes_map;
    
    std::string explicit_membind_str;
    if (!traffic_gen_mem_nodes.empty()) {
        for (size_t i = 0; i < traffic_gen_mem_nodes.size(); ++i) {
            explicit_membind_str += std::to_string(traffic_gen_mem_nodes[i]);
            if (i < traffic_gen_mem_nodes.size() - 1) explicit_membind_str += ",";
        }
    }

    std::map<std::string, std::vector<int>> node_to_cores;


    if (!ready_fifo_path_.empty()) {
        ::unlink(ready_fifo_path_.c_str());
        if (::mkfifo(ready_fifo_path_.c_str(), 0666) != 0 && errno != EEXIST) {
            std::cerr << "Warning: Failed to create ready FIFO " << ready_fifo_path_
                      << ": " << std::strerror(errno) << std::endl;
        }
    }

    std::string worker_args_base = " -r " + std::to_string(static_cast<int>(ratio_pct)) +
                                   " -p " + std::to_string(pause) +
                                   " -w " + std::to_string(cores.size());

    if (config_.verbosity >= 3) {
        worker_args_base += " -v 1";
    }


    if (!ready_fifo_path_.empty()) {
        worker_args_base += " -f " + ready_fifo_path_;
    }

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
                membind_arg = "0";
            }
        }
        
        if (core_id != -1) {
            node_to_cores[membind_arg].push_back(core_id);
        }
        
        const std::string& worker_args = worker_args_base;

        std::string cmd;
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
        std::cout << "    Launching TrafficGen (" << mode_label << " - " << cores.size() << " procs)";
        if (config_.verbosity < 3) std::cout << '\n';
        
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
            std::cout << '\n';
            size_t preview = std::min<size_t>(worker_cmds.size(), 5);
            for (size_t i = 0; i < preview; ++i) {
                std::cout << "      cmd[" << i << "]: " << worker_cmds[i] << '\n';
            }
            if (worker_cmds.size() > preview) {
                std::cout << "      ... (" << (worker_cmds.size() - preview)
                          << " more)" << '\n';
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

    expected_ready_count_ = traffic_gen_cores;

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
    

    if (!ready_fifo_path_.empty()) {
        ::unlink(ready_fifo_path_.c_str());
        if (::mkfifo(ready_fifo_path_.c_str(), 0666) != 0 && errno != EEXIST) {
            std::cerr << "Warning: Failed to create ready FIFO " << ready_fifo_path_
                      << ": " << std::strerror(errno) << std::endl;
        }
        worker_args += " -f " + ready_fifo_path_;
    }
    
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
        if (config_.verbosity < 3) std::cout << '\n';
        
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
            std::cout << '\n';
            std::cout << "      - Method: " << (use_srun ? "srun (SLURM)" : "mpirun") << '\n';
            std::cout << "      - Cores: " << core_list << '\n';
            std::cout << "      - " << launcher << '\n';
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
    std::vector<pid_t> targets;
    if (active_traffic_gen_pid_ > 0) {
        targets.push_back(active_traffic_gen_pid_);
    }
    if (current_traffic_gen_pid_ > 0 && current_traffic_gen_pid_ != active_traffic_gen_pid_) {
        targets.push_back(current_traffic_gen_pid_);
    }
    auto scanned = list_traffic_gen_pids();
    targets.insert(targets.end(), scanned.begin(), scanned.end());
    std::sort(targets.begin(), targets.end());
    targets.erase(std::unique(targets.begin(), targets.end()), targets.end());

    if (targets.empty()) {
        return true;
    }

    bool all_stopped = true;
    for (pid_t pid : targets) {
        if (!stop_process_tree(pid, std::chrono::milliseconds(500))) {
            all_stopped = false;
        }
    }
    if (all_stopped && list_traffic_gen_pids().empty()) {
        return true;
    }

    // Last-resort compatibility fallback for systems where process groups don't tear down cleanly.
    if (run_command_success("command -v pkill >/dev/null 2>&1")) {
        const std::string uid = std::to_string(getuid());
        const std::string term_multiseq = "pkill -u " + uid + " -TERM -x traffic_gen_multiseq.x >/dev/null 2>&1";
        const std::string term_rand = "pkill -u " + uid + " -TERM -x traffic_gen_rand.x >/dev/null 2>&1";
        const std::string kill_multiseq = "pkill -u " + uid + " -KILL -x traffic_gen_multiseq.x >/dev/null 2>&1";
        const std::string kill_rand = "pkill -u " + uid + " -KILL -x traffic_gen_rand.x >/dev/null 2>&1";

        system(term_multiseq.c_str());
        system(term_rand.c_str());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        system(kill_multiseq.c_str());
        system(kill_rand.c_str());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    auto survivors = list_traffic_gen_pids();
    for (pid_t pid : survivors) {
        if (!stop_process_tree(pid, std::chrono::milliseconds(200))) {
            return false;
        }
    }
    return list_traffic_gen_pids().empty();
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
