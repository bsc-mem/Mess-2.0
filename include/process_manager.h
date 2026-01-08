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

#ifndef PROCESS_MANAGER_H
#define PROCESS_MANAGER_H

#include "benchmark_config.h"
#include "system_info.h"
#include "ptrchase_perf_helper.h"
#include "utils.h"

#include <semaphore.h>
#include <string>
#include <sys/types.h>
#include <vector>

class PtrChaseProcessManager {
public:
    PtrChaseProcessManager(const BenchmarkConfig& config, const system_info& sys_info);
    ~PtrChaseProcessManager();

    bool ensure_running();
    bool wait_for_ready(int timeout_seconds = 30);
    void cleanup();

    void cleanup_perf_counters();

    bool ensure_semaphores();
    void drain_done_sem();
    void drain_start_sem();
    bool wait_for_done(int timeout_ms);

    pid_t pid() const { return ptr_chase_pid_; }
    sem_t* start_sem() const { return ptrchase_start_sem_; }
    sem_t* done_sem() const { return ptrchase_done_sem_; }
    ptrchase_perf::PtrchasePerfFDs& perf_fds() { return ptrchase_perf_fds_; }
    const ptrchase_perf::PtrchasePerfFDs& perf_fds() const { return ptrchase_perf_fds_; }
    const std::string& pipe_path() const { return pipe_path_; }

private:
    const BenchmarkConfig& config_;
    const system_info& sys_info_;

    pid_t ptr_chase_pid_;
    sem_t* ptrchase_start_sem_;
    sem_t* ptrchase_done_sem_;
    ptrchase_perf::PtrchasePerfFDs ptrchase_perf_fds_;
    bool perf_initialized_;

    std::string start_sem_name_;
    std::string done_sem_name_;
    std::string ready_flag_path_;
    std::string pipe_path_;
    std::string unique_id_;

    void generate_unique_names();
    bool launch_process();
};

struct TrafficGenPreparation {
    pid_t pid = 0;
    bool reused_existing = false;
    long long prev_cas_rd = 0;
    long long prev_cas_wr = 0;
    double prev_elapsed = 0.0;
    double prev_bw_gb_s = 0.0;
};

class TrafficGenProcessManager {
public:
    TrafficGenProcessManager(const BenchmarkConfig& config, const system_info& sys_info);
    ~TrafficGenProcessManager() = default;

    bool prepare_traffic_gen(double ratio_pct,
                           int pause,
                           int traffic_gen_cores,
                           const std::vector<int>& traffic_gen_mem_nodes,
                           const std::string& traffic_gen_log_file,
                           TrafficGenPreparation& prep,
                           ExecutionMode mode = ExecutionMode::MULTISEQUENTIAL);

    bool relaunch_traffic_gen(double ratio_pct,
                         int pause,
                         int traffic_gen_cores,
                         const std::vector<int>& traffic_gen_mem_nodes,
                         const std::string& traffic_gen_log_file,
                         ExecutionMode mode = ExecutionMode::MULTISEQUENTIAL);

    bool is_traffic_gen_running(pid_t pid) const;
    void terminate_traffic_gen(pid_t pid);
    void kill_all_traffic_gen();

    void stash_bandwidth_retry(pid_t traffic_gen_pid,
                               long long cas_rd,
                               long long cas_wr,
                               double elapsed,
                               double bw_gb_s);
    bool consume_bandwidth_retry(long long& cas_rd,
                                 long long& cas_wr,
                                 double& elapsed,
                                 double& bw_gb_s);

    pid_t current_traffic_gen_pid() const { return current_traffic_gen_pid_; }
    pid_t active_traffic_gen_pid() const { return active_traffic_gen_pid_; }

private:
    const BenchmarkConfig& config_;


    bool keep_traffic_gen_on_retry_;
    long long retry_bw_cas_rd_;
    long long retry_bw_cas_wr_;
    double retry_bw_elapsed_;
    double retry_bw_gb_s_;
    pid_t current_traffic_gen_pid_;
    pid_t active_traffic_gen_pid_;

    bool persistent_traffic_gen_valid_;
    double persistent_ratio_pct_;
    int persistent_pause_;
    ExecutionMode persistent_mode_;
    int persistent_traffic_gen_cores_;
    std::vector<int> persistent_traffic_gen_mem_nodes_;

    bool launch_traffic_gen_multiseq(double ratio_pct, int pause, int cores, const std::vector<int>& mem_nodes, const std::string& log_file, pid_t& out_pid) const;
    
#if USE_MPI_SPAWN
    bool launch_traffic_gen_mpi(double ratio_pct, int pause, int cores, const std::vector<int>& mem_nodes, const std::string& log_file, pid_t& out_pid, ExecutionMode mode);
#endif
    bool read_traffic_gen_pid(const std::string& pid_filename, pid_t& traffic_gen_pid) const;
    bool cleanup_zombie_traffic_gen() const;
    std::string get_membind_arg(int default_node) const;

    mutable std::vector<std::string> cached_core_list_;
    mutable int cached_traffic_gen_cores_;
    mutable std::vector<std::string> cached_explicit_cores_;
    
    // Helper to get cores with caching
    bool resolve_core_list(int traffic_gen_cores, const std::vector<std::string>& explicit_cores, std::vector<std::string>& out_cores) const;
};

#endif
