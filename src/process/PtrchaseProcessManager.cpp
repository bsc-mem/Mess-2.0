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

#include "ProcessManager.h"
#include "Utils.h"

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <fcntl.h>
#include <unistd.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <cstring>

namespace {

bool IsProcessRunning(pid_t pid) {
    if (pid <= 0) return false;
    
    int status;
    int result = waitpid(pid, &status, WNOHANG);
    if (result == pid) {
        return false;
    }
    
    if (result == 0) {
        return true;
    }
    
    return (::kill(pid, 0) == 0);
}

bool WaitForProcessExit(pid_t pid, std::chrono::milliseconds timeout) {
    if (pid <= 0) return true;
    auto deadline = std::chrono::steady_clock::now() + timeout;
    while (true) {
        if (::kill(pid, 0) == -1 && errno == ESRCH) {
            return true;
        }
        if (std::chrono::steady_clock::now() >= deadline) {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

}

PtrChaseProcessManager::PtrChaseProcessManager(const BenchmarkConfig& config,
                                               const system_info& sys_info)
    : config_(config),
      ptr_chase_pid_(0),
      ptrchase_start_sem_(nullptr),
      ptrchase_done_sem_(nullptr) {
    (void)sys_info;
    ptrchase_perf_fds_.fd_cycles = -1;
    ptrchase_perf_fds_.fd_insts = -1;
    ptrchase_perf_fds_.fd_tlb1 = -1;
    ptrchase_perf_fds_.fd_tlb2 = -1;
    generate_unique_names();
}

void PtrChaseProcessManager::generate_unique_names() {
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
    
    start_sem_name_ = "/mess_pc_start_" + unique_id_;
    done_sem_name_ = "/mess_pc_done_" + unique_id_;
    ready_flag_path_ = "/tmp/ptr_chase_ready_" + unique_id_ + ".flag";
    pipe_path_ = "/tmp/mess_ptrchase_pipe_" + unique_id_;
}

PtrChaseProcessManager::~PtrChaseProcessManager() {
    cleanup();
}

bool PtrChaseProcessManager::launch_process() {
    const auto& cache = SystemToolsCache::instance();

    // Determine which CPU and NUMA node to bind ptr_chase to.
    // When no explicit binding is configured, auto-detect a compute core
    // whose NUMA node has memory (avoids assistant cores on e.g. A64FX).
    auto [ptr_chase_cpu, auto_mem_node] = cache.initialized
        ? std::make_pair(cache.ptr_chase_cpu, cache.ptr_chase_node)
        : find_ptr_chase_cpu();
    std::string ptr_chase_cpu_str = std::to_string(ptr_chase_cpu);

    std::string mem_node_str;
    if (!config_.memory_bind_nodes.empty()) {
        for (size_t i = 0; i < config_.memory_bind_nodes.size(); ++i) {
            mem_node_str += std::to_string(config_.memory_bind_nodes[i]);
            if (i < config_.memory_bind_nodes.size() - 1) mem_node_str += ",";
        }
    } else {
        mem_node_str = std::to_string(auto_mem_node);
    }

    if (ptrchase_start_sem_) {
        sem_close(ptrchase_start_sem_);
        ptrchase_start_sem_ = nullptr;
    }
    if (ptrchase_done_sem_) {
        sem_close(ptrchase_done_sem_);
        ptrchase_done_sem_ = nullptr;
    }

    std::remove(ready_flag_path_.c_str());
    std::remove(pipe_path_.c_str());

    sem_unlink(start_sem_name_.c_str());
    sem_unlink(done_sem_name_.c_str());

    if (mkfifo(pipe_path_.c_str(), 0666) != 0 && errno != EEXIST) {
        std::cerr << "ERROR: Failed to create pipe " << pipe_path_ << ": " << std::strerror(errno) << std::endl;
        return false;
    }

    std::string log_path = "/tmp/ptr_chase_" + unique_id_ + ".log";
    std::string cmd_prefix;
    std::string ptr_chase_bin;
    std::string working_dir;

    std::filesystem::path root = get_project_root();

    // Search for ptr_chase binary in multiple locations
    std::vector<std::filesystem::path> binary_search_paths = {
        root / "build" / "bin" / "ptr_chase",
        root / "bin" / "ptr_chase",
        root / "ptr_chase",
        std::filesystem::current_path() / "ptr_chase",
        std::filesystem::path("./ptr_chase")
    };

    std::filesystem::path binary_path;
    for (const auto& p : binary_search_paths) {
        if (std::filesystem::exists(p)) {
            binary_path = p;
            break;
        }
    }

    // Search for array.dat in multiple locations
    std::vector<std::filesystem::path> array_search_paths = {
        root / "build" / "bin" / "array.dat",
        root / "build" / "array.dat",
        root / "bin" / "array.dat",
        root / "array.dat",
        std::filesystem::current_path() / "array.dat"
    };

    std::filesystem::path array_dat_path;
    for (const auto& p : array_search_paths) {
        if (std::filesystem::exists(p)) {
            array_dat_path = p;
            break;
        }
    }

    if (!binary_path.empty()) {
        ptr_chase_bin = binary_path.string();
        
        if (!array_dat_path.empty()) {
            // Use the directory containing array.dat as working directory
            working_dir = array_dat_path.parent_path().string();
            cmd_prefix = "cd " + working_dir + " && ";
            
            // Use relative path to binary if it's in or below the working directory
            std::filesystem::path rel_binary = std::filesystem::relative(binary_path, array_dat_path.parent_path());
            if (!rel_binary.empty() && rel_binary.string().compare(0, 2, "..") != 0) {
                ptr_chase_bin = "./" + rel_binary.string();
            }
        } else {
            // No array.dat found - use binary directory as working dir
            working_dir = binary_path.parent_path().string();
            cmd_prefix = "cd " + working_dir + " && ";
            ptr_chase_bin = "./" + binary_path.filename().string();
        }
    } else {
        ptr_chase_bin = "ptr_chase";
        working_dir = std::filesystem::current_path().string();
    }

    bool have_taskset = cache.have_taskset;
    bool have_numactl = cache.have_numactl;

    std::string binding_cmd;
    if (have_numactl && have_taskset) {
        binding_cmd = "numactl --membind=" + mem_node_str + " taskset -c " + ptr_chase_cpu_str + " " + ptr_chase_bin;
    } else if (have_numactl) {
        binding_cmd = "numactl --membind=" + mem_node_str + " --physcpubind=" + ptr_chase_cpu_str + " " + ptr_chase_bin;
    } else if (have_taskset) {
        binding_cmd = "taskset -c " + ptr_chase_cpu_str + " " + ptr_chase_bin;
        if (config_.verbosity >= 2) {
            std::cerr << "  [NOTICE] numactl not found, using taskset only (no memory binding)" << std::endl;
        }
    } else {
        binding_cmd = ptr_chase_bin;
        if (config_.verbosity >= 1) {
            std::cerr << "  [WARNING] Neither numactl nor taskset found. ptr_chase will run without CPU/memory binding." << std::endl;
        }
    }
    
    std::string ptr_chase_cmd = "{ " + cmd_prefix + binding_cmd + " >> " + log_path + " 2>&1 & echo $!; } 2>/dev/null";

    FILE* ptr_pipe = popen(ptr_chase_cmd.c_str(), "r");
    if (!ptr_pipe) {
        std::cerr << "ERROR: Failed to launch persistent ptr_chase" << std::endl;
        return false;
    }

    char ptr_pid_str[32];
    if (fgets(ptr_pid_str, sizeof(ptr_pid_str), ptr_pipe) != nullptr) {
        ptr_chase_pid_ = atoi(ptr_pid_str);
    }
    pclose_success(ptr_pipe);

    if (ptr_chase_pid_ <= 0) {
        std::cerr << "ERROR: Failed to get ptr_chase PID" << std::endl;
        return false;
    }

    if (config_.verbosity >= 3) {
        std::cout << "PointerChase settings:" << std::endl;
        std::cout << "    Launched ptr_chase process (PID: " << ptr_chase_pid_ << ")" << std::endl;
        std::cout << "    Session ID: " << unique_id_ << std::endl;
        std::cout << "    Pipe: " << pipe_path_ << std::endl;
        std::cout << "    Log: " << log_path << std::endl;
    }

    return true;
}

/**
 * @brief Ensures the pointer-chaser worker process is actively running and ready.
 * 
 * In scenarios where pointer chasing is used for latency measurement, the `ptr_chase` binary
 * runs as a separate process. This function verifies if it is already alive; if not, it cleans
 * up any stale state, launches a new instance, waits for it to signal readiness via a flag file,
 * and initializes the POSIX semaphores used to synchronize traffic generation and latency probing.
 * 
 * @return bool True if the process is running and synchronized; false on failure.
 */
bool PtrChaseProcessManager::ensure_running() {
  
    if (IsProcessRunning(ptr_chase_pid_)) {
        return true;
    }

    cleanup();

    if (!launch_process()) {
        return false;
    }


    if (!wait_for_ready(10)) {
        std::cerr << "ERROR: ptr_chase did not signal ready within timeout" << std::endl;
        if (ptr_chase_pid_ > 0) {
            kill(ptr_chase_pid_, SIGKILL);
            waitpid(ptr_chase_pid_, nullptr, WNOHANG);
            ptr_chase_pid_ = 0;
        }
        return false;
    }

    if (!ensure_semaphores()) {
        std::cerr << "ERROR: Failed to open ptr_chase semaphores" << std::endl;
        cleanup();
        return false;
    }


    return true;
}

bool PtrChaseProcessManager::wait_for_ready(int timeout_seconds) {
    auto start_time = std::chrono::steady_clock::now();
    const std::string& ready_flag = ready_flag_path_;

    // Give the child process a moment to start and create the file
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    while (true) {
        // Use open/read instead of filesystem::exists to avoid caching issues
        int fd = open(ready_flag.c_str(), O_RDONLY);
        if (fd != -1) {
            char buf[32];
            ssize_t n = read(fd, buf, sizeof(buf) - 1);
            close(fd);
            
            if (n > 0) {
                buf[n] = '\0';
                pid_t real_pid = atoi(buf);
                
                if (real_pid > 0) {
                    if (real_pid != ptr_chase_pid_) {
                        ptr_chase_pid_ = real_pid;
                    }
                    return true;
                }
            }
        }

        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time);
        if (elapsed.count() >= timeout_seconds) {
            if (config_.verbosity >= 1) {
                std::string log_path = "/tmp/ptr_chase_" + unique_id_ + ".log";
                std::ifstream log_file(log_path);
                if (log_file.is_open()) {
                    std::string line;
                    std::cerr << "  [DEBUG] ptr_chase log contents:" << std::endl;
                    while (std::getline(log_file, line)) {
                        std::cerr << "    " << line << std::endl;
                    }
                }
            }
            return false;
        }

        if (!IsProcessRunning(ptr_chase_pid_)) {
            if (config_.verbosity >= 1) {
                std::cerr << "  [DEBUG] ptr_chase process (" << ptr_chase_pid_ << ") died unexpectedly" << std::endl;
                
                int status;
                if (waitpid(ptr_chase_pid_, &status, WNOHANG) == ptr_chase_pid_) {
                    if (WIFSIGNALED(status)) {
                        std::cerr << "  [DEBUG] ptr_chase killed by signal " << WTERMSIG(status) 
                                  << " (" << strsignal(WTERMSIG(status)) << ")" << std::endl;
                    } else if (WIFEXITED(status)) {
                        std::cerr << "  [DEBUG] ptr_chase exited with code " << WEXITSTATUS(status) << std::endl;
                    }
                }

                std::string log_path = "/tmp/ptr_chase_" + unique_id_ + ".log";
                std::cerr << "  [DEBUG] Checking log file: " << log_path << std::endl;
                std::ifstream log_file(log_path);
                if (log_file.is_open()) {
                    std::string line;
                    bool has_content = false;
                    while (std::getline(log_file, line)) {
                        if (!has_content) {
                            std::cerr << "  [DEBUG] ptr_chase log contents:" << std::endl;
                            has_content = true;
                        }
                        std::cerr << "    " << line << std::endl;
                    }
                    if (!has_content) {
                        std::cerr << "  [DEBUG] Log file is empty" << std::endl;
                    }
                } else {
                    std::cerr << "  [DEBUG] Log file does not exist or cannot be opened" << std::endl;
                    std::cerr << "  [DEBUG] Checking old hardcoded path /tmp/ptr_chase.log..." << std::endl;
                    std::ifstream old_log("/tmp/ptr_chase.log");
                    if (old_log.is_open()) {
                        std::cerr << "  [DEBUG] WARNING: Old /tmp/ptr_chase.log exists - ptr_chase binary may not have been regenerated!" << std::endl;
                    }
                }
            }
            return false;
        }

        // Check more frequently to reduce race window
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void PtrChaseProcessManager::cleanup() {
    
    if (ptr_chase_pid_ > 0) {
        kill(ptr_chase_pid_, SIGTERM);
        bool exited = WaitForProcessExit(ptr_chase_pid_, std::chrono::seconds(3));
        if (!exited) {
            kill(ptr_chase_pid_, SIGKILL);
            WaitForProcessExit(ptr_chase_pid_, std::chrono::seconds(1));
        }
        ptr_chase_pid_ = 0;
    }

    cleanup_perf_counters();

    if (ptrchase_start_sem_) {
        sem_close(ptrchase_start_sem_);
        ptrchase_start_sem_ = nullptr;
    }
    if (ptrchase_done_sem_) {
        sem_close(ptrchase_done_sem_);
        ptrchase_done_sem_ = nullptr;
    }

    if (!start_sem_name_.empty()) {
        sem_unlink(start_sem_name_.c_str());
    }
    if (!done_sem_name_.empty()) {
        sem_unlink(done_sem_name_.c_str());
    }

    std::remove(ready_flag_path_.c_str());
    std::remove(pipe_path_.c_str());
}


void PtrChaseProcessManager::cleanup_perf_counters() {
    ptrchase_perf::cleanup_ptrchase_perf_counters(ptrchase_perf_fds_);
}

bool PtrChaseProcessManager::ensure_semaphores() {
    if (ptrchase_start_sem_ && ptrchase_done_sem_) {
        return true;
    }

    const int max_attempts = 50;
    for (int attempt = 0; attempt < max_attempts; ++attempt) {
        if (!ptrchase_start_sem_) {
            ptrchase_start_sem_ = sem_open(start_sem_name_.c_str(), O_CREAT, 0666, 0);
            if (ptrchase_start_sem_ == SEM_FAILED) {
                ptrchase_start_sem_ = nullptr;
            }
        }
        if (!ptrchase_done_sem_) {
            ptrchase_done_sem_ = sem_open(done_sem_name_.c_str(), O_CREAT, 0666, 0);
            if (ptrchase_done_sem_ == SEM_FAILED) {
                ptrchase_done_sem_ = nullptr;
            }
        }
        if (ptrchase_start_sem_ && ptrchase_done_sem_) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (!ptrchase_start_sem_ || !ptrchase_done_sem_) {
        std::cerr << "ERROR: Unable to open ptr_chase semaphores" << std::endl;
        return false;
    }

    drain_done_sem();
    drain_start_sem();

    return true;
}

void PtrChaseProcessManager::drain_start_sem() {
    if (!ptrchase_start_sem_) {
        return;
    }
    while (sem_trywait(ptrchase_start_sem_) == 0) {
    }
}

void PtrChaseProcessManager::drain_done_sem() {
    if (!ptrchase_done_sem_) {
        return;
    }
    while (sem_trywait(ptrchase_done_sem_) == 0) {
    }
}

bool PtrChaseProcessManager::wait_for_done(int timeout_ms) {
    if (!ptrchase_done_sem_) {
        return false;
    }
    auto start = std::chrono::steady_clock::now();
    while (true) {
        if (sem_trywait(ptrchase_done_sem_) == 0) {
            return true;
        }
        if (errno != EAGAIN && errno != EINTR) {
            return false;
        }
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start).count();
        if (elapsed_ms >= timeout_ms) {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
