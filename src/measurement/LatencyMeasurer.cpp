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
#include "Measurement.h"
#include <algorithm>
#ifdef __linux__
#include <asm/unistd.h>
#endif
#include <chrono>

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cctype>
#include <cstring>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#ifdef __linux__
#include <linux/perf_event.h>
#endif
#include <semaphore.h>
#include <sstream>
#include <sys/ioctl.h>
#include <thread>
#include <unistd.h>

LatencyMeasurer::LatencyMeasurer(const BenchmarkConfig& config, MeasurementStorage* storage, PtrChaseProcessManager* ptrchase_manager, std::function<std::vector<int>()> numa_resolver, int cache_line_size)
    : config_(config),
      ptrchase_manager_(ptrchase_manager),
      numa_resolver_(std::move(numa_resolver)),
      ptrchase_accesses_per_burst_(0.0),
      ptrchase_insts_per_access_(0.0) {
    (void)storage;
    (void)cache_line_size;
    //init_ptrchase_scaling();
}

void LatencyMeasurer::init_ptrchase_scaling() {
    if (ptrchase_accesses_per_burst_ > 0 && ptrchase_insts_per_access_ > 0) {
        return;
    }

    auto parse_value = [](const std::string& token, const std::string& line) -> double {
        size_t pos = line.find(token);
        if (pos == std::string::npos) {
            return 0.0;
        }
        pos = line.find('=', pos);
        if (pos == std::string::npos) {
            return 0.0;
        }
        std::string value_str = line.substr(pos + 1);
        value_str.erase(std::remove_if(value_str.begin(), value_str.end(),
                                       [](unsigned char c) { return std::isspace(c) != 0; }),
                        value_str.end());
        char* end_ptr = nullptr;
        double parsed = std::strtod(value_str.c_str(), &end_ptr);
        return (end_ptr != value_str.c_str()) ? parsed : 0.0;
    };

    double total_instructions = 0.0;
    double iterations = 0.0;
    double burst_iterations = 0.0;

    if (const char* env_instr = std::getenv("PTRCHASE_NUM_INSTRUCTIONS")) {
        total_instructions = std::strtod(env_instr, nullptr);
    }
    if (const char* env_iters = std::getenv("PTRCHASE_NUM_ITERATIONS")) {
        iterations = std::strtod(env_iters, nullptr);
    }
    if (const char* env_burst = std::getenv("PTRCHASE_BURST_ITERATIONS")) {
        burst_iterations = std::strtod(env_burst, nullptr);
    }

    if (total_instructions <= 0.0 || iterations <= 0.0 || burst_iterations <= 0.0) {
        std::filesystem::path root = get_project_root();
        std::filesystem::path makefile_path = root / "build/lib/ptr_chase_src/Makefile";
        std::ifstream mf(makefile_path);
        if (mf.is_open()) {
            std::string line;
            while (std::getline(mf, line)) {
                if (line.find("instructions") != std::string::npos &&
                    total_instructions <= 0.0) {
                    total_instructions = parse_value("instructions", line);
                } else if (line.find("iterations") != std::string::npos &&
                           iterations <= 0.0) {
                    iterations = parse_value("iterations", line);
                } else if (line.find("burst_iterations") != std::string::npos &&
                           burst_iterations <= 0.0) {
                    burst_iterations = parse_value("burst_iterations", line);
                }
            }
            mf.close();
        }
    }

    if (iterations <= 0.0) {
        iterations = 1.0;
    }
    if (total_instructions <= 0.0) {
        total_instructions = iterations * 10000.0;
    }
    if (burst_iterations <= 0.0) {
        burst_iterations = 1.0;
    }

    ptrchase_burst_iters_ = static_cast<uint64_t>(burst_iterations);

    const char* insts_env = std::getenv("PTRCHASE_NUM_INSTRUCTIONS");
    const char* iters_env = std::getenv("PTRCHASE_NUM_ITERATIONS");

    if (insts_env && iters_env) {
        ptrchase_insts_per_iter_ = std::stoull(insts_env) / std::stoull(iters_env);
    } else {
        ptrchase_insts_per_iter_ = (iterations > 0.0) ? static_cast<uint64_t>(total_instructions / iterations) : 100000; // Default
    }
    if (ptrchase_insts_per_iter_ == 0) { 
        ptrchase_insts_per_iter_ = 100000;
    }
    
    ptrchase_accesses_per_burst_ = 
        (double)ptrchase_burst_iters_ * ptrchase_insts_per_iter_;


    ptrchase_insts_per_access_ = static_cast<double>(ptrchase_insts_per_iter_);
    if (ptrchase_insts_per_access_ <= 0.0) {
        ptrchase_insts_per_access_ = 5000.0; 
    }

}

bool LatencyMeasurer::start_burst_async() {
    if (burst_running_) {
        return false;
    }
    
    if (ptrchase_manager_->pid() <= 0) {
        return false;
    }
    
    if (!ptrchase_manager_->ensure_semaphores()) {
        return false;
    }

    ptrchase_manager_->drain_done_sem();

    pipe_fd_ = open(ptrchase_manager_->pipe_path().c_str(), O_RDONLY | O_NONBLOCK);
    if (pipe_fd_ == -1) {
        if (config_.verbosity >= 2) {
            std::cerr << "    [BURST] Failed to open pipe async (" << ptrchase_manager_->pipe_path() << "): " << std::strerror(errno) << std::endl;
        }
        return false;
    }

    if (sem_post(ptrchase_manager_->start_sem()) == -1) {
        close(pipe_fd_);
        pipe_fd_ = -1;
        return false;
    }

    burst_start_time_ = std::chrono::steady_clock::now();
    burst_running_ = true;
    
    return true;
}

bool LatencyMeasurer::try_collect_burst_async(PerfBurstCounters& out) {
    if (!burst_running_) {
        return false;
    }

    int sem_res = sem_trywait(ptrchase_manager_->done_sem());
    if (sem_res == 0) {
        int flags = fcntl(pipe_fd_, F_GETFL);
        if (flags != -1) {
            fcntl(pipe_fd_, F_SETFL, flags & ~O_NONBLOCK);
        }

        char buffer[1024];
        memset(buffer, 0, sizeof(buffer));
        ssize_t bytes_read = read(pipe_fd_, buffer, sizeof(buffer) - 1);
        
        close(pipe_fd_);
        pipe_fd_ = -1;
        burst_running_ = false;

        if (bytes_read > 0) {
            std::string line(buffer);
            auto parse_val = [&](const std::string& key) -> long long {
                size_t pos = line.find(key + "=");
                if (pos != std::string::npos) {
                    try {
                        return std::stoll(line.substr(pos + key.length() + 1));
                    } catch (...) { return 0; }
                }
                return 0;
            };

            out.cycles = static_cast<double>(parse_val("cycles"));
            out.instructions = static_cast<double>(parse_val("instructions"));
            out.tlb1miss = static_cast<double>(parse_val("tlb1"));
            out.tlb2miss = static_cast<double>(parse_val("tlb2"));
            out.using_hugepages = (parse_val("hugepages") != 0);
            long long duration_ns = parse_val("duration_ns");

            if (duration_ns > 0) {
                out.duration_s = static_cast<double>(duration_ns) / 1e9;
            } else {
                auto burst_end = std::chrono::steady_clock::now();
                std::chrono::duration<double> diff = burst_end - burst_start_time_;
                out.duration_s = diff.count();
            }
            
            out.tlb1_event_name = expected_tlb1_event_;
            out.tlb2_event_name = expected_tlb2_event_;
            
            return true;
        } else {
             return false;
        }
    }
    return false;
}

void LatencyMeasurer::reset() {
    if (pipe_fd_ != -1) {
        close(pipe_fd_);
        pipe_fd_ = -1;
    }
    burst_running_ = false;
}
