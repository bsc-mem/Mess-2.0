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
#include "measurement.h"
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
      storage_(storage),
      ptrchase_manager_(ptrchase_manager),
      numa_resolver_(std::move(numa_resolver)),
      cache_line_size_(cache_line_size),
      ptrchase_accesses_per_burst_(0.0),
      ptrchase_insts_per_access_(0.0) {
        //init_ptrchase_scaling();
}

bool LatencyMeasurer::measure_latency_with_bursts(int burst_count, PerfBurstCounters& totals, double& total_accesses) {
    if (burst_count <= 0) {
        if (config_.verbosity >= 2) {
            std::cerr << "    [BURST] Invalid burst_count: " << burst_count << std::endl;
        }
        return false;
    }
    init_ptrchase_scaling();
    if (ptrchase_manager_->pid() <= 0 || ptrchase_accesses_per_burst_ <= 0.0) {
        if (config_.verbosity >= 2) {
            std::cerr << "    [BURST] Invalid ptr_chase state before bursts (pid="
                      << ptrchase_manager_->pid() << ", accesses_per_burst="
                      << ptrchase_accesses_per_burst_ << ")" << std::endl;
        }
        return false;
    }
    if (!ptrchase_manager_->ensure_semaphores()) {
        if (config_.verbosity >= 2) {
            std::cerr << "    [BURST] ensure_ptrchase_semaphores() failed in measure_latency_with_bursts" << std::endl;
        }
        return false;
    }

    // Initialize the output totals struct
    totals = {0, 0, 0, 0, 0, false, "", ""};
    total_accesses = 0.0;

    const int warmup_bursts = 2;
    int total_bursts = warmup_bursts + burst_count;

    for (int burst = 0; burst < total_bursts; ++burst) {
        PerfBurstCounters sample{0, 0, 0, 0, 0, false, "", ""};
        if (!run_single_ptrchase_burst(sample)) {
            if (config_.verbosity >= 2) {
                std::cerr << "    [BURST] run_single_ptrchase_burst() failed for burst "
                          << (burst + 1) << "/" << total_bursts << std::endl;
            }
            return false;
        }

        if (config_.verbosity >= 3) {
            double burst_latency_ns = 0.0;
            if (ptrchase_accesses_per_burst_ > 0.0 && sample.duration_s > 0.0) {
                double cpu_freq_hz = sample.cycles / sample.duration_s;
                double cpu_freq_ghz = cpu_freq_hz / 1e9;
                
                double pagewalk_ns = (cpu_freq_ghz > 0.0) ? (sample.tlb2miss / cpu_freq_ghz) : 0.0;
                double stlb_ns = tlb_hit_latency_ns_ * sample.tlb1miss;
                
                double total_ns = sample.duration_s * 1e9;
                double effective_ns = total_ns - pagewalk_ns - stlb_ns;
                
                if (effective_ns > 0.0) {
                    burst_latency_ns = effective_ns / ptrchase_accesses_per_burst_;
                }
            }

            double accesses_per_s =
                (sample.duration_s > 0.0 && ptrchase_accesses_per_burst_ > 0.0)
                    ? (ptrchase_accesses_per_burst_ / sample.duration_s)
                    : 0.0;
            double cpi = (sample.instructions > 0.0)
                             ? (sample.cycles / sample.instructions)
                             : 0.0;

            std::cout << "      [Burst " << (burst + 1) << "/" << total_bursts << "] "
                      << "cycles: " << std::fixed << std::setprecision(0) << sample.cycles
                      << " | instr: " << std::fixed << std::setprecision(0) << sample.instructions
                      << " | tlb1: " << std::fixed << std::setprecision(0) << sample.tlb1miss
                      << " | tlb2: " << std::fixed << std::setprecision(0) << sample.tlb2miss
                      << " | dur: " << std::fixed << std::setprecision(3) << sample.duration_s << " s"
                      << " | acc/s: " << std::fixed << std::setprecision(2) << accesses_per_s
                      << " | lat: " << std::fixed << std::setprecision(1) << burst_latency_ns << " ns"
                      << " | CPI: " << std::fixed << std::setprecision(1) << cpi
                      << std::endl;
        }

        if (burst >= warmup_bursts) {
            totals.cycles += sample.cycles;
            totals.instructions += sample.instructions;
            totals.tlb1miss += sample.tlb1miss;
            totals.tlb2miss += sample.tlb2miss;
            totals.duration_s += sample.duration_s;
            totals.using_hugepages = sample.using_hugepages;
            total_accesses += ptrchase_accesses_per_burst_;
        }
    }

    totals.tlb1_event_name = expected_tlb1_event_;
    totals.tlb2_event_name = expected_tlb2_event_;

    return totals.cycles > 0 && totals.duration_s > 0 && total_accesses > 0;
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

bool LatencyMeasurer::run_single_ptrchase_burst(PerfBurstCounters& out) {
    if (ptrchase_manager_->pid() <= 0) {
        if (config_.verbosity >= 2) {
            std::cerr << "    [BURST] Invalid ptr_chase PID in run_single_ptrchase_burst: "
                      << ptrchase_manager_->pid() << std::endl;
        }
        return false;
    }
    if (!ptrchase_manager_->ensure_semaphores()) {
        if (config_.verbosity >= 2) {
            std::cerr << "    [BURST] ensure_ptrchase_semaphores() failed in run_single_ptrchase_burst" << std::endl;
        }
        return false;
    }

    auto burst_start = std::chrono::steady_clock::now();

    ptrchase_manager_->drain_done_sem();

    if (sem_post(ptrchase_manager_->start_sem()) == -1) {
        if (config_.verbosity >= 2) {
            std::cerr << "    [BURST] sem_post(start) failed in run_single_ptrchase_burst: "
                      << std::strerror(errno) << std::endl;
        }
        return false;
    }

    long long cycles = 0;
    long long insts = 0;
    long long tlb1 = 0;
    long long tlb2 = 0;
    long long hugepages = 0;
    
    std::ifstream pipe_file(ptrchase_manager_->pipe_path());
    if (pipe_file.is_open()) {
        std::string line;
        if (std::getline(pipe_file, line)) {
            auto parse_val = [&](const std::string& key) -> long long {
                size_t pos = line.find(key + "=");
                if (pos != std::string::npos) {
                    return std::stoll(line.substr(pos + key.length() + 1));
                }
                return 0;
            };
            
            cycles = parse_val("cycles");
            insts = parse_val("instructions");
            tlb1 = parse_val("tlb1");
            tlb2 = parse_val("tlb2");
            hugepages = parse_val("hugepages");
        }
        pipe_file.close();
    } else {
        if (config_.verbosity >= 2) {
            std::cerr << "    [BURST] Failed to open pipe " << ptrchase_manager_->pipe_path() << std::endl;
        }
    }

    bool completed = ptrchase_manager_->wait_for_done(2500);
    auto burst_end = std::chrono::steady_clock::now();

    if (!completed) {
        if (config_.verbosity >= 2) {
            bool running = ptrchase_manager_->pid() > 0;
            std::cerr << "    [BURST] Timeout waiting for ptr_chase done semaphore (running="
                      << (running ? "yes" : "no") << ")" << std::endl;
        }
        return false;
    }

    std::chrono::duration<double> dur = burst_end - burst_start;
    double duration_s = dur.count();

    out.cycles = static_cast<double>(cycles);
    out.instructions = static_cast<double>(insts);
    out.tlb1miss = static_cast<double>(tlb1);
    out.tlb2miss = static_cast<double>(tlb2);
    out.duration_s = duration_s;
    out.using_hugepages = (hugepages != 0);
    
    out.tlb1_event_name = expected_tlb1_event_;
    out.tlb2_event_name = expected_tlb2_event_;

    return true;
}

bool LatencyMeasurer::parse_perf_burst_output(const std::string& filepath,
                                              PerfBurstCounters& out) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        return false;
    }

    double cycles = 0.0;
    double instructions = 0.0;
    double tlb1 = 0.0;
    double tlb2 = 0.0;
    double seconds = 0.0;

    std::string line;
    while (std::getline(file, line)) {
        std::string clean = line;
        clean.erase(std::remove(clean.begin(), clean.end(), ','), clean.end());
        std::istringstream iss(clean);
        double value = 0.0;

        if (line.find("cycles") != std::string::npos &&
            line.find("seconds") == std::string::npos) {
            if (iss >> value) {
                cycles = value;
            }
        } else if (line.find("instructions") != std::string::npos) {
            if (iss >> value) {
                instructions = value;
            }
        } else if (!expected_tlb1_event_.empty() && line.find(expected_tlb1_event_) != std::string::npos) {
            if (iss >> value) {
                tlb1 = value;
            }
        } else if (!expected_tlb2_event_.empty() && line.find(expected_tlb2_event_) != std::string::npos) {
            if (iss >> value) {
                tlb2 = value;
            }
        } else if (line.find("seconds time elapsed") != std::string::npos) {
            if (iss >> value) {
                seconds = value;
            }
        }
    }
    file.close();

    if (cycles <= 0.0 || seconds <= 0.0) {
        if (config_.verbosity >= 3) {
            std::cerr << "    [BURST] Raw perf output for failed parse (" << filepath
                      << "):" << std::endl;
            std::ifstream dump_file(filepath);
            if (dump_file.is_open()) {
                std::string dump_line;
                while (std::getline(dump_file, dump_line)) {
                    std::cerr << "        " << dump_line << std::endl;
                }
                dump_file.close();
            } else {
                std::cerr << "        (could not reopen perf output file)" << std::endl;
            }
        }
        return false;
    }

    out.cycles = cycles;
    out.instructions = instructions;
    out.tlb1miss = tlb1;
    out.tlb2miss = tlb2;
    out.duration_s = seconds;
    
    out.tlb1_event_name = expected_tlb1_event_;
    out.tlb2_event_name = expected_tlb2_event_;
    
    return true;
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

