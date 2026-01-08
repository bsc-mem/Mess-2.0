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

#include "ptrchase_perf_helper.h"
#include "architecture/ArchitectureRegistry.h"
#include "architecture/Architecture.h"
#include "architecture/PerformanceCounterStrategy.h"

#include <cstring>
#include <string>
#include <sys/syscall.h>
#include <unistd.h>
#ifdef __linux__
#include <linux/perf_event.h>
#endif

#ifndef __NR_perf_event_open
#define __NR_perf_event_open 298
#endif

#ifdef __linux__
static int sys_perf_event_open(struct perf_event_attr* attr, pid_t pid,
                               int cpu, int group_fd, unsigned long flags) {
    return static_cast<int>(syscall(__NR_perf_event_open, attr, pid, cpu, group_fd, flags));
}
#endif

namespace ptrchase_perf {

void select_tlb_events_for_ptrchase(const system_info& info,
                                    uint64_t& tlb1_raw,
                                    uint64_t& tlb2_raw,
                                    bool& use_tlb1,
                                    bool& use_tlb2) {
    // Default to no TLB counters
    tlb1_raw = 0;
    tlb2_raw = 0;
    use_tlb1 = false;
    use_tlb2 = false;

    CPUCapabilities caps;
    
    std::string arch_str(info.arch);
    if (arch_str.find("x86_64") != std::string::npos || arch_str.find("x86") != std::string::npos) {
        caps.arch = CPUArchitecture::X86_64;
    } else if (arch_str.find("aarch64") != std::string::npos || arch_str.find("arm64") != std::string::npos) {
        caps.arch = CPUArchitecture::ARM64;
    } else if (arch_str.find("ppc64") != std::string::npos || arch_str.find("power") != std::string::npos) {
        caps.arch = CPUArchitecture::POWER;
    } else if (arch_str.find("riscv") != std::string::npos) {
        caps.arch = CPUArchitecture::RISCV64;
    } else {
        return; // Unknown architecture
    }
    
    // Map vendor string
    std::string vendor_str(info.cpu_vendor);
    if (vendor_str.find("Intel") != std::string::npos || vendor_str.find("GenuineIntel") != std::string::npos) {
        caps.vendor = CPUVendor::INTEL;
    } else if (vendor_str.find("AMD") != std::string::npos || vendor_str.find("AuthenticAMD") != std::string::npos) {
        caps.vendor = CPUVendor::AMD;
    } else if (vendor_str.find("ARM") != std::string::npos) {
        caps.vendor = CPUVendor::ARM;
    } else {
        caps.vendor = CPUVendor::UNKNOWN;
    }
    
    caps.model_name = std::string(info.cpu_model);
    
    // Get architecture and create counter strategy
    auto arch = ArchitectureRegistry::instance().getArchitecture(caps);
    if (arch) {
        auto strategy = arch->createCounterStrategy(caps);
        if (strategy) {
            strategy->getTlbMissCounters(tlb1_raw, tlb2_raw, use_tlb1, use_tlb2);
        }
    }
}

bool setup_ptrchase_perf_counters(const system_info& info,
                                  pid_t ptr_chase_pid,
                                  PtrchasePerfFDs& fds) {
#ifdef __linux__
    if (ptr_chase_pid <= 0) {
        return false;
    }

    perf_event_attr attr;
    std::memset(&attr, 0, sizeof(attr));
    attr.size = sizeof(attr);
    attr.type = PERF_TYPE_HARDWARE;
    attr.config = PERF_COUNT_HW_CPU_CYCLES;
    attr.disabled = 1;
    attr.exclude_kernel = 0;
    attr.exclude_hv = 1;

    int fd_cycles = sys_perf_event_open(&attr, ptr_chase_pid, -1, -1, 0);
    if (fd_cycles < 0) {
        return false;
    }

    attr.config = PERF_COUNT_HW_INSTRUCTIONS;
    int fd_insts = sys_perf_event_open(&attr, ptr_chase_pid, -1, fd_cycles, 0);
    if (fd_insts < 0) {
        close(fd_cycles);
        return false;
    }

    std::memset(&attr, 0, sizeof(attr));
    attr.size = sizeof(attr);
    attr.type = PERF_TYPE_RAW;
    attr.disabled = 1;
    attr.exclude_kernel = 0;
    attr.exclude_hv = 1;

    uint64_t tlb1_raw = 0, tlb2_raw = 0;
    bool use_tlb1 = false, use_tlb2 = false;
    select_tlb_events_for_ptrchase(info, tlb1_raw, tlb2_raw, use_tlb1, use_tlb2);

    int fd_tlb1 = -1;
    int fd_tlb2 = -1;
    if (use_tlb1) {
        attr.config = tlb1_raw;
        fd_tlb1 = sys_perf_event_open(&attr, ptr_chase_pid, -1, fd_cycles, 0);
        if (fd_tlb1 < 0) {
            fd_tlb1 = -1;
        }
    }
    if (use_tlb2) {
        attr.config = tlb2_raw;
        fd_tlb2 = sys_perf_event_open(&attr, ptr_chase_pid, -1, fd_cycles, 0);
        if (fd_tlb2 < 0) {
            fd_tlb2 = -1;
        }
    }

    fds.fd_cycles = fd_cycles;
    fds.fd_insts  = fd_insts;
    fds.fd_tlb1   = fd_tlb1;
    fds.fd_tlb2   = fd_tlb2;
    return true;
#else
    (void)info; (void)ptr_chase_pid; (void)fds;
    return false;
#endif
}

void cleanup_ptrchase_perf_counters(PtrchasePerfFDs& fds) {
    if (fds.fd_cycles >= 0) {
        close(fds.fd_cycles);
        fds.fd_cycles = -1;
    }
    if (fds.fd_insts >= 0) {
        close(fds.fd_insts);
        fds.fd_insts = -1;
    }
    if (fds.fd_tlb1 >= 0) {
        close(fds.fd_tlb1);
        fds.fd_tlb1 = -1;
    }
    if (fds.fd_tlb2 >= 0) {
        close(fds.fd_tlb2);
        fds.fd_tlb2 = -1;
    }
}
}
