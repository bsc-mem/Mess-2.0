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

#include "PtrchasePerfHelper.h"
#include "architecture/ArchitectureRegistry.h"
#include "architecture/Architecture.h"
#include "architecture/BandwidthCounterStrategy.h"

#include <string>
#include <unistd.h>

namespace ptrchase_perf {

/**
 * @brief Identifies and retrieves the correct hardware PMU event codes for tracking TLB misses.
 * 
 * This function translates the generic `system_info` struct into a `CPUCapabilities` object 
 * to determine the hardware architecture and vendor (e.g., Intel, AMD, ARM). It then queries 
 * the corresponding `Architecture` backend (e.g., `ArmArchitecture`, `X86Architecture`) to fetch 
 * the specific raw event codes required to monitor D-TLB (Data Translation Lookaside Buffer) misses.
 * 
 * These events help distinguish whether high memory access latencies are caused by
 * actual DRAM fetches or by the overhead of page table walks.
 * 
 * @param info The global system hardware information struct.
 * @param tlb1_raw Output parameter for the primary TLB miss event code.
 * @param tlb2_raw Output parameter for a secondary TLB miss event code (if applicable).
 * @param use_tlb1 Output flag indicating if the primary TLB event should be monitored.
 * @param use_tlb2 Output flag indicating if the secondary TLB event should be monitored.
 */
void select_tlb_events_for_ptrchase(const system_info& info,
                                    uint64_t& tlb1_raw, uint64_t& tlb2_raw,
                                    bool& use_tlb1, bool& use_tlb2) {
    tlb1_raw = 0;
    tlb2_raw = 0;
    use_tlb1 = false;
    use_tlb2 = false;

    auto& strategy = BandwidthCounterStrategy::instance();
    if (strategy.is_initialized()) {
        strategy.get_tlb_counters(tlb1_raw, tlb2_raw, use_tlb1, use_tlb2);
        return;
    }

    CPUCapabilities caps;

    std::string arch_str(info.arch);
    if (arch_str.find("x86_64") != std::string::npos || arch_str.find("x86") != std::string::npos) {
        caps.arch = CPUArchitecture::X86_64;
    } else if (arch_str.find("aarch64") != std::string::npos || arch_str.find("arm64") != std::string::npos) {
        caps.arch = CPUArchitecture::ARM64;
    } else if (arch_str.find("ppc64") != std::string::npos || arch_str.find("power") != std::string::npos) {
        caps.arch = CPUArchitecture::POWERPC;
    } else if (arch_str.find("riscv") != std::string::npos) {
        caps.arch = CPUArchitecture::RISCV64;
    } else {
        return;
    }

    std::string vendor_str(info.cpu_vendor);
    if (vendor_str.find("Intel") != std::string::npos || vendor_str.find("GenuineIntel") != std::string::npos) {
        caps.vendor = CPUVendor::INTEL;
    } else if (vendor_str.find("AMD") != std::string::npos || vendor_str.find("AuthenticAMD") != std::string::npos) {
        caps.vendor = CPUVendor::AMD;
    } else if (vendor_str.find("ARM") != std::string::npos
            || vendor_str.find("Fujitsu") != std::string::npos || vendor_str.find("FUJITSU") != std::string::npos
            || vendor_str.find("Ampere") != std::string::npos || vendor_str.find("AMPERE") != std::string::npos
            || vendor_str.find("NVIDIA") != std::string::npos || vendor_str.find("Nvidia") != std::string::npos) {
        caps.vendor = CPUVendor::ARM;
    } else {
        caps.vendor = CPUVendor::UNKNOWN;
    }

    caps.model_name = std::string(info.cpu_model);

    auto arch = ArchitectureRegistry::instance().getArchitecture(caps);
    if (arch) {
        auto counter_strategy = arch->createCounterStrategy(caps);
        if (counter_strategy) {
            counter_strategy->get_tlb_counters(tlb1_raw, tlb2_raw, use_tlb1, use_tlb2);
        }
    }
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
