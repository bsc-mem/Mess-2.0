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

#include "arch/arm/counters/NvidiaGraceCounters.h"
#include <unistd.h>

CasCounterSelection NvidiaGraceCounters::detectCasCounters() {
    std::vector<std::string> read_events;
    std::vector<std::string> write_events;
    
    for (int i = 0; i < 16; ++i) {
        std::string pmu_name = "nvidia_scf_pmu_" + std::to_string(i);
        std::string pmu_path = "/sys/bus/event_source/devices/" + pmu_name;
        if (access((pmu_path + "/type").c_str(), F_OK) == 0) {
            read_events.push_back(pmu_name + "/cmem_rd_data/");
            write_events.push_back(pmu_name + "/cmem_wr_total_bytes/");
        } else if (!read_events.empty()) {
            break;
        }
    }
    
    if (!read_events.empty() && !write_events.empty()) {
        CasCounterSelection selection;
        selection.perf_available = true;
        selection.read_events = read_events;
        selection.write_events = write_events;
        selection.has_read_write = true;
        return selection;
    }
    
    return discoverFromPerf();
}

void NvidiaGraceCounters::getTlbMissCounters(uint64_t& tlb1_raw, uint64_t& tlb2_raw, bool& use_tlb1, bool& use_tlb2) {
    tlb1_raw = 0x2012;
    tlb2_raw = 0x1008;
    use_tlb1 = true;
    use_tlb2 = true;
}
