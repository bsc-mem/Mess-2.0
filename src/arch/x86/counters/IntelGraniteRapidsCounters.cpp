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

#include "arch/x86/counters/IntelGraniteRapidsCounters.h"

CasCounterSelection IntelGraniteRapidsCounters::detectCasCounters() {
    std::vector<std::string> preferred_read;
    std::vector<std::string> preferred_write;

    // Generate candidates for up to 32 sub-channels
    for (int i = 0; i < 32; ++i) {
        preferred_read.push_back("unc_m_cas_count_sch" + std::to_string(i) + ".rd");
        preferred_write.push_back("unc_m_cas_count_sch" + std::to_string(i) + ".wr");
    }
    
    CasCounterSelection result = discoverFromPerf(preferred_read, preferred_write, {});
    
    if (result.read_events.empty() || result.write_events.empty()) {
        result = discoverFromPerf();
    }
    
    if (!result.read_events.empty() && !result.write_events.empty()) {
        bool uses_sch_counters = false;
        for (const auto& evt : result.read_events) {
            if (evt.find("unc_m_cas_count_sch") != std::string::npos) {
                uses_sch_counters = true;
                break;
            }
        }
        result.requires_channel_aggregation = uses_sch_counters;
    }
    
    return result;
}

void IntelGraniteRapidsCounters::getTlbMissCounters(uint64_t& tlb1_raw, uint64_t& tlb2_raw, bool& use_tlb1, bool& use_tlb2) {
    tlb1_raw = 0x1012;
    tlb2_raw = 0x2012;
    use_tlb1 = true;
    use_tlb2 = true;
}
