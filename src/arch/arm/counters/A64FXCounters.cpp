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

#include "arch/arm/counters/A64FXCounters.h"

CasCounterSelection A64FXCounters::detectCasCounters() {
    auto result = discoverCasCountersForRequestedMeasurer();

    const MeasurerType requested = get_requested_measurer_type();
    const bool use_perf_events = (requested == MeasurerType::AUTO || requested == MeasurerType::PERF);

    if (use_perf_events &&
        !result.has_read_write && !result.has_combined_counter) {
        // A64FX local-memory bandwidth is derived from core PMU events, not CAS counts.
        // Official references:
        //   - A64FX Microarchitecture Manual, p.85
        //      --> https://www.stonybrook.edu/commcms/ookami/support/_docs/A64FX_Microarchitecture_Manual_en_1.3.pdf
        //      --> Fujitsu A64FX documentation: https://github.com/fujitsu/A64FX/tree/master/doc
        //   - LIKWID arm64fx MEM group: groups/arm64fx/MEM.txt
        //      --> https://github.com/RRZE-HPC/likwid/blob/master/groups/arm64fx/MEM.txt
        
        // Formula:
        //   read  = (L2D_CACHE_REFILL - (L2D_SWAP_DM + L2D_CACHE_MIBMCH_PRF)) * 256 B
        //   write =  L2D_CACHE_WB * 256 B
        
        // perf raw event mapping:
        //   r17  = L2D_CACHE_REFILL
        //   r18  = L2D_CACHE_WB
        //   r325 = L2D_SWAP_DM
        //   r326 = L2D_CACHE_MIBMCH_PRF
        
        // AUTO resolves to perf on A64FX because these counters come from the
        // Linux core PMU rather than from socket-level CAS events.
        
        // The perf measurer applies the subtractive read formula only when
        // uses_read_subtract_formula is set, so all other systems keep the
        // standard read/write interpretation unchanged.
        result.read_events = {"r17"};
        result.write_events = {"r18"};
        result.read_subtract_events = {"r325", "r326"};
        result.has_read_write = true;
        result.uses_read_subtract_formula = true;
        result.perf_available = true;
        result.failure_reason = "";
    }
    
    return result;
}

void A64FXCounters::getTlbMissCounters(uint64_t& tlb1_raw, uint64_t& tlb2_raw, bool& use_tlb1, bool& use_tlb2) {
    // A64FX PMU Events v1.2:
    //   https://github.com/fujitsu/A64FX/blob/master/doc/A64FX_PMU_Events_v1.2.pdf
    tlb1_raw = 0x002D;  // L2D_TLB_REFILL: "Attributable L2 data or unified TLB refill"
    tlb2_raw = 0x0005;  // L1D_TLB_REFILL: "Attributable L1 data or unified TLB refill"
    use_tlb1 = true;
    use_tlb2 = true;
}
