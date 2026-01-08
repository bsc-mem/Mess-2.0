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

#include "arch/power/PowerCounters.h"

CasCounterSelection PowerCounters::detectCasCounters() {
    auto result = discoverFromPerf(
        {"PM_MBA0_READ_BYTES", "PM_MBA1_READ_BYTES", "PM_MBA2_READ_BYTES", "PM_MBA3_READ_BYTES",
         "PM_MBA4_READ_BYTES", "PM_MBA5_READ_BYTES", "PM_MBA6_READ_BYTES", "PM_MBA7_READ_BYTES"},
        {"PM_MBA0_WRITE_BYTES", "PM_MBA1_WRITE_BYTES", "PM_MBA2_WRITE_BYTES", "PM_MBA3_WRITE_BYTES",
         "PM_MBA4_WRITE_BYTES", "PM_MBA5_WRITE_BYTES", "PM_MBA6_WRITE_BYTES", "PM_MBA7_WRITE_BYTES"},
        {}
    );
    
    if (!result.has_read_write && !result.has_combined_counter) {
        result.failure_reason = "Power9 MBA (Memory Bus Analyzer) counters not found";
    }
    
    return result;
}

void PowerCounters::getTlbMissCounters(uint64_t& tlb1_raw, uint64_t& tlb2_raw, bool& use_tlb1, bool& use_tlb2) {
    tlb1_raw = 0x300FC;
    tlb2_raw = 0x002D;
    use_tlb1 = true;
    use_tlb2 = true;
}
