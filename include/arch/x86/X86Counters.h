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

#ifndef X86_COUNTERS_H
#define X86_COUNTERS_H

#include "architecture/BandwidthCounterStrategy.h"

/**
 * @file X86Counters.h
 * @brief Generic x86 counter strategy.
 */

/** @brief Counter discovery strategy for generic x86 systems. */
class X86Counters : public BandwidthCounterStrategy {
public:
    /**
     * @brief Detects and returns the appropriate CAS counters for x86.
     * 
     * The term "CAS" is kept for historical continuity. On x86 architectures, this function 
     * determines the required uncore performance monitoring events (such as Intel IMC or AMD UMC) 
     * to accurately track read and write bandwidth to main memory.
     * 
     * @return CasCounterSelection The selected counters and their configuration.
     */
    CasCounterSelection detectCasCounters() override;

    /**
     * @brief Retrieves the raw event codes for x86 TLB miss counters.
     * 
     * This function provides the hardware-specific event codes needed to monitor
     * Data TLB misses on x86, which is crucial for analyzing the performance penalty 
     * of page walks during memory-intensive operations.
     * 
     * @param tlb1_raw Reference to store the first TLB miss counter event code.
     * @param tlb2_raw Reference to store the second TLB miss counter event code.
     * @param use_tlb1 Reference to a boolean indicating if the first counter should be used.
     * @param use_tlb2 Reference to a boolean indicating if the second counter should be used.
     */
    void getTlbMissCounters(uint64_t& tlb1_raw, uint64_t& tlb2_raw, bool& use_tlb1, bool& use_tlb2) override;
};

#endif
