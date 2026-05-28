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

#ifndef KERNEL_TYPES_H
#define KERNEL_TYPES_H

#include <string>
#include <vector>

/**
 * @file KernelTypes.h
 * @brief Shared enums and configuration structs for traffic-generator kernel generation.
 */

// Make sure to read through the wiki before tuning any of these attributes!

/**
 * @brief Supported traffic generation modes.
 */
enum class ExecutionMode {
    MULTISEQUENTIAL,
};

/**
 * @brief ISA backends that can be targeted explicitly or selected automatically.
 */
enum class ISAMode {
    // Generic
    AUTO,

    // x86_64
    AVX512,
    AVX2,
    AVX,
    SSE2,
    SCALAR,

    // ARM64
    NEON_PAIR,
    NEON_NATIVE,
    SVE,
    SVE_MAX,
    SVE128,
    SVE256,
    SVE512,

    // RISC-V
    RVV1_0,
    RVV0_7,

    // POWERPC
    VSX,
    VMX
};

/**
 * @brief Address-generation policy used by generated load/store instructions.
 */
enum class AddressingPolicy {
    AUTO,
    POST_INCREMENT,
    INDEXED
};

/**
 * @brief Store expansion policy for one logical write step.
 */
enum class StoreLinePolicy {
    AUTO,
    ISA_NATIVE,
    CACHELINE_64B,
    CACHELINE_NATIVE
};

/**
 * @brief Low-level memory operation parameters for a generated kernel instance.
 */
struct KernelParameters {
    size_t array_size;
    double read_ratio;
    int stride;
    bool use_nontemporal;
};

/** 
 * @brief Configuration for the sequential multi-ratio traffic generator.
 * 
 * Please refer to the official wiki for detailed documentation of each attribute:
 * (e.g., how ops_per_pause_block, ratio_granularity, and num_simd_registers affect the generation).
 */
struct KernelConfigMultiSeq {
    int ratio_granularity = 2;
    int ops_per_pause_block = 6;
    int total_ops = 100;
    
    int num_simd_registers = 30;
    
    ISAMode isa_mode = ISAMode::AUTO;
    AddressingPolicy addressing_policy = AddressingPolicy::AUTO;
    StoreLinePolicy store_line_policy = StoreLinePolicy::AUTO;
    bool enable_interleaving = true;
    bool use_nontemporal_stores = false;
    bool single_registers = true;

    // Internal generator-resolved value. 0 means "use ISA default width".
    int resolved_bytes_per_mem_op = 0;
    int resolved_cache_line_bytes = 0;
};


using KernelConfig = KernelConfigMultiSeq;

/**
 * @brief Architecture-specific template and capability metadata for code generation.
 */
struct ArchitectureConfig {
    std::string arch_name;
    std::string pointer_chase_template;
    std::string utils_multiseq_file;
    bool has_multisequential;
};

#endif
