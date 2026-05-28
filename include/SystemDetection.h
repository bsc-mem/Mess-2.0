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

#ifndef SYSTEM_DETECTION_H
#define SYSTEM_DETECTION_H

#include <string>
#include <vector>
#include <iostream>
#include <cstdint>

#include "SystemInfo.h"

/**
 * @file SystemDetection.h
 * @brief Host architecture and capability detection used to configure Mess automatically.
 */

/** @brief CPU architecture families recognized by Mess. */
enum class CPUArchitecture {
    X86_64,
    ARM64,
    POWERPC,
    RISCV64,
    UNKNOWN
};

/** @brief CPU vendors recognized by Mess. */
enum class CPUVendor {
    INTEL,
    AMD,
    ARM,
    IBM,
    SIFIVE,
    UNKNOWN
};

/** @brief ISA extensions used to select kernel implementations. */
enum class ISAExtension {
    // x86
    AVX512,
    AVX2,
    AVX,
    SSE4_2,
    SSE4_1,
    SSSE3,
    SSE3,
    SSE2,
    SSE,

    // ARM
    NEON,
    SVE,
    SVE2,

    // RISC-V
    RVV1_0,
    RVV0_7,

    // PowerPC
    VSX,
    VMX,

    UNKNOWN
};

/**
 * @brief Normalized hardware capabilities derived from the host system.
 */
struct CPUCapabilities {
    CPUArchitecture arch;
    CPUVendor vendor;
    std::string model_name;
    int physical_cores;
    int logical_cores;
    int sockets;
    std::vector<ISAExtension> extensions;

    bool isA64FX() const { return model_name.find("A64FX") != std::string::npos; }

    bool hasOnPackageHBM() const;

    // Runtime vector lengths (when detectable).
    // 0 means unknown/unavailable.
    int sve_vector_bits = 0;
    int sve_max_vector_bits = 0;
    int rvv_vector_bits = 0;

    size_t l1d_size;
    size_t l1i_size;
    size_t l2_size;
    size_t l3_size;
    int cache_line_size_bytes = 0;

    size_t total_memory;
    int memory_channels;
    std::string memory_type;
    std::string memory_frequency;
    int bus_width;

    double upi_freq;
    int n_data_lanes;
    int flit_bit;
    int data_flit_bit;
    int n_upi_channels;

    double nvlink_bw_gb_s = 0.0;
};

/**
 * @brief Detects hardware capabilities and exposes them in Mess's normalized format.
 */
/**
 * @brief Central component for querying and parsing hardware capabilities at runtime.
 * 
 * The `SystemDetector` relies on both generic OS abstractions (like `uname`) and OS-specific 
 * techniques (e.g., parsing `/proc/cpuinfo` or `/sys/devices/system/cpu/...` on Linux, `sysctl` on macOS)
 * to populate the `system_info` and `CPUCapabilities` structures. 
 * It identifies the architecture, vendor, cache hierarchy, SIMD extensions, and system topology, 
 * driving critical downstream decisions like ISA fallback and PMU counter selection.
 */
class SystemDetector {
private:
    system_info sys_info_;
    CPUCapabilities capabilities_;

    bool detect_x86_capabilities();
    bool detect_arm_capabilities();
    bool detect_powerpc_capabilities();
    bool detect_riscv_capabilities();

public:
    SystemDetector();
    ~SystemDetector() = default;

    bool detect();
    const system_info& get_system_info() const { return sys_info_; }
    const CPUCapabilities& get_capabilities() const { return capabilities_; }

    void print(std::ostream& os, int verbosity) const;
};

#endif
