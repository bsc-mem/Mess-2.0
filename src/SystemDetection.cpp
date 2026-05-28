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

#include "SystemDetection.h"
#include "SystemInfo.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstring>
#include <algorithm>
#include <string>
#include <cctype>
#include <cstdlib>
#include <limits>
#include <sys/utsname.h>

#if defined(__x86_64__) || defined(_M_X64)
#include <cpuid.h>
#include <stdint.h>
#elif defined(__linux__) && defined(__aarch64__)
#include <sys/prctl.h>
#include <linux/prctl.h>
#elif defined(__aarch64__) || defined(__arm__) || defined(_M_ARM)
#elif defined(__powerpc__) || defined(__ppc__) || defined(_M_PPC)
#endif

bool CPUCapabilities::hasOnPackageHBM() const {
    return cpu_has_onpackage_hbm(model_name);
}

namespace {
#if defined(__x86_64__) || defined(_M_X64)
    static inline uint64_t xgetbv(uint32_t xcr) {
        uint32_t eax, edx;
        __asm__ volatile ("xgetbv" : "=a"(eax), "=d"(edx) : "c"(xcr));
        return (static_cast<uint64_t>(edx) << 32) | eax;
    }
#endif
    std::string read_file(const std::string& path) {
        std::ifstream file(path);
        if (!file) return "";
        std::stringstream buffer;
        buffer << file.rdbuf();
        return buffer.str();
    }

    bool parse_first_uint64(const std::string& text, uint64_t& out) {
        size_t i = 0;
        while (i < text.size() && !std::isdigit(static_cast<unsigned char>(text[i]))) {
            ++i;
        }
        if (i >= text.size()) {
            return false;
        }
        uint64_t value = 0;
        while (i < text.size() && std::isdigit(static_cast<unsigned char>(text[i]))) {
            uint64_t digit = static_cast<uint64_t>(text[i] - '0');
            if (value > (std::numeric_limits<uint64_t>::max() - digit) / 10) {
                return false;
            }
            value = value * 10 + digit;
            ++i;
        }
        out = value;
        return true;
    }

    size_t parse_size_with_unit(const std::string& str) {
        if (str.empty()) return 0;
        
        size_t multiplier = 1;
        size_t value = 0;
        
        try {
            size_t pos = 0;
            while (pos < str.length() && std::isdigit(str[pos])) {
                value = value * 10 + (str[pos] - '0');
                pos++;
            }
            
            if (pos < str.length()) {
                switch (std::toupper(str[pos])) {
                    case 'K': multiplier = 1000; break;
                    case 'M': multiplier = 1000 * 1000; break;
                    case 'G': multiplier = 1000 * 1000 * 1000; break;
                    default: break;
                }
            }
            
            return value * multiplier;
        } catch (...) {
            return 0;
        }
    }

    /**
     * @brief Parses the sysfs interface to determine the CPU cache size for a given level.
     * 
     * Iterates through `/sys/devices/system/cpu/cpu0/cache/index*` to find the cache
     * matching the requested level (e.g., 1 for L1, 2 for L2). When found, it reads
     * the `size` file, converting strings (e.g., "32K", "1M") into exact byte values.
     * 
     * @param level The cache level to detect (1, 2, or 3).
     * @return size_t The size of the cache in bytes, or 0 if not found.
     */
    size_t detect_cache_size_sysfs(int level) {
        // First try matching by actual cache level (not just index)
        for (int idx = 0; idx < 16; ++idx) {
            std::string base = "/sys/devices/system/cpu/cpu0/cache/index" + std::to_string(idx);
            std::string level_content = read_file(base + "/level");
            if (level_content.empty()) break; // no more cache indices

            // Trim whitespace
            while (!level_content.empty() && (level_content.back() == '\n' || level_content.back() == ' '))
                level_content.pop_back();

            int cache_level = 0;
            try { cache_level = std::stoi(level_content); } catch (...) { continue; }

            if (cache_level == level) {
                std::string size_content = read_file(base + "/size");
                if (!size_content.empty()) {
                    size_t size = parse_size_with_unit(size_content);
                    if (size > 0) return size;
                }
            }
        }
        return 0;
    }


#ifndef __APPLE__
#if defined(__x86_64__) || defined(_M_X64)
    size_t detect_cache_size_x86(int level) {
        unsigned int eax, ebx, ecx, edx;
        
        if (level == 3) {
            __cpuid(0x80000006, eax, ebx, ecx, edx);
            if ((ecx >> 18) > 0) {
                size_t size = ((ecx >> 18) + 1) * 512 * 1024;
                return size;
            }
        }
        
        return detect_cache_size_sysfs(level);
    }
#endif
#endif
#ifdef __APPLE__
#include <sys/sysctl.h>
    size_t detect_cache_size_macos(int level) {
        std::string name;
        if (level == 1) name = "hw.l1dcachesize";
        else if (level == 2) name = "hw.l2cachesize";
        else if (level == 3) name = "hw.l3cachesize";
        else return 0;

        uint64_t size = 0;
        size_t len = sizeof(size);
        if (sysctlbyname(name.c_str(), &size, &len, nullptr, 0) == 0) {
            return static_cast<size_t>(size);
        }
        return 0;
    }
#endif

#if defined(__linux__) && defined(__aarch64__) && defined(PR_SVE_GET_VL) && defined(PR_SVE_VL_LEN_MASK)
    int detect_current_sve_bits_via_prctl() {
        long vl = prctl(PR_SVE_GET_VL);
        if (vl < 0) {
            return 0;
        }
        return static_cast<int>((vl & PR_SVE_VL_LEN_MASK) * 8);
    }
#endif
}

size_t detect_cache_size(int level) {
    size_t size = detect_cache_size_sysfs(level);
    if (size > 0) return size;
    
#if defined(__APPLE__)
    return detect_cache_size_macos(level);
#elif defined(__x86_64__) || defined(_M_X64)
    return detect_cache_size_x86(level);
#else
    return 0;
#endif
}

int detect_cache_line_size_fallback() {
#if defined(__APPLE__)
    uint64_t line_size = 0;
    size_t len = sizeof(line_size);
    if (sysctlbyname("hw.cachelinesize", &line_size, &len, nullptr, 0) == 0 && line_size > 0) {
        return static_cast<int>(line_size);
    }
#else
    uint64_t line_size = 0;
    if (parse_first_uint64(read_file("/sys/devices/system/cpu/cpu0/cache/index0/coherency_line_size"), line_size) &&
        line_size > 0) {
        return static_cast<int>(line_size);
    }
#endif
    return 64;
}

SystemDetector::SystemDetector() {
    memset(&sys_info_, 0, sizeof(sys_info_));
    capabilities_ = {};
}

/**
 * @brief Runs the hardware discovery process.
 * 
 * This method runs a series of detection passes to populate `sys_info_` and `capabilities_`.
 * It detects the CPU architecture, vendor, ISA extensions (e.g., AVX, SVE, RVV), and specific
 * CPU models (e.g., A64FX, Grace). 
 * 
 * After gathering CPU metadata, it runs `detect_caches()` to map out the memory hierarchy,
 * which informs cache-line sizes, array dimensions, and stride offsets
 * used during the benchmark generation.
 * 
 * @return bool True if detection was broadly successful, false on failure.
 */
bool SystemDetector::detect() {
    if (system_info_detect(&sys_info_) != 0) {
        return false;
    }

    std::string vendor = sys_info_.cpu_vendor;
    std::transform(vendor.begin(), vendor.end(), vendor.begin(), ::tolower);
    if (vendor.find("intel") != std::string::npos) {
        capabilities_.vendor = CPUVendor::INTEL;
    } else if (vendor.find("amd") != std::string::npos) {
        capabilities_.vendor = CPUVendor::AMD;
    } else if (vendor.find("arm") != std::string::npos
            || vendor.find("fujitsu") != std::string::npos
            || vendor.find("ampere") != std::string::npos
            || vendor.find("nvidia") != std::string::npos) {
        capabilities_.vendor = CPUVendor::ARM;
    } else if (vendor.find("ibm") != std::string::npos) {
        capabilities_.vendor = CPUVendor::IBM;
    } else if (vendor.find("sifive") != std::string::npos) {
        capabilities_.vendor = CPUVendor::SIFIVE;
    } else {
        capabilities_.vendor = CPUVendor::UNKNOWN;
    }

    {
        std::string cpu_model = sys_info_.cpu_model;
        std::string cpu_vendor = sys_info_.cpu_vendor;
        
        size_t pos = 0;
        while ((pos = cpu_model.find("(R)", pos)) != std::string::npos) {
            cpu_model.erase(pos, 3);
        }
        
        pos = cpu_vendor.find("GenuineIntel");
        if (pos != std::string::npos) {
            cpu_vendor.erase(pos, 12);
        }
        
        auto cleanup_string = [](std::string& str) {
            size_t pos = 0;
            while ((pos = str.find("  ", pos)) != std::string::npos) {
                str.erase(pos, 1);
            }
            pos = str.find_first_not_of(" ");
            if (pos != std::string::npos) {
                str.erase(0, pos);
            } else {
                str.clear();
            }
            pos = str.find_last_not_of(" ");
            if (pos != std::string::npos) {
                str.erase(pos + 1);
            }
        };
        
        cleanup_string(cpu_model);
        cleanup_string(cpu_vendor);
        
        strncpy(sys_info_.cpu_model, cpu_model.c_str(), sizeof(sys_info_.cpu_model) - 1);
        sys_info_.cpu_model[sizeof(sys_info_.cpu_model) - 1] = '\0';
        strncpy(sys_info_.cpu_vendor, cpu_vendor.c_str(), sizeof(sys_info_.cpu_vendor) - 1);
        sys_info_.cpu_vendor[sizeof(sys_info_.cpu_vendor) - 1] = '\0';
    }

    capabilities_.arch = CPUArchitecture::UNKNOWN;

#if defined(__x86_64__) || defined(_M_X64)
    capabilities_.arch = CPUArchitecture::X86_64;
#elif defined(__aarch64__) || defined(__arm64__) || defined(__arm__)
    capabilities_.arch = CPUArchitecture::ARM64;
#elif defined(__powerpc__) || defined(__ppc__) || defined(_M_PPC)
    capabilities_.arch = CPUArchitecture::POWERPC;
#elif defined(__riscv) && (__riscv_xlen == 64)
    capabilities_.arch = CPUArchitecture::RISCV64;
#endif

    if (capabilities_.arch == CPUArchitecture::UNKNOWN) {
        struct utsname uname_info;
        if (uname(&uname_info) == 0) {
            std::string machine = uname_info.machine;
            if (machine == "x86_64") {
                capabilities_.arch = CPUArchitecture::X86_64;
            } else if (machine == "aarch64" || machine == "arm64") {
                capabilities_.arch = CPUArchitecture::ARM64;
            } else if (machine.find("ppc64") != std::string::npos) {
                capabilities_.arch = CPUArchitecture::POWERPC;
            } else if (machine.find("riscv64") != std::string::npos) {
                capabilities_.arch = CPUArchitecture::RISCV64;
            }
        }
    }

    capabilities_.model_name = sys_info_.cpu_model;
    capabilities_.physical_cores = sys_info_.total_physical_cores;
    capabilities_.logical_cores = sys_info_.total_logical_cores;
    capabilities_.sockets = sys_info_.socket_count;
    capabilities_.total_memory = static_cast<size_t>(sys_info_.total_mem_bytes);
    capabilities_.memory_type = sys_info_.mem_technology;
    capabilities_.memory_frequency = sys_info_.mem_frequency;

    bool has_cache_info = false;
    int detected_cache_line_size = 0;
    for (int i = 0; i < sys_info_.socket_count; ++i) {
        const auto& socket = sys_info_.sockets[i];
        for (int j = 0; j < socket.cache_count; ++j) {
            const auto& cache = socket.caches[j];
            if (cache.size_bytes > 0) {
                has_cache_info = true;
                switch (cache.level) {
                    case 1:
                        if (cache.type == si_cache_type::DATA) {
                            capabilities_.l1d_size = static_cast<size_t>(cache.size_bytes);
                        } else if (cache.type == si_cache_type::INSTRUCTION) {
                            capabilities_.l1i_size = static_cast<size_t>(cache.size_bytes);
                        }
                        break;
                    case 2:
                        capabilities_.l2_size = static_cast<size_t>(cache.size_bytes);
                        break;
                    case 3:
                        capabilities_.l3_size = static_cast<size_t>(cache.size_bytes);
                        break;
                }
            }
            if (detected_cache_line_size <= 0 && cache.line_size_bytes > 0) {
                detected_cache_line_size = cache.line_size_bytes;
            }
        }
        capabilities_.memory_channels = socket.mem_channels;
    }
    capabilities_.cache_line_size_bytes = (detected_cache_line_size > 0)
        ? detected_cache_line_size
        : detect_cache_line_size_fallback();
    
    auto get_bus_width_bytes = [](const std::string& mem_type, const std::string& vendor) -> int {
        std::string type_upper = mem_type;
        std::transform(type_upper.begin(), type_upper.end(), type_upper.begin(), ::toupper);
        
        if (type_upper.find("HBM") != std::string::npos) {
            return 128;
        }
        
        if (vendor.find("Apple") != std::string::npos) {
            return 16;
        }
        
        return 8;
    };

    capabilities_.bus_width = get_bus_width_bytes(capabilities_.memory_type, sys_info_.cpu_vendor) * 8;

    std::string model_upper = sys_info_.cpu_model;
    std::transform(model_upper.begin(), model_upper.end(), model_upper.begin(), ::toupper);
    auto is_unknown_or_empty = [](const std::string& value) {
        return value.empty() || value == "Unknown";
    };

    if (model_upper.find("A64FX") != std::string::npos) {
        if (is_unknown_or_empty(capabilities_.memory_type)) {
            capabilities_.memory_type = "HBM2";
        }
        if (is_unknown_or_empty(capabilities_.memory_frequency)) {
            capabilities_.memory_frequency = "2000 MT/s";
        }
        if (capabilities_.memory_channels <= 0 || capabilities_.memory_channels == 2) {
            capabilities_.memory_channels = 4;
        }
        capabilities_.bus_width = 1024;
    } else if (model_upper.find("NEOVERSE-V2") != std::string::npos ||
               model_upper.find("NEOVERSE V2") != std::string::npos ||
               model_upper.find("GRACE") != std::string::npos) {
        if (is_unknown_or_empty(capabilities_.memory_type)) {
            capabilities_.memory_type = "LPDDR5X";
        }
        if (is_unknown_or_empty(capabilities_.memory_frequency)) {
            capabilities_.memory_frequency = "4266";
        }
        if (capabilities_.memory_channels <= 0 || capabilities_.memory_channels == 2) {
            capabilities_.memory_channels = 16;
        }
    }

    capabilities_.upi_freq = 0;
    capabilities_.n_data_lanes = 0;
    capabilities_.flit_bit = 0;
    capabilities_.data_flit_bit = 0;
    capabilities_.n_upi_channels = 0;

    std::string model = sys_info_.cpu_model; 
    // Sapphire Rapids (8480/9480)
    if (model.find("8480") != std::string::npos || model.find("9480") != std::string::npos || model.find("9462") != std::string::npos) {
        capabilities_.upi_freq = 16.0;
        capabilities_.n_data_lanes = 20;
        capabilities_.flit_bit = 80;
        capabilities_.data_flit_bit = 64;
        capabilities_.n_upi_channels = 4;
    }
    // Emerald Rapids (8568)
    else if (model.find("8568") != std::string::npos) {
        capabilities_.upi_freq = 20.0;
        capabilities_.n_data_lanes = 20;
        capabilities_.flit_bit = 80;
        capabilities_.data_flit_bit = 64;
        capabilities_.n_upi_channels = 4;
    }
    
    if (!has_cache_info || capabilities_.l3_size == 0) {
        capabilities_.l1d_size = detect_cache_size(1);
        capabilities_.l1i_size = capabilities_.l1i_size ? capabilities_.l1i_size : capabilities_.l1d_size;
        capabilities_.l2_size = capabilities_.l2_size ? capabilities_.l2_size : detect_cache_size(2);
        capabilities_.l3_size = capabilities_.l3_size ? capabilities_.l3_size : detect_cache_size(3);
    }

    // When sysfs doesn't expose cache sizes, try to identify the CPU via
    // the ARM MIDR register and use known cache geometry.
    if (capabilities_.arch == CPUArchitecture::ARM64 &&
        (!capabilities_.l1d_size || !capabilities_.l2_size || !capabilities_.l3_size)) {
        std::string midr_str = read_file("/sys/devices/system/cpu/cpu0/regs/identification/midr_el1");
        if (!midr_str.empty()) {
            uint64_t midr = 0;
            try { midr = std::stoull(midr_str, nullptr, 0); } catch (...) {}

            if (midr != 0) {
                uint8_t  implementer = (midr >> 24) & 0xFF;
                uint16_t part        = (midr >> 4)  & 0xFFF;

                struct midr_cache_info {
                    uint8_t  implementer;
                    uint16_t part;
                    size_t   l1d;           // per core
                    size_t   l1i;           // per core
                    size_t   l2_shared;     // shared L2 per cluster/CMG
                    int      cores_per_group; // cores sharing the L2
                    size_t   l3;            // total L3 (0 = no L3, LLC is L2)
                };

                // MIDR implementer codes: 0x46=Fujitsu, 0x41=ARM, 0x4E=NVIDIA
                static const midr_cache_info known[] = {
                    // Fujitsu A64FX: impl=0x46, part=0x001
                    { 0x46, 0x001, 64*1024, 64*1024, 8*1024*1024, 12, 0 },
                };

                for (const auto& k : known) {
                    if (implementer == k.implementer && part == k.part) {
                        if (!capabilities_.l1d_size) capabilities_.l1d_size = k.l1d;
                        if (!capabilities_.l1i_size) capabilities_.l1i_size = k.l1i;
                        if (!capabilities_.l2_size)  capabilities_.l2_size  = k.l2_shared;
                        if (!capabilities_.l3_size) {
                            if (k.l3 > 0) {
                                capabilities_.l3_size = k.l3;
                            } else {
                                // No L3 — total LLC = L2 × number of groups
                                int cores = sys_info_.total_physical_cores > 0 ? sys_info_.total_physical_cores : 1;
                                int groups = (cores + k.cores_per_group - 1) / k.cores_per_group;
                                capabilities_.l3_size = k.l2_shared * static_cast<size_t>(groups);
                            }
                        }
                        break;
                    }
                }
            }
        }
    }

    if (capabilities_.l3_size == 0) {
        if (capabilities_.l2_size > 0) {
            // No L3 and not a known CPU — use L2 as a rough LLC estimate.
            capabilities_.l3_size = capabilities_.l2_size;
        } else {
            std::cerr << "Warning: Could not determine LLC size from system. Using default 32MB." << std::endl;
            capabilities_.l3_size = 32 * 1000 * 1000;
        }
    }

    switch (capabilities_.arch) {
        case CPUArchitecture::X86_64:
            return detect_x86_capabilities();
        case CPUArchitecture::ARM64:
            return detect_arm_capabilities();
        case CPUArchitecture::POWERPC:
            return detect_powerpc_capabilities();
        case CPUArchitecture::RISCV64:
            return detect_riscv_capabilities();
        default:
            return true;
    }
}

bool SystemDetector::detect_x86_capabilities() {
#ifdef __x86_64__
    unsigned int eax, ebx, ecx, edx;

    if (__get_cpuid_count(7, 0, &eax, &ebx, &ecx, &edx)) {
        bool hw_avx512f = (ebx & bit_AVX512F);
        unsigned int eax1, ebx1, ecx1, edx1;
        bool os_supports_state = false;
        if (__get_cpuid(1, &eax1, &ebx1, &ecx1, &edx1)) {
            bool osxsave = (ecx1 & bit_OSXSAVE);
            if (osxsave) {
                uint64_t xcr0 = xgetbv(0);
                const uint64_t need = (1ULL<<1) | (1ULL<<2) | (1ULL<<5) | (1ULL<<6) | (1ULL<<7);
                os_supports_state = (xcr0 & need) == need;
            }
        }
        if (hw_avx512f && os_supports_state) {
            capabilities_.extensions.push_back(ISAExtension::AVX512);
        }
    }

    if (__get_cpuid_count(7, 0, &eax, &ebx, &ecx, &edx)) {
        if (ebx & bit_AVX2) {
            capabilities_.extensions.push_back(ISAExtension::AVX2);
        }
    }

    if (__get_cpuid(1, &eax, &ebx, &ecx, &edx)) {
        bool hw_avx = (ecx & bit_AVX);
        bool osxsave = (ecx & bit_OSXSAVE);
        bool os_avx = false;
        if (osxsave) {
            uint64_t xcr0 = xgetbv(0);
            const uint64_t need = (1ULL<<1) | (1ULL<<2);
            os_avx = (xcr0 & need) == need;
        }
        if (hw_avx && os_avx) {
            capabilities_.extensions.push_back(ISAExtension::AVX);
        }
    }

    if (__get_cpuid(1, &eax, &ebx, &ecx, &edx)) {
        if (ecx & bit_SSE4_2) capabilities_.extensions.push_back(ISAExtension::SSE4_2);
        if (ecx & bit_SSE4_1) capabilities_.extensions.push_back(ISAExtension::SSE4_1);
        if (ecx & bit_SSSE3) capabilities_.extensions.push_back(ISAExtension::SSSE3);
        if (ecx & bit_SSE3) capabilities_.extensions.push_back(ISAExtension::SSE3);
        if (edx & bit_SSE2) capabilities_.extensions.push_back(ISAExtension::SSE2);
        if (edx & bit_SSE) capabilities_.extensions.push_back(ISAExtension::SSE);
    }
#endif
    return true;
}

bool SystemDetector::detect_arm_capabilities() {
#ifdef __linux__
    std::ifstream cpuinfo("/proc/cpuinfo");
    if (cpuinfo.is_open()) {
        std::string line;
        bool has_neon = false;
        bool has_sve = false;
        bool has_sve2 = false;
        bool is_grace = false;
        bool is_a64fx = false;
        while (std::getline(cpuinfo, line)) {
            std::string lower_line = line;
            std::transform(lower_line.begin(), lower_line.end(), lower_line.begin(),
                           [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

            if (lower_line.find("asimd") != std::string::npos ||
                lower_line.find(" neon") != std::string::npos ||
                lower_line.rfind("neon", 0) == 0) {
                has_neon = true;
            }

            if (lower_line.find("sve2") != std::string::npos) {
                has_sve2 = true;
                has_sve = true;
            } else if (lower_line.find("sve") != std::string::npos) {
                has_sve = true;
            }

            std::string upper_line = line;
            std::transform(upper_line.begin(), upper_line.end(), upper_line.begin(),
                           [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
            if (upper_line.find("NEOVERSE-V2") != std::string::npos || 
                upper_line.find("NEOVERSE V2") != std::string::npos ||
                upper_line.find("GRACE") != std::string::npos) {
                is_grace = true;
            }
            if (upper_line.find("A64FX") != std::string::npos) {
                is_a64fx = true;
            }
        }

        if (has_neon) {
            capabilities_.extensions.push_back(ISAExtension::NEON);
        }
        if (has_sve2) {
            capabilities_.extensions.push_back(ISAExtension::SVE2);
        }
        if (has_sve) {
            capabilities_.extensions.push_back(ISAExtension::SVE);
        }

        if (has_sve) {
            // Linux reports SVE VL in bytes through this tunable.
            // Convert to bits for ISA width resolution in the generator.
            const std::string sve_vl_text = read_file("/proc/sys/abi/sve_default_vector_length");
            uint64_t sve_vl_bytes = 0;
            if (parse_first_uint64(sve_vl_text, sve_vl_bytes) && sve_vl_bytes > 0) {
                capabilities_.sve_vector_bits = static_cast<int>(sve_vl_bytes * 8);
            } else if (is_a64fx || capabilities_.isA64FX()) {
                // A64FX has a fixed 512-bit SVE implementation.
                capabilities_.sve_vector_bits = 512;
            }

#if defined(__linux__) && defined(__aarch64__) && defined(PR_SVE_GET_VL) && defined(PR_SVE_VL_LEN_MASK)
            const int current_sve_bits = detect_current_sve_bits_via_prctl();
            if (current_sve_bits > 0) {
                capabilities_.sve_vector_bits = current_sve_bits;
            }
#endif
        }

        if (is_grace) {
            capabilities_.nvlink_bw_gb_s = 450.0;
        }
    }
#endif

    return true;
}

bool SystemDetector::detect_powerpc_capabilities() {
    capabilities_.extensions.push_back(ISAExtension::VSX);
    capabilities_.extensions.push_back(ISAExtension::VMX);
    return true;
}

bool SystemDetector::detect_riscv_capabilities() {
    std::ifstream cpuinfo("/proc/cpuinfo");
    if (cpuinfo.is_open()) {
        std::string line;
        bool found_rvv = false;
        while (std::getline(cpuinfo, line)) {
            if (line.find("rvv") != std::string::npos) {
                if (line.find("rvv1.0") != std::string::npos) {
                    capabilities_.extensions.push_back(ISAExtension::RVV1_0);
                } else if (line.find("rvv0.7") != std::string::npos) {
                    capabilities_.extensions.push_back(ISAExtension::RVV0_7);
                }
                found_rvv = true;
            }

            // Common CPUinfo fields: "vlenb : 32" or "vlen : 256"
            if (line.find("vlenb") != std::string::npos) {
                uint64_t vlenb = 0;
                if (parse_first_uint64(line, vlenb) && vlenb > 0) {
                    capabilities_.rvv_vector_bits = static_cast<int>(vlenb * 8);
                }
            } else if (line.find("vlen") != std::string::npos) {
                uint64_t vlen_bits = 0;
                if (parse_first_uint64(line, vlen_bits) && vlen_bits > 0) {
                    capabilities_.rvv_vector_bits = static_cast<int>(vlen_bits);
                }
            }
        }

        if (found_rvv && capabilities_.rvv_vector_bits == 0) {
            capabilities_.rvv_vector_bits = 256;
        }
    }
    return true;
}

void SystemDetector::print(std::ostream& os, int verbosity) const {
    os << "System Information:" << std::endl;
    os << "  OS: " << sys_info_.os_name << " " << sys_info_.os_release << std::endl;
    os << "  Architecture: ";
    switch (capabilities_.arch) {
        case CPUArchitecture::X86_64: os << "x86_64"; break;
        case CPUArchitecture::ARM64: os << "ARM64"; break;
        case CPUArchitecture::POWERPC: os << "PowerPC"; break;
        case CPUArchitecture::RISCV64: os << "RISC-V 64"; break;
        default: os << "Unknown"; break;
    }
    os << std::endl;

    os << "  CPU: " << sys_info_.cpu_vendor << " " << sys_info_.cpu_model << std::endl;
    os << "  Cores: " << capabilities_.physical_cores << " physical, "
       << capabilities_.logical_cores << " logical" << std::endl;
    os << "  Sockets: " << capabilities_.sockets << std::endl;

    if (verbosity >= 2) {
        os << "  Memory: " << (capabilities_.total_memory / (1000*1000*1000)) << " GB" << std::endl;
        if (!capabilities_.memory_type.empty()) {
            os << "  Memory Type: " << capabilities_.memory_type;
            if (!capabilities_.memory_frequency.empty()) {
                os << " @ " << capabilities_.memory_frequency;
            }
            os << std::endl;
        }
        os << "  Memory Channels: " << capabilities_.memory_channels << std::endl;

        if (capabilities_.l1d_size > 0) os << "  L1D Cache: " << (capabilities_.l1d_size / 1000) << " KB" << std::endl;
        if (capabilities_.l1i_size > 0) os << "  L1I Cache: " << (capabilities_.l1i_size / 1000) << " KB" << std::endl;
        if (capabilities_.l2_size > 0) os << "  L2 Cache: " << (capabilities_.l2_size / 1000) << " KB" << std::endl;
        if (capabilities_.l3_size > 0) os << "  L3 Cache: " << (capabilities_.l3_size / (1000*1000)) << " MB" << std::endl;

        if (verbosity >= 3 && !capabilities_.extensions.empty()) {
            os << "  ISA Extensions: ";
            for (size_t i = 0; i < capabilities_.extensions.size(); ++i) {
                if (i > 0) os << ", ";
                switch (capabilities_.extensions[i]) {
                    case ISAExtension::AVX512: os << "AVX512"; break;
                    case ISAExtension::AVX2: os << "AVX2"; break;
                    case ISAExtension::AVX: os << "AVX"; break;
                    case ISAExtension::SSE4_2: os << "SSE4.2"; break;
                    case ISAExtension::SSE4_1: os << "SSE4.1"; break;
                    case ISAExtension::SSSE3: os << "SSSE3"; break;
                    case ISAExtension::SSE3: os << "SSE3"; break;
                    case ISAExtension::SSE2: os << "SSE2"; break;
                    case ISAExtension::SSE: os << "SSE"; break;
                    case ISAExtension::NEON: os << "NEON"; break;
                    case ISAExtension::SVE2: os << "SVE2"; break;
                    case ISAExtension::SVE: os << "SVE"; break;
                    case ISAExtension::RVV0_7: os << "RVV0.7"; break;
                    case ISAExtension::RVV1_0: os << "RVV1.0"; break;
                    case ISAExtension::VMX: os << "VMX"; break;
                    case ISAExtension::VSX: os << "VSX"; break;
                    default: os << "Unknown"; break;
                }
            }
            os << std::endl;
        }

        if (capabilities_.sve_vector_bits > 0) {
            os << "  SVE Vector Length: " << capabilities_.sve_vector_bits << " bits" << std::endl;
        }
        if (capabilities_.rvv_vector_bits > 0) {
            os << "  RVV Vector Length: " << capabilities_.rvv_vector_bits << " bits" << std::endl;
        }
    }
}
