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

#include "system_detection.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstring>
#include <algorithm>
#include <string>
#include <cctype>
#include <cstdlib>
#include <sys/utsname.h>

#if defined(__x86_64__) || defined(_M_X64)
#include <cpuid.h>
#include <stdint.h>
#elif defined(__aarch64__) || defined(__arm__) || defined(_M_ARM)
#elif defined(__powerpc__) || defined(__ppc__) || defined(_M_PPC)
#endif

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

    size_t detect_cache_size_sysfs(int level) {
        std::string path = "/sys/devices/system/cpu/cpu0/cache/index" + 
                          std::to_string(level) + "/size";
        
        std::string content = read_file(path);
        if (!content.empty()) {
            size_t size = parse_size_with_unit(content);
            
            return size;
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

SystemDetector::SystemDetector() {
    memset(&sys_info_, 0, sizeof(sys_info_));
    capabilities_ = {};
}

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
    } else if (vendor.find("arm") != std::string::npos) {
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
    capabilities_.arch = CPUArchitecture::POWER;
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
                capabilities_.arch = CPUArchitecture::POWER;
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
        }
        capabilities_.memory_channels = socket.mem_channels;
    }
    
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

    if (capabilities_.l3_size == 0) {
        std::cerr << "Warning: Could not determine L3 cache size from system. Using default 32MB." << std::endl;
        capabilities_.l3_size = 32 * 1000 * 1000;
    }

    switch (capabilities_.arch) {
        case CPUArchitecture::X86_64:
            return detect_x86_capabilities();
        case CPUArchitecture::ARM64:
            return detect_arm_capabilities();
        case CPUArchitecture::POWER:
            return detect_power_capabilities();
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
    capabilities_.extensions.push_back(ISAExtension::NEON);

#ifdef __linux__
    std::ifstream cpuinfo("/proc/cpuinfo");
    if (cpuinfo.is_open()) {
        std::string line;
        bool has_sve = false;
        bool has_sve2 = false;
        bool is_grace = false;
        while (std::getline(cpuinfo, line)) {
            if (line.find("sve2") != std::string::npos) {
                has_sve2 = true;
                has_sve = true;
            } else if (line.find("sve") != std::string::npos) {
                has_sve = true;
            }
            std::string upper_line = line;
            std::transform(upper_line.begin(), upper_line.end(), upper_line.begin(), ::toupper);
            if (upper_line.find("NEOVERSE-V2") != std::string::npos || 
                upper_line.find("NEOVERSE V2") != std::string::npos ||
                upper_line.find("GRACE") != std::string::npos) {
                is_grace = true;
            }
        }
        if (has_sve2) {
            capabilities_.extensions.push_back(ISAExtension::SVE2);
        }
        if (has_sve) {
            capabilities_.extensions.push_back(ISAExtension::SVE);
        }
        if (is_grace) {
            capabilities_.nvlink_bw_gb_s = 450.0;
        }
    }
#endif

    return true;
}

bool SystemDetector::detect_power_capabilities() {
    capabilities_.extensions.push_back(ISAExtension::VSX);
    capabilities_.extensions.push_back(ISAExtension::VMX);
    return true;
}

bool SystemDetector::detect_riscv_capabilities() {
    std::ifstream cpuinfo("/proc/cpuinfo");
    if (cpuinfo.is_open()) {
        std::string line;
        while (std::getline(cpuinfo, line)) {
            if (line.find("rvv") != std::string::npos) {
                if (line.find("rvv1.0") != std::string::npos) {
                    capabilities_.extensions.push_back(ISAExtension::RVV1_0);
                } else if (line.find("rvv0.7") != std::string::npos) {
                    capabilities_.extensions.push_back(ISAExtension::RVV0_7);
                }
                break;
            }
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
        case CPUArchitecture::POWER: os << "Power"; break;
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
                    case ISAExtension::NEON: os << "NEON"; break;
                    case ISAExtension::SVE: os << "SVE"; break;
                    case ISAExtension::RVV1_0: os << "RVV1.0"; break;
                    case ISAExtension::VSX: os << "VSX"; break;
                    default: os << "Unknown"; break;
                }
            }
            os << std::endl;
        }
    }
}
