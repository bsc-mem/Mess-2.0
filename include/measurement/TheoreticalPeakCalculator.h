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

#pragma once

#include "system_detection.h"
#include <string>
#include <cstdlib>
#include <dirent.h>

enum class MemoryBindingType {
    LOCAL,
    REMOTE_UPI,
    NVLINK_GRACE
};

struct TheoreticalPeakConfig {
    MemoryBindingType binding_type = MemoryBindingType::LOCAL;
    
    double mem_freq_mhz = 0.0;
    int memory_channels = 0;
    int bus_width = 64;
    
    double upi_freq_gt_s = 0.0;
    int n_data_lanes = 0;
    int flit_bit = 0;
    int data_flit_bit = 0;
    int n_upi_channels = 0;
    
    double nvlink_bw_gb_s = 0.0;
};

class TheoreticalPeakCalculator {
public:
    static double calculate(const TheoreticalPeakConfig& config) {
        switch (config.binding_type) {
            case MemoryBindingType::NVLINK_GRACE:
                return config.nvlink_bw_gb_s;
            
            case MemoryBindingType::REMOTE_UPI:
                if (config.flit_bit > 0 && config.data_flit_bit > 0) {
                    // UPI bandwidth per direction (not bidirectional - memory traffic flows primarily one way)
                    return (config.upi_freq_gt_s * config.n_data_lanes * 
                           (static_cast<double>(config.data_flit_bit) / config.flit_bit) * 
                           config.n_upi_channels) / 8.0;
                }
                return 0.0;
            
            case MemoryBindingType::LOCAL:
            default:
                if (config.bus_width > 0 && config.mem_freq_mhz > 0) {
                    return (static_cast<double>(config.bus_width) / 8.0) * 
                           config.mem_freq_mhz * config.memory_channels / 1000.0;
                }
                return 0.0;
        }
    }

    static int infer_memory_channels_from_type(const std::string& mem_type) {
        std::string type_upper = mem_type;
        for (auto& c : type_upper) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
        
        if (type_upper.find("HBM") != std::string::npos) {
            return 8;
        }
        if (type_upper.find("DDR5") != std::string::npos) {
            return 8;
        }
        if (type_upper.find("DDR4") != std::string::npos) {
            return 6;
        }
        if (type_upper.find("LPDDR5") != std::string::npos) {
            return 8;
        }
        return 8;
    }

    static int count_imc_pmus() {
        int count = 0;
#ifdef __linux__
        DIR* dir = opendir("/sys/bus/event_source/devices");
        if (dir) {
            struct dirent* entry;
            while ((entry = readdir(dir)) != nullptr) {
                std::string name = entry->d_name;
                if (name.find("uncore_imc") != std::string::npos) {
                    count++;
                }
            }
            closedir(dir);
        }
#endif
        return count;
    }

    static TheoreticalPeakConfig from_capabilities(
        const CPUCapabilities& caps,
        bool is_remote_memory = false)
    {
        TheoreticalPeakConfig config;
        
        if (caps.nvlink_bw_gb_s > 0) {
            config.binding_type = MemoryBindingType::NVLINK_GRACE;
            config.nvlink_bw_gb_s = caps.nvlink_bw_gb_s;
        } else if (is_remote_memory) {
            config.binding_type = MemoryBindingType::REMOTE_UPI;
            config.upi_freq_gt_s = caps.upi_freq;
            config.n_data_lanes = caps.n_data_lanes;
            config.flit_bit = caps.flit_bit;
            config.data_flit_bit = caps.data_flit_bit;
            config.n_upi_channels = caps.n_upi_channels;
        } else {
            config.binding_type = MemoryBindingType::LOCAL;
            if (!caps.memory_frequency.empty()) {
                try {
                    config.mem_freq_mhz = std::stod(caps.memory_frequency);
                } catch (...) {}
            }
            config.memory_channels = caps.memory_channels;
            if (config.memory_channels <= 0) {
                int imc_count = count_imc_pmus();
                if (imc_count > 0) {
                    config.memory_channels = imc_count * 2;
                } else {
                    config.memory_channels = infer_memory_channels_from_type(caps.memory_type);
                }
            }
            config.bus_width = caps.bus_width > 0 ? caps.bus_width : 64;
        }
        
        return config;
    }

    static double calculate_from_capabilities(
        const CPUCapabilities& caps,
        bool is_remote_memory = false)
    {
        return calculate(from_capabilities(caps, is_remote_memory));
    }

    static double scale_for_core_count(
        double theoretical_peak_gb_s,
        int cores_used,
        int memory_channels,
        int total_socket_cores = 0)
    {
        if (cores_used <= 0) return theoretical_peak_gb_s;
        
        int saturation_cores = memory_channels * 2;
        if (saturation_cores <= 0) saturation_cores = 16;
        
        if (total_socket_cores > 0 && total_socket_cores < saturation_cores) {
            saturation_cores = total_socket_cores;
        }
        
        double scale_factor = static_cast<double>(cores_used) / saturation_cores;
        if (scale_factor > 1.0) scale_factor = 1.0;
        
        return theoretical_peak_gb_s * scale_factor;
    }

    static double scale_upi_for_core_count(
        double upi_peak_gb_s,
        double local_peak_gb_s,
        int cores_used,
        int memory_channels,
        int total_socket_cores = 0)
    {
        if (cores_used <= 0) return upi_peak_gb_s;
        if (local_peak_gb_s <= 0) return upi_peak_gb_s;
        
        int local_saturation_cores = memory_channels * 2;
        if (local_saturation_cores <= 0) local_saturation_cores = 16;
        
        if (total_socket_cores > 0 && total_socket_cores < local_saturation_cores) {
            local_saturation_cores = total_socket_cores;
        }
        
        int upi_saturation_cores = std::max(1, static_cast<int>(local_saturation_cores * (upi_peak_gb_s / local_peak_gb_s)));
        
        double scale_factor = static_cast<double>(cores_used) / upi_saturation_cores;
        if (scale_factor > 1.0) scale_factor = 1.0;
        
        return upi_peak_gb_s * scale_factor;
    }

    static double calculate_achievable_peak(
        const CPUCapabilities& caps,
        int cores_used,
        int total_socket_cores = 0,
        bool is_remote_memory = false)
    {
        double theoretical_peak = calculate_from_capabilities(caps, is_remote_memory);
        if (is_remote_memory) {
            double local_peak = calculate_from_capabilities(caps, false);
            return scale_upi_for_core_count(theoretical_peak, local_peak, cores_used, 
                                            caps.memory_channels, total_socket_cores);
        }
        return scale_for_core_count(theoretical_peak, cores_used, caps.memory_channels, total_socket_cores);
    }

    static double get_noise_floor(double achievable_peak_gb_s) {
        double floor = achievable_peak_gb_s * 0.0005;
        return (floor < 0.05) ? 0.05 : floor;
    }

    static double get_warmup_threshold(double achievable_peak_gb_s) {
        double threshold;
        if(achievable_peak_gb_s < 700){
            threshold = achievable_peak_gb_s * 0.40;
        } else {
            threshold = achievable_peak_gb_s * 0.20;
        }

        const double max_threshold = 64.0;
        if (threshold > max_threshold) {
            threshold = max_threshold;
        }
        return threshold;
    }
};
