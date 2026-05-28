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

#include "SystemInfo.h"
#include "Utils.h"
#include <unistd.h>
#include <sys/utsname.h>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <cctype>
#include <sstream>
#include <set>

#ifdef __APPLE__
#include <sys/sysctl.h>
#include <mach/mach.h>
#include <sys/param.h>
#include <sys/mount.h>
#endif

static void strip_cpu_frequency(char* model) {
    if (!model) return;

    char* at_pos = std::strchr(model, '@');
    if (at_pos) {
        *at_pos = '\0';

        int len = std::strlen(model);
        while (len > 0 && (model[len-1] == ' ' || model[len-1] == '\t')) {
            model[--len] = '\0';
        }
    }
}

#ifndef __APPLE__
static std::string run_command_for_output(const char* cmd) {
    FILE* pipe = popen(cmd, "r");
    if (!pipe) return "";
    
    std::string result;
    char buffer[256];
    while (fgets(buffer, sizeof(buffer), pipe)) {
        result += buffer;
    }
    
    if (!pclose_success(pipe)) return "";
    
    return result;
}

static bool has_nvidia_scf_pmu() {
#if defined(__linux__)
    for (int i = 0; i < 64; ++i) {
        const std::string pmu_path =
            "/sys/bus/event_source/devices/nvidia_scf_pmu_" + std::to_string(i);
        if (access((pmu_path + "/type").c_str(), F_OK) == 0 ||
            access(pmu_path.c_str(), F_OK) == 0) {
            return true;
        }
    }
#endif
    return false;
}

static bool looks_like_grace_core(const std::string& model) {
    std::string upper = model;
    for (auto& c : upper) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return upper.find("NEOVERSE-V2") != std::string::npos ||
           upper.find("NEOVERSE V2") != std::string::npos ||
           upper.find("GRACE") != std::string::npos;
}

static int safe_stoi(const std::string& s, int fallback_val = 0) {
    try {
        size_t first = s.find_first_not_of(" \t");
        if (first == std::string::npos) return fallback_val;
        return std::stoi(s.substr(first));
    } catch (...) {
        return fallback_val;
    }
}

static int detect_cores_fallback(int* physical_cores, int* logical_cores, int* sockets, int* cores_per_socket) {
    std::string output = run_command_for_output("lscpu 2>/dev/null");
    if (!output.empty()) {
        std::istringstream iss(output);
        std::string line;
        int lscpu_sockets = 0;
        int lscpu_cores_per_socket = 0;
        int lscpu_clusters = 0;
        int lscpu_cores_per_cluster = 0;
        int lscpu_threads_per_core = 1;
        int lscpu_cpus = 0;

        while (std::getline(iss, line)) {
            if (line.find("Socket(s):") != std::string::npos) {
                size_t pos = line.find(':');
                if (pos != std::string::npos) {
                    lscpu_sockets = safe_stoi(line.substr(pos + 1));
                }
            } else if (line.find("Core(s) per socket:") != std::string::npos) {
                size_t pos = line.find(':');
                if (pos != std::string::npos) {
                    lscpu_cores_per_socket = safe_stoi(line.substr(pos + 1));
                }
            } else if (line.find("Core(s) per cluster:") != std::string::npos) {
                size_t pos = line.find(':');
                if (pos != std::string::npos) {
                    lscpu_cores_per_cluster = safe_stoi(line.substr(pos + 1));
                }
            } else if (line.find("Cluster(s):") != std::string::npos) {
                size_t pos = line.find(':');
                if (pos != std::string::npos) {
                    lscpu_clusters = safe_stoi(line.substr(pos + 1));
                }
            } else if (line.find("Thread(s) per core:") != std::string::npos) {
                size_t pos = line.find(':');
                if (pos != std::string::npos) {
                    lscpu_threads_per_core = safe_stoi(line.substr(pos + 1), 1);
                }
            } else if (line.find("CPU(s):") != std::string::npos && line.find("NUMA") == std::string::npos && line.find("On-line") == std::string::npos) {
                size_t pos = line.find(':');
                if (pos != std::string::npos) {
                    lscpu_cpus = safe_stoi(line.substr(pos + 1));
                }
            }
        }

        // For cluster-based architectures (e.g. A64FX)
        if (lscpu_sockets <= 0 && lscpu_cores_per_socket <= 0 &&
            lscpu_clusters > 0 && lscpu_cores_per_cluster > 0) {
            lscpu_sockets = 1;
            lscpu_cores_per_socket = lscpu_clusters * lscpu_cores_per_cluster;
        }

        if (lscpu_sockets > 0 && lscpu_cores_per_socket > 0) {
            if (sockets) *sockets = lscpu_sockets;
            if (cores_per_socket) *cores_per_socket = lscpu_cores_per_socket;
            if (physical_cores) *physical_cores = lscpu_sockets * lscpu_cores_per_socket;
            if (logical_cores) *logical_cores = lscpu_sockets * lscpu_cores_per_socket * lscpu_threads_per_core;
            return 1;
        }

        if (lscpu_cpus > 0) {
            if (logical_cores) *logical_cores = lscpu_cpus;
            if (physical_cores) *physical_cores = lscpu_cpus / (lscpu_threads_per_core > 0 ? lscpu_threads_per_core : 1);
            if (sockets) *sockets = lscpu_sockets > 0 ? lscpu_sockets : 1;
            if (cores_per_socket && *sockets > 0) *cores_per_socket = *physical_cores / *sockets;
            return 1;
        }
    }

    output = run_command_for_output("nproc --all 2>/dev/null");
    if (!output.empty()) {
        int nproc_count = safe_stoi(output);
        if (nproc_count > 0) {
            if (logical_cores) *logical_cores = nproc_count;
            if (physical_cores) *physical_cores = nproc_count;
            if (sockets) *sockets = 1;
            if (cores_per_socket) *cores_per_socket = nproc_count;
            return 1;
        }
    }
    
    long nprocs = sysconf(_SC_NPROCESSORS_ONLN);
    if (nprocs > 0) {
        if (logical_cores) *logical_cores = static_cast<int>(nprocs);
        if (physical_cores) *physical_cores = static_cast<int>(nprocs);
        if (sockets) *sockets = 1;
        if (cores_per_socket) *cores_per_socket = static_cast<int>(nprocs);
        return 1;
    }
    
    return 0;
}

static bool parse_memory_speed_from_output(const std::string& output, char* freq, int* channels) {
    int speed = 0;
    int channel_count = 0;
    
    std::istringstream iss(output);
    std::string line;
    
    while (std::getline(iss, line)) {
        size_t pos = line.find("Configured Memory Speed:");
        if (pos == std::string::npos) {
            pos = line.find("Configured Clock Speed:");
        }
        if (pos == std::string::npos) {
            pos = line.find("Speed:");
            if (pos != std::string::npos && line.find("Configured") != std::string::npos) {
                continue;
            }
        }
        
        if (pos != std::string::npos) {
            size_t num_start = line.find_first_of("0123456789", pos);
            if (num_start != std::string::npos) {
                int val = std::stoi(line.substr(num_start));
                if (val > speed && val > 100) {
                    speed = val;
                    channel_count++;
                }
            }
        }
        
        if (line.find("clock:") != std::string::npos && line.find("MHz") != std::string::npos) {
            size_t num_start = line.find("clock:") + 6;
            num_start = line.find_first_of("0123456789", num_start);
            if (num_start != std::string::npos) {
                int val = std::stoi(line.substr(num_start));
                int mts = val * 2;
                if (mts > speed && mts > 100) {
                    speed = mts;
                    channel_count++;
                }
            }
        }
    }
    
    if (speed > 0) {
        if (freq && !freq[0]) {
            snprintf(freq, 32, "%d MT/s", speed);
        }
        if (channels && channel_count > 0) {
            *channels = channel_count;
        }
        return true;
    }
    return false;
}

static bool try_command_with_sudo_fallback(const char* cmd, const char* sudo_cmd, 
                                           char* freq, int* channels,
                                           bool (*parser)(const std::string&, char*, int*)) {
    std::string output = run_command_for_output(cmd);
    if (!output.empty() && parser(output, freq, channels)) {
        return true;
    }
    
    output = run_command_for_output(sudo_cmd);
    if (!output.empty() && parser(output, freq, channels)) {
        return true;
    }
    
    return false;
}

static bool parse_decode_dimms_output(const std::string& output, char* freq, int* channels) {
    (void)channels;
    std::istringstream iss(output);
    std::string line;
    while (std::getline(iss, line)) {
        if (line.find("Maximum module speed") != std::string::npos ||
            line.find("Configured Memory Speed") != std::string::npos) {
            size_t num_start = line.find_first_of("0123456789");
            if (num_start != std::string::npos) {
                int val = std::stoi(line.substr(num_start));
                if (val > 100 && freq && !freq[0]) {
                    snprintf(freq, 32, "%d MT/s", val);
                    return true;
                }
            }
        }
    }
    return false;
}

static bool try_detect_memory_freq(char* freq, int* channels) {
    if (try_command_with_sudo_fallback(
            "dmidecode -t memory 2>/dev/null",
            "sudo -n dmidecode -t memory 2>/dev/null",
            freq, channels, parse_memory_speed_from_output)) {
        return true;
    }
    
    if (try_command_with_sudo_fallback(
            "lshw -C memory 2>/dev/null",
            "sudo -n lshw -C memory 2>/dev/null",
            freq, channels, parse_memory_speed_from_output)) {
        return true;
    }
    
    std::string output = run_command_for_output("decode-dimms 2>/dev/null");
    if (!output.empty() && parse_decode_dimms_output(output, freq, channels)) {
        return true;
    }
    
    output = run_command_for_output("hwinfo --memory 2>/dev/null");
    if (!output.empty() && parse_memory_speed_from_output(output, freq, channels)) {
        return true;
    }
    
    output = run_command_for_output("inxi -m 2>/dev/null");
    if (!output.empty()) {
        std::istringstream iss(output);
        std::string line;
        while (std::getline(iss, line)) {
            size_t speed_pos = line.find("speed:");
            if (speed_pos == std::string::npos) {
                speed_pos = line.find("Speed:");
            }
            if (speed_pos != std::string::npos) {
                size_t num_start = line.find_first_of("0123456789", speed_pos);
                if (num_start != std::string::npos) {
                    int val = std::stoi(line.substr(num_start));
                    if (val > 100 && freq && !freq[0]) {
                        snprintf(freq, 32, "%d MT/s", val);
                        return true;
                    }
                }
            }
        }
    }
    
    return false;
}
#endif

#ifndef __APPLE__
bool cpu_has_onpackage_hbm(const std::string& cpu_model) {
    std::string upper = cpu_model;
    for (auto& c : upper) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    const bool intel_xeon_max =
        upper.find("XEON") != std::string::npos && upper.find("MAX") != std::string::npos;
    const bool fujitsu_a64fx = upper.find("A64FX") != std::string::npos;
    return intel_xeon_max || fujitsu_a64fx;
}

static void infer_memory_info_from_cpu(const char* cpu_model, char* tech, char* freq, int* channels) {
    if (!cpu_model) return;
    
    std::string model = cpu_model;
    for (auto & c: model) c = toupper(c);

    if (model.find("PLATINUM") != std::string::npos || model.find("GOLD") != std::string::npos || model.find("SILVER") != std::string::npos || model.find("BRONZE") != std::string::npos) {
        if (model.find("84") != std::string::npos || model.find("64") != std::string::npos || 
            model.find("54") != std::string::npos || model.find("34") != std::string::npos ||
            model.find("85") != std::string::npos) {
            if (tech && !tech[0]) strcpy(tech, "DDR5");
            if (freq && !freq[0]) strcpy(freq, "4800 MT/s");
            if (channels) *channels = 8;
            return;
        }
        if (model.find("83") != std::string::npos || model.find("63") != std::string::npos || 
            model.find("53") != std::string::npos || model.find("43") != std::string::npos) {
            if (tech && !tech[0]) strcpy(tech, "DDR4");
            if (freq && !freq[0]) strcpy(freq, "3200 MT/s");
            if (channels) *channels = 8;
            return;
        }
        if (model.find("82") != std::string::npos || model.find("62") != std::string::npos || 
            model.find("52") != std::string::npos || model.find("42") != std::string::npos ||
            model.find("32") != std::string::npos) {
            if (tech && !tech[0]) strcpy(tech, "DDR4");
            if (freq && !freq[0]) strcpy(freq, "2933 MT/s");
            if (channels) *channels = 6;
            return;
        }
        if (model.find("81") != std::string::npos || model.find("61") != std::string::npos || 
            model.find("51") != std::string::npos || model.find("41") != std::string::npos ||
            model.find("31") != std::string::npos) {
            if (tech && !tech[0]) strcpy(tech, "DDR4");
            if (freq && !freq[0]) strcpy(freq, "2666 MT/s");
            if (channels) *channels = 6;
            return;
        }
    }
    
    // AMD EPYC
    if (model.find("EPYC") != std::string::npos) {
        if (model.find("9") != std::string::npos && model.find("4") != std::string::npos) {
             if (tech && !tech[0]) strcpy(tech, "DDR5");
             if (freq && !freq[0]) strcpy(freq, "4800 MT/s");
             if (channels) *channels = 12;
             return;
        }
        if (model.find("7") != std::string::npos) {
             if (tech && !tech[0]) strcpy(tech, "DDR4");
             if (freq && !freq[0]) strcpy(freq, "3200 MT/s");
             if (channels) *channels = 8;
             return;
        }
    }

    // Fujitsu A64FX (HBM2) - e.g. Fugaku supercomputer
    if (model.find("A64FX") != std::string::npos) {
        if (tech && !tech[0]) strcpy(tech, "HBM2");
        if (freq && !freq[0]) strcpy(freq, "2000 MT/s");
        if (channels) *channels = 4;
        return;
    }

    // Intel Xeon Max can expose both DDR and HBM as separate NUMA nodes.
    // Do not force a system-wide HBM label here; per-node HMAT/DAX detection
    // below decides whether a bound node is DDR or HBM.
    if (model.find("MAX") != std::string::npos || (model.find("94") != std::string::npos && model.find("HBM") != std::string::npos)) {
        if (freq && !freq[0]) strcpy(freq, "3200 MT/s");
        if (channels) *channels = 4;
        return;
    }

    // Intel Emerald Rapids (85xx)
    if (model.find("85") != std::string::npos) {
        if (tech && !tech[0]) strcpy(tech, "DDR5");
        if (freq && !freq[0]) strcpy(freq, "5600 MT/s");
        if (channels) *channels = 8;
        return;
    }

    // NVIDIA Grace (Neoverse-V2)
    if (model.find("NEOVERSE-V2") != std::string::npos || model.find("NEOVERSE V2") != std::string::npos || model.find("GRACE") != std::string::npos) {
        if (tech && !tech[0]) strcpy(tech, "LPDDR5X");
        if (freq && !freq[0]) strcpy(freq, "4266");
        if (channels) *channels = 16;
        return;
    }

    if (model.find("XEON") != std::string::npos) {
        size_t xeon_pos = model.find("XEON");
        std::string after_xeon = model.substr(xeon_pos + 4);
        
        size_t digit_start = after_xeon.find_first_of("0123456789");
        if (digit_start != std::string::npos) {
            std::string num_str;
            for (size_t i = digit_start; i < after_xeon.size() && std::isdigit(after_xeon[i]); ++i) {
                num_str += after_xeon[i];
            }
            
            if (!num_str.empty()) {
                int model_num = std::stoi(num_str);
                
                if (model_num >= 6000 && model_num < 7000) {
                    if (tech && !tech[0]) strcpy(tech, "DDR5");
                    if (freq && !freq[0]) strcpy(freq, "6400 MT/s");
                    if (channels) *channels = 12;
                    return;
                }
            }
        }
    }
}
#endif

int system_info_detect(system_info *out) {
    if (!out) return -1;

    std::memset(out, 0, sizeof(system_info));

    struct utsname uname_info;
    if (uname(&uname_info) == 0) {
        std::snprintf(out->os_name, sizeof(out->os_name), "%.*s", (int)sizeof(out->os_name)-1, uname_info.sysname);
        std::snprintf(out->os_release, sizeof(out->os_release), "%.*s", (int)sizeof(out->os_release)-1, uname_info.release);
        std::snprintf(out->os_version, sizeof(out->os_version), "%.*s", (int)sizeof(out->os_version)-1, uname_info.version);
        std::snprintf(out->arch, sizeof(out->arch), "%.*s", (int)sizeof(out->arch)-1, uname_info.machine);
    }

#ifdef __APPLE__
    size_t len;
    
    len = sizeof(out->cpu_model);
    sysctlbyname("machdep.cpu.brand_string", out->cpu_model, &len, NULL, 0);
    
    out->cpu_model[std::strcspn(out->cpu_model, "\n")] = 0;
    strip_cpu_frequency(out->cpu_model);
    
    std::snprintf(out->cpu_vendor, sizeof(out->cpu_vendor), "Apple");
    
    int physical_cores = 0;
    int logical_cores = 0;
    len = sizeof(physical_cores);
    sysctlbyname("hw.physicalcpu", &physical_cores, &len, NULL, 0);
    len = sizeof(logical_cores);
    sysctlbyname("hw.logicalcpu", &logical_cores, &len, NULL, 0);
    
    out->total_physical_cores = physical_cores;
    out->total_logical_cores = logical_cores;
    
    uint64_t mem_bytes = 0;
    len = sizeof(mem_bytes);
    sysctlbyname("hw.memsize", &mem_bytes, &len, NULL, 0);
    out->total_mem_bytes = mem_bytes;
    
    std::snprintf(out->mem_technology, sizeof(out->mem_technology), "LPDDR5");
    
#else
    FILE *cpuinfo = std::fopen("/proc/cpuinfo", "r");
    if (cpuinfo) {
        char line[256];
        char vendor[32] = "";
        char model[128] = "";
        int cores_per_socket = 0;
        int siblings = 0;

        while (std::fgets(line, sizeof(line), cpuinfo)) {
            if (std::strstr(line, "vendor_id")) {
                std::sscanf(line, "vendor_id : %31s", vendor);
            } else if (std::strstr(line, "model name")) {
                char *colon = std::strchr(line, ':');
                if (colon) {
                    std::snprintf(model, sizeof(model), "%s", colon + 2);
                    model[std::strcspn(model, "\n")] = 0;
                    strip_cpu_frequency(model);
                }
            } else if (std::strstr(line, "cpu cores")) {
                std::sscanf(line, "cpu cores : %d", &cores_per_socket);
            } else if (std::strstr(line, "siblings")) {
                std::sscanf(line, "siblings : %d", &siblings);
            }
        }
        
        std::rewind(cpuinfo);
        int socket_ids[64] = {0};
        int max_socket_id = -1;
        
        while (std::fgets(line, sizeof(line), cpuinfo)) {
            if (std::strstr(line, "physical id")) {
                int socket_id;
                if (std::sscanf(line, "physical id : %d", &socket_id) == 1) {
                    if (socket_id >= 0 && socket_id < 64) {
                        socket_ids[socket_id] = 1;
                        if (socket_id > max_socket_id) {
                            max_socket_id = socket_id;
                        }
                    }
                }
            }
        }
        
        std::fclose(cpuinfo);

        int socket_count = 0;
        for (int i = 0; i <= max_socket_id; i++) {
            if (socket_ids[i]) {
                socket_count++;
            }
        }
        
        if (socket_count == 0 && cores_per_socket > 0) {
            FILE *cpuinfo2 = std::fopen("/proc/cpuinfo", "r");
            int total_physical_cores = 0;
            if (cpuinfo2) {
                while (std::fgets(line, sizeof(line), cpuinfo2)) {
                    if (std::strstr(line, "cpu cores")) {
                        int cores;
                        if (std::sscanf(line, "cpu cores : %d", &cores) == 1) {
                            total_physical_cores += cores;
                        }
                    }
                }
                std::fclose(cpuinfo2);
            }
            if (total_physical_cores > 0 && cores_per_socket > 0) {
                socket_count = total_physical_cores / cores_per_socket;
            }
        }
        
        if (socket_count < 1) socket_count = 1;

        if (cores_per_socket <= 0 || siblings <= 0) {
            int fb_physical = 0, fb_logical = 0, fb_sockets = 0, fb_cores_per_socket = 0;
            if (detect_cores_fallback(&fb_physical, &fb_logical, &fb_sockets, &fb_cores_per_socket)) {
                if (cores_per_socket <= 0) cores_per_socket = fb_cores_per_socket;
                if (siblings <= 0) siblings = fb_logical / (fb_sockets > 0 ? fb_sockets : 1);
                if (socket_count <= 1 && fb_sockets > 0) socket_count = fb_sockets;
            }
        }

        if (cores_per_socket <= 0) {
            long nprocs = sysconf(_SC_NPROCESSORS_ONLN);
            if (nprocs > 0) {
                cores_per_socket = static_cast<int>(nprocs);
                if (siblings <= 0) siblings = static_cast<int>(nprocs);
            } else {
                cores_per_socket = 1;
                siblings = 1;
            }
        }

        out->total_physical_cores = cores_per_socket * socket_count;
        out->total_logical_cores = siblings > 0 ? siblings * socket_count : cores_per_socket * socket_count;

        // Fallback: If model or vendor is empty, try to get them from lscpu
        // This is needed for ARM systems where /proc/cpuinfo lacks vendor_id and model name
        if (model[0] == '\0' || vendor[0] == '\0') {
             std::string lscpu_output = run_command_for_output("lscpu 2>/dev/null");
             std::istringstream iss(lscpu_output);
             std::string line;
             while (std::getline(iss, line)) {
                 if (model[0] == '\0' && line.find("Model name:") != std::string::npos) {
                     size_t pos = line.find(':');
                     if (pos != std::string::npos) {
                         std::string m = line.substr(pos + 1);
                         size_t first = m.find_first_not_of(" \t");
                         if (first != std::string::npos) {
                             m = m.substr(first);
                         }
                         std::snprintf(model, sizeof(model), "%s", m.c_str());
                         model[std::strcspn(model, "\n")] = 0;
                         strip_cpu_frequency(model);
                     }
                 }
                 if (vendor[0] == '\0' && line.find("Vendor ID:") != std::string::npos) {
                     size_t pos = line.find(':');
                     if (pos != std::string::npos) {
                         std::string v = line.substr(pos + 1);
                         size_t first = v.find_first_not_of(" \t");
                         if (first != std::string::npos) {
                             v = v.substr(first);
                         }
                         std::snprintf(vendor, sizeof(vendor), "%s", v.c_str());
                         vendor[std::strcspn(vendor, "\n")] = 0;
                     }
                 }
             }
        }

        if (has_nvidia_scf_pmu() && looks_like_grace_core(model)) {
            std::snprintf(vendor, sizeof(vendor), "NVIDIA");
            std::snprintf(model, sizeof(model), "Grace (Neoverse-V2)");
        }

        std::snprintf(out->cpu_vendor, sizeof(out->cpu_vendor), "%s", vendor);
        std::snprintf(out->cpu_model, sizeof(out->cpu_model), "%s", model);
        
        int inferred_channels = 0;

        // Fast path first: infer from known CPU families and only run expensive probes when needed.
        infer_memory_info_from_cpu(out->cpu_model, out->mem_technology, out->mem_frequency, &inferred_channels);

        bool force_slow_memory_probe = false;
        if (const char* env = std::getenv("MESS_SLOW_MEM_PROBE")) {
            force_slow_memory_probe = (std::strcmp(env, "0") != 0);
        }

        if (!out->mem_frequency[0] || force_slow_memory_probe) {
            int detected_channels = inferred_channels;
            if (try_detect_memory_freq(out->mem_frequency, &detected_channels) && detected_channels > 0) {
                inferred_channels = detected_channels;
            }
        }

        if (!out->mem_technology[0]) {
            std::snprintf(out->mem_technology, sizeof(out->mem_technology), "Unknown");
        }
        
        if (!out->mem_frequency[0]) {
            std::snprintf(out->mem_frequency, sizeof(out->mem_frequency), "Unknown");
        }

        int detected_base_mhz = 0, detected_max_mhz = 0, detected_cur_mhz = 0;
        {
            auto read_khz = [](const char *path) -> int {
                FILE *f = std::fopen(path, "r");
                if (!f) return 0;
                int khz = 0;
                std::fscanf(f, "%d", &khz);
                std::fclose(f);
                return khz / 1000;
            };
            
            // Read frequencies from all CPUs to find maximum values across system
            for (int cpu = 0; cpu < out->total_logical_cores; cpu++) {
                char path[256];
                std::snprintf(path, sizeof(path), "/sys/devices/system/cpu/cpu%d/cpufreq/cpuinfo_max_freq", cpu);
                int cpu_max = read_khz(path);
                if (cpu_max > detected_max_mhz) detected_max_mhz = cpu_max;
                
                std::snprintf(path, sizeof(path), "/sys/devices/system/cpu/cpu%d/cpufreq/base_frequency", cpu);
                int cpu_base = read_khz(path);
                if (cpu_base > detected_base_mhz) detected_base_mhz = cpu_base;
                
                std::snprintf(path, sizeof(path), "/sys/devices/system/cpu/cpu%d/cpufreq/scaling_cur_freq", cpu);
                int cpu_cur = read_khz(path);
                if (cpu_cur > detected_cur_mhz) detected_cur_mhz = cpu_cur;
            }
            
            // Fallback to cpu0 if no frequencies were detected
            if (detected_max_mhz == 0) {
                detected_max_mhz = read_khz("/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq");
                detected_base_mhz = read_khz("/sys/devices/system/cpu/cpu0/cpufreq/base_frequency");
                detected_cur_mhz = read_khz("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq");
            }
            
        }

        // Store at system level (maximum values across all sockets)
        out->cpu_base_mhz = detected_base_mhz;
        out->cpu_max_mhz = detected_max_mhz;
        out->cpu_cur_mhz = detected_cur_mhz;

        out->socket_count = socket_count;
        for (int i = 0; i < socket_count && i < SI_MAX_SOCKETS; i++) {
            out->sockets[i].id = i;
            out->sockets[i].core_count = cores_per_socket;
            out->sockets[i].thread_count = siblings;
            out->sockets[i].mem_total_bytes = out->total_mem_bytes / socket_count;
            out->sockets[i].mem_channels = (inferred_channels > 0) ? inferred_channels : 2;
        }
    }

    FILE *meminfo = std::fopen("/proc/meminfo", "r");
    if (meminfo) {
        char line[256];
        while (std::fgets(line, sizeof(line), meminfo)) {
            if (std::strstr(line, "MemTotal")) {
                unsigned long kb;
                std::sscanf(line, "MemTotal: %lu kB", &kb);
                out->total_mem_bytes = kb * 1024;
                break;
            }
        }
        std::fclose(meminfo);
    }
#endif

    out->page_size = sysconf(_SC_PAGESIZE);

#ifndef __APPLE__
    const auto numa_memory_info =
        detect_numa_memory_info(out->cpu_model, cpu_has_onpackage_hbm(out->cpu_model));
    std::set<std::string> detected_types;
    int node_index = 0;
    for (const auto& kv : numa_memory_info) {
        if (node_index >= SI_MAX_NUMA_NODES) {
            break;
        }

        const NumaNodeMemoryInfo& node = kv.second;
        si_numa_node& out_node = out->numa_nodes[node_index++];
        out_node.id = node.id;
        out_node.mem_total_bytes = node.mem_total_bytes;
        out_node.has_cpus = node.has_cpus ? 1 : 0;
        out_node.is_dax_backed = node.is_dax_backed ? 1 : 0;
        out_node.read_bandwidth_mb_s = node.read_bandwidth_mb_s;
        out_node.read_latency_ns = node.read_latency_ns;
        std::snprintf(out_node.memory_type, sizeof(out_node.memory_type), "%s",
                      node.memory_type.empty() ? "UNKNOWN" : node.memory_type.c_str());

        if (!node.memory_type.empty() && node.memory_type != "UNKNOWN") {
            detected_types.insert(node.memory_type);
        }
    }
    out->numa_node_count = node_index;

    if (!detected_types.empty()) {
        const std::string scanned =
            (detected_types.size() == 1) ? *detected_types.begin() : std::string("Mixed");
        const bool scanned_is_ddr = scanned.find("DDR") != std::string::npos;
        const bool model_set_tech = out->mem_technology[0] != '\0';
        if (!scanned_is_ddr || !model_set_tech) {
            std::snprintf(out->mem_technology, sizeof(out->mem_technology), "%s", scanned.c_str());
        }
    }
#endif

    return 0;
}
