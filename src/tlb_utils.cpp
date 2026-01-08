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

#include "utils.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <cstdio>
#include <string>
#include <sys/wait.h>
#include <unistd.h>
#ifdef __APPLE__
#include <sys/sysctl.h>
#endif

int get_cache_line_size() {
#if defined(__APPLE__)
    size_t line_size = 0;
    size_t sizeof_line_size = sizeof(line_size);
    if (sysctlbyname("hw.cachelinesize", &line_size, &sizeof_line_size, 0, 0) == 0) {
        return line_size;
    }
    return 64;
#else
    std::ifstream cache_file("/sys/devices/system/cpu/cpu0/cache/index0/coherency_line_size");
    if (cache_file.is_open()) {
        int line_size;
        cache_file >> line_size;
        cache_file.close();
        if (line_size > 0) {
            return line_size;
        }
    }
    
    FILE* fp = popen("getconf LEVEL1_DCACHE_LINESIZE", "r");
    if (fp) {
        char buffer[128];
        if (fgets(buffer, sizeof(buffer), fp)) {
            int line_size = atoi(buffer);
            pclose_success(fp);
            if (line_size > 0) {
                return line_size;
            }
        }
        pclose_success(fp);
    }
    
    return 64;
#endif
}

TLBMeasurement measure_and_set_tlb_latency(int attempt) {
    TLBMeasurement result;
    result.cache_line_size = get_cache_line_size();
    
#ifdef __APPLE__
    result.latency_ns = 1.0; 
    return result;
#endif

    const char* tlb_env = std::getenv("TLB_HIT_LATENCY_NS");
    if (tlb_env) {
        result.latency_ns = std::stod(tlb_env);
        return result;
    }
    
    if (attempt > 1) {
        std::cout << "TLB measurement attempt " << attempt << std::endl;
    }
    
    result.latency_ns = 0.0;
    result.cache_line_size = get_cache_line_size();
    
    std::string tlb_binary_path;
    bool binary_found = false;
    
    std::filesystem::path root = get_project_root();
    
    std::vector<std::filesystem::path> possible_paths = {
        root / "build/bin/tlb",
        root / "tlb",
        root / "bin/tlb"
    };
    
    for (const auto& path : possible_paths) {
        if (access(path.c_str(), X_OK) == 0) {
            tlb_binary_path = path.string();
            binary_found = true;
            break;
        }
    }
    
    if (!binary_found) {
        bool patch_success = false;
        std::string patch_error;
        std::filesystem::path bench_header_path = root / "tools/lmbench/src/bench.h";
        std::ifstream bench_file(bench_header_path);
        if (bench_file.is_open()) {
            std::stringstream buffer;
            std::string line;
            bool patched = false;
            bool already_patched = true; 
            
            while (std::getline(bench_file, line)) {
                if (line.find("#define PORTMAP") != std::string::npos && line.find("/*") == std::string::npos) {
                    buffer << "/* " << line << " */\n";
                    patched = true;
                    already_patched = false;
                } else if (line.find("#include") != std::string::npos && line.find("rpc/") != std::string::npos && line.find("/*") == std::string::npos) {
                    buffer << "/* " << line << " */\n";
                    patched = true;
                    already_patched = false;
                } else {
                    buffer << line << "\n";
                }
            }
            bench_file.close();
            
            if (already_patched) {
                patch_success = true;
                patch_error.clear();
            } else if (patched) {
                std::ofstream out_file(bench_header_path);
                if (out_file.is_open()) {
                    out_file << buffer.str();
                    out_file.close();
                    patch_success = true;
                    patch_error.clear();
                } else {
                    patch_error = "bench.h could not be rewritten (check permissions).";
                }
            } else {
                patch_error = "Expected PORTMAP/rpc includes were not found in bench.h.";
            }
        } else {
            patch_error = "bench.h is missing or unreadable.";
        }
        
        if (!patch_success) {
            std::cerr << "ERROR: Failed to patch " << bench_header_path << ". "
                      << (patch_error.empty() ? "Unknown issue." : patch_error) << std::endl;
            std::cerr << "LMBench could not be compiled. Make sure to have the necessary files under the tools directory.\n"
                      << "If you don't, try running: \"git submodule update --init --recursive\" to download them." << std::endl;
            std::exit(EXIT_FAILURE);
        }
        
        std::filesystem::path tools_src = root / "tools/lmbench/src";
        std::filesystem::path build_bin_tlb = root / "build/bin/tlb";
        
        std::string tlb_compile_cmd = "gcc -O2 -lm -I " + tools_src.string() + " -o " + build_bin_tlb.string() + " " +
                                      (tools_src / "tlb.c").string() + " " +
                                      (tools_src / "lib_timing.c").string() + " " +
                                      (tools_src / "lib_stats.c").string() + " " +
                                      (tools_src / "lib_mem.c").string() + " " +
                                      (tools_src / "lib_debug.c").string() + " " +
                                      (tools_src / "getopt.c").string() + " " +
                                      (tools_src / "lib_sched.c").string() + " 2>&1";
        
        FILE* compile_pipe = popen(tlb_compile_cmd.c_str(), "r");
        if (compile_pipe) {
            char compile_buffer[256];
            std::string compile_output;
            while (fgets(compile_buffer, sizeof(compile_buffer), compile_pipe)) {
                compile_output += compile_buffer;
            }
            int compile_ret = pclose_success(compile_pipe) ? 0 : -1;
            
            if (compile_ret != 0) {
                std::cerr << "ERROR: Failed to compile TLB measurement tool";
                if (attempt > 0) std::cerr << " (attempt " << attempt << "/5)";
                std::cerr << std::endl;
                if (!compile_output.empty()) {
                    std::cerr << "Compilation output:" << std::endl;
                    std::cerr << compile_output << std::endl;
                }
                std::cerr << "LMBench could not be compiled. Make sure to have the necessary files under the tools directory.\n"
                          << "If you don't, try running: \"git submodule update --init --recursive\" to download them." << std::endl;
                std::exit(EXIT_FAILURE);
            }
        } else {
            std::cerr << "ERROR: Failed to execute compilation command";
            if (attempt > 0) std::cerr << " (attempt " << attempt << "/5)";
            std::cerr << std::endl;
            std::cerr << "LMBench could not be compiled. Make sure to have the necessary files under the tools directory.\n"
                      << "If you don't, try running: \"git submodule update --init --recursive\" to download them." << std::endl;
            std::exit(EXIT_FAILURE);
        }
        tlb_binary_path = build_bin_tlb.string();
    }

    
    std::string tlb_run_cmd = tlb_binary_path + " -c -L " + std::to_string(result.cache_line_size) + " -N 10 2>&1";

    
    FILE* pipe = popen(tlb_run_cmd.c_str(), "r");
    if (!pipe) {
        std::cerr << "WARNING: Failed to run TLB measurement command: " << tlb_run_cmd;
        if (attempt > 0) std::cerr << " (attempt " << attempt << "/5)";
        std::cerr << std::endl;
        std::cerr << "Using default TLB hit latency: 2.33921 ns" << std::endl;
        result.latency_ns = 2.33921;
        return result;
    }
    
    char buffer[256];
    std::string output;
    while (fgets(buffer, sizeof(buffer), pipe)) {
        output += buffer;
    }
    pclose_success(pipe);

    double tlb_ns = 0.0;
    
    size_t pages_pos = output.find("pages");
    if (pages_pos != std::string::npos) {
        size_t ns_start = output.find_first_of("0123456789.", pages_pos);
        size_t ns_end = output.find("nanoseconds", ns_start);
        if (ns_start != std::string::npos && ns_end != std::string::npos) {
            std::string ns_str = output.substr(ns_start, ns_end - ns_start);
            try {
                tlb_ns = std::stod(ns_str);
            } catch (...) {
                std::cerr << "Failed to parse: '" << ns_str << "'" << std::endl;
            }
        }
    }
    
    if (tlb_ns == 0.0) {
        size_t search_pos = 0;
        while ((search_pos = output.find("nanoseconds", search_pos)) != std::string::npos) {
            size_t num_end = search_pos;
            while (num_end > 0 && (output[num_end - 1] == ' ' || output[num_end - 1] == '\t')) {
                num_end--;
            }
            size_t num_start = num_end;
            while (num_start > 0 && (isdigit(output[num_start - 1]) || output[num_start - 1] == '.')) {
                num_start--;
            }
            
            if (num_start < num_end) {
                std::string ns_str = output.substr(num_start, num_end - num_start);
                try {
                    double candidate = std::stod(ns_str);
                    if (candidate > 0.0 && candidate < 1000.0) { 
                        tlb_ns = candidate;
                        std::cout << "Parsed TLB latency (strategy 2): " << tlb_ns << " ns" << std::endl;
                        break;
                    }
                } catch (...) {}
            }
            search_pos++;
        }
    }
    
    if (tlb_ns == 0.0) {
        std::cerr << "WARNING: Failed to parse TLB measurement output";
        if (attempt > 0) std::cerr << " (attempt " << attempt << "/5)";
        std::cerr << std::endl;
        tlb_ns = 2.33921;
    }
    
    if (setenv("TLB_HIT_LATENCY_NS", std::to_string(tlb_ns).c_str(), 1) == 0) {
        result.latency_ns = tlb_ns;
    } else {
        std::cerr << "ERROR: Failed to set TLB_HIT_LATENCY_NS environment variable" << std::endl;
        result.latency_ns = tlb_ns;
    }
    
    return result;
}
