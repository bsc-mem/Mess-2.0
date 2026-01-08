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

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <regex>
#include <sstream>
#include <string>
#include <vector>
#include <unistd.h>
#include <libgen.h>
#include <limits.h>

#include "codegen.h"
#include "utils.h"
#include "architecture/ArchitectureRegistry.h"
#include "architecture/ISA.h"
#include "architecture/KernelAssembler.h"
#include "ptrchase_perf_helper.h"
#include "system_detection.h"


std::string read_template(const std::string& filename) {
    return codegen::TemplateEngine::Read(filename);
}

std::string replace_template_variables(const std::string& template_content, 
                                      const std::map<std::string, std::string>& replacements) {
    return codegen::TemplateEngine::Replace(template_content, replacements);
}


int main(int argc, char* argv[]) {
    bool debug_mode = false;
    bool build_multisequential = true;
    
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--debug") {
            debug_mode = true;
        }
    }
    

    std::filesystem::path root = get_project_root();

    bool installation_success = true;
    std::vector<std::string> pending_issues;

    std::cout << "\n\n╔════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║             Mess · Code Generator              ║" << std::endl;
    std::cout << "║  \033[3m      Detecting system configuration    \033[0m      ║" << std::endl;
    std::cout << "║  \033[3m    and generating executables for Mess...  \033[0m  ║" << std::endl;
    std::cout << "╚════════════════════════════════════════════════╝\n" << std::endl;

    SystemDetector detector;
    if (!detector.detect()) {
        std::cerr << "ERROR: Failed to detect system information" << std::endl;
        return 1;
    }
    
    auto capabilities = detector.get_capabilities();
    const system_info& sys_info = detector.get_system_info();

    KernelConfig config;
    KernelGenerator generator(config, capabilities);
    ArchitectureConfig arch_config = generator.setup_architecture_config();
    
    auto architecture = ArchitectureRegistry::instance().getArchitecture(capabilities);
    auto assembler = architecture->createAssembler(config);

    // Calculate TrafficGen array size and core count
    uint64_t arg1 = 5ULL * 1000 * 1000 * 1000;
    uint64_t arg2 = static_cast<uint64_t>(capabilities.l3_size) * 8;
    uint64_t min_traffic_gen_size = std::max(arg1, arg2) / sizeof(double);

    bool using_avx512 = (config.isa_mode == ISAMode::AVX512 || 
                        (config.isa_mode == ISAMode::AUTO && 
                         std::find(capabilities.extensions.begin(), 
                                  capabilities.extensions.end(), 
                                  ISAExtension::AVX512) != capabilities.extensions.end()));
    
    if (using_avx512) {
        min_traffic_gen_size *= 2;
    }

    int num_cores = sysconf(_SC_NPROCESSORS_ONLN);
    if (num_cores < 1) num_cores = 1;

    std::string output_dir = (root / "src/traffic_gen/src").string();
    generator.generate_kernel(output_dir);

    std::string ptr_chase_asm = assembler->getPointerChaseInstruction();
    std::string ptr_chase_loop = assembler->getPointerChaseLoopAsm();
    
    std::cout << "\n┌─ System Detection" << std::endl;
    std::cout << "│ CPU: " << capabilities.model_name << std::endl;
    
    auto detected_extensions = capabilities.extensions;
    std::cout << "│ Detected CPU extensions: ";
    for (const auto& ext : detected_extensions) {
        switch (ext) {
            case ISAExtension::SSE: std::cout << "SSE "; break;
            case ISAExtension::SSE2: std::cout << "SSE2 "; break;
            case ISAExtension::SSE3: std::cout << "SSE3 "; break;
            case ISAExtension::SSSE3: std::cout << "SSSE3 "; break;
            case ISAExtension::SSE4_1: std::cout << "SSE4.1 "; break;
            case ISAExtension::SSE4_2: std::cout << "SSE4.2 "; break;
            case ISAExtension::AVX: std::cout << "AVX "; break;
            case ISAExtension::AVX2: std::cout << "AVX2 "; break;
            case ISAExtension::AVX512: std::cout << "AVX512 "; break;
            case ISAExtension::NEON: std::cout << "NEON "; break;
            case ISAExtension::SVE: std::cout << "SVE "; break;
            case ISAExtension::SVE2: std::cout << "SVE2 "; break;
            case ISAExtension::RVV1_0: std::cout << "RVV1.0 "; break;
            case ISAExtension::RVV0_7: std::cout << "RVV0.7 "; break;
            case ISAExtension::VSX: std::cout << "VSX "; break;
            case ISAExtension::VMX: std::cout << "VMX "; break;
            default: std::cout << "Unknown "; break;
        }
    }
    std::cout << std::endl;
    
    std::cout << "│ Architecture: ";
    switch (capabilities.arch) {
        case CPUArchitecture::X86_64: std::cout << "x86_64"; break;
        case CPUArchitecture::ARM64: std::cout << "aarch64"; break;
        case CPUArchitecture::POWER: std::cout << "power"; break;
        case CPUArchitecture::RISCV64: std::cout << "riscv64"; break;
        default: std::cout << "unknown"; break;
    }
    std::cout << std::endl;
    
    TLBMeasurement tlb_measurement = measure_and_set_tlb_latency();
    double measured_tlb_ns = tlb_measurement.latency_ns;
    int cache_line_size = tlb_measurement.cache_line_size;

    std::cout << "│ Cache line: " << cache_line_size << " bytes" << std::endl;

    if (measured_tlb_ns < 0) {
        pending_issues.push_back("TLB measurement failed (using default values)");
        measured_tlb_ns = 0; 
        cache_line_size = get_cache_line_size();
    }
    std::cout << "│ TLB latency: " << std::fixed << std::setprecision(5) << measured_tlb_ns << " ns" << std::endl;
    
    std::ifstream paranoid_file("/proc/sys/kernel/perf_event_paranoid");
    int paranoid_level = 3;
    if (paranoid_file.is_open()) {
        paranoid_file >> paranoid_level;
        paranoid_file.close();
    }
    
    bool hw_counter_accessible = false;
    if (paranoid_level <= 1) {
        if (run_command_success("perf stat -e cycles:k true 2>/dev/null >/dev/null")) {
            hw_counter_accessible = true;
        }
    }
    
    if (!hw_counter_accessible) {
        std::cout << "│ \033[33mWarning: No access to hardware performance counters (paranoid level: " << paranoid_level << ")\033[0m" << std::endl;
        std::cout << "│ \033[33mThe benchmark will compile, but make sure you have access to hardware counters when running the benchmark\033[0m" << std::endl;
        pending_issues.push_back("Performance counter access denied (paranoid level: " + std::to_string(paranoid_level) + ")");
    } else {
        std::cout << "│ Access to HWC: OK (paranoid level: " << paranoid_level << ")" << std::endl;
    }
    std::cout << "└─" << std::endl;

    codegen::BuildManager build_manager;
    build_manager.EnsureDirectories({(root / "build").string(),
                                     (root / "build/bin").string(),
                                     (root / "build/lib").string(),
                                     (root / "build/lib/ptr_chase_src").string(),
                                     (root / "measuring").string()});
    
    uint64_t default_array_size = 1ULL * 1000 * 1000 * 1000;
    uint64_t l3_array = capabilities.l3_size * 4;
    uint64_t array_size = std::max(default_array_size, l3_array); 
    
    KernelParameters pc_params = {
        .array_size = array_size, 
        .read_ratio = 1.0,                       
        .stride = 1,                             
        .use_prefetch = false,                
        .use_nontemporal = false                 
    };

    uint64_t array_elements = pc_params.array_size / cache_line_size;
    uint64_t iterations = 10;
 
    uint64_t loop_accesses = 1024; 
    uint64_t burst_iterations = 1000;
    
    uint64_t total_accesses = iterations * loop_accesses;

    std::cout << "\n┌─ PointerChase Compilation" << std::endl;
    std::cout << "│ L3 Cache: " << (capabilities.l3_size / (1000*1000)) << " MB" << std::endl;
    std::cout << "│ Array size: " << (pc_params.array_size / (1000*1000)) << " MB (" << array_elements << " elements)" << std::endl;
    std::cout << "│ Loop accesses: " << loop_accesses << std::endl;
    std::cout << "│ Iterations per cycle: " << iterations << std::endl;
    uint64_t tlb1_raw = 0;
    uint64_t tlb2_raw = 0;
    bool use_tlb1 = false;
    bool use_tlb2 = false;
    ptrchase_perf::select_tlb_events_for_ptrchase(sys_info, tlb1_raw, tlb2_raw, use_tlb1, use_tlb2);

    std::map<std::string, std::string> replacements = {
        {"ARRAY_ELEMS_VALUE", std::to_string(array_elements)},
        {"ITERS_VALUE", std::to_string(iterations)},
        {"BURST_ITERS_VALUE", std::to_string(burst_iterations)},
        {"ARRAY_SIZE_VALUE", std::to_string(pc_params.array_size)},
        {"INSTRUCTIONS_VALUE", std::to_string(total_accesses)},
        {"ITERATIONS_VALUE", std::to_string(iterations)},
        {"CACHE_LINE_SIZE_VALUE", std::to_string(cache_line_size)},
        {"ASSEMBLY_INSTRUCTION", ptr_chase_asm},
        {"PTRCHASE_USE_TLB1", use_tlb1 ? "1" : "0"},
        {"PTRCHASE_TLB1_EVENT", use_tlb1 ? std::to_string(tlb1_raw) : "0"},
        {"PTRCHASE_USE_TLB2", use_tlb2 ? "1" : "0"},
        {"PTRCHASE_TLB2_EVENT", use_tlb2 ? std::to_string(tlb2_raw) : "0"},
        {"PTRCHASE_BURST_LOOP", assembler->generatePointerChaseBurstLoop()}
    };

    std::string ptr_chase_template = read_template((root / "templates/ptr_chase_template.c").string());
    std::string ptr_chase_code = replace_template_variables(ptr_chase_template, replacements);
    build_manager.WriteFile((root / "build/lib/ptr_chase_src/ptr_chase.c").string(), ptr_chase_code);
    
    std::cout << "│ Generated kernel: ptr_chase" << std::endl;

    std::string arraygen_template_content = read_template((root / "templates/arraygen_template.c").string());
    std::string arraygen_template_code = replace_template_variables(arraygen_template_content, replacements);
    
    build_manager.WriteFile((root / "build/lib/ptr_chase_src/arraygen.c").string(), arraygen_template_code);

    std::string loop_template_content = read_template((root / "templates/loop_template.h").string());
    std::string loop_template_code = replace_template_variables(loop_template_content, replacements);
    
    build_manager.WriteFile((root / "build/lib/ptr_chase_src/loop_template.h").string(), loop_template_code);

    std::string makefile_template = read_template((root / "templates/Makefile_template").string());
    std::string makefile_code = replace_template_variables(makefile_template, replacements);
    
    build_manager.WriteFile((root / "build/lib/ptr_chase_src/Makefile").string(), makefile_code);

    std::string ptr_array_env = std::to_string(pc_params.array_size);
    setenv("PTRCHASE_ARRAY_SIZE", ptr_array_env.c_str(), 1);
    std::string ptr_instr_env = std::to_string(total_accesses);
    std::string ptr_iters_env = std::to_string(iterations);
    setenv("PTRCHASE_NUM_INSTRUCTIONS", ptr_instr_env.c_str(), 1);
    setenv("PTRCHASE_NUM_ITERATIONS", ptr_iters_env.c_str(), 1);
    std::string ptr_burst_iters_env = std::to_string(burst_iterations);
    setenv("PTRCHASE_BURST_ITERATIONS", ptr_burst_iters_env.c_str(), 1);

    std::string make_cmd =
        "cd " + (root / "build/lib/ptr_chase_src").string() + " && " + std::string(debug_mode ? "make clean && make" : "make -s clean && make -s");
    if (!debug_mode) {
        make_cmd += " >/dev/null 2>&1";
    }

    if (!run_command_success(make_cmd)) {
        installation_success = false;
        std::string arch_name = capabilities.arch == CPUArchitecture::X86_64 ? "x86_64" :
                                capabilities.arch == CPUArchitecture::ARM64 ? "ARM64" :
                                capabilities.arch == CPUArchitecture::POWER ? "POWER" :
                                capabilities.arch == CPUArchitecture::RISCV64 ? "RISC-V" : "unknown";
        pending_issues.push_back("Pointer chase kernel compilation failed (" + arch_name + " architecture)");
        std::cout << "│ compiled successfully: No (compilation failed for " << arch_name << ")" << std::endl;
        
        std::string placeholder = "#!/bin/bash\n"
                                  "# Placeholder -compilation failed\n"
                                  "echo \"ERROR: ptr_chase compilation failed for this architecture.\"\n"
                                  "exit 1\n";
        build_manager.WriteFile((root / "build/bin/ptr_chase").string(), placeholder);
        build_manager.RunCommand("chmod +x " + (root / "build/bin/ptr_chase").string());
        build_manager.WriteFile((root / "build/bin/array.dat").string(), "");
    } else {
        build_manager.CopyFile((root / "build/lib/ptr_chase_src/ptr_chase").string(), (root / "build/bin/ptr_chase").string());
        if (std::filesystem::exists(root / "build/lib/ptr_chase_src/array.dat")) {
            build_manager.CopyFile((root / "build/lib/ptr_chase_src/array.dat").string(), (root / "build/bin/array.dat").string());
        }
        
        std::cout << "│ Compiled successfully: Yes" << std::endl;
        std::cout << "│ Generated random walk data: " << array_elements << " elements" << std::endl;
    }
    std::cout << "└─" << std::endl;

    KernelGenerator kernel_gen(config, capabilities);
    
    build_manager.EnsureDirectory((root / "src/traffic_gen/src").string());
    
    std::cout << "\n┌─ Generating Architecture-Specific NOP" << std::endl;
    try {
        std::string output_dir = (root / "src/traffic_gen/src").string();
        kernel_gen.generate_kernel(output_dir);

        std::string nop_path = (root / "src/traffic_gen/src/nop.c").string();
        if (std::filesystem::exists(nop_path)) {
            std::cout << "│ Architecture: " << architecture->getName() << std::endl;
        } else {
            std::cout << "│ ERROR: Failed to create nop.c" << std::endl;
        }
    } catch (const std::exception& e) {
        std::cout << "│ ERROR: " << e.what() << std::endl;
    }
    std::cout << "└─" << std::endl;
    
    std::cout << "\n┌─ Generating Optimized MultiSequential Utils" << std::endl;
    try {
        std::string arch_name = architecture->getName();
        std::string output_path = (root / "src/traffic_gen/src" / ("generated_utils_multiseq_" + arch_name + ".c")).string();
        
        if (std::filesystem::exists(output_path)) {
            std::shared_ptr<ISA> selected_isa;
            if (config.isa_mode == ISAMode::AUTO) {
                selected_isa = architecture->selectBestISA(capabilities);
            } else {
                auto supported_isas = architecture->getSupportedISAs();
                for (const auto& isa : supported_isas) {
                    if (isa->getMode() == config.isa_mode) {
                        selected_isa = isa;
                        break;
                    }
                }
                if (!selected_isa) {
                    selected_isa = architecture->selectBestISA(capabilities);
                }
            }
            
            std::string isa_name;
            if (config.isa_mode == ISAMode::AUTO) {
                isa_name = "AUTO (selected: " + selected_isa->getName() + ")";
            } else {
                isa_name = selected_isa->getName();
            }
            std::cout << "│ Architecture: " << arch_name << std::endl;
            std::cout << "│ Functions: 51 (ratios 0-100%)" << std::endl;
            std::cout << "│ Ops per pause block: " << config.ops_per_pause_block << std::endl;
            std::cout << "│ ISA mode: " << isa_name << std::endl;
            std::cout << "│ Vector width: " << selected_isa->getVectorWidthBits() << " bits" << std::endl;
            std::cout << "│ Max registers: " << selected_isa->getMaxSimdRegisters() << " (using: " << (config.single_registers ? 1 : config.num_simd_registers) << ")" << std::endl;
            std::cout << "│ Interleaving: " << (config.enable_interleaving ? "enabled" : "disabled") << std::endl;
            std::cout << "│ Store type: " << (config.use_nontemporal_stores ? "non-temporal" : "temporal") << std::endl;
        } else {
            std::cout << "│ ERROR: Output file not found at " << output_path << std::endl;
        }
    } catch (const std::exception& e) {
        std::cout << "│ ERROR: Exception caught: " << e.what() << std::endl;
    }
    std::cout << "└─" << std::endl;


    std::cout << "\n┌─ TrafficGen Compilation" << std::endl;
    

    auto describe_mode = [](bool has_mode, bool build_mode) {
        if (!has_mode) return "Not available";
        return build_mode ? "Selected" : "Skipped";
    };
    
    std::cout << "│ Architecture: " << architecture->getName() << std::endl;
    std::cout << "│ MultiSequential utils: " << describe_mode(arch_config.has_multisequential, build_multisequential) << std::endl;


    std::vector<std::string> target_names;
    target_names.push_back("traffic_gen_multiseq.x");
    
    
    std::string target_line = target_names.empty() ? "(none)" : target_names.front();
    for (size_t i = 1; i < target_names.size(); ++i) {
        target_line += ", " + target_names[i];
    }
    std::cout << "│ Target: " << target_line << std::endl;
    
        std::filesystem::create_directories(root / "src/traffic_gen/src");
        
        bool all_compiled = true;


            std::string mode_name = "multiseq";
            std::string utils_file = arch_config.utils_multiseq_file;


            std::filesystem::path utils_path = root / utils_file;
            std::string target_name = "traffic_gen_" + mode_name + ".x";
            std::string utils_link = (root / "src/traffic_gen/src/utils.c").string();
                        
            if (!std::filesystem::exists(utils_path)) {
                all_compiled = false;
                installation_success = false;
                pending_issues.push_back("Utils file not found: " + utils_path.string());
                std::cout << "│ ERROR: Utils file not found: " << utils_path << std::endl;
                std::cout << "│ ERROR: Please check if file exists or path is correct" << std::endl;
            }
            
            std::filesystem::remove(utils_link);
            std::filesystem::copy_file(utils_path, utils_link, std::filesystem::copy_options::overwrite_existing);
            
            if (!std::filesystem::exists(utils_link)) {
                std::cout << "│ ERROR: Failed to copy utils file to " << utils_link << std::endl;
                all_compiled = false;
                installation_success = false;
            }
            
            std::cout << "│ Compiling " << target_name << " (array: " << (min_traffic_gen_size * sizeof(double) / (1000*1000)) << " MB)" << std::endl;
            
            std::filesystem::create_directories(root / "src/traffic_gen/build");
            
            std::string clean_cmd = "cd " + (root / "src/traffic_gen").string() + " && make clean >/dev/null 2>&1";
            system(clean_cmd.c_str());
            
std::string build_cmd = "cd " + (root / "src/traffic_gen").string() + " && mkdir -p build && make -j" + std::to_string(num_cores) + " " + target_name + 
                                   " CFLAGS=\"-DTrafficGen_ARRAY_SIZE=" + std::to_string(min_traffic_gen_size) + "\"" +
                                   (debug_mode ? "" : " >/dev/null 2>&1");
            
            int build_result = system(build_cmd.c_str());
            
            if (WIFEXITED(build_result) && WEXITSTATUS(build_result) == 0) {
                std::string source_binary = (root / "src/traffic_gen" / target_name).string();
                std::string dest_binary = (root / "build/bin" / target_name).string();
                
                if (std::filesystem::exists(source_binary)) {
                    std::filesystem::copy_file(source_binary, dest_binary,
                                              std::filesystem::copy_options::overwrite_existing);
                    std::filesystem::remove(source_binary);
                    std::cout << "│ Compiled: " << target_name << " ✓" << std::endl;
                } else {
                    all_compiled = false;
                    pending_issues.push_back("TrafficGen " + mode_name + " binary not found after compilation");
                    std::cout << "│ Compiled: " << target_name << " ✗ (binary not found)" << std::endl;
                }
            } else {
                all_compiled = false;
                installation_success = false;
                pending_issues.push_back("TrafficGen " + mode_name + " compilation failed");
                std::cout << "│ Compiled: " << target_name << " ✗ (compilation failed)" << std::endl;
                
                std::ofstream fallback_mpi((root / "build/bin" / target_name).string());
                fallback_mpi << "#!/bin/bash\n";
                fallback_mpi << "# Fallback - compilation failed\n";
                fallback_mpi << "echo \"ERROR: " << target_name << " compilation failed.\"\n";
                fallback_mpi << "exit 1\n";
                fallback_mpi.close();
                system(("chmod +x " + (root / "build/bin" / target_name).string()).c_str());
            }
        
        system(("rm -rf " + (root / "src/traffic_gen/build").string()).c_str());
        std::filesystem::remove(root / "src/traffic_gen/src/utils.c");
        
        std::cout << "│ Status: " << (all_compiled ? "All variants compiled successfully" : "Some variants failed") << std::endl;
        std::cout << "└─" << std::endl;
    

    bool only_hw_counter_issue = (pending_issues.size() == 1 && 
                                pending_issues[0].find("Performance counter access denied") != std::string::npos);
    
    if ((installation_success && pending_issues.empty()) || 
        (installation_success && only_hw_counter_issue)) {
        std::cout << "\nInstallation complete. Generated kernels are ready in build/bin/" << std::endl;
    } else {
        std::cout << "\n✗ Installation failed. Pending issues to correct:" << std::endl;
        for (const auto& issue : pending_issues) {
            std::cout << "  - " << issue << std::endl;
        }
        std::cout << "\nSome components may not function correctly until these issues are resolved." << std::endl;
    }

    return 0;
}