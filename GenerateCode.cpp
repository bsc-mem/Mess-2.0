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
#include <thread>
#include <unistd.h>
#include <libgen.h>
#include <limits.h>

#include "Codegen.h"
#include "Utils.h"
#include "arch/arm/ArmISAUtils.h"
#include "architecture/ArchitectureRegistry.h"
#include "architecture/ISA.h"
#include "architecture/KernelAssembler.h"
#include "PtrchasePerfHelper.h"
#include "SystemDetection.h"

namespace {

ArchitectureConfig default_architecture_config(const Architecture& architecture) {
    ArchitectureConfig arch_config;
    arch_config.arch_name = architecture.getName();
    arch_config.utils_multiseq_file = "src/traffic_gen/src/generated_utils_multiseq_" + arch_config.arch_name + ".c";
    arch_config.has_multisequential = true;
    arch_config.pointer_chase_template = "templates/ptr_chase_template.c";
    return arch_config;
}

bool isTrueVLIssue(const std::string& reason) {
    return reason.rfind("Requested true-VL mode '", 0) == 0;
}

std::string compactConsoleIssue(const std::string& reason) {
    if (isTrueVLIssue(reason)) {
        const std::string prefix = "Requested true-VL mode '";
        const size_t mode_begin = prefix.size();
        const size_t mode_end = reason.find('\'', mode_begin);
        if (mode_end != std::string::npos) {
            return "true-VL mode '" + reason.substr(mode_begin, mode_end - mode_begin) +
                   "' is unsupported on this machine";
        }
    }

    const size_t newline_pos = reason.find('\n');
    if (newline_pos != std::string::npos) {
        return reason.substr(0, newline_pos);
    }
    return reason;
}

std::string extractIssueDetail(const std::string& reason) {
    const size_t open_paren = reason.find('(');
    const size_t close_paren = reason.rfind(')');
    if (open_paren != std::string::npos && close_paren != std::string::npos && close_paren > open_paren + 1) {
        return reason.substr(open_paren + 1, close_paren - open_paren - 1);
    }
    return "";
}

void printConsoleIssue(const std::string& label, const std::string& reason) {
    std::cout << "│ " << label << compactConsoleIssue(reason) << std::endl;
    const std::string detail = extractIssueDetail(reason);
    if (!detail.empty()) {
        std::cout << "│ Detail: " << detail << std::endl;
    }
}


std::string to_string(AddressingPolicy policy) {
    switch (policy) {
        case AddressingPolicy::AUTO: return "AUTO";
        case AddressingPolicy::POST_INCREMENT: return "POST_INCREMENT";
        case AddressingPolicy::INDEXED: return "INDEXED";
        default: return "UNKNOWN";
    }
}

std::string to_string(StoreLinePolicy policy) {
    switch (policy) {
        case StoreLinePolicy::AUTO: return "AUTO";
        case StoreLinePolicy::ISA_NATIVE: return "ISA_NATIVE";
        case StoreLinePolicy::CACHELINE_64B: return "CACHELINE_64B";
        case StoreLinePolicy::CACHELINE_NATIVE: return "CACHELINE_NATIVE";
        default: return "UNKNOWN";
    }
}

int resolve_vector_width_bits(ISAMode mode, const CPUCapabilities& caps, int fallback_bits) {
    switch (arm_isa_utils::canonicalizePublicMode(mode)) {
        case ISAMode::SVE128: return 128;
        case ISAMode::SVE256: return 256;
        case ISAMode::SVE512: return 512;
        case ISAMode::SVE:
            if (caps.sve_vector_bits > 0) {
                return caps.sve_vector_bits;
            }
            if (caps.sve_max_vector_bits > 0) {
                return caps.sve_max_vector_bits;
            }
            break;
        case ISAMode::NEON_PAIR:
            return 256;
        default:
            break;
    }
    if ((mode == ISAMode::RVV1_0 || mode == ISAMode::RVV0_7) && caps.rvv_vector_bits > 0) {
        return caps.rvv_vector_bits;
    }
    return fallback_bits;
}

StoreLinePolicy resolve_store_line_policy(StoreLinePolicy policy) {
    return (policy == StoreLinePolicy::AUTO) ? StoreLinePolicy::ISA_NATIVE : policy;
}

ISAMode resolve_effective_isa_mode(const KernelConfig& config,
                                   const CPUCapabilities& caps,
                                   const Architecture& architecture) {
    if (config.isa_mode != ISAMode::AUTO) {
        return arm_isa_utils::canonicalizePublicMode(config.isa_mode);
    }
    return arm_isa_utils::canonicalizePublicMode(architecture.selectBestISA(caps)->getMode());
}


AddressingPolicy resolve_addressing_policy(AddressingPolicy policy, CPUArchitecture arch) {
    if (policy != AddressingPolicy::AUTO) {
        return policy;
    }
    if (arch == CPUArchitecture::ARM64) {
        return AddressingPolicy::POST_INCREMENT;
    }
    if (arch == CPUArchitecture::X86_64) {
        return AddressingPolicy::INDEXED;
    }
    // Non-x86/ARM backends do not currently use addressing-policy forms.
    return AddressingPolicy::INDEXED;
}

}


std::string read_template(const std::string& filename) {
    return codegen::TemplateEngine::Read(filename);
}

std::string replace_template_variables(const std::string& template_content, 
                                      const std::map<std::string, std::string>& replacements) {
    return codegen::TemplateEngine::Replace(template_content, replacements);
}


int main(int argc, char* argv[]) {
    try {
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
    auto architecture = ArchitectureRegistry::instance().getArchitecture(capabilities);
    if (!architecture) {
        throw std::runtime_error("Unsupported architecture");
    }
    config.isa_mode = arm_isa_utils::canonicalizePublicMode(config.isa_mode);
    ArchitectureConfig arch_config = default_architecture_config(*architecture);
    auto assembler = architecture->createAssembler(config);
    const std::string preflight_multiseq_sve_error = arm_isa_utils::validateTrueSVEMode(config.isa_mode, capabilities);

    int num_cores = (sys_info.socket_count > 0) ? sys_info.sockets[0].core_count : 1;
    if (num_cores < 1) num_cores = 1;

    bool using_avx512 = (config.isa_mode == ISAMode::AVX512);

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
    if (capabilities.sve_vector_bits > 0) {
        std::cout << "│ SVE vector length: " << capabilities.sve_vector_bits << " bits" << std::endl;
    }
    if (capabilities.sve_max_vector_bits > 0 && capabilities.sve_max_vector_bits != capabilities.sve_vector_bits) {
        std::cout << "│ SVE max vector length: " << capabilities.sve_max_vector_bits << " bits" << std::endl;
    }
    if (capabilities.rvv_vector_bits > 0) {
        std::cout << "│ RVV vector length: " << capabilities.rvv_vector_bits << " bits" << std::endl;
    }
    
    std::cout << "│ Architecture: ";
    switch (capabilities.arch) {
        case CPUArchitecture::X86_64: std::cout << "x86_64"; break;
        case CPUArchitecture::ARM64: std::cout << "aarch64"; break;
        case CPUArchitecture::POWERPC: std::cout << "power"; break;
        case CPUArchitecture::RISCV64: std::cout << "riscv64"; break;
        default: std::cout << "unknown"; break;
    }
    std::cout << std::endl;
    
    int cache_line_size = 64;
    double measured_tlb_ns = 0;

    if(capabilities.arch == CPUArchitecture::RISCV64) {
        std::cout << "│ \033[33mWarning: RISC-V support is work-in-progress. TLB miss latency couldn't be measured.\033[0m" << std::endl;
        std::cout << "│ \033[33mThe benchmark will compile, but make sure you have access to hardware counters when running the benchmark\033[0m" << std::endl;
    }else{
        TLBMeasurement tlb_measurement = measure_and_set_tlb_latency();
        measured_tlb_ns = tlb_measurement.latency_ns;
        cache_line_size = tlb_measurement.cache_line_size;
    }


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
    
    const bool is_a64fx_system = capabilities.isA64FX();

    uint64_t default_array_size = 1ULL * 1024 * 1024 * 1024;
    uint64_t a64fx_array_size = 512ULL * 1024 * 1024;
    uint64_t l3_array = capabilities.l3_size * 4;
    uint64_t array_size = is_a64fx_system ? a64fx_array_size : std::max(default_array_size, l3_array);
    
    KernelParameters pc_params = {
        .array_size = array_size,
        .read_ratio = 1.0,
        .stride = 1,
        .use_nontemporal = false
    };

    const uint64_t ptrchase_cache_line = is_a64fx_system ? 128ULL : 64ULL;
    uint64_t array_elements = pc_params.array_size / ptrchase_cache_line;
    uint64_t active_array_elements = array_elements;
    uint64_t iterations = is_a64fx_system ? 50000ULL : 10ULL;
    uint64_t ptrchase_alloc_alignment = is_a64fx_system ? ptrchase_cache_line : 2ULL * 1024 * 1024;

    uint64_t loop_accesses = is_a64fx_system ? 4000ULL : 1024ULL;
    uint64_t burst_iterations = is_a64fx_system ? 250ULL : 1000ULL;
    
    uint64_t total_accesses = iterations * loop_accesses;

    std::cout << "\n┌─ PointerChase Compilation" << std::endl;
    std::cout << "│ LLC: " << (capabilities.l3_size / (1000*1000)) << " MB" << std::endl;
    std::cout << "│ Array size: " << (pc_params.array_size / (1000*1000)) << " MB (" << array_elements << " allocated elements)" << std::endl;
    std::cout << "│ PtrChase walk line: " << ptrchase_cache_line << " bytes" << std::endl;
    if (active_array_elements != array_elements) {
        const uint64_t active_bytes = active_array_elements * ptrchase_cache_line;
        std::cout << "│ PtrChase active walk: " << (active_bytes / (1000 * 1000)) << " MB (" << active_array_elements << " elements)" << std::endl;
    }
    std::cout << "│ Loop accesses: " << loop_accesses << std::endl;
    std::cout << "│ Iterations per cycle: " << iterations << std::endl;
    uint64_t tlb1_raw = 0;
    uint64_t tlb2_raw = 0;
    bool use_tlb1 = false;
    bool use_tlb2 = false;

    // Use the already-resolved architecture to get TLB events directly,
    // rather than going through select_tlb_events_for_ptrchase(sys_info)
    // which can fail on ARM when /proc/cpuinfo lacks vendor_id/model name fields.
    if (architecture) {
        auto counter_strategy = architecture->createCounterStrategy(capabilities);
        if (counter_strategy) {
            counter_strategy->getTlbMissCounters(tlb1_raw, tlb2_raw, use_tlb1, use_tlb2);
        }
    }
    if (!use_tlb1 && !use_tlb2) {
        // Fallback to the sys_info-based path
        ptrchase_perf::select_tlb_events_for_ptrchase(sys_info, tlb1_raw, tlb2_raw, use_tlb1, use_tlb2);
    }

    std::map<std::string, std::string> replacements = {
        {"ARRAY_ELEMS_VALUE", std::to_string(array_elements)},
        {"ACTIVE_ARRAY_ELEMS_VALUE", std::to_string(active_array_elements)},
        {"ITERS_VALUE", std::to_string(iterations)},
        {"BURST_ITERS_VALUE", std::to_string(burst_iterations)},
        {"ARRAY_SIZE_VALUE", std::to_string(pc_params.array_size)},
        {"INSTRUCTIONS_VALUE", std::to_string(total_accesses)},
        {"ITERATIONS_VALUE", std::to_string(iterations)},
        {"CACHE_LINE_SIZE_VALUE", std::to_string(ptrchase_cache_line)},
        {"PTRCHASE_CACHE_LINE_VALUE", std::to_string(ptrchase_cache_line)},
        {"ASSEMBLY_INSTRUCTION", ptr_chase_asm},
        {"PTRCHASE_USE_TLB1", use_tlb1 ? "1" : "0"},
        {"PTRCHASE_TLB1_EVENT", use_tlb1 ? std::to_string(tlb1_raw) : "0"},
        {"PTRCHASE_USE_TLB2", use_tlb2 ? "1" : "0"},
        {"PTRCHASE_TLB2_EVENT", use_tlb2 ? std::to_string(tlb2_raw) : "0"},
        {"PTRCHASE_DISABLE_HUGEPAGE_VALUE", "0"},
        {"PTRCHASE_ALLOC_ALIGNMENT_VALUE", std::to_string(ptrchase_alloc_alignment)},
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
    const unsigned int host_threads = std::max(1u, std::thread::hardware_concurrency());
    const int compile_jobs = static_cast<int>(std::max(1u, std::min(8u, host_threads)));

    std::string make_cmd =
        "cd " + (root / "build/lib/ptr_chase_src").string() + " && " + std::string(debug_mode ? "make clean && make" : "make -s clean && make -s");
    if (!debug_mode) {
        make_cmd += " >/dev/null 2>&1";
    }

    if (!run_command_success(make_cmd)) {
        installation_success = false;
        std::string arch_name = capabilities.arch == CPUArchitecture::X86_64 ? "x86_64" :
                                capabilities.arch == CPUArchitecture::ARM64 ? "ARM64" :
                                capabilities.arch == CPUArchitecture::POWERPC ? "POWERPC" :
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
        std::cout << "│ Generated random walk data: " << active_array_elements << " active elements";
        if (active_array_elements != array_elements) {
            std::cout << " (" << array_elements << " allocated)";
        }
        std::cout << std::endl;
    }
    std::cout << "└─" << std::endl;

    build_manager.EnsureDirectory((root / "src/traffic_gen/src").string());
    std::string utils_generation_error;
    if (!preflight_multiseq_sve_error.empty()) {
        utils_generation_error = preflight_multiseq_sve_error;
    }
    if (!utils_generation_error.empty()) {
        installation_success = false;
        pending_issues.push_back(utils_generation_error);
        std::filesystem::remove(root / arch_config.utils_multiseq_file);
    }
    
    std::cout << "\n┌─ Generating Architecture-Specific NOP" << std::endl;
    if (!utils_generation_error.empty()) {
        printConsoleIssue("ERROR: ", utils_generation_error);
    } else {
        try {
            KernelGenerator kernel_gen(config, capabilities);
            arch_config = kernel_gen.setup_architecture_config();
            std::string output_dir = (root / "src/traffic_gen/src").string();
            kernel_gen.generate_kernel(output_dir);

            std::string nop_path = (root / "src/traffic_gen/src/nop.c").string();
            if (std::filesystem::exists(nop_path)) {
                std::cout << "│ Architecture: " << architecture->getName() << std::endl;
            } else {
                std::cout << "│ ERROR: Failed to create nop.c" << std::endl;
            }
        } catch (const std::exception& e) {
            installation_success = false;
            utils_generation_error = e.what();
            pending_issues.push_back(utils_generation_error);
            std::cout << "│ ERROR: " << compactConsoleIssue(utils_generation_error) << std::endl;
            std::filesystem::remove(root / arch_config.utils_multiseq_file);
        }
    }
    std::cout << "└─" << std::endl;
    
    std::cout << "\n┌─ Generating Optimized MultiSequential Utils" << std::endl;
    try {
        std::string arch_name = architecture->getName();
        std::string output_path = (root / "src/traffic_gen/src" / ("generated_utils_multiseq_" + arch_name + ".c")).string();
        
        if (!utils_generation_error.empty()) {
            printConsoleIssue("ERROR: ", utils_generation_error);
        } else if (std::filesystem::exists(output_path)) {
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

            const ISAMode effective_mode = (config.isa_mode == ISAMode::AUTO) ? selected_isa->getMode() : config.isa_mode;
            const int effective_bits = resolve_vector_width_bits(effective_mode, capabilities, selected_isa->getVectorWidthBits());
            const int effective_bytes = std::max(1, effective_bits / 8);
            const int max_regs = std::max(1, selected_isa->getMaxSimdRegisters());
            const int used_regs = config.single_registers ? 1 : std::max(1, std::min(config.num_simd_registers, max_regs));
            const StoreLinePolicy resolved_store_policy = resolve_store_line_policy(config.store_line_policy);
            const AddressingPolicy selected_addressing_policy = resolve_addressing_policy(config.addressing_policy, capabilities.arch);

            std::cout << "│ Architecture: " << arch_name << std::endl;
            std::cout << "│ Functions: 51 (ratios 0-100%)" << std::endl;
            std::cout << "│ Ops per pause block: " << config.ops_per_pause_block << std::endl;
            std::cout << "│ ISA mode: " << isa_name << std::endl;
            std::cout << "│ Vector width: " << selected_isa->getVectorWidthBits() << " bits";
            if (effective_bits != selected_isa->getVectorWidthBits()) {
                std::cout << " (machine: " << effective_bits << " bits)";
            }
            std::cout << std::endl;
            std::cout << "│ Resolved bytes/op: " << effective_bytes << " bytes" << std::endl;
            std::cout << "│ Max registers: " << max_regs << " (using: " << used_regs << ")" << std::endl;
            std::cout << "│ Interleaving: " << (config.enable_interleaving ? "enabled" : "disabled") << std::endl;
            std::cout << "│ Addressing policy: " << to_string(config.addressing_policy);
            if (config.addressing_policy == AddressingPolicy::AUTO) {
                std::cout << " (selected: " << to_string(selected_addressing_policy) << ")";
            }
            std::cout << std::endl;
            std::cout << "│ Store line policy: " << to_string(config.store_line_policy);
            if (config.store_line_policy == StoreLinePolicy::AUTO) {
                std::cout << " (selected: " << to_string(resolved_store_policy) << ")";
            }
            std::cout << std::endl;
            std::cout << "│ Store type: " << (config.use_nontemporal_stores ? "non-temporal" : "temporal") << std::endl;
        } else {
            std::cout << "│ ERROR: Output file not found at " << output_path << std::endl;
        }
    } catch (const std::exception& e) {
        std::cout << "│ ERROR: Exception caught: " << e.what() << std::endl;
    }
    std::cout << "└─" << std::endl;


    std::cout << "\n┌─ TrafficGen Compilation" << std::endl;

    std::vector<ExecutionMode> available_modes;
    if (arch_config.has_multisequential && build_multisequential) {
        available_modes.push_back(ExecutionMode::MULTISEQUENTIAL);
    }

    auto describe_mode = [](bool has_mode, bool build_mode) {
        if (!has_mode) return "Not available";
        return build_mode ? "Available" : "Skipped";
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

    auto write_fallback_trafficgen_binary = [&](const std::string& target_name, const std::string& reason) {
        const std::filesystem::path fallback_path = root / "build/bin" / target_name;
        std::string sanitized_reason = reason;
        std::replace(sanitized_reason.begin(), sanitized_reason.end(), '\n', ' ');
        std::replace(sanitized_reason.begin(), sanitized_reason.end(), '"', '\'');
        std::ofstream fallback(fallback_path);
        fallback << "#!/bin/bash\n";
        fallback << "echo \"ERROR: " << target_name << " is unavailable.\" >&2\n";
        if (!sanitized_reason.empty()) {
            fallback << "echo \"Reason: " << sanitized_reason << "\" >&2\n";
        }
        fallback << "exit 1\n";
        fallback.close();
        system(("chmod +x " + fallback_path.string()).c_str());
    };

    if (available_modes.empty()) {
        installation_success = false;
        pending_issues.push_back("No utils files available for this architecture");
        std::cout << "│ Status: No utils files found" << std::endl;
        std::cout << "└─" << std::endl;
    } else {
        std::filesystem::create_directories(root / "src/traffic_gen/src");
        
        bool all_compiled = true;

        for (size_t mode_idx = 0; mode_idx < available_modes.size(); ++mode_idx) {

            std::string mode_name = "multiseq";
            std::string utils_file = arch_config.utils_multiseq_file;


            std::filesystem::path utils_path = root / utils_file;
            std::string target_name = "traffic_gen_" + mode_name + ".x";
            std::string utils_link = (root / "src/traffic_gen/src/utils.c").string();
                        
            if (!std::filesystem::exists(utils_path)) {
                all_compiled = false;
                installation_success = false;
                const std::string reason = !utils_generation_error.empty()
                    ? utils_generation_error
                    : ("Required utils file was not generated: " + utils_path.string());
                if (utils_generation_error.empty()) {
                    pending_issues.push_back("Utils file not found: " + utils_path.string());
                }
                std::cout << "│ Compiled: " << target_name << " ✗ (" << compactConsoleIssue(reason) << ")" << std::endl;
                write_fallback_trafficgen_binary(target_name, reason);
            }
            
            std::filesystem::remove(utils_link);
            std::filesystem::copy_file(utils_path, utils_link, std::filesystem::copy_options::overwrite_existing);
            
            if (!std::filesystem::exists(utils_link)) {
                std::cout << "│ ERROR: Failed to copy utils file to " << utils_link << std::endl;
                all_compiled = false;
                installation_success = false;
            }
            uint64_t min_traffic_gen_size = calculate_traffic_gen_array_size(capabilities.l3_size);
            if (using_avx512) {
                min_traffic_gen_size *= 2;
            }

            std::cout << "│ Compiling " << target_name << " (array: " << (min_traffic_gen_size * sizeof(double) / (1000*1000)) << " MB)" << std::endl;

            std::filesystem::create_directories(root / "src/traffic_gen/build");

            // Determine architecture-specific compiler flags
            std::string arch_flags = "-march=native";
            if (capabilities.arch == CPUArchitecture::ARM64) {
                ISAMode compile_mode = resolve_effective_isa_mode(config, capabilities, *architecture);
                arch_flags = arm_isa_utils::compileFlagsForMode(compile_mode);
            }else if (capabilities.arch == CPUArchitecture::RISCV64) {
                arch_flags = "-march=rv64g";
            }
            const int trafficgen_compile_jobs = std::max(1, std::min(compile_jobs, num_cores));
            std::string build_cmd = "cd " + (root / "src/traffic_gen").string() + " && mkdir -p build && make -j" + std::to_string(trafficgen_compile_jobs) + " " + target_name +
                                   " CFLAGS=\"" + arch_flags + " -Wall -fopenmp -DTrafficGen_ARRAY_SIZE=" + std::to_string(min_traffic_gen_size) + "\"" +
                                   " LDFLAGS=\"-pthread -lrt -fopenmp\"" +
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
                write_fallback_trafficgen_binary(target_name, "Compilation failed");
            }
        }
        
        system(("rm -rf " + (root / "src/traffic_gen/build").string()).c_str());
        std::filesystem::remove(root / "src/traffic_gen/src/utils.c");
        
        std::cout << "│ Status: " << (all_compiled ? "All variants compiled successfully" : "Some variants failed") << std::endl;
        std::cout << "└─" << std::endl;
    }
    

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
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "ERROR: Unknown failure during code generation." << std::endl;
        return 1;
    }
}
