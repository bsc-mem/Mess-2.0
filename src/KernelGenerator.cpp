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

#include "Codegen.h"
#include "architecture/Architecture.h"
#include "architecture/ISA.h"
#include "architecture/KernelAssembler.h"
#include "architecture/ArchitectureRegistry.h"
#include "arch/arm/ArmISAUtils.h"
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sys/stat.h>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <cerrno>
#include <algorithm>
#include <stdexcept>
#include "Utils.h"

namespace {

ISAExtension isaExtensionFromMode(ISAMode mode) {
    switch (mode) {
        // x86_64
        case ISAMode::AVX512: return ISAExtension::AVX512;
        case ISAMode::AVX2:   return ISAExtension::AVX2;
        case ISAMode::AVX:    return ISAExtension::AVX;
        case ISAMode::SSE2:   return ISAExtension::SSE2;
        // ARM64
        case ISAMode::SVE:    return ISAExtension::SVE;
        case ISAMode::SVE_MAX:return ISAExtension::SVE;
        case ISAMode::SVE128: return ISAExtension::SVE;
        case ISAMode::SVE256: return ISAExtension::SVE;
        case ISAMode::SVE512: return ISAExtension::SVE;
        case ISAMode::NEON_NATIVE:
        case ISAMode::NEON_PAIR:
            return ISAExtension::NEON;
        // RISC-V
        case ISAMode::RVV1_0: return ISAExtension::RVV1_0;
        case ISAMode::RVV0_7: return ISAExtension::RVV0_7;
        // POWERPC
        case ISAMode::VSX:    return ISAExtension::VSX;
        case ISAMode::VMX:    return ISAExtension::VMX;
        default:
            return ISAExtension::UNKNOWN;
    }
}

bool isISAAvailable(ISAMode mode, const CPUCapabilities& caps) {
    mode = arm_isa_utils::canonicalizePublicMode(mode);
    if (mode == ISAMode::AUTO || mode == ISAMode::SCALAR) {
        return true;
    }

    if (arm_isa_utils::requestedSVEBits(mode) > 0) {
        return std::find(caps.extensions.begin(), caps.extensions.end(), ISAExtension::SVE) != caps.extensions.end();
    }

    ISAExtension required = isaExtensionFromMode(mode);
    if (required == ISAExtension::UNKNOWN) {
        return false;
    }
    return std::find(caps.extensions.begin(), caps.extensions.end(), required) != caps.extensions.end();
}

/**
 * @brief Resolves the appropriate ISA implementation for a given requested mode.
 * 
 * This function determines the best matching ISA backend. If `AUTO` is specified,
 * it delegates to the architecture's native `selectBestISA` mechanism which
 * evaluates CPU capabilities to pick the best available ISA.
 * Otherwise, it searches the supported ISAs for an exact match, falling back
 * to the best available ISA if the specific mode is not found.
 * 
 * @param arch Pointer to the target architecture.
 * @param mode The requested ISA mode (e.g., AVX512, SVE, AUTO).
 * @param caps The hardware capabilities detected at runtime.
 * @return std::shared_ptr<ISA> A pointer to the resolved ISA backend.
 */
std::shared_ptr<ISA> resolveISAForMode(const Architecture* arch, ISAMode mode, const CPUCapabilities& caps) {
    mode = arm_isa_utils::canonicalizePublicMode(mode);
    if (mode == ISAMode::AUTO) {
        return arch->selectBestISA(caps);
    }
    for (const auto& candidate : arch->getSupportedISAs()) {
        if (candidate->getMode() == mode) {
            return candidate;
        }
    }
    return arch->selectBestISA(caps);
}

int resolveVectorWidthBits(ISAMode mode, const CPUCapabilities& caps, int fallback_bits) {
    mode = arm_isa_utils::canonicalizePublicMode(mode);
    const int requested_sve_bits = arm_isa_utils::requestedSVEBits(mode);
    if (requested_sve_bits > 0) {
        return requested_sve_bits;
    }
    if (mode == ISAMode::SVE) {
        if (caps.sve_vector_bits > 0) {
            return caps.sve_vector_bits;
        }
        if (caps.sve_max_vector_bits > 0) {
            return caps.sve_max_vector_bits;
        }
    }
    if (arm_isa_utils::isNeonPairMode(mode)) {
        return 256;
    }
    if ((mode == ISAMode::RVV1_0 || mode == ISAMode::RVV0_7) && caps.rvv_vector_bits > 0) {
        return caps.rvv_vector_bits;
    }
    return fallback_bits;
}

StoreLinePolicy resolveStoreLinePolicy(StoreLinePolicy policy) {
    if (policy == StoreLinePolicy::AUTO) {
        return StoreLinePolicy::ISA_NATIVE;
    }
    return policy;
}

int resolvedStoreLineBytes(StoreLinePolicy policy, const CPUCapabilities& caps) {
    switch (policy) {
        case StoreLinePolicy::CACHELINE_64B:
            return 64;
        case StoreLinePolicy::CACHELINE_NATIVE:
            return (caps.cache_line_size_bytes > 0) ? caps.cache_line_size_bytes : 64;
        default:
            return 0;
    }
}

int storeBurstCount(StoreLinePolicy policy, int bytes_per_store_op, const CPUCapabilities& caps) {
    const int target_line_bytes = resolvedStoreLineBytes(policy, caps);
    if (target_line_bytes <= 0 || bytes_per_store_op <= 0 || bytes_per_store_op >= target_line_bytes) {
        return 1;
    }
    return std::max(1, target_line_bytes / bytes_per_store_op);
}

}

/**
 * @brief Constructs a KernelGenerator and initializes the architecture and ISA backends.
 * 
 * This constructor handles the benchmark setup phase. It:
 * 1. Resolves the `Architecture` instance based on runtime `CPUCapabilities`.
 * 2. Determines the best `ISA` mode. If a specific ISA is requested but unavailable,
 *    it either falls back to the next best available ISA or, for explicit true-VL
 *    SVE width requests, fails with a diagnostic.
 * 3. Calculates the `resolved_bytes_per_mem_op` dynamically based on the target
 *    vector register width (e.g., 64 bytes for AVX-512, variable for SVE).
 * 4. Instantiates the appropriate `KernelAssembler` which will generate the 
 *    raw inline assembly loops.
 * 
 * @param config Configuration specifying requested ISA, ratios, unroll factors, etc.
 * @param capabilities The hardware capabilities detected at runtime.
 */
KernelGenerator::KernelGenerator(const KernelConfig& config, const CPUCapabilities& capabilities)
    : kernel_config_(config), capabilities_(capabilities) {

    kernel_config_.isa_mode = arm_isa_utils::canonicalizePublicMode(kernel_config_.isa_mode);

    architecture_ = ArchitectureRegistry::instance().getArchitecture(capabilities_);
    if (!architecture_) {
        throw std::runtime_error("Unsupported architecture");
    }

    if (kernel_config_.isa_mode == ISAMode::AUTO) {
        auto best_isa = architecture_->selectBestISA(capabilities_);
        kernel_config_.isa_mode = best_isa->getMode();
    } else {
        const int requested_sve_bits = arm_isa_utils::requestedSVEBits(kernel_config_.isa_mode);
        if (requested_sve_bits > 0) {
            const std::string validation_error =
                arm_isa_utils::validateTrueSVEMode(kernel_config_.isa_mode, capabilities_);
            if (!validation_error.empty()) {
                throw std::runtime_error(validation_error);
            }
        }
        if (!isISAAvailable(kernel_config_.isa_mode, capabilities_)) {
            if (requested_sve_bits > 0) {
                throw std::runtime_error(arm_isa_utils::formatUnsupportedTrueSVEMessage(kernel_config_.isa_mode, capabilities_, ""));
            } else {
            std::string requested_name = "Unknown";
            for (const auto& isa : architecture_->getSupportedISAs()) {
                if (isa->getMode() == kernel_config_.isa_mode) {
                    requested_name = isa->getName();
                    break;
                }
            }

            auto fallback_isa = architecture_->selectBestISA(capabilities_);
            std::cerr << "│ \033[33mWarning: Requested ISA '" << requested_name
                      << "' is not available on this system.\033[0m" << std::endl;
            std::cerr << "│ \033[33mFalling back to '" << fallback_isa->getName() << "'.\033[0m" << std::endl;

            kernel_config_.isa_mode = fallback_isa->getMode();
            }
        }
    }

    {
        auto active_isa = resolveISAForMode(architecture_.get(), kernel_config_.isa_mode, capabilities_);
        int vector_bits = resolveVectorWidthBits(kernel_config_.isa_mode, capabilities_, active_isa->getVectorWidthBits());
        kernel_config_.resolved_bytes_per_mem_op = std::max(1, vector_bits / 8);
        kernel_config_.resolved_cache_line_bytes = std::max(0, resolvedStoreLineBytes(resolveStoreLinePolicy(kernel_config_.store_line_policy), capabilities_));
    }

    assembler_ = architecture_->createAssembler(kernel_config_);
}

KernelGenerator::~KernelGenerator() = default;

KernelAssembler* KernelGenerator::get_assembler() {
    return assembler_.get();
}

ArchitectureConfig KernelGenerator::setup_architecture_config() {
    ArchitectureConfig arch_config;
    arch_config.arch_name = architecture_->getName();
    

    arch_config.utils_multiseq_file = "src/traffic_gen/src/generated_utils_multiseq_" + arch_config.arch_name + ".c";
    arch_config.has_multisequential = true;
    
    arch_config.pointer_chase_template = "templates/ptr_chase_template.c";
    
    return arch_config;
}
/**
 * @brief Generates a C function containing inline assembly for the multi-sequential memory access pattern.
 * 
 * This is the code generation loop for linear memory traffic. Based on the requested `ratio` 
 * (read/write mix percentage), `bytes_per_op`, and unrolling factors (`total_ops`), it
 * builds an assembly loop tailored to the chosen ISA.
 * 
 * If `enable_interleaving` is set, reads and writes are interleaved based on error-accumulation 
 * (Bresenham-like) logic to ensure an even mix. Otherwise, reads and writes are grouped. 
 * `ops_per_pause_block` logic is injected to insert hardware pauses if configured.
 * 
 * @param ratio The read percentage (0 to 100). E.g., 70 means 70% reads, 30% writes.
 * @return std::string The generated C function source code as a string.
 */
std::string KernelGenerator::generate_multiseq_function(int ratio) {
    std::ostringstream oss;
    auto asm_gen = get_assembler();
    
    std::string function_name = "TrafficGen_copy_" + std::to_string(ratio);
    oss << "void " << function_name << "(double *a_array, double *b_array, ssize_t *array_size, int *pause) {\n";
    const int ops_per_block = kernel_config_.ops_per_pause_block;
    const int total_ops = kernel_config_.total_ops;
    int num_reads = (total_ops * ratio + 50) / 100;
    int num_writes = total_ops - num_reads;

    oss << asm_gen->generateRegisterSetup();

    if (arm_isa_utils::isSVEFamilyMode(kernel_config_.isa_mode)) {
        const int target_line_bytes = resolvedStoreLineBytes(resolveStoreLinePolicy(kernel_config_.store_line_policy), capabilities_);
        oss << "    mess_sve_store_burst_count = mess_arm_store_burst_count(mess_sve_bytes_per_op, "
            << target_line_bytes << ");\n";
        oss << "    mess_sve_loop_increment_elements = (ssize_t)((" << num_reads << "));\n";
        oss << "    if ((ssize_t)(" << num_writes << " * mess_sve_store_burst_count) > mess_sve_loop_increment_elements) "
            << "mess_sve_loop_increment_elements = (ssize_t)(" << num_writes << " * mess_sve_store_burst_count);\n";
        oss << "    mess_sve_loop_increment_elements *= mess_sve_elements_per_op;\n";
    }

    oss << asm_gen->generateAsmStart();
    oss << asm_gen->generateVectorRegisterInit();
    oss << "      \"..L_" << ratio << ":\\n\"\n";

    std::shared_ptr<ISA> isa = resolveISAForMode(architecture_.get(), kernel_config_.isa_mode, capabilities_);
    int bytes_per_op = kernel_config_.resolved_bytes_per_mem_op;
    if (bytes_per_op <= 0) {
        int vector_bits = resolveVectorWidthBits(kernel_config_.isa_mode, capabilities_, isa->getVectorWidthBits());
        bytes_per_op = std::max(1, vector_bits / 8);
    }

    int num_regs = std::max(1, std::min(kernel_config_.num_simd_registers, isa->getMaxSimdRegisters()));
    const StoreLinePolicy store_policy = resolveStoreLinePolicy(kernel_config_.store_line_policy);
    int stores_per_write = storeBurstCount(store_policy, bytes_per_op, capabilities_);
    if (arm_isa_utils::isSVEFamilyMode(kernel_config_.isa_mode)) {
        stores_per_write = 1;
    }

    int offset = 0;
    int current_reg = 0;
    int ops_in_block = 0;
    
    auto emit_pause_if_needed = [&]() {
        if (ops_per_block > 0 && ops_in_block >= ops_per_block) {
            oss << asm_gen->generatePause();
            ops_in_block = 0;
        }
    };

    auto emit_store_burst = [&](int& write_offset, int reg) {
        for (int burst = 0; burst < stores_per_write; ++burst) {
            oss << asm_gen->generateStore(write_offset, reg);
            write_offset += bytes_per_op;
            ++ops_in_block;
            emit_pause_if_needed();
        }
    };

    if (kernel_config_.enable_interleaving) {
        int read_error = 0;
        int write_error = 0;
        int read_offset = 0;
        int write_offset = 0;
        
        for (int op_idx = 0; op_idx < total_ops; ++op_idx) {
            bool do_read = false;
            bool do_write = false;
            
            read_error += num_reads;
            if (read_error >= total_ops) {
                do_read = true;
                read_error -= total_ops;
            }
            
            write_error += num_writes;
            if (write_error >= total_ops) {
                do_write = true;
                write_error -= total_ops;
            }
            
            int reg = 0;
            if (!kernel_config_.single_registers && num_regs > 1) {
                reg = current_reg;
                current_reg = (current_reg + 1) % num_regs;
            }
            
            if (do_read) {
                oss << asm_gen->generateLoad(read_offset, reg);
                read_offset += bytes_per_op;
                ++ops_in_block;
                emit_pause_if_needed();
            }
            
            if (do_write) {
                emit_store_burst(write_offset, reg);
            }
        }
        offset = std::max(read_offset, write_offset);
    } else {
        int num_common = std::min(num_reads, num_writes);
        int num_only_read = num_reads - num_common;
        int num_only_write = num_writes - num_common;
        
        int read_offset = 0;
        int write_offset = 0;
        
        for (int i = 0; i < num_common; ++i) {
            int reg = 0;
            if (!kernel_config_.single_registers && num_regs > 1) {
                reg = current_reg;
                current_reg = (current_reg + 1) % num_regs;
            }
            
            oss << asm_gen->generateLoad(read_offset, reg);
            read_offset += bytes_per_op;
            ++ops_in_block;
            emit_pause_if_needed();

            emit_store_burst(write_offset, reg);
        }
        
        for (int i = 0; i < num_only_read; ++i) {
            int reg = 0;
            if (!kernel_config_.single_registers && num_regs > 1) {
                reg = current_reg;
                current_reg = (current_reg + 1) % num_regs;
            }
            
            oss << asm_gen->generateLoad(read_offset, reg);
            read_offset += bytes_per_op;
            ++ops_in_block;
            emit_pause_if_needed();
        }
        
        for (int i = 0; i < num_only_write; ++i) {
            int reg = 0;
            if (!kernel_config_.single_registers && num_regs > 1) {
                reg = current_reg;
                current_reg = (current_reg + 1) % num_regs;
            }
            
            emit_store_burst(write_offset, reg);
        }
        offset = std::max(read_offset, write_offset);
    }
    
    if (ops_per_block > 0 && ops_in_block > 0) {
        oss << asm_gen->generatePause();
    }
    
    oss << asm_gen->generateLoopControl(offset, ratio);
    
    oss << asm_gen->generateAsmEnd();
    oss << "}\n\n";
    
    return oss.str();
}

std::string KernelGenerator::generate_all_multiseq_functions() {
    std::ostringstream oss;
    
    oss << assembler_->generateHeader();
    
    for (int ratio = 0; ratio <= 100; ratio += kernel_config_.ratio_granularity) {
        oss << generate_multiseq_function(ratio);
    }

    oss << "int TrafficGen_get_ratio_granularity(void)\n";
    oss << "{\n";
    oss << "    return " << kernel_config_.ratio_granularity << ";\n";
    oss << "}\n\n";

    oss << "int TrafficGen_get_loop_increment(int ratio)\n";
    oss << "{\n";
    if (arm_isa_utils::isSVEFamilyMode(kernel_config_.isa_mode)) {
        const int target_line_bytes = resolvedStoreLineBytes(resolveStoreLinePolicy(kernel_config_.store_line_policy), capabilities_);
        if (kernel_config_.isa_mode == ISAMode::SVE) {
            oss << "    int mess_sve_bytes_per_op = mess_arm_current_sve_vl_bytes();\n";
        } else {
            oss << "    int mess_sve_bytes_per_op = mess_arm_configure_sve_vl_bytes(" << arm_isa_utils::requestedSVEBits(kernel_config_.isa_mode) << ", " << (arm_isa_utils::requestedSVEBits(kernel_config_.isa_mode) > 0 ? 1 : 0) << ");\n";
        }
        oss << "    ssize_t mess_sve_elements_per_op = (ssize_t)(mess_sve_bytes_per_op / 8);\n";
        oss << "    int mess_sve_store_burst_count = mess_arm_store_burst_count(mess_sve_bytes_per_op, "
            << target_line_bytes << ");\n";
    }
    oss << "    switch (ratio) {\n";
    
    int total_ops = kernel_config_.total_ops;
    
    std::shared_ptr<ISA> isa = resolveISAForMode(architecture_.get(), kernel_config_.isa_mode, capabilities_);
    int bytes_per_op = kernel_config_.resolved_bytes_per_mem_op;
    if (bytes_per_op <= 0) {
        int vector_bits = resolveVectorWidthBits(kernel_config_.isa_mode, capabilities_, isa->getVectorWidthBits());
        bytes_per_op = std::max(1, vector_bits / 8);
    }
    const StoreLinePolicy store_policy = resolveStoreLinePolicy(kernel_config_.store_line_policy);
    const int stores_per_write = storeBurstCount(store_policy, bytes_per_op, capabilities_);

    for (int ratio = 0; ratio <= 100; ratio += kernel_config_.ratio_granularity) {
        int increment_elements = 0;
        int num_reads = (total_ops * ratio + 50) / 100;
        int num_writes = total_ops - num_reads;

        if (kernel_config_.enable_interleaving) {
             int offset = 0;
             int read_offset = 0;
             int write_offset = 0;
             int read_error = 0;
             int write_error = 0;
             for (int op_idx = 0; op_idx < total_ops; ++op_idx) {
                 bool do_read = false;
                 bool do_write = false;
                 read_error += num_reads;
                 if (read_error >= total_ops) { do_read = true; read_error -= total_ops; }
                 write_error += num_writes;
                 if (write_error >= total_ops) { do_write = true; write_error -= total_ops; }
                 if (do_read) read_offset += bytes_per_op;
                 if (do_write) write_offset += bytes_per_op * stores_per_write;
             }
             offset = std::max(read_offset, write_offset);
             increment_elements = offset / 8;
        } else {
            int num_common = std::min(num_reads, num_writes);
            int num_only_read = num_reads - num_common;
            int num_only_write = num_writes - num_common;
            int read_offset = num_common * bytes_per_op + num_only_read * bytes_per_op;
            int write_offset = (num_common + num_only_write) * bytes_per_op * stores_per_write;
            int offset = std::max(read_offset, write_offset);
            increment_elements = offset / 8;
        }
        if (arm_isa_utils::isSVEFamilyMode(kernel_config_.isa_mode)) {
            oss << "        case " << ratio << ": {\n";
            oss << "            ssize_t mess_sve_loop_increment_elements = " << num_reads << ";\n";
            oss << "            if ((ssize_t)(" << num_writes << " * mess_sve_store_burst_count) > mess_sve_loop_increment_elements) "
                << "mess_sve_loop_increment_elements = (ssize_t)(" << num_writes << " * mess_sve_store_burst_count);\n";
            oss << "            mess_sve_loop_increment_elements *= mess_sve_elements_per_op;\n";
            oss << "            return (int)mess_sve_loop_increment_elements;\n";
            oss << "        }\n";
        } else {
            oss << "        case " << ratio << ": return " << increment_elements << ";\n";
        }
    }
    oss << "        default: return 0;\n";
    oss << "    }\n";
    oss << "}\n\n";

    oss << "void TrafficGen_copy_rw(double *a_array, double *b_array, ssize_t *array_size, int *pause, int rd_percentage)\n";
    oss << "{\n";
    oss << "    switch (rd_percentage) {\n";
    for (int ratio = 0; ratio <= 100; ratio += kernel_config_.ratio_granularity) {
        oss << "        case " << ratio << ": TrafficGen_copy_" << ratio << "(a_array, b_array, array_size, pause); break;\n";
        
    }
    oss << "        default: break;\n";
    oss << "    }\n";
    oss << "}\n";
    
    oss << assembler_->generateFooter();

    return oss.str();
}

void KernelGenerator::generate_kernel(const std::string& output_dir_str) { 
    std::filesystem::path output_dir(output_dir_str);
    if (output_dir.is_relative()) {
        output_dir = get_project_root() / output_dir;
    }
    
    std::filesystem::create_directories(output_dir);
    
    std::string output_dir_string = output_dir.string();

    ArchitectureConfig arch_config = setup_architecture_config();
    
    if (arch_config.has_multisequential) {
        std::string filename = output_dir_string + "/generated_utils_multiseq_" + arch_config.arch_name + ".c";
        std::ofstream outfile(filename);
        if (outfile.is_open()) {
            outfile << generate_all_multiseq_functions();
            outfile.close();
        } else {
            std::cerr << "Unable to open file " << filename << std::endl;
        }
    }
    
    
    std::string nop_filename = output_dir_string + "/nop.c";
    
    std::ofstream nop_file(nop_filename);
    if (nop_file.is_open()) {
        nop_file << assembler_->generateNopFile();
        nop_file.close();
    } else {
        std::cerr << "│ ERROR: Failed to create " << nop_filename << ": " << strerror(errno) << std::endl;
    }
}
