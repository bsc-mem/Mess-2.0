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

#include "codegen.h"
#include "architecture/Architecture.h"
#include "architecture/ISA.h"
#include "architecture/KernelAssembler.h"
#include "architecture/ArchitectureRegistry.h"
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
#include "utils.h"

namespace {

ISAExtension isaExtensionFromMode(ISAMode mode) {
    switch (mode) {
        // x86_64
        case ISAMode::AVX512: return ISAExtension::AVX512;
        case ISAMode::AVX2:   return ISAExtension::AVX2;
        case ISAMode::AVX:    return ISAExtension::AVX;
        // ARM64
        case ISAMode::SVE:    return ISAExtension::SVE;
        case ISAMode::NEON:   return ISAExtension::NEON;
        // RISC-V
        case ISAMode::RVV1_0: return ISAExtension::RVV1_0;
        case ISAMode::RVV0_7: return ISAExtension::RVV0_7;
        // POWER
        case ISAMode::VSX:    return ISAExtension::VSX;
        case ISAMode::VMX:    return ISAExtension::VMX;
        default:
            return ISAExtension::UNKNOWN;
    }
}

bool isISAAvailable(ISAMode mode, const CPUCapabilities& caps) {
    if (mode == ISAMode::AUTO || mode == ISAMode::SCALAR) {
        return true;
    }

    ISAExtension required = isaExtensionFromMode(mode);
    if (required == ISAExtension::UNKNOWN) {
        return false;
    }
    return std::find(caps.extensions.begin(), caps.extensions.end(), required) != caps.extensions.end();
}

}

KernelGenerator::KernelGenerator(const KernelConfig& config, const CPUCapabilities& capabilities)
    : kernel_config_(config), capabilities_(capabilities) {

    architecture_ = ArchitectureRegistry::instance().getArchitecture(capabilities_);
    if (!architecture_) {
        throw std::runtime_error("Unsupported architecture");
    }

    if (kernel_config_.isa_mode == ISAMode::AUTO) {
        auto best_isa = architecture_->selectBestISA(capabilities_);
        kernel_config_.isa_mode = best_isa->getMode();
    } else {
        if (!isISAAvailable(kernel_config_.isa_mode, capabilities_)) {
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



ISAMode KernelGenerator::select_best_isa_for_bandwidth() const {
    return architecture_->selectBestISA(capabilities_)->getMode();
}

std::string KernelGenerator::generate_multiseq_function(int ratio) {
    std::ostringstream oss;
    auto asm_gen = get_assembler();
    
    std::string function_name = "TrafficGen_copy_" + std::to_string(ratio);
    oss << "void " << function_name << "(double *a_array, double *b_array, ssize_t *array_size, int *pause) {\n";
    oss << asm_gen->generateRegisterSetup();
    
    oss << asm_gen->generateAsmStart();
    oss << asm_gen->generateVectorRegisterInit();
    oss << "      \"..L_" << ratio << ":\\n\"\n";
    
    int ops_per_block = kernel_config_.ops_per_pause_block;
    int num_regs = kernel_config_.num_simd_registers;
    int total_ops = kernel_config_.total_ops;
    
    std::shared_ptr<ISA> isa;
    if (kernel_config_.isa_mode == ISAMode::AUTO) {
        isa = architecture_->selectBestISA(capabilities_);
    } else {
        auto supported_isas = architecture_->getSupportedISAs();
        for (const auto& candidate : supported_isas) {
            if (candidate->getMode() == kernel_config_.isa_mode) {
                isa = candidate;
                break;
            }
        }
        if (!isa) {
            isa = architecture_->selectBestISA(capabilities_);
        }
    }
    
    int bytes_per_op = isa->getVectorWidthBits() / 8;

    int offset = 0;
    int current_reg = 0;
    int ops_in_block = 0;
    
    
    int num_reads = (total_ops * ratio + 50) / 100;
    int num_writes = total_ops - num_reads;
    
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
                ops_in_block++;
            }
            
            if (do_write) {
                oss << asm_gen->generateStore(write_offset, reg);
                write_offset += bytes_per_op;
                ops_in_block++;
            }
            
            if (ops_per_block > 0 && ops_in_block >= ops_per_block) {
                oss << asm_gen->generatePause();
                ops_in_block = 0;
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
            
            oss << asm_gen->generateStore(write_offset, reg);
            write_offset += bytes_per_op;
            
            ops_in_block += 2;
            
            if (ops_per_block > 0 && ops_in_block >= ops_per_block) {
                oss << asm_gen->generatePause();
                ops_in_block = 0;
            }
        }
        
        for (int i = 0; i < num_only_read; ++i) {
            int reg = 0;
            if (!kernel_config_.single_registers && num_regs > 1) {
                reg = current_reg;
                current_reg = (current_reg + 1) % num_regs;
            }
            
            oss << asm_gen->generateLoad(read_offset, reg);
            read_offset += bytes_per_op;
            
            ops_in_block++;
            
            if (ops_per_block > 0 && ops_in_block >= ops_per_block) {
                oss << asm_gen->generatePause();
                ops_in_block = 0;
            }
        }
        
        for (int i = 0; i < num_only_write; ++i) {
            int reg = 0;
            if (!kernel_config_.single_registers && num_regs > 1) {
                reg = current_reg;
                current_reg = (current_reg + 1) % num_regs;
            }
            
            oss << asm_gen->generateStore(write_offset, reg);
            write_offset += bytes_per_op;
            
            ops_in_block++;
            
            if (ops_per_block > 0 && ops_in_block >= ops_per_block) {
                oss << asm_gen->generatePause();
                ops_in_block = 0;
            }
        }
        offset = std::max(read_offset, write_offset);
    }
    
    if (ops_in_block > 0) {
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
    oss << "    switch (ratio) {\n";
    
    int total_ops = kernel_config_.total_ops;
    
    std::shared_ptr<ISA> isa;
    if (kernel_config_.isa_mode == ISAMode::AUTO) {
        isa = architecture_->selectBestISA(capabilities_);
    } else {
        auto supported_isas = architecture_->getSupportedISAs();
        for (const auto& candidate : supported_isas) {
            if (candidate->getMode() == kernel_config_.isa_mode) {
                isa = candidate;
                break;
            }
        }
        if (!isa) {
            isa = architecture_->selectBestISA(capabilities_);
        }
    }
    int bytes_per_op = isa->getVectorWidthBits() / 8;

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
                 if (do_write) write_offset += bytes_per_op;
             }
             offset = std::max(read_offset, write_offset);
             increment_elements = offset / 8;
        } else {
            int num_common = std::min(num_reads, num_writes);
            int num_only_read = num_reads - num_common;
            int num_only_write = num_writes - num_common;
            int read_offset = num_common * bytes_per_op + num_only_read * bytes_per_op;
            int write_offset = num_common * bytes_per_op + num_only_write * bytes_per_op;
            int offset = std::max(read_offset, write_offset);
            increment_elements = offset / 8;
        }
        oss << "        case " << ratio << ": return " << increment_elements << ";\n";
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



std::string KernelGenerator::generate_nop_file() {
    return assembler_->generateNopFile();
}

