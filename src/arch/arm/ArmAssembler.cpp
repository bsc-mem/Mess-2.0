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

#include "arch/arm/ArmAssembler.h"
#include <algorithm>
#include <iomanip>
#include <set>

namespace {

bool isSVEFamily(ISAMode mode) {
    return mode == ISAMode::SVE ||
           mode == ISAMode::SVE_MAX ||
           mode == ISAMode::SVE128 ||
           mode == ISAMode::SVE256 ||
           mode == ISAMode::SVE512;
}

bool isNeonPairMode(ISAMode mode) {
    return mode == ISAMode::NEON_PAIR;
}

int requestedSVEBits(ISAMode mode) {
    switch (mode) {
        case ISAMode::SVE128: return 128;
        case ISAMode::SVE256: return 256;
        case ISAMode::SVE512: return 512;
        default: return 0;
    }
}

AddressingPolicy resolveAddressingPolicy(const KernelConfig& config) {
    if (config.addressing_policy == AddressingPolicy::AUTO) {
        return AddressingPolicy::POST_INCREMENT;
    }
    return config.addressing_policy;
}

std::string emitAddressAdd(const char* base, int offset) {
    std::ostringstream oss;
    if (offset == 0) {
        oss << "        \"mov x8, " << base << ";\\n\"\n";
    } else if (offset > 0 && offset < 4096) {
        oss << "        \"add x8, " << base << ", #" << offset << ";\\n\"\n";
    } else {
        oss << "        \"movz x9, #0x" << std::hex << (offset & 0xFFFF) << ";\\n\"\n";
        if (offset > 0xFFFF) {
            oss << "        \"movk x9, #0x" << std::hex << ((offset >> 16) & 0xFFFF) << ", LSL #16;\\n\"\n";
        }
        oss << "        \"add x8, " << base << ", x9;\\n\"\n";
    }
    return oss.str();
}

int resolvedArmBytesPerMemOp(const KernelConfig& config) {
    if (config.resolved_bytes_per_mem_op > 0) {
        return config.resolved_bytes_per_mem_op;
    }
    if (isSVEFamily(config.isa_mode)) {
        return 64;
    }
    return isNeonPairMode(config.isa_mode) ? 32 : 16;
}


int nextArmLabelId() {
    static int next_id = 0;
    return ++next_id;
}

std::string emitSveSequentialDynamicStoreBurst(int reg, const char* op) {
    std::ostringstream oss;
    const int label_id = nextArmLabelId();
    const std::string done = ".L_mess_sve_seq_store_done_" + std::to_string(label_id);
    oss << "        \"" << op << " {z" << reg << ".d}, p0, [x20];\\n\"\n"
        << "        \"addvl x20, x20, #1;\\n\"\n";
    for (int burst = 1; burst < 8; ++burst) {
        oss << "        \"cmp x26, #" << burst << ";\\n\"\n"
            << "        \"b.le " << done << ";\\n\"\n"
            << "        \"" << op << " {z" << reg << ".d}, p0, [x20];\\n\"\n"
            << "        \"addvl x20, x20, #1;\\n\"\n";
    }
    oss << "        \"" << done << ":\\n\"\n";
    return oss.str();
}


std::string emitArmVlConfigurationHelpers() {
    return R"ARM(#include <stdlib.h>
#include <stdio.h>
#if defined(__GNUC__)
#define MESS_MAYBE_UNUSED __attribute__((unused))
#else
#define MESS_MAYBE_UNUSED
#endif
#if defined(__linux__) && defined(__aarch64__)
#include <sys/prctl.h>
#include <linux/prctl.h>
#include <asm/sigcontext.h>
#if defined(__ARM_FEATURE_SVE)
#include <arm_sve.h>
#endif
static MESS_MAYBE_UNUSED int mess_arm_instruction_sve_vl_bytes(void)
{
#if defined(__ARM_FEATURE_SVE)
    return (int)svcntb();
#else
    return -1;
#endif
}
static MESS_MAYBE_UNUSED int mess_arm_current_sve_vl_bytes(void)
{
#if defined(PR_SVE_GET_VL) && defined(PR_SVE_VL_LEN_MASK)
    long current = prctl(PR_SVE_GET_VL);
    if (current < 0) {
        perror("prctl(PR_SVE_GET_VL)");
        abort();
    }
    int active_bytes = (int)(current & PR_SVE_VL_LEN_MASK);
    int instruction_bytes = mess_arm_instruction_sve_vl_bytes();
    if (instruction_bytes > 0 && instruction_bytes != active_bytes) {
        fprintf(stderr,
                "SVE VL validation mismatch: PR_SVE_GET_VL reports %d bits but instructions report %d bits\n",
                active_bytes * 8, instruction_bytes * 8);
        abort();
    }
    return active_bytes;
#else
    return 0;
#endif
}
static MESS_MAYBE_UNUSED int mess_arm_configure_sve_vl_bytes(int requested_bits, int require_exact)
{
#if defined(PR_SVE_SET_VL) && defined(PR_SVE_VL_LEN_MASK) && (defined(PR_SVE_VL_MAX) || defined(SVE_VL_MAX))
#if defined(PR_SVE_VL_MAX)
    const unsigned long mess_sve_max_vl = (unsigned long)PR_SVE_VL_MAX;
#else
    const unsigned long mess_sve_max_vl = (unsigned long)SVE_VL_MAX;
#endif
    unsigned long requested = (requested_bits > 0) ? (unsigned long)(requested_bits / 8) : mess_sve_max_vl;
    long configured = prctl(PR_SVE_SET_VL, requested);
    if (configured < 0) {
        perror("prctl(PR_SVE_SET_VL)");
        abort();
    }
    long current = prctl(PR_SVE_GET_VL);
    if (current < 0) {
        perror("prctl(PR_SVE_GET_VL)");
        abort();
    }
    int active_bytes = (int)(current & PR_SVE_VL_LEN_MASK);
    if (active_bytes <= 0) {
        active_bytes = (int)(configured & PR_SVE_VL_LEN_MASK);
    }
    if (active_bytes <= 0) {
        fprintf(stderr, "Failed to resolve active SVE VL after PR_SVE_SET_VL\n");
        abort();
    }
    int instruction_bytes = mess_arm_instruction_sve_vl_bytes();
    if (instruction_bytes > 0 && instruction_bytes != active_bytes) {
        fprintf(stderr,
                "SVE VL validation mismatch after PR_SVE_SET_VL: PR_SVE_GET_VL reports %d bits but instructions report %d bits\n",
                active_bytes * 8, instruction_bytes * 8);
        abort();
    }
    if (require_exact && requested_bits > 0 && active_bytes != (requested_bits / 8)) {
        fprintf(stderr, "Requested SVE VL %d bits but kernel configured %d bits\n",
                requested_bits, active_bytes * 8);
        abort();
    }
    return active_bytes;
#else
    (void)requested_bits;
    (void)require_exact;
    return 0;
#endif
}
#else
static MESS_MAYBE_UNUSED int mess_arm_instruction_sve_vl_bytes(void)
{
    return -1;
}
static MESS_MAYBE_UNUSED int mess_arm_current_sve_vl_bytes(void)
{
    return 0;
}
static MESS_MAYBE_UNUSED int mess_arm_configure_sve_vl_bytes(int requested_bits, int require_exact)
{
    (void)requested_bits;
    (void)require_exact;
    return 0;
}
#endif

static MESS_MAYBE_UNUSED int mess_arm_store_burst_count(int bytes_per_op, int target_line_bytes)
{
    if (target_line_bytes <= 0 || bytes_per_op <= 0 || bytes_per_op >= target_line_bytes) {
        return 1;
    }
    return target_line_bytes / bytes_per_op;
}

)ARM";
}

std::string emitMovImmediate(const char* reg, int value) {
    std::ostringstream oss;
    oss << "        \"movz " << reg << ", #0x" << std::hex << (value & 0xFFFF) << ";\\n\"\n";
    if (value > 0xFFFF) {
        oss << "        \"movk " << reg << ", #0x" << std::hex << ((value >> 16) & 0xFFFF) << ", LSL #16;\\n\"\n";
    }
    return oss.str();
}

std::string emitSveIndexedAddress(const char* base, int op_index) {
    std::ostringstream oss;
    if (op_index == 0) {
        oss << "        \"mov x15, x21;\\n\"\n";
    } else {
        oss << emitMovImmediate("x9", op_index);
        oss << "        \"madd x15, x9, x28, x21;\\n\"\n";
    }
    oss << "        \"lsl x15, x15, #3;\\n\"\n"
        << "        \"add x8, " << base << ", x15;\\n\"\n";
    return oss.str();
}

std::string emitSveSequentialDynamicStoreBurstIndexed(int reg, const char* op, int op_index) {
    std::ostringstream oss;
    const int label_id = nextArmLabelId();
    const std::string done = ".L_mess_sve_seq_store_idx_done_" + std::to_string(label_id);
    oss << emitSveIndexedAddress("x20", op_index);
    oss << "        \"" << op << " {z" << reg << ".d}, p0, [x8];\\n\"\n";
    for (int burst = 1; burst < 8; ++burst) {
        oss << "        \"cmp x26, #" << burst << ";\\n\"\n"
            << "        \"b.le " << done << ";\\n\"\n"
            << "        \"add x8, x8, x28, lsl #3;\\n\"\n"
            << "        \"" << op << " {z" << reg << ".d}, p0, [x8];\\n\"\n";
    }
    oss << "        \"" << done << ":\\n\"\n";
    return oss.str();
}

int maxArmVectorRegistersForMode(ISAMode mode) {
    return isNeonPairMode(mode) ? 16 : 32;
}

int usedArmVectorRegisters(const KernelConfig& config) {
    if (config.single_registers) {
        return 1;
    }
    return std::max(1, std::min(config.num_simd_registers, maxArmVectorRegistersForMode(config.isa_mode)));
}


std::string armSequentialVectorClobbers(const KernelConfig& config) {
    std::ostringstream oss;
    const int used_regs = usedArmVectorRegisters(config);
    if (isSVEFamily(config.isa_mode)) {
        for (int reg = 0; reg < used_regs; ++reg) {
            oss << ", \"z" << reg << "\"";
        }
        oss << ", \"p0\"";
        return oss.str();
    }

    for (int reg = 0; reg < used_regs; ++reg) {
        if (isNeonPairMode(config.isa_mode)) {
            const int qreg = (reg * 2) % 32;
            oss << ", \"q" << qreg << "\", \"q" << (qreg + 1) << "\"";
        } else {
            oss << ", \"q" << reg << "\"";
        }
    }
    return oss.str();
}

}

std::string ArmAssembler::generateLoad(int offset, int reg) const {
    std::ostringstream oss;
    AddressingPolicy policy = resolveAddressingPolicy(config_);

    if (isSVEFamily(config_.isa_mode)) {
        if (policy == AddressingPolicy::POST_INCREMENT) {
            oss << "        \"ld1d {z" << reg << ".d}, p0/z, [x19];\\n\"\n"
                << "        \"addvl x19, x19, #1;\\n\"\n";
        } else {
            const int op_index = offset / std::max(1, resolvedArmBytesPerMemOp(config_));
            oss << emitSveIndexedAddress("x19", op_index)
                << "        \"ld1d {z" << reg << ".d}, p0/z, [x8];\\n\"\n";
        }
    } else {
        const bool pair_mode = isNeonPairMode(config_.isa_mode);
        int qreg = pair_mode ? ((reg * 2) % 32) : (reg % 32);
        if (policy == AddressingPolicy::POST_INCREMENT) {
            int step = resolvedArmBytesPerMemOp(config_);
            if (pair_mode) {
                oss << "        \"ldp q" << qreg << ", q" << (qreg + 1) << ", [x19], #" << step << ";\\n\"\n";
            } else {
                oss << "        \"ldr q" << qreg << ", [x19], #" << step << ";\\n\"\n";
            }
        } else {
            oss << "        \"lsl x15, x21, #3;\\n\"\n";
            oss << emitAddressAdd("x19", offset);
            oss << "        \"add x8, x8, x15;\\n\"\n";
            if (pair_mode) {
                oss << "        \"ldp q" << qreg << ", q" << (qreg + 1) << ", [x8];\\n\"\n";
            } else {
                oss << "        \"ldr q" << qreg << ", [x8];\\n\"\n";
            }
        }
    }
    return oss.str();
}

std::string ArmAssembler::generateStore(int offset, int reg) const {
    std::ostringstream oss;
    AddressingPolicy policy = resolveAddressingPolicy(config_);

    if (isSVEFamily(config_.isa_mode)) {
        std::string op = config_.use_nontemporal_stores ? "stnt1d" : "st1d";
        if (policy == AddressingPolicy::POST_INCREMENT) {
            oss << emitSveSequentialDynamicStoreBurst(reg, op.c_str());
        } else {
            const int op_index = offset / std::max(1, resolvedArmBytesPerMemOp(config_));
            oss << emitSveSequentialDynamicStoreBurstIndexed(reg, op.c_str(), op_index);
        }
    } else {
        const bool pair_mode = isNeonPairMode(config_.isa_mode);
        int qreg = pair_mode ? ((reg * 2) % 32) : (reg % 32);
        if (policy == AddressingPolicy::POST_INCREMENT) {
            int step = resolvedArmBytesPerMemOp(config_);
            if (pair_mode) {
                oss << "        \"stp q" << qreg << ", q" << (qreg + 1) << ", [x20], #" << step << ";\\n\"\n";
            } else {
                oss << "        \"str q" << qreg << ", [x20], #" << step << ";\\n\"\n";
            }
        } else {
            oss << "        \"lsl x15, x21, #3;\\n\"\n";
            oss << emitAddressAdd("x20", offset);
            oss << "        \"add x8, x8, x15;\\n\"\n";
            if (pair_mode) {
                oss << "        \"stp q" << qreg << ", q" << (qreg + 1) << ", [x8];\\n\"\n";
            } else {
                oss << "        \"str q" << qreg << ", [x8];\\n\"\n";
            }
        }
    }
    return oss.str();
}

std::string ArmAssembler::generateLoopControl(int increment, int labelId) const {
    std::ostringstream oss;

    auto safe_add = [](const char* reg, int val) {
        std::ostringstream ss;
        if (val < 4096 && val > -4096) {
             ss << "        \"add " << reg << ", " << reg << ", #" << val << ";\\n\"\n";
        } else {
             ss << "        \"movz x9, #0x" << std::hex << (val & 0xFFFF) << ";\\n\"\n";
             if (val > 0xFFFF) ss << "        \"movk x9, #0x" << std::hex << ((val >> 16) & 0xFFFF) << ", LSL #16;\\n\"\n";
             ss << "        \"add " << reg << ", " << reg << ", x9;\\n\"\n";
        }
        return ss.str();
    };

    oss << "\n";
    if (isSVEFamily(config_.isa_mode)) {
        oss << "        \"add x21, x21, x27;\\n\"\n";
    } else {
        // Kernel generator passes increments in bytes. The outer loop counter x21 tracks elements.
        int loopIncrement = increment / 8;
        oss << safe_add("x21", loopIncrement);
    }

    oss << "        \"cmp x21, x22;\\n\"\n";
    oss << "        \"blt ..L_" << labelId << ";\\n\"\n";
    return oss.str();
}

std::string ArmAssembler::generatePause() const {
    // Avoid call overhead when pause is zero; execute the delay inline only when needed.
    return "        \"cbz x4, 2f;\\n\"\n"
           "        \"mov x10, x4;\\n\"\n"
           "        \"1:\\n\"\n"
           "        \"nop;\\n\"\n"
           "        \"subs x10, x10, #1;\\n\"\n"
           "        \"b.ne 1b;\\n\"\n"
           "        \"2:\\n\"\n";
}

std::string ArmAssembler::generateHeader() const {
    std::ostringstream oss;
    oss << "/*\n"
        << " * Generated by Mess Kernel Generator (ARM)\n"
        << " * Mode: MultiSequential access\n"
        << " */\n\n"
        << "#include <stdio.h>\n"
        << "#include <unistd.h>\n"
        << emitArmVlConfigurationHelpers()
        << "#include \"utils.h\"\n\n"
        << "void print_usage(char *argv[], char* usage)\n"
        << "{\n"
        << "    fprintf(stderr,\"Usage: %s %s\", argv[0], usage);\n"
        << "}\n\n";
    return oss.str();
}

std::string ArmAssembler::generateRegisterSetup() const {
    std::ostringstream oss;
    if (isSVEFamily(config_.isa_mode)) {
        oss << "    int mess_sve_bytes_per_op = 0;\n";
        oss << "    int mess_sve_store_burst_count = 1;\n";
        oss << "    ssize_t mess_sve_loop_increment_elements = 0;\n";
        oss << "    ssize_t mess_sve_elements_per_op = 0;\n";
        const int requested_bits = requestedSVEBits(config_.isa_mode);
        if (config_.isa_mode == ISAMode::SVE) {
            oss << "    mess_sve_bytes_per_op = mess_arm_current_sve_vl_bytes();\n";
        } else if (config_.isa_mode == ISAMode::SVE_MAX || requested_bits > 0) {
            const int require_exact = requested_bits > 0 ? 1 : 0;
            oss << "    mess_sve_bytes_per_op = mess_arm_configure_sve_vl_bytes("
                << requested_bits << ", " << require_exact << ");\n"
                << "    (void)mess_sve_bytes_per_op;\n";
        }
        oss << "    mess_sve_elements_per_op = (ssize_t)(mess_sve_bytes_per_op / 8);\n";
    }
    oss << "\n";
    return oss.str();
}

std::string ArmAssembler::generateVectorRegisterInit() const {
    if (isSVEFamily(config_.isa_mode)) {
        return "      \"ptrue p0.d;\\n\"\n";
    }
    return "";
}

std::string ArmAssembler::generateFooter() const {
    return "";
}

std::string ArmAssembler::generateAsmStart() const {
    if (isSVEFamily(config_.isa_mode)) {
        return "    asm __volatile__ (\n"
               "      \"mov x21, #0x0;\\n\"\n"
               "      \"mov x19, %0;\\n\"\n"
               "      \"mov x20, %3;\\n\"\n"
               "      \"mov x22, %1;\\n\"\n"
               "      \"mov x4, %2;\\n\"\n"
               "      \"mov x26, %4;\\n\"\n"
               "      \"mov x27, %5;\\n\"\n"
               "      \"mov x28, %6;\\n\"";
    }
    return "    asm __volatile__ (\n"
           "      \"mov x21, #0x0;\\n\"\n"
           "      \"mov x19, %0;\\n\"\n"
           "      \"mov x20, %3;\\n\"\n"
           "      \"mov x22, %1;\\n\"\n"
           "      \"mov x4, %2;\\n\"";
}

std::string ArmAssembler::generateAsmEnd() const {
    std::ostringstream oss;
    if (isSVEFamily(config_.isa_mode)) {
        oss << "      :\n"
            << "      : \"r\" (a_array), \"r\" (*array_size), \"r\" (*pause), \"r\" (b_array), \"r\" (mess_sve_store_burst_count), \"r\" (mess_sve_loop_increment_elements), \"r\" (mess_sve_elements_per_op)\n"
            << "      : \"x4\", \"x8\", \"x9\", \"x10\", \"x15\", \"x19\", \"x20\", \"x21\", \"x22\", \"x26\", \"x27\", \"x28\""
            << armSequentialVectorClobbers(config_)
            << ", \"memory\", \"cc\"\n"
            << "    );\n";
    } else {
        oss << "      :\n"
            << "      : \"r\" (a_array), \"r\" (*array_size), \"r\" (*pause), \"r\" (b_array)\n"
            << "      : \"x4\", \"x8\", \"x9\", \"x10\", \"x15\", \"x19\", \"x20\", \"x21\", \"x22\""
            << armSequentialVectorClobbers(config_)
            << ", \"memory\", \"cc\"\n"
            << "    );\n";
    }
    return oss.str();
}

std::string ArmAssembler::getPointerChaseLoopAsm() const {
    return "            \"subs %2, %2, #1;\"\n"
           "            \"b.ne start_loop_%%=;\"";
}

std::string ArmAssembler::getPointerChaseInstruction() const {
    return "        \"adds x3, %2, %1; ldr %1, [x3];\"";
}

std::string ArmAssembler::generatePointerChaseBurstLoop() const {
    return R"(
        register uint64_t i asm("x0") = BURST_ITERS;
        register struct line *start asm("x1") = array;
        register uint64_t next asm("x2") = current_offset;

        __asm__ __volatile__ (
            "start_loop_%=:"
            #include "loop.h"
            "subs %0, %0, #1;"
            "bne start_loop_%=;"
            : "+r" (i), "+r" (next)
            : "r" (start)
            : "x3", "cc", "memory"
        );

        current_offset = next;
)";
}

std::string ArmAssembler::generateNopFile() const {
    return R"(#include <stdlib.h>
#include <stdio.h>


void volatile nop_(void) {

    asm __volatile__ (
      "cmp x4, #0x0;\n"
      "bne 1f;\n"
      "b 3f;\n"
      "1:\n"
      "mov x10, x4;\n"
      "2:\n"
      "nop;\n"
      "subs x10, x10, #0x01;\n"
      "cmp x10, #0x0;\n"
      "bne 2b;\n"
      "3:\n"
      :
      :
      : "x30", "x4", "x10"
    );

}
)";
}

