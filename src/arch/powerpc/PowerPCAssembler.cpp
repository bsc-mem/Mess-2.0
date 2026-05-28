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

#include "arch/powerpc/PowerPCAssembler.h"
#include <algorithm>

namespace {

bool isVmxMode(ISAMode mode) {
    return mode == ISAMode::VMX;
}

bool isVsxMode(ISAMode mode) {
    return mode == ISAMode::VSX;
}

bool isPowerVectorMode(ISAMode mode) {
    return isVmxMode(mode) || isVsxMode(mode);
}

int resolvedPowerPCBytesPerMemOp(const KernelConfig& config) {
    if (config.resolved_bytes_per_mem_op > 0) {
        return config.resolved_bytes_per_mem_op;
    }
    return isPowerVectorMode(config.isa_mode) ? 16 : 8;
}

}

std::string PowerPCAssembler::generateLoad(int /*offset*/, int reg) const {
    std::ostringstream oss;
    const int step = resolvedPowerPCBytesPerMemOp(config_);
    if (isVsxMode(config_.isa_mode)) {
        oss << "        \"lxvd2x " << reg << ", 0, %1;\\n\"\n";
    } else if (isVmxMode(config_.isa_mode)) {
        oss << "        \"lvx v" << reg << ", 0, %1;\\n\"\n";
    } else {
        oss << "        \"ld 31, 0(%1);\\n\"\n";
    }
    oss << "        \"addi %1, %1, " << step << ";\\n\"\n";
    return oss.str();
}

std::string PowerPCAssembler::generateStore(int /*offset*/, int reg) const {
    std::ostringstream oss;
    const int step = resolvedPowerPCBytesPerMemOp(config_);
    if (isVsxMode(config_.isa_mode)) {
        oss << "        \"stxvd2x " << reg << ", 0, %0;\\n\"\n";
    } else if (isVmxMode(config_.isa_mode)) {
        oss << "        \"stvx v" << reg << ", 0, %0;\\n\"\n";
    } else {
        oss << "        \"std 31, 0(%0);\\n\"\n";
    }
    oss << "        \"addi %0, %0, " << step << ";\\n\"\n";
    return oss.str();
}

std::string PowerPCAssembler::generateLoopControl(int increment, int labelId) const {
    (void)increment;
    std::ostringstream oss;
    oss << "\n        \"bdnz .L_" << labelId << ";\\n\"\n";
    return oss.str();
}

std::string PowerPCAssembler::generatePause() const { return ""; }

std::string PowerPCAssembler::generateHeader() const { return ""; }
std::string PowerPCAssembler::generateRegisterSetup() const { return ""; }

std::string PowerPCAssembler::generateVectorRegisterInit() const { return ""; }

std::string PowerPCAssembler::generateFooter() const { return ""; }

std::string PowerPCAssembler::generateAsmStart() const {
    return "    asm (\n";
}

std::string PowerPCAssembler::generateAsmEnd() const {
    return "\n    );\n";
}

std::string PowerPCAssembler::getPointerChaseLoopAsm() const {
    return "            \"bdnz start_loop_%%=;\"";
}

std::string PowerPCAssembler::getPointerChaseInstruction() const {
    return "        \"add %3, %2, %1; ld %1, 0(%3);\"";
}

std::string PowerPCAssembler::generatePointerChaseBurstLoop() const {
    return R"(
        register uint64_t i = BURST_ITERS;
        register struct line *start = array;
        register uint64_t next = current_offset;
        register uint64_t tmp;

        __asm__ __volatile__ (
            "start_loop_%=:!"
            #include "loop.h"
            "addi %0, %0, -1;"
            "cmpdi %0, 0;"
            "bne start_loop_%=;"
            : "+r" (i), "+r" (next)
            : "r" (start), "r" (tmp)
            : "cc", "memory"
        );

        current_offset = next;
)";
}


std::string PowerPCAssembler::generateNopFile() const {
    return R"(#include <stdlib.h>
#include <stdio.h>

extern "C" void volatile nop_(void) {
    asm __volatile__ (
        "cmpli 0, 1, 4, 0;\n"
        "bne start_pause;\n"
        "b end;\n"
        "start_pause:"
        "start_loop2:\n"
        "nop;\n"
        "addi 4, 4, -0x01;\n"
        "cmpli 0, 1, 4, 0x00;\n"
        "bne start_loop2;\n"
        "end:"
        :
        :
        : "4", "10"
    );
}

int nop(int *ntimes) {
    return 0;
}
)";

}

