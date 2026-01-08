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

#include "arch/riscv/RiscvAssembler.h"

std::string RiscvAssembler::generateLoad(int offset, int reg) const {
    std::ostringstream oss;
    oss << "        \"vle64.v v" << reg << ", (%1);\\n\"\n"
        << "        \"addi %1, %1, " << offset << ";\\n\"\n";
    return oss.str();
}

std::string RiscvAssembler::generateStore(int offset, int reg) const {
    std::ostringstream oss;
    oss << "        \"vse64.v v" << reg << ", (%0);\\n\"\n"
        << "        \"addi %0, %0, " << offset << ";\\n\"\n";
    return oss.str();
}

std::string RiscvAssembler::generateLoopControl(int increment, int labelId) const {
    std::ostringstream oss;
    oss << "\n        \"sub %2, %2, " << increment << ";\\n\"\n"
        << "        \"bnez %2, .L_" << labelId << ";\\n\"\n";
    return oss.str();
}

std::string RiscvAssembler::generatePause() const { return ""; }

std::string RiscvAssembler::generateHeader() const { return ""; }
std::string RiscvAssembler::generateRegisterSetup() const { return ""; }
std::string RiscvAssembler::generateVectorRegisterInit() const { return ""; }
std::string RiscvAssembler::generateFooter() const { return ""; }

std::string RiscvAssembler::generateAsmStart() const {
    return "    asm (\n";
}

std::string RiscvAssembler::generateAsmEnd() const {
    return "\n    );\n";
}

std::string RiscvAssembler::getPointerChaseLoopAsm() const {
    return "            \"addi %2, %2, -1;\"\n"
           "            \"bnez %2, start_loop_%%=;\"";
}

std::string RiscvAssembler::getPointerChaseInstruction() const {
    return "        \"add x21, %2, %1; ld %1, 0(x21);\"";
}

std::string RiscvAssembler::generatePointerChaseBurstLoop() const {
    return R"(
        register uint64_t i = BURST_ITERS;
        register struct line *start = array;
        register uint64_t next = current_offset;

        __asm__ __volatile__ (
            "li x20, 1;"
            "start_loop_%=:"
            #include "loop.h"
            "sub %0, %0, x20;"
            "bgt %0, x0, start_loop_%=;"
            : "+r" (i), "+r" (next)
            : "r" (start)
            : "x20", "x21", "memory"
        );

        current_offset = next;
)";
}


