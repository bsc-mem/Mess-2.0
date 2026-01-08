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

#include "arch/power/PowerAssembler.h"

std::string PowerAssembler::generateLoad(int offset, int reg) const {
    std::ostringstream oss;
    oss << "        \"lxvd2x " << reg << ", 0, %1;\\n\"\n"
        << "        \"addi %1, %1, " << offset << ";\\n\"\n";
    return oss.str();
}

std::string PowerAssembler::generateStore(int offset, int reg) const {
    std::ostringstream oss;
    oss << "        \"stxvd2x " << reg << ", 0, %0;\\n\"\n"
        << "        \"addi %0, %0, " << offset << ";\\n\"\n";
    return oss.str();
}

std::string PowerAssembler::generateLoopControl([[maybe_unused]] int increment, int labelId) const {
    std::ostringstream oss;
    oss << "\n        \"bdnz .L_" << labelId << ";\\n\"\n";
    return oss.str();
}

std::string PowerAssembler::generatePause() const { return ""; }

std::string PowerAssembler::generateHeader() const { return ""; }
std::string PowerAssembler::generateRegisterSetup() const { return ""; }

std::string PowerAssembler::generateVectorRegisterInit() const { return ""; }

std::string PowerAssembler::generateFooter() const { return ""; }

std::string PowerAssembler::generateAsmStart() const {
    return "    asm (\n";
}

std::string PowerAssembler::generateAsmEnd() const {
    return "\n    );\n";
}

std::string PowerAssembler::getPointerChaseLoopAsm() const {
    return "            \"bdnz start_loop_%%=;\"";
}

std::string PowerAssembler::getPointerChaseInstruction() const {
    return "        \"add %3, %2, %1; ld %1, 0(%3);\"";
}

std::string PowerAssembler::generatePointerChaseBurstLoop() const {
    return R"(
        register uint64_t i = BURST_ITERS;
        register struct line *start = array;
        register uint64_t next = current_offset;
        register uint64_t tmp;

        __asm__ __volatile__ (
            "start_loop_%=:"
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


