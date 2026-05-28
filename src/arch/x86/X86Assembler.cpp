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

#include "arch/x86/X86Assembler.h"
#include <algorithm>

namespace {

AddressingPolicy resolveAddressingPolicy(const KernelConfig& config) {
    if (config.addressing_policy == AddressingPolicy::AUTO) {
        // x86 has no post-increment memory addressing mode; default to indexed.
        return AddressingPolicy::INDEXED;
    }
    return config.addressing_policy;
}

int x86BytesPerOp(ISAMode mode) {
    if (mode == ISAMode::SCALAR) {
        return 8;
    }
    if (mode == ISAMode::SSE2) {
        return 16;
    }
    return (mode == ISAMode::AVX512) ? 64 : 32;
}

const char* x86VectorRegPrefix(ISAMode mode) {
    if (mode == ISAMode::AVX512) return "zmm";
    if (mode == ISAMode::SSE2) return "xmm";
    return "ymm";
}

const char* x86VectorLoadInstr(ISAMode mode) {
    return (mode == ISAMode::SSE2) ? "movupd" : "vmovupd";
}

const char* x86VectorStoreInstr(ISAMode mode, bool nontemporal) {
    if (mode == ISAMode::SSE2) {
        return nontemporal ? "movntpd" : "movupd";
    }
    return nontemporal ? "vmovntpd" : "vmovupd";
}

}

static const char* getScalarRegName(int reg) {
    static const char* regs[] = {"rax", "rcx", "rdx", "rsi", "rdi", "r8", "r9", "r13", "r14"};
    return regs[reg % 9];
}

std::string X86Assembler::generateLoad(int offset, int reg) const {
    std::ostringstream oss;
    AddressingPolicy policy = resolveAddressingPolicy(config_);

    if (config_.isa_mode == ISAMode::SCALAR) {
        if (policy == AddressingPolicy::POST_INCREMENT) {
            oss << "        \"movq      (%%r11), %%" << getScalarRegName(reg) << ";\\n\"\n"
                << "        \"addq      $8, %%r11;\\n\"\n";
        } else {
            oss << "        \"movq      " << offset << "(%%r11,%%rbx,8), %%" << getScalarRegName(reg) << ";\\n\"\n";
        }
    } else {
        const char* instr = x86VectorLoadInstr(config_.isa_mode);
        const char* prefix = x86VectorRegPrefix(config_.isa_mode);
        if (policy == AddressingPolicy::POST_INCREMENT) {
            int bytes = x86BytesPerOp(config_.isa_mode);
            oss << "        \"" << instr << "   (%%r11), %%" << prefix << reg << ";\\n\"\n"
                << "        \"addq      $" << bytes << ", %%r11;\\n\"\n";
        } else {
            oss << "        \"" << instr << "   " << offset << "(%%r11,%%rbx,8), %%" << prefix << reg << ";\\n\"\n";
        }
    }
    return oss.str();
}

std::string X86Assembler::generateStore(int offset, int reg) const {
    std::ostringstream oss;
    AddressingPolicy policy = resolveAddressingPolicy(config_);

    if (config_.isa_mode == ISAMode::SCALAR) {
        const char* instr = config_.use_nontemporal_stores ? "movnti" : "movq";
        if (policy == AddressingPolicy::POST_INCREMENT) {
            oss << "        \"" << instr << "    %%" << getScalarRegName(reg) << ", (%%r10);\\n\"\n"
                << "        \"addq      $8, %%r10;\\n\"\n";
        } else {
            oss << "        \"" << instr << "    %%" << getScalarRegName(reg) << ", " << offset << "(%%r10,%%rbx,8);\\n\"\n";
        }
    } else {
        const char* instr = x86VectorStoreInstr(config_.isa_mode, config_.use_nontemporal_stores);
        const char* prefix = x86VectorRegPrefix(config_.isa_mode);
        if (policy == AddressingPolicy::POST_INCREMENT) {
            int bytes = x86BytesPerOp(config_.isa_mode);
            oss << "        \"" << instr << "  %%" << prefix << reg << ", (%%r10);\\n\"\n"
                << "        \"addq      $" << bytes << ", %%r10;\\n\"\n";
        } else {
            oss << "        \"" << instr << "  %%" << prefix << reg << ", " << offset << "(%%r10,%%rbx,8);\\n\"\n";
        }
    }
    return oss.str();
}

std::string X86Assembler::generateLoopControl(int increment, int labelId) const {
    std::ostringstream oss;
    oss << "\n        \"//-----------------------------------------------\\n\"\n"
        << "\n"
        << "        \"add       $" << (increment / 8) << ", %%rbx;\\n\"\n"
        << "\n"
        << "        \"cmp       %%r12, %%rbx;\\n\"\n"
        << "\n"
        << "        \"jb        ..L_" << labelId << ";\\n\"\n";
    return oss.str();
}

std::string X86Assembler::generatePause() const {
    // Avoid function call overhead for pause==0 by inlining the delay loop.
    return "        \"movl      (%%r15), %%edi;\\n\"\n"
           "        \"testl     %%edi, %%edi;\\n\"\n"
           "        \"jz        2f;\\n\"\n"
           "        \"1:\\n\"\n"
           "        \"nop;\\n\"\n"
           "        \"decl      %%edi;\\n\"\n"
           "        \"jnz       1b;\\n\"\n"
           "        \"2:\\n\"\n";
}

std::string X86Assembler::generateHeader() const {
    std::ostringstream oss;
    oss << "/*\n"
        << " * Generated by Mess Kernel Generator (x86)\n"
        << " * Mode: MultiSequential access\n"
        << " */\n\n"
        << "#include <stdio.h>\n"
        << "#include <unistd.h>\n"
        << "#include \"utils.h\"\n\n"
        << "void print_usage(char *argv[], char* usage)\n"
        << "{\n"
        << "    fprintf(stderr,\"Usage: %s %s\", argv[0], usage);\n"
        << "}\n\n";
    return oss.str();
}

std::string X86Assembler::generateRegisterSetup() const {
    return "    register double *a asm(\"r10\");\n"
           "    a = a_array;\n"
           "    register double *b asm(\"r11\");\n"
           "    b = b_array;\n"
           "    register ssize_t i asm(\"rbx\");\n"
           "    i = 0;\n"
           "    register ssize_t n asm(\"r12\");\n"
           "    n = *array_size;\n"
           "    register int *p asm(\"r15\");\n"
           "    p = pause;\n\n";
}

std::string X86Assembler::generateVectorRegisterInit() const {
    if (config_.isa_mode == ISAMode::SCALAR) {
        return "";
    }
    if (config_.isa_mode == ISAMode::AVX512) {
        return "        \"vpxorq %%zmm1, %%zmm1, %%zmm1;\\n\"\n\n";
    }
    if (config_.isa_mode == ISAMode::SSE2) {
        return "        \"xorpd %%xmm1, %%xmm1;\\n\"\n\n";
    }
    return "        \"vxorpd %%ymm1, %%ymm1, %%ymm1;\\n\"\n\n";
}

std::string X86Assembler::generateFooter() const {
    return "";
}

std::string X86Assembler::generateAsmStart() const {
    return "    asm volatile (\n";
}

std::string X86Assembler::generateAsmEnd() const {
    return "      :\n"
           "      : \"r\" (a), \"r\" (b), \"r\" (i), \"r\" (n), \"r\" (p)\n"
           "      : \"rdi\", \"memory\", \"cc\"\n"
           "    );\n";
}

std::string X86Assembler::getPointerChaseLoopAsm() const {
    return "            \"dec %%rcx;\"\n"
           "            \"jnz start_loop_%=;\"";
}

std::string X86Assembler::getPointerChaseInstruction() const {
    return "        \"mov (%%rbx,%%rax), %%rax;\"";
}

std::string X86Assembler::generatePointerChaseBurstLoop() const {
    return R"(
        __asm__ __volatile__ (
            "start_loop_%=:\n"
            #include "loop.h"
            "dec %%rcx;\n"
            "jnz start_loop_%=;\n"
            : "+a" (current_offset), "=c" (dummy_rcx)
            : "b" (array), "c" (BURST_ITERS)
            : "cc", "memory"
        );
)";
}

std::string X86Assembler::generateNopFile() const {
    return R"(#include <stdlib.h>
#include <stdio.h>

int nop(int *ntimes) {
	unsigned int i  = *ntimes;
        if ( !i ) {
            return 0;
        } else {
            asm(
                "mov %0, %%ecx;\n"
                "the_loop%=:\n"
                "nop;\n"
                "dec %%ecx;\n"
                "jnz the_loop%=;\n"
                :
                : "r" (i)
                :"ecx"
            );
        }

	return 0;
}

int nop_(int *ntimes)
{
    return nop(ntimes);
}
)";
}

