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

#ifndef X86_ASSEMBLER_H
#define X86_ASSEMBLER_H

#include "architecture/KernelAssembler.h"
#include "architecture/ISA.h"
#include <sstream>

class X86Assembler : public KernelAssembler {
    KernelConfig config_;
public:
    X86Assembler(const KernelConfig& config) : config_(config) {}

    std::string generateLoad(int offset, int reg) const override;
    std::string generateStore(int offset, int reg) const override;
    std::string generateLoopControl(int increment, int labelId) const override;
    std::string generatePause() const override;
    
    std::string generateHeader() const override;
    std::string generateRegisterSetup() const override;
    std::string generateVectorRegisterInit() const override;
    std::string generateFooter() const override;
    std::string generateAsmStart() const override;
    std::string generateAsmEnd() const override;
    
    
    std::string getPointerChaseLoopAsm() const override;
    std::string getPointerChaseInstruction() const override;
    std::string generatePointerChaseBurstLoop() const override;
    
    std::string generateNopFile() const override;
    
};

#endif
