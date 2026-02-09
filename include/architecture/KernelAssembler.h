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

#ifndef KERNEL_ASSEMBLER_H
#define KERNEL_ASSEMBLER_H

#include <string>
#include "kernel_types.h"

class KernelAssembler {
public:
    virtual ~KernelAssembler() = default;

    virtual std::string generateLoad(int offset, int reg) const = 0;
    virtual std::string generateStore(int offset, int reg) const = 0;
    virtual std::string generateLoopControl(int increment, int labelId) const = 0;
    virtual std::string generatePause() const = 0;
    
    virtual std::string generateHeader() const = 0;
    virtual std::string generateRegisterSetup() const = 0;
    virtual std::string generateVectorRegisterInit() const = 0;
    virtual std::string generateFooter() const = 0;
    virtual std::string generateAsmStart() const = 0;
    virtual std::string generateAsmEnd() const = 0;
    
    virtual std::string getPointerChaseLoopAsm() const = 0;
    virtual std::string getPointerChaseInstruction() const = 0;
    virtual std::string generatePointerChaseBurstLoop() const = 0;

    virtual std::string generateNopFile() const = 0;
    
};

#endif
