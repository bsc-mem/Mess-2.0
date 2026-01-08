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

#include "arch/riscv/RiscvArchitecture.h"
#include "arch/riscv/RiscvAssembler.h"
#include "arch/riscv/counters/SiFiveCounters.h"
#include "arch/riscv/RiscvCounters.h"  // Generic fallback
#include "architecture/ArchitectureRegistry.h"
#include <algorithm>
#include <string>
#include <cctype>
#include <algorithm>

std::unique_ptr<KernelAssembler> RiscvArchitecture::createAssembler(const KernelConfig& config) const {
    return std::make_unique<RiscvAssembler>(config);
}

std::unique_ptr<PerformanceCounterStrategy> RiscvArchitecture::createCounterStrategy(const CPUCapabilities& caps) const {
    auto toLower = [](const std::string& str) {
        std::string result = str;
        std::transform(result.begin(), result.end(), result.begin(),
                       [](unsigned char c) { return std::tolower(c); });
        return result;
    };
    
    auto modelContains = [&](const std::string& token) {
        return toLower(caps.model_name).find(toLower(token)) != std::string::npos;
    };
    
    if (modelContains("fu740") || modelContains("hifive")) {
        return std::make_unique<SiFiveCounters>();
    }
    
    if (modelContains("sifive")) {
        return std::make_unique<SiFiveCounters>();
    }
    
    return std::make_unique<RiscvCounters>();
}

std::vector<std::shared_ptr<ISA>> RiscvArchitecture::getSupportedISAs() const { return {}; }
std::shared_ptr<ISA> RiscvArchitecture::selectBestISA(const CPUCapabilities&) const { return nullptr; }

std::string RiscvArchitecture::generateNopFile() const {
    return R"(#include <stdlib.h>
#include <stdio.h>

extern "C" void volatile nop_(void) {
    asm __volatile__ (
        "bne x22, x0, start_pause;\n"
        "j end;\n"
        "start_pause:"
        "mv x23, x22;\n"
        "start_loop:\n"
        "nop;\n"
        "addi x23, x23, -1;\n"
        "bne x23, x0, start_loop;\n"
        "end:"
        :
        :
        : "x30", "x4", "x10", "x23"
    );
}

int nop(int *ntimes) {
    return 0;
}
)";
}

double RiscvArchitecture::getUpiScalingFactor(const CPUCapabilities&) const {
    return 1.0;
}

static ArchitectureRegistrar<RiscvArchitecture> riscv_registrar;
