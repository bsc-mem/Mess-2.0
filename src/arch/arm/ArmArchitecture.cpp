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

#include "arch/arm/ArmArchitecture.h"
#include "arch/arm/ArmAssembler.h"
#include "arch/arm/counters/A64FXCounters.h"
#include "arch/arm/counters/Graviton3Counters.h"
#include "arch/arm/counters/NvidiaGraceCounters.h"
#include "arch/arm/ArmCounters.h"
#include "architecture/ArchitectureRegistry.h"
#include <algorithm>
#include <string>
#include <cctype>

static ArchitectureRegistrar<ArmArchitecture> arm_registrar;

namespace {

class ArmISA : public ISA {
    ISAMode mode_;
    std::string name_;
    int width_bits_;
    int regs_;
public:
    ArmISA(ISAMode mode, std::string name, int width_bits, int regs)
        : mode_(mode), name_(std::move(name)), width_bits_(width_bits), regs_(regs) {}

    ISAMode getMode() const override { return mode_; }
    std::string getName() const override { return name_; }
    int getVectorWidthBits() const override { return width_bits_; }
    int getMaxSimdRegisters() const override { return regs_; }
};

std::shared_ptr<ISA> makeArmISA(ISAMode mode, const std::string& name, int width_bits, int regs) {
    return std::make_shared<ArmISA>(mode, name, width_bits, regs);
}

}

std::unique_ptr<KernelAssembler> ArmArchitecture::createAssembler(const KernelConfig& config) const {
    return std::make_unique<ArmAssembler>(config);
}

std::unique_ptr<PerformanceCounterStrategy> ArmArchitecture::createCounterStrategy(const CPUCapabilities& caps) const {
    auto toLower = [](const std::string& str) {
        std::string result = str;
        std::transform(result.begin(), result.end(), result.begin(),
                       [](unsigned char c) { return std::tolower(c); });
        return result;
    };
    
    auto modelContains = [&](const std::string& token) {
        return toLower(caps.model_name).find(toLower(token)) != std::string::npos;
    };
    
    if (modelContains("a64fx")) {
        return std::make_unique<A64FXCounters>();
    } else if (modelContains("graviton3") || modelContains("graviton4")) {
        return std::make_unique<Graviton3Counters>();
    } else if (modelContains("grace")) {
        return std::make_unique<NvidiaGraceCounters>(caps);
    }
    
    if (modelContains("fujitsu")) {
        return std::make_unique<A64FXCounters>();
    } else if (modelContains("graviton") || modelContains("neoverse")) {
        return std::make_unique<Graviton3Counters>();
    } else if (modelContains("nvidia")) {
        return std::make_unique<NvidiaGraceCounters>(caps);
    }
    
    return std::make_unique<ArmCounters>(caps);
}

std::vector<std::shared_ptr<ISA>> ArmArchitecture::getSupportedISAs() const {
    return {
        makeArmISA(ISAMode::SVE, "SVE", 512, 32),
        makeArmISA(ISAMode::NEON, "NEON", 256, 32)
    };
}

std::shared_ptr<ISA> ArmArchitecture::selectBestISA(const CPUCapabilities& /*caps*/) const {
    return makeArmISA(ISAMode::NEON, "NEON", 256, 32);
}

std::string ArmArchitecture::generateNopFile() const {
    return R"(#include <stdlib.h>
#include <stdio.h>


void volatile nop_(void) {

    asm __volatile__ (
      "cmp x4, #0x0;\n"
      "bne start_pause;\n"
      "b end;\n"
      "start_pause:"
      "mov x10, x4;\n"
      "start_loop:\n"
      "nop;\n"
      "subs x10, x10, #0x01;\n"
      "cmp x10, #0x0;\n"
      "bne start_loop;\n"
      // "blr x30;\n"
      "end:"
      :
      :
      : "x30", "x4", "x10"
    );

}
)";
}

double ArmArchitecture::getUpiScalingFactor(const CPUCapabilities&) const {
    return 1.0;
}

