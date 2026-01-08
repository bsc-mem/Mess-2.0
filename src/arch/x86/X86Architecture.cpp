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

#include "arch/x86/X86Architecture.h"
#include "arch/x86/X86Assembler.h"
#include "arch/x86/counters/IntelSkylakeCounters.h"
#include "arch/x86/counters/IntelSPRCounters.h"
#include "arch/x86/counters/IntelEmeraldRapidsCounters.h"
#include "arch/x86/counters/IntelGraniteRapidsCounters.h"
#include "arch/x86/counters/AmdZenCounters.h"
#include "arch/x86/counters/AmdZen4Counters.h"
#include "arch/x86/X86Counters.h"
#include "architecture/ArchitectureRegistry.h"
#include <algorithm>
#include <string>
#include <cctype>

static ArchitectureRegistrar<X86Architecture> x86_registrar;

namespace {

class X86ISA : public ISA {
    ISAMode mode_;
    std::string name_;
    int width_bits_;
    int regs_;
public:
    X86ISA(ISAMode mode, std::string name, int width_bits, int regs)
        : mode_(mode), name_(std::move(name)), width_bits_(width_bits), regs_(regs) {}

    ISAMode getMode() const override { return mode_; }
    std::string getName() const override { return name_; }
    int getVectorWidthBits() const override { return width_bits_; }
    int getMaxSimdRegisters() const override { return regs_; }
};



std::string toLower(const std::string& str) {
    std::string result = str;
    std::transform(result.begin(), result.end(), result.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    return result;
}

bool modelContains(const std::string& model, const std::string& token) {
    return toLower(model).find(toLower(token)) != std::string::npos;
}

}

bool X86Architecture::supports(const CPUCapabilities& caps) const {
    return caps.arch == CPUArchitecture::X86_64;
}

std::unique_ptr<KernelAssembler> X86Architecture::createAssembler(const KernelConfig& config) const {
    return std::make_unique<X86Assembler>(config);
}

std::unique_ptr<PerformanceCounterStrategy> X86Architecture::createCounterStrategy([[maybe_unused]] const CPUCapabilities& caps) const {
    std::string model = caps.model_name;
    
    if (caps.vendor == CPUVendor::INTEL) {
        if (modelContains(model, "granite") || modelContains(model, "gnr")) {
            return std::make_unique<IntelGraniteRapidsCounters>();
        } else if (modelContains(model, "emerald") || modelContains(model, "emr")) {
            return std::make_unique<IntelEmeraldRapidsCounters>();
        } else if (modelContains(model, "sapphire") || modelContains(model, "spr")) {
            return std::make_unique<IntelSPRCounters>();
        } else if (modelContains(model, "skylake")) {
            return std::make_unique<IntelSkylakeCounters>();
        }
        
        return std::make_unique<IntelGraniteRapidsCounters>();
        
    } else if (caps.vendor == CPUVendor::AMD) {
        if (modelContains(model, "genoa") || modelContains(model, "bergamo") || modelContains(model, "zen4") || modelContains(model, "zen 4")) {
            return std::make_unique<AmdZen4Counters>();
        } else if (modelContains(model, "milan") || modelContains(model, "rome") || modelContains(model, "zen3") || modelContains(model, "zen 3") || modelContains(model, "zen2") || modelContains(model, "zen 2")) {
            return std::make_unique<AmdZenCounters>();
        }
        
        return std::make_unique<AmdZen4Counters>();
    }
    
    return std::make_unique<X86Counters>();
}

std::vector<std::shared_ptr<ISA>> X86Architecture::getSupportedISAs() const {
    return {
        std::make_shared<X86ISA>(ISAMode::AVX512, "AVX-512", 512, 32),
        std::make_shared<X86ISA>(ISAMode::AVX2,   "AVX2",    256, 16),
        std::make_shared<X86ISA>(ISAMode::AVX,    "AVX",     256, 16)
    };
}

std::shared_ptr<ISA> X86Architecture::selectBestISA(const CPUCapabilities& caps) const {
    auto has = [&](ISAExtension e){ 
        return std::find(caps.extensions.begin(), caps.extensions.end(), e) != caps.extensions.end(); 
    };

    if (has(ISAExtension::AVX2))   return std::make_shared<X86ISA>(ISAMode::AVX2,   "AVX2",    256, 16);
    if (has(ISAExtension::AVX512)) return std::make_shared<X86ISA>(ISAMode::AVX512, "AVX-512", 512, 32);
    if (has(ISAExtension::AVX))    return std::make_shared<X86ISA>(ISAMode::AVX,    "AVX",     256, 16);
    
    return std::make_shared<X86ISA>(ISAMode::AVX, "AVX", 256, 16); // Fallback
}

std::string X86Architecture::generateNopFile() const {
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

double X86Architecture::getUpiScalingFactor(const CPUCapabilities& caps) const {
    if (caps.vendor == CPUVendor::INTEL) {
        // Intel UPI typically uses 1 header flit + 8 data flits = 9 flits per 64B cache line
        return 1.0 / 9.0;
    } else if (caps.vendor == CPUVendor::AMD) {
        return 1.0;
    }
    return 1.0;
}

