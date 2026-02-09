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

#include "arch/power/PowerArchitecture.h"
#include "arch/power/PowerAssembler.h"
#include "arch/power/PowerCounters.h"
#include "architecture/ArchitectureRegistry.h"
#include "architecture/ISA.h"
#include <algorithm>

namespace {

class PowerISA : public ISA {
public:
    PowerISA(ISAMode mode, std::string name, int width_bits, int regs)
        : mode_(mode), name_(std::move(name)), width_bits_(width_bits), regs_(regs) {}

    ISAMode getMode() const override { return mode_; }
    std::string getName() const override { return name_; }
    int getVectorWidthBits() const override { return width_bits_; }
    int getMaxSimdRegisters() const override { return regs_; }

private:
    ISAMode mode_;
    std::string name_;
    int width_bits_;
    int regs_;
};

}

std::unique_ptr<KernelAssembler> PowerArchitecture::createAssembler(const KernelConfig& config) const {
    return std::make_unique<PowerAssembler>(config);
}

std::unique_ptr<BandwidthCounterStrategy> PowerArchitecture::createCounterStrategy(const CPUCapabilities&) const {
    return std::make_unique<PowerCounters>();
}

std::vector<std::shared_ptr<ISA>> PowerArchitecture::getSupportedISAs() const {
    return {
        std::make_shared<PowerISA>(ISAMode::VSX, "VSX", 128, 64),
        std::make_shared<PowerISA>(ISAMode::VMX, "VMX", 128, 32),
        std::make_shared<PowerISA>(ISAMode::SCALAR, "SCALAR", 64, 16)
    };
}

std::shared_ptr<ISA> PowerArchitecture::selectBestISA(const CPUCapabilities& caps) const {
    auto has = [&](ISAExtension e) {
        return std::find(caps.extensions.begin(), caps.extensions.end(), e) != caps.extensions.end();
    };

    if (has(ISAExtension::VSX)) {
        return std::make_shared<PowerISA>(ISAMode::VSX, "VSX", 128, 64);
    }
    if (has(ISAExtension::VMX)) {
        return std::make_shared<PowerISA>(ISAMode::VMX, "VMX", 128, 32);
    }
    return std::make_shared<PowerISA>(ISAMode::SCALAR, "SCALAR", 64, 16);
}

double PowerArchitecture::getUpiScalingFactor(const CPUCapabilities&) const {
    return 1.0;
}

static ArchitectureRegistrar<PowerArchitecture> power_registrar;
