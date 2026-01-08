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
#include <algorithm>

std::unique_ptr<KernelAssembler> PowerArchitecture::createAssembler(const KernelConfig& config) const {
    return std::make_unique<PowerAssembler>(config);
}

std::unique_ptr<PerformanceCounterStrategy> PowerArchitecture::createCounterStrategy(const CPUCapabilities&) const {
    return std::make_unique<PowerCounters>();
}

std::vector<std::shared_ptr<ISA>> PowerArchitecture::getSupportedISAs() const { return {}; }
std::shared_ptr<ISA> PowerArchitecture::selectBestISA(const CPUCapabilities&) const { return nullptr; }

std::string PowerArchitecture::generateNopFile() const {
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

double PowerArchitecture::getUpiScalingFactor(const CPUCapabilities&) const {
    return 1.0;
}

static ArchitectureRegistrar<PowerArchitecture> power_registrar;
