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

 #include "measurement/InstructionSamplerFactory.h"
#include "measurement/instruction_samplers/IntelPebsSampler.h"
#include "measurement/instruction_samplers/ArmSpeSampler.h"
#include <iostream>
#include <sstream>

std::string InstructionSamplerFactory::backend_name(SamplerBackend b) {
    switch (b) {
        case SamplerBackend::INTEL_PEBS: return "Intel PEBS";
        case SamplerBackend::ARM_SPE:    return "ARM SPE";
        case SamplerBackend::NONE:       return "None";
        case SamplerBackend::AUTO:       return "Auto";
    }
    return "Unknown";
}

SamplerBackend InstructionSamplerFactory::detect_best_backend() {
    {
        IntelPebsSampler probe(50, 997, 0);
        if (probe.is_available()) return SamplerBackend::INTEL_PEBS;
    }
    {
        ArmSpeSampler probe(50, 4096, 0);
        if (probe.is_available()) return SamplerBackend::ARM_SPE;
    }
    return SamplerBackend::NONE;
}

std::string InstructionSamplerFactory::availability_diagnostic(CPUArchitecture arch) {
    if (detect_best_backend() != SamplerBackend::NONE) {
        return "";
    }

    std::ostringstream oss;
    oss << "Instruction sampling not supported on this CPU/Kernel (no PEBS or SPE found)";

    if (arch == CPUArchitecture::ARM64) {
        oss << "\n    ARM SPE diagnostic: " << ArmSpeSampler::availability_diagnostic();
    }

    return oss.str();
}

std::unique_ptr<InstructionSampler> InstructionSamplerFactory::create(
    SamplerBackend backend,
    uint64_t latency_threshold,
    int sample_rate,
    int verbosity) {

    SamplerBackend resolved = backend;
    if (resolved == SamplerBackend::AUTO) {
        resolved = detect_best_backend();
        if (verbosity >= 1) {
            std::cout << "    Instruction sampler auto-detected: "
                      << backend_name(resolved) << "\n";
        }
    }

    switch (resolved) {
        case SamplerBackend::INTEL_PEBS: {
            int freq = (sample_rate > 0) ? sample_rate : 997;
            auto s = std::make_unique<IntelPebsSampler>(latency_threshold, freq, verbosity);
            if (s->is_available()) return s;
            if (verbosity >= 1) {
                std::cerr << "    Warning: Intel PEBS requested but not available\n";
            }
            return nullptr;
        }
        case SamplerBackend::ARM_SPE: {
            int period = (sample_rate > 0) ? sample_rate : 4096;
            auto s = std::make_unique<ArmSpeSampler>(latency_threshold, period, verbosity);
            if (s->is_available()) return s;
            if (verbosity >= 1) {
                std::cerr << "    Warning: ARM SPE requested but not available\n";
            }
            return nullptr;
        }
        case SamplerBackend::NONE:
        case SamplerBackend::AUTO:
            return nullptr;
    }
    return nullptr;
}

