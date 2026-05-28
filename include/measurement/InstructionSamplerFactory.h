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

#ifndef INSTRUCTION_SAMPLER_FACTORY_H
#define INSTRUCTION_SAMPLER_FACTORY_H

#include "InstructionSampler.h"
#include "../SystemDetection.h"
#include <memory>
#include <string>

/**
 * @file InstructionSamplerFactory.h
 * @brief Factory helpers for selecting an instruction-sampling backend.
 */

/** @brief Instruction-sampling backends supported by Mess. */
enum class SamplerBackend {
    AUTO,
    INTEL_PEBS,
    ARM_SPE,
    NONE
};

/**
 * @brief Creates and auto-detects instruction-sampling backends.
 */
class InstructionSamplerFactory {
public:
    static std::unique_ptr<InstructionSampler> create(
        SamplerBackend backend = SamplerBackend::AUTO,
        uint64_t latency_threshold = 50,
        int sample_rate = 0,
        int verbosity = 0);

    static SamplerBackend detect_best_backend();

    static std::string backend_name(SamplerBackend b);

    /**
     * @brief Returns a user-facing diagnostic when no backend is available.
     */
    static std::string availability_diagnostic(CPUArchitecture arch);
};

#endif

