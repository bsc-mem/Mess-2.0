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

#ifndef ARCHITECTURE_REGISTRY_H
#define ARCHITECTURE_REGISTRY_H

#include <memory>
#include <vector>
#include <functional>
#include "Architecture.h"

/**
 * @file ArchitectureRegistry.h
 * @brief Registration and lookup support for architecture plugins.
 */

/**
 * @brief Global registry of architecture factories.
 */
/**
 * @brief Singleton registry for architecture-specific code generation backends.
 * 
 * Uses a Factory pattern to maintain a list of supported `Architecture` implementations 
 * (e.g., `X86Architecture`, `ArmArchitecture`). During initialization, modules register their 
 * factories here. At runtime, the registry evaluates the detected `CPUCapabilities` to instantiate 
 * the correct architecture plugin capable of producing valid inline assembly and configuring 
 * the appropriate hardware counters.
 */
class ArchitectureRegistry {
public:
    static ArchitectureRegistry& instance();

    void registerFactory(std::function<std::unique_ptr<Architecture>()> factory);
    
    std::unique_ptr<Architecture> getArchitecture(const CPUCapabilities& caps) const;

private:
    ArchitectureRegistry() = default;
    std::vector<std::function<std::unique_ptr<Architecture>()>> factories_;
};

/**
 * @brief Helper that registers an architecture type during static initialization.
 */
template<typename T>
struct ArchitectureRegistrar {
    ArchitectureRegistrar() {
        ArchitectureRegistry::instance().registerFactory([]() {
            return std::make_unique<T>();
        });
    }
};

#endif
