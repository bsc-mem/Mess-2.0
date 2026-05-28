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

#include "arch/arm/ArmISAUtils.h"

#include <algorithm>
#include <sstream>

#include "arch/arm/ArmSVEVLProbe.h"

namespace arm_isa_utils {

bool isSVEFamilyMode(ISAMode mode) {
    return mode == ISAMode::SVE ||
           mode == ISAMode::SVE_MAX ||
           mode == ISAMode::SVE128 ||
           mode == ISAMode::SVE256 ||
           mode == ISAMode::SVE512;
}

bool isNeonPairMode(ISAMode mode) {
    return mode == ISAMode::NEON_PAIR;
}

ISAMode canonicalizePublicMode(ISAMode mode) {
    return (mode == ISAMode::SVE_MAX) ? ISAMode::SVE : mode;
}

int requestedSVEBits(ISAMode mode) {
    switch (canonicalizePublicMode(mode)) {
        case ISAMode::SVE128: return 128;
        case ISAMode::SVE256: return 256;
        case ISAMode::SVE512: return 512;
        default: return 0;
    }
}

std::string sveModeName(ISAMode mode) {
    switch (canonicalizePublicMode(mode)) {
        case ISAMode::SVE: return "SVE";
        case ISAMode::SVE128: return "SVE128";
        case ISAMode::SVE256: return "SVE256";
        case ISAMode::SVE512: return "SVE512";
        default: return "SVE";
    }
}

bool requiresTrueVLValidation(ISAMode mode) {
    return requestedSVEBits(mode) > 0;
}

std::string validateTrueSVEMode(ISAMode mode, const CPUCapabilities& caps) {
    const int requested_bits = requestedSVEBits(mode);
    if (requested_bits <= 0) {
        return "";
    }

    const bool has_sve = std::find(caps.extensions.begin(), caps.extensions.end(), ISAExtension::SVE) != caps.extensions.end();
    if (!has_sve) {
        return formatUnsupportedTrueSVEMessage(mode, caps, "SVE extension is not available");
    }

    const arm_sve_probe::SVEVLProbeResult probe = arm_sve_probe::probeTrueVLRequest(requested_bits);
    if (!probe.supported) {
        return formatUnsupportedTrueSVEMessage(mode, caps, probe.detail);
    }

    return "";
}

std::string formatUnsupportedTrueSVEMessage(ISAMode mode,
                                            const CPUCapabilities& caps,
                                            const std::string& detail) {
    std::ostringstream oss;
    oss << "Requested true-VL mode '" << sveModeName(mode) << "' is not supported on this machine";
    if (!detail.empty()) {
        oss << " (" << detail << ")";
    } else if (caps.sve_vector_bits > 0) {
        oss << " (current SVE VL: " << caps.sve_vector_bits << " bits)";
    }
    return oss.str();
}

std::string compileFlagsForMode(ISAMode mode) {
    switch (canonicalizePublicMode(mode)) {
        case ISAMode::SVE:
        case ISAMode::SVE128:
        case ISAMode::SVE256:
        case ISAMode::SVE512:
            return "-march=armv8-a+sve";
        default:
            return "-march=native";
    }
}

}  // namespace arm_isa_utils
