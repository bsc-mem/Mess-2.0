#ifndef ARM_ISA_UTILS_H
#define ARM_ISA_UTILS_H

#include <string>

#include "SystemDetection.h"
#include "architecture/ISA.h"

namespace arm_isa_utils {

bool isSVEFamilyMode(ISAMode mode);
bool isNeonPairMode(ISAMode mode);
ISAMode canonicalizePublicMode(ISAMode mode);
int requestedSVEBits(ISAMode mode);
std::string sveModeName(ISAMode mode);
bool requiresTrueVLValidation(ISAMode mode);
std::string validateTrueSVEMode(ISAMode mode, const CPUCapabilities& caps);
std::string formatUnsupportedTrueSVEMessage(ISAMode mode,
                                            const CPUCapabilities& caps,
                                            const std::string& detail);
std::string compileFlagsForMode(ISAMode mode);

}

#endif
