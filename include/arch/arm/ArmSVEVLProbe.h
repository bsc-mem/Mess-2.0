#ifndef ARM_SVE_VL_PROBE_H
#define ARM_SVE_VL_PROBE_H

#include <cerrno>
#include <cstring>
#include <sstream>
#include <string>

#if defined(__linux__) && defined(__aarch64__)
#include <sys/prctl.h>
#include <linux/prctl.h>
#include <asm/sigcontext.h>
#if defined(__ARM_FEATURE_SVE)
#include <arm_sve.h>
#endif
#endif

namespace arm_sve_probe {

struct SVEVLProbeResult {
    bool supported = false;
    int active_bits = 0;
    int prctl_bits = 0;
    int instruction_bits = 0;
    int restored_bits = 0;
    bool restore_verified = false;
    std::string detail;
};

inline int currentVLBytesFromInstruction() {
#if defined(__linux__) && defined(__aarch64__) && defined(__ARM_FEATURE_SVE)
    return static_cast<int>(svcntb());
#else
    return -1;
#endif
}

inline int currentVLBytesFromPrctl(std::string* detail = nullptr) {
#if defined(__linux__) && defined(__aarch64__) && defined(PR_SVE_GET_VL) && defined(PR_SVE_VL_LEN_MASK)
    errno = 0;
    const long current = prctl(PR_SVE_GET_VL);
    const int get_errno = errno;
    if (current < 0) {
        if (detail) {
            std::ostringstream oss;
            oss << "PR_SVE_GET_VL failed";
            if (get_errno != 0) {
                oss << ": " << std::strerror(get_errno);
            }
            *detail = oss.str();
        }
        return -1;
    }
    return static_cast<int>(current & PR_SVE_VL_LEN_MASK);
#else
    if (detail) {
        *detail = "PR_SVE_GET_VL is not available on this build";
    }
    return -1;
#endif
}

inline SVEVLProbeResult probeTrueVLRequest(int requested_bits) {
    SVEVLProbeResult result;

#if defined(__linux__) && defined(__aarch64__) && defined(PR_SVE_GET_VL) && defined(PR_SVE_SET_VL) && defined(PR_SVE_VL_LEN_MASK)
    if (requested_bits <= 0 || (requested_bits % 128) != 0) {
        result.detail = "Requested SVE VL must be a positive multiple of 128 bits";
        return result;
    }

    std::string detail;
    const int original_bytes = currentVLBytesFromPrctl(&detail);
    if (original_bytes <= 0) {
        result.detail = detail;
        return result;
    }

    const unsigned long requested_bytes = static_cast<unsigned long>(requested_bits / 8);
    errno = 0;
    const long configured = prctl(PR_SVE_SET_VL, requested_bytes);
    const int set_errno = errno;
    if (configured < 0) {
        std::ostringstream oss;
        oss << "PR_SVE_SET_VL(" << requested_bits << ") failed";
        if (set_errno != 0) {
            oss << ": " << std::strerror(set_errno);
        }
        result.detail = oss.str();
        return result;
    }

    const int current_bytes = currentVLBytesFromPrctl(&detail);
    if (current_bytes <= 0) {
        result.detail = detail;
        if (original_bytes > 0) {
            (void)prctl(PR_SVE_SET_VL, static_cast<unsigned long>(original_bytes));
        }
        return result;
    }

    const int instruction_bytes = currentVLBytesFromInstruction();
    result.prctl_bits = current_bytes * 8;
    result.instruction_bits = (instruction_bytes > 0) ? (instruction_bytes * 8) : 0;
    result.active_bits = result.prctl_bits;

    bool restore_ok = false;
    if (original_bytes > 0) {
        errno = 0;
        const long restored = prctl(PR_SVE_SET_VL, static_cast<unsigned long>(original_bytes));
        const int restore_errno = errno;
        if (restored < 0) {
            std::ostringstream oss;
            oss << "Failed to restore original SVE VL";
            if (restore_errno != 0) {
                oss << ": " << std::strerror(restore_errno);
            }
            result.detail = oss.str();
            return result;
        }

        int restored_bytes = currentVLBytesFromPrctl(&detail);
        if (restored_bytes <= 0) {
            result.detail = detail;
            return result;
        }

        const int restored_instruction_bytes = currentVLBytesFromInstruction();
        if (restored_instruction_bytes > 0 && restored_instruction_bytes != restored_bytes) {
            std::ostringstream oss;
            oss << "SVE restore verification mismatch: PR_SVE_GET_VL reports "
                << (restored_bytes * 8) << " bits but instructions report "
                << (restored_instruction_bytes * 8) << " bits";
            result.detail = oss.str();
            return result;
        }

        result.restored_bits = restored_bytes * 8;
        restore_ok = (restored_bytes == original_bytes);
        result.restore_verified = restore_ok;
        if (!restore_ok) {
            std::ostringstream oss;
            oss << "Failed to restore original SVE VL: expected " << (original_bytes * 8)
                << " bits, got " << (restored_bytes * 8) << " bits";
            result.detail = oss.str();
            return result;
        }
    }

    if (instruction_bytes > 0 && instruction_bytes != current_bytes) {
        std::ostringstream oss;
        oss << "SVE VL validation mismatch: PR_SVE_GET_VL reports "
            << (current_bytes * 8) << " bits but instructions report "
            << (instruction_bytes * 8) << " bits";
        result.detail = oss.str();
        return result;
    }

    if (current_bytes != static_cast<int>(requested_bytes)) {
        std::ostringstream oss;
        oss << "kernel granted " << (current_bytes * 8) << " bits after requesting " << requested_bits;
        if (instruction_bytes > 0) {
            oss << "; instruction-visible VL is " << (instruction_bytes * 8) << " bits";
        }
        result.detail = oss.str();
        return result;
    }

    result.supported = true;
    return result;
#else
    (void)requested_bits;
    result.detail = "PR_SVE_SET_VL/PR_SVE_GET_VL are not available on this build";
    return result;
#endif
}

}

#endif
