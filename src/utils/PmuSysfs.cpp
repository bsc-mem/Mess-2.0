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

#include "utils/PmuSysfs.h"

#include <algorithm>
#include <cctype>
#include <fstream>

#if defined(__linux__)
#include <dirent.h>
#include <unistd.h>
#endif

namespace pmu_sysfs {

namespace {

constexpr const char* kRoot = "/sys/bus/event_source/devices";

#if defined(__linux__)

bool for_each_entry(const std::string& dir_path,
                    const std::function<void(const std::string&)>& visit) {
    DIR* dir = opendir(dir_path.c_str());
    if (!dir) return false;
    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        const char* name = entry->d_name;
        if (name[0] == '.' && (name[1] == '\0' || (name[1] == '.' && name[2] == '\0'))) {
            continue;
        }
        visit(name);
    }
    closedir(dir);
    return true;
}

bool has_only_digits(const std::string& s) {
    if (s.empty()) return false;
    for (char c : s) {
        if (!std::isdigit(static_cast<unsigned char>(c))) return false;
    }
    return true;
}
#endif 

}

const char* root() {
    return kRoot;
}

std::string path(const std::string& pmu_name) {
    return std::string(kRoot) + "/" + pmu_name;
}

bool exists(const std::string& pmu_name) {
#if defined(__linux__)
    const std::string type_path = path(pmu_name) + "/type";
    return access(type_path.c_str(), F_OK) == 0;
#else
    (void)pmu_name;
    return false;
#endif
}

std::vector<std::string> list(std::function<bool(const std::string&)> filter) {
    std::vector<std::string> names;
#if defined(__linux__)
    for_each_entry(kRoot, [&](const std::string& name) {
        if (!filter || filter(name)) names.push_back(name);
    });
    std::sort(names.begin(), names.end());
#else
    (void)filter;
#endif
    return names;
}

std::vector<std::string> list_prefixed(const std::string& prefix) {
    std::vector<std::string> names;
#if defined(__linux__)
    const std::string needle = prefix + "_";
    for_each_entry(kRoot, [&](const std::string& name) {
        if (name.rfind(needle, 0) != 0) return;
        if (!has_only_digits(name.substr(needle.size()))) return;
        names.push_back(name);
    });
    std::sort(names.begin(), names.end());
#else
    (void)prefix;
#endif
    return names;
}

std::vector<std::string> enumerate_indexed(const std::string& prefix, int max_index) {
    std::vector<std::string> names;
#if defined(__linux__)
    for (int i = 0; i < max_index; ++i) {
        std::string candidate = prefix + "_" + std::to_string(i);
        if (exists(candidate)) {
            names.push_back(std::move(candidate));
        } else if (!names.empty()) {
            break;
        }

    }
#else
    (void)prefix;
    (void)max_index;
#endif
    return names;
}

bool read_type(const std::string& pmu_name, uint32_t& out) {
    std::ifstream f(path(pmu_name) + "/type");
    if (!f.is_open()) return false;
    f >> out;
    return f.good() || f.eof();
}

bool read_cpumask(const std::string& pmu_name, int& first_cpu) {
    std::ifstream f(path(pmu_name) + "/cpumask");
    if (!f.is_open()) return false;

    std::string line;
    if (!std::getline(f, line)) return false;

    const size_t pos = line.find_first_of(",-");
    const std::string head = (pos != std::string::npos) ? line.substr(0, pos) : line;

    try {
        first_cpu = std::stoi(head);
        return true;
    } catch (...) {
        return false;
    }
}

std::vector<std::string> list_events(const std::string& pmu_name,
                                     std::function<bool(const std::string&)> filter) {
    std::vector<std::string> events;
#if defined(__linux__)
    const std::string events_dir = path(pmu_name) + "/events";
    for_each_entry(events_dir, [&](const std::string& event_name) {
        if (filter && !filter(event_name)) return;
        events.push_back(pmu_name + "/" + event_name + "/");
    });
#else
    (void)pmu_name;
    (void)filter;
#endif
    return events;
}

}
