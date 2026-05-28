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

#ifndef SYSTEM_INFO_H
#define SYSTEM_INFO_H

#include <cstddef>
#include <cstdint>
#include <string>

/**
 * @file SystemInfo.h
 * @brief Low-level C-style topology and memory information structures.
 */

#define SI_MAX_SOCKETS 8
#define SI_MAX_NUMA_NODES 32
#define SI_MAX_CACHE_LEVELS 4

/** @brief Cache types reported by @ref system_info_detect. */
enum class si_cache_type {
    UNIFIED,
    INSTRUCTION,
    DATA,
    TRACE
};

/** @brief Describes one cache level discovered for a socket. */
struct si_cache {
    si_cache_type type;
    int level;
    long long size_bytes;
    int line_size_bytes;
    int associativity;
};

/** @brief Describes one physical socket. */
struct si_socket {
    int id;
    int core_count;
    int thread_count;

    int cache_count;
    si_cache caches[SI_MAX_CACHE_LEVELS];

    long long mem_total_bytes;
    int mem_channels;
    int mem_dimm_count;
    int mem_speed_mts;
};

/** @brief Describes one NUMA node. */
struct si_numa_node {
    int id;
    long long mem_total_bytes;
    /** @brief Detected memory technology for this node (e.g. "DDR5", "HBM2E", "CXL", "PMEM", "UNKNOWN"). */
    char memory_type[16];
    /** @brief Whether at least one CPU is attached to this node. */
    int has_cpus;
    /** @brief Whether this node is backed by a DAX device (HBM in flat mode, CXL, PMEM). */
    int is_dax_backed;
    /** @brief HMAT read bandwidth in MB/s (0 if unavailable). */
    long long read_bandwidth_mb_s;
    /** @brief HMAT read latency in nanoseconds (0 if unavailable). */
    long long read_latency_ns;
};

/** @brief Aggregated system information used as the raw input to capability normalization. */
struct system_info {
    char os_name[64];
    char os_release[64];
    char os_version[128];
    char arch[64];
    char cpu_vendor[32];
    char cpu_model[128];

    int total_logical_cores;
    int total_physical_cores;

    long page_size;
    long long total_mem_bytes;
    char mem_technology[64];
    char mem_frequency[32];

    int cpu_base_mhz;
    int cpu_max_mhz;
    int cpu_cur_mhz;

    int socket_count;
    si_socket sockets[SI_MAX_SOCKETS];

    int numa_node_count;
    si_numa_node numa_nodes[SI_MAX_NUMA_NODES];
};

/**
 * @brief Fills a @ref system_info structure with detected host information.
 * @param out Destination structure.
 * @return `0` on success, non-zero on failure.
 */
int  system_info_detect(system_info *out);

bool cpu_has_onpackage_hbm(const std::string& cpu_model);

#endif
