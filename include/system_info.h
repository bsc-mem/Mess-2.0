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

#define SI_MAX_SOCKETS 8
#define SI_MAX_NUMA_NODES 8
#define SI_MAX_CACHE_LEVELS 4

enum class si_cache_type {
    UNIFIED,
    INSTRUCTION,
    DATA,
    TRACE
};

struct si_cache {
    si_cache_type type;
    int level;
    long long size_bytes;
    int line_size_bytes;
    int associativity;
};

struct si_socket {
    int id;
    int core_count;
    int thread_count;

    int cpu_base_mhz;
    int cpu_max_mhz;
    int cpu_cur_mhz;

    int cache_count;
    si_cache caches[SI_MAX_CACHE_LEVELS];

    long long mem_total_bytes;
    int mem_channels;
    int mem_dimm_count;
    int mem_speed_mts;
};

struct si_numa_node {
    int id;
    long long mem_total_bytes;
};

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

    int socket_count;
    si_socket sockets[SI_MAX_SOCKETS];

    int numa_node_count;
    si_numa_node numa_nodes[SI_MAX_NUMA_NODES];
};

int  system_info_detect(system_info *out);
void system_info_print(const system_info *info, int verbosity);

#endif
