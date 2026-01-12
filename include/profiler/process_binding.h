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

#ifndef PROCESS_BINDING_H
#define PROCESS_BINDING_H

#include <vector>
#include <set>
#include <string>

struct ProcessBinding {
    std::set<int> allowed_cpus;
    std::set<int> allowed_mem_nodes;
    std::set<int> inferred_local_nodes;
    std::set<int> inferred_remote_nodes;
    
    bool is_cpu_bound = false;
    bool is_mem_bound = false;
    bool has_remote_access = false;
    
    std::string binding_source;
    
    std::string describe() const;
    std::vector<int> get_target_mem_nodes() const;
    int get_representative_cpu() const;
};

class ProcessBindingDetector {
public:
    ProcessBindingDetector();
    
    ProcessBinding detect();
    ProcessBinding detect_for_pid(pid_t pid);
    
    static std::set<int> get_cpu_affinity_mask();
    static std::set<int> get_cpu_affinity_mask_for_pid(pid_t pid);
    
    static std::set<int> get_membind_nodes();
    static std::set<int> get_membind_nodes_for_pid(pid_t pid);
    
    static std::set<int> cpus_to_numa_nodes(const std::set<int>& cpus);
    static std::set<int> get_all_system_nodes();
    
    static bool is_remote_access(int cpu, int mem_node);
    
private:
    static std::set<int> parse_cpu_list(const std::string& list_str);
    static std::set<int> parse_node_mask(const std::string& mask_str);
};

std::string format_cpu_set(const std::set<int>& cpus);
std::string format_node_set(const std::set<int>& nodes);

#endif
