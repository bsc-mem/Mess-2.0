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

#ifndef CODEGEN_H
#define CODEGEN_H

#include "KernelTypes.h"
#include "SystemDetection.h"
#include "SystemInfo.h"

#include <string>
#include <vector>
#include <memory>
#include <functional>
#include <sstream>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <iostream>
#include <map>
#include <libgen.h>
#include <stdexcept>
#include <limits.h>
#include <unistd.h>

/**
 * @file Codegen.h
 * @brief Helpers for generating, materializing, and building traffic-generator kernels.
 */

namespace codegen {

/**
 * @brief Wraps filesystem and shell operations used by kernel code generation.
 */
class BuildManager {
public:
    /** @brief Creates a directory tree if it does not already exist. */
    void EnsureDirectory(const std::string& path) const {
        std::filesystem::create_directories(path);
    }

    /** @brief Creates multiple directory trees in sequence. */
    void EnsureDirectories(std::initializer_list<std::string> dirs) const {
        for (const auto& dir : dirs) {
            EnsureDirectory(dir);
        }
    }

    /**
     * @brief Writes text content to a file.
     * @return `true` on success.
     */
    bool WriteFile(const std::string& path, const std::string& content) const {
        std::ofstream file(path);
        if (!file.is_open()) {
            std::cerr << "ERROR: Failed to write file " << path << std::endl;
            return false;
        }
        file << content;
        return true;
    }

    /**
     * @brief Executes a shell command needed for code generation or builds.
     * @param cmd Command string to execute.
     * @param verbose When true, echoes the command before running it.
     * @return `true` when the command exits successfully.
     */
    bool RunCommand(const std::string& cmd, bool verbose = false) const {
        if (verbose) {
            std::cout << "      - " << cmd << std::endl;
        }
        int ret = system(cmd.c_str());
        if (ret != 0) {
            std::cerr << "ERROR: Command failed (" << cmd << ")" << std::endl;
            return false;
        }
        return true;
    }

    /**
     * @brief Copies a file while overwriting any previous destination.
     * @return `true` on success.
     */
    bool CopyFile(const std::string& from, const std::string& to) const {
        std::error_code ec;
        std::filesystem::copy_file(from, to,
                                   std::filesystem::copy_options::overwrite_existing,
                                   ec);
        if (ec) {
            std::cerr << "ERROR: Failed to copy " << from << " to " << to << ": "
                      << ec.message() << std::endl;
            return false;
        }
        return true;
    }
};

/**
 * @brief Loads code-generation templates and substitutes placeholder values.
 */
class TemplateEngine {
public:
    /**
     * @brief Reads a template file from the given path or from the installed template directory.
     * @throws std::runtime_error When the template cannot be located.
     */
    static std::string Read(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            char exe_path[PATH_MAX];
            ssize_t len = readlink("/proc/self/exe", exe_path, sizeof(exe_path) - 1);
            if (len != -1) {
                exe_path[len] = '\0';
                std::string exe_dir = std::string(dirname(exe_path));
                std::string template_path = exe_dir + "/../templates/" +
                                            filename.substr(filename.find_last_of('/') + 1);
                file.open(template_path);
            }
            if (!file.is_open()) {
                throw std::runtime_error("Cannot open template file: " + filename);
            }
        }

        std::stringstream buffer;
        buffer << file.rdbuf();
        return buffer.str();
    }

    /**
     * @brief Replaces `{{key}}` placeholders in a template body.
     * @param template_content Template body to expand.
     * @param replacements Mapping from placeholder keys to replacement values.
     * @return Expanded template content.
     */
    static std::string Replace(const std::string& template_content,
                               const std::map<std::string, std::string>& replacements) {
        std::string result = template_content;
        for (const auto& [key, value] : replacements) {
            std::string placeholder = "{{" + key + "}}";
            size_t pos = 0;
            while ((pos = result.find(placeholder, pos)) != std::string::npos) {
                result.replace(pos, placeholder.length(), value);
                pos += value.length();
            }
        }
        return result;
    }
};

}

class KernelAssembler; 
class Architecture;   

/**
 * @brief Selects the active architecture plugin and emits traffic-generator source files.
 */
class KernelGenerator {
private:
    KernelConfig kernel_config_;
    CPUCapabilities capabilities_;
    
    std::shared_ptr<Architecture> architecture_;
    std::unique_ptr<KernelAssembler> assembler_;

    KernelAssembler* get_assembler();

public:
    /**
     * @brief Builds a generator for the given kernel configuration and host capabilities.
     */
    KernelGenerator(const KernelConfig& config, const CPUCapabilities& capabilities);
    ~KernelGenerator();

    /**
     * @brief Resolves architecture-specific template and utility paths.
     * @return Resolved architecture code-generation configuration.
     */
    ArchitectureConfig setup_architecture_config();
    
    /**
     * @brief Generates the requested kernel sources into an output directory.
     * @param output_dir Destination directory for generated artifacts.
     */
    void generate_kernel(const std::string& output_dir);

    /** @brief Generates one multi-sequential kernel function for a requested ratio. */
    std::string generate_multiseq_function(int ratio);
    /** @brief Generates the full multi-sequential kernel dispatch body. */
    std::string generate_all_multiseq_functions();
    
    
};

#endif
