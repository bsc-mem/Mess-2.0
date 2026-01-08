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

#include "measurement.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <system_error>
#include <unistd.h>
#include <fcntl.h>
#include <sys/file.h>

#include "utils.h"

namespace fs = std::filesystem;

MeasurementStorage::MeasurementStorage(const BenchmarkConfig& config, int cache_line_size, int verbosity, const std::string& output_root_override)
    : config_(config),
      cache_line_size_(cache_line_size),
      verbosity_(verbosity),
      output_root_override_(output_root_override),
      measurement_root_is_temp_(!config.profile_output),
      directories_created_(false) {
    initialize_storage();
}

MeasurementStorage::~MeasurementStorage() {
    if (measurement_root_is_temp_ && !measurement_root_.empty()) {
        std::error_code ec;
        fs::remove_all(measurement_root_, ec);
        if (ec && verbosity_ >= 2) {
            std::cerr << "Warning: Failed to remove temporary measurement directory '"
                      << measurement_root_ << "': " << ec.message() << std::endl;
        }
    }
}

void MeasurementStorage::ensure_directories_exist() const {
    if (directories_created_) return;

    fs::create_directories(measurement_logs_dir_);
    fs::create_directories(measurement_bw_dir_);
    fs::create_directories(measurement_lat_dir_);

    directories_created_ = true;
}

void MeasurementStorage::initialize_storage() {
    auto assign_paths = [this](const std::string& root) {
        measurement_root_ = root;
        measurement_logs_dir_ = measurement_root_ + "/logs";

        measurement_bw_dir_ = measurement_root_ + "/bw";
        measurement_lat_dir_ = measurement_root_ + "/lat";



    };

    if (measurement_root_is_temp_) {
        assign_paths(create_temp_measurement_root());
    } else {
        if (!output_root_override_.empty()) {
            assign_paths(output_root_override_);
        } else {
            assign_paths(config_.output_root + "/multisequential");
        }
    }

    try {
        ensure_directories_exist();

    } catch (const fs::filesystem_error& ex) {
        if (measurement_root_is_temp_) {
            throw std::runtime_error(
                std::string("Failed to create temporary measurement directory at '") +
                measurement_root_ + "': " + ex.what());
        }
        throw;
    }

    if (measurement_root_is_temp_ && verbosity_ >= 2) {
        std::cout << "  Profile output disabled: staging measurement artifacts under "
                  << measurement_root_ << std::endl;
    }
}

std::string MeasurementStorage::create_temp_measurement_root() const {
    fs::path temp_base;
    try {
        temp_base = fs::temp_directory_path();
    } catch (const fs::filesystem_error&) {
        temp_base = "/tmp";
    }

    auto timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch())
                         .count();
    std::ostringstream oss;
    oss << "mess_measurements_" << getpid() << "_" << timestamp;
    return (temp_base / oss.str()).string();
}

std::string MeasurementStorage::bw_file_path(double ratio_pct, int pause) const {
    return measurement_bw_dir_ + "/bw_" + std::to_string(static_cast<int>(ratio_pct)) +
           "_" + std::to_string(pause) + ".txt";
}

std::string MeasurementStorage::lat_file_path(double ratio_pct, int pause) const {
    return measurement_lat_dir_ + "/lat_" + std::to_string(static_cast<int>(ratio_pct)) +
           "_" + std::to_string(pause) + ".txt";
}

std::string MeasurementStorage::traffic_gen_log_file_path(double ratio_pct, int pause) const {
    return measurement_logs_dir_ + "/traffic_gen_log_" +
           std::to_string(static_cast<int>(ratio_pct)) + "_" +
           std::to_string(pause) + ".txt";
}

void MeasurementStorage::sync_plotter_file() const {
    const fs::path source_plotter = fs::path(measurement_root_) / "plotter.txt";

    if (!fs::exists(source_plotter)) {
        if (verbosity_ >= 3) {
            std::cerr << "Warning: plotter.txt not found at '" << source_plotter.string()
                      << "'" << std::endl;
        }
        return;
    }

    if (fs::path(source_plotter).parent_path() == fs::path(measurement_root_)) {
        return;
    }

    copy_plotter_to(source_plotter.string(), measurement_root_);
}

void MeasurementStorage::cleanup_measurement_files(double ratio_pct, int pause) const {

    std::remove(bw_file_path(ratio_pct, pause).c_str());
    std::remove(lat_file_path(ratio_pct, pause).c_str());

    std::remove(traffic_gen_log_file_path(ratio_pct, pause).c_str());
}

std::vector<double> MeasurementStorage::parse_bw_measurements(
    const std::string& filepath, double scaling_factor) const {
    std::vector<double> bw_values;
    std::ifstream file(filepath);
    if (!file.is_open()) {
        return bw_values;
    }

    auto is_bw_read_counter = [](const std::string& line) -> bool {
        bool has_read_pattern = line.find(".rd") != std::string::npos ||
                               line.find("_rxl") != std::string::npos ||
                               line.find("cmem_rd") != std::string::npos ||
                               line.find("remote_socket_rd_data") != std::string::npos ||
                               line.find("rd_bytes") != std::string::npos ||
                               line.find("dram_channel_data_controller") != std::string::npos;
        if (!has_read_pattern && line.find("read") != std::string::npos) {
            has_read_pattern = line.find("cas_count") != std::string::npos ||
                              line.find("mba") != std::string::npos;
        }
        return has_read_pattern;
    };

    auto is_bw_write_counter = [](const std::string& line) -> bool {
        bool has_write_pattern = line.find(".wr") != std::string::npos ||
                                line.find("_txl") != std::string::npos ||
                                line.find("cmem_wr") != std::string::npos ||
                                line.find("remote_socket_wr_total_bytes") != std::string::npos ||
                                line.find("wr_bytes") != std::string::npos;
        if (!has_write_pattern && line.find("write") != std::string::npos) {
            has_write_pattern = line.find("cas_count") != std::string::npos ||
                               line.find("mba") != std::string::npos;
        }
        return has_write_pattern;
    };

    std::string line;
    long long rd_count = 0, wr_count = 0;
    double elapsed = 0;
    bool has_rd = false, has_wr = false, has_time = false;
    bool is_grace_rd = false, is_grace_wr = false;

    while (std::getline(file, line)) {
        if (line.find("S0") != std::string::npos && is_bw_read_counter(line)) {
            std::istringstream iss(line);
            std::string socket, column, count_str;
            iss >> socket >> column >> count_str;
            try {
                rd_count = std::stoll(count_str);
                has_rd = true;
                if (line.find("cmem_rd") != std::string::npos) is_grace_rd = true;
                if (line.find("remote_socket_rd_data") != std::string::npos) is_grace_rd = true;
            } catch (...) {}
        } 
        else if (line.find("S0") != std::string::npos && is_bw_write_counter(line)) {
            std::istringstream iss(line);
            std::string socket, column, count_str;
            iss >> socket >> column >> count_str;
            try {
                wr_count = std::stoll(count_str);
                has_wr = true;
                if (line.find("cmem_wr") != std::string::npos) is_grace_wr = true;
                if (line.find("remote_socket_wr_total_bytes") != std::string::npos) is_grace_wr = true;
            } catch (...) {}
        } 
        else if (line.find("seconds time elapsed") != std::string::npos) {
            std::istringstream iss(line);
            iss >> elapsed;
            has_time = true;
        }

        if (has_rd && has_wr && has_time) {
            double bw_gb_s = 0.0;
            if (is_grace_rd || is_grace_wr) {
                bw_gb_s = (static_cast<double>(rd_count) * 32.0 + static_cast<double>(wr_count)) / (elapsed * 1e9);
            } else {
                bw_gb_s = ((rd_count + wr_count) * cache_line_size_ * scaling_factor) / (elapsed * 1e9);
            }
            bw_values.push_back(bw_gb_s);
            rd_count = wr_count = 0;
            elapsed = 0;
            has_rd = has_wr = has_time = false;
            is_grace_rd = is_grace_wr = false;
        }
    }

    file.close();
    return bw_values;
}

std::vector<double> MeasurementStorage::parse_lat_measurements(
    const std::string& filepath,
    double cached_tlb_hit_latency_ns) const {
    std::vector<double> lat_values;
    std::ifstream file(filepath);
    if (!file.is_open()) {
        return lat_values;
    }


    std::string line;
    double cycles = 0, instructions = 0, accesses = 0, tlb1miss = 0, tlb2miss = 0,
           seconds_user = 0;
    bool has_cycles = false, has_instr = false, has_accesses = false,
         has_tlb1 = false, has_tlb2 = false, has_time = false;
    double max_cpu_freq_ghz = 0.0;

    while (std::getline(file, line)) {
        if (line.find("cycles") != std::string::npos) {
            std::istringstream iss(line);
            iss >> cycles;
            has_cycles = true;
        } else if (line.find("instructions") != std::string::npos) {
            std::istringstream iss(line);
            iss >> instructions;
            has_instr = true;
        } else if (line.find("accesses") != std::string::npos) {
            std::istringstream iss(line);
            iss >> accesses;
            has_accesses = true;
        } else if (line.find("seconds time elapsed") != std::string::npos) {
            std::istringstream iss(line);
            iss >> seconds_user;
            has_time = true;
        } else {
            std::istringstream iss(line);
            double value;
            if (iss >> value) {
                if (value >= 0) {
                     if (!has_tlb1) {
                        tlb1miss = value;
                        has_tlb1 = true;
                    } else if (!has_tlb2) {
                        tlb2miss = value;
                        has_tlb2 = true;
                    }
                }
            }
        }

        if (has_cycles && has_accesses && has_tlb1 && has_tlb2 && has_time &&
            seconds_user > 0 && accesses > 0) {
            double cpu_freq_hz = cycles / seconds_user;
            double cpu_freq_ghz = cpu_freq_hz / 1e9;
            max_cpu_freq_ghz = std::max(max_cpu_freq_ghz, cpu_freq_ghz);

            double stlb_hit_latency_ns = cached_tlb_hit_latency_ns;
            double stlb_hit_latency = stlb_hit_latency_ns * cpu_freq_ghz;

            double tlb2_penalty_cycles = tlb2miss;
            double tlb1_penalty_cycles = stlb_hit_latency * tlb1miss;
            double effective_cycles = cycles - tlb2_penalty_cycles - tlb1_penalty_cycles;

            double cycles_per_access = effective_cycles / accesses;
            double latency_ns = cycles_per_access / cpu_freq_ghz;

            lat_values.push_back(latency_ns);

            cycles = instructions = accesses = tlb1miss = tlb2miss = 0;
            seconds_user = 0;
            has_cycles = has_instr = has_accesses = has_tlb1 = has_tlb2 = has_time =
                false;
        }
    }

    file.close();


    return lat_values;
}

bool MeasurementStorage::append_to_file_with_lock(const std::string& filepath, const std::string& content) const {
    int fd = open(filepath.c_str(), O_WRONLY | O_CREAT | O_APPEND, 0644);
    if (fd == -1) {
        std::cerr << "Error opening file for locking: " << filepath << std::endl;
        return false;
    }

    struct flock fl;
    fl.l_type = F_WRLCK;  
    fl.l_whence = SEEK_SET; 
    fl.l_start = 0;
    fl.l_len = 0;        

    if (fcntl(fd, F_SETLKW, &fl) == -1) {
        std::cerr << "Error acquiring lock for file: " << filepath << std::endl;
        close(fd);
        return false;
    }

    ssize_t bytes_written = write(fd, content.c_str(), content.length());
    if (bytes_written == -1) {
        std::cerr << "Error writing to file: " << filepath << std::endl;
    }

    fl.l_type = F_UNLCK;
    if (fcntl(fd, F_SETLK, &fl) == -1) {
        std::cerr << "Error releasing lock for file: " << filepath << std::endl;
    }

    close(fd);
    return (bytes_written != -1);
}

void MeasurementStorage::copy_plotter_to(const std::string& source_plotter,
                                         const std::string& destination_dir) const {
    if (destination_dir.empty()) {
        return;
    }

    try {
        fs::create_directories(destination_dir);
        fs::path destination_file = fs::path(destination_dir) / "plotter.txt";
        fs::copy_file(source_plotter, destination_file,
                      fs::copy_options::overwrite_existing);
    } catch (const fs::filesystem_error& ex) {
        if (verbosity_ >= 2) {
            std::cerr << "Warning: Failed to copy plotter.txt to '"
                      << destination_dir << "': " << ex.what() << std::endl;
        }
    }
}


void MeasurementStorage::write_plotter_file(const CPUCapabilities& caps, const std::string& binding_type, double tlb_ns, int cache_line_size, double upi_scaling_factor) const {
    if (measurement_root_is_temp_) {
        return;
    }
    
    std::filesystem::path plotter_path = std::filesystem::path(measurement_root_) / "plotter.txt";
    append_memory_config_to_plotter(plotter_path, caps, binding_type, tlb_ns, cache_line_size, upi_scaling_factor);
}
