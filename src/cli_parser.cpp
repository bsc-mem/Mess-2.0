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

#include "cli_parser.h"
#include <iostream>
#include <sstream>
#include <algorithm>
#include <cstring>
#include <getopt.h>
#include <unistd.h>
#include <cstdlib>
#include <iomanip>
#include <cmath>

void CLIParser::display_configuration(const BenchmarkConfig& config, bool perf_accessible, int paranoid_level, bool tlb_ok, double tlb_latency_ns) const {
    if (config.verbosity < 1) {
        return;
    }

    std::cout << "\n=== Configuration ===" << std::endl;
    std::cout << "Performance counter access: " << (perf_accessible ? "OK" : "KO") 
              << " (paranoid: " << paranoid_level << ")" << std::endl;
    
    if (config.verbosity >= 2) {
        if (tlb_ok) {
            std::cout << "TLB hit latency: " << tlb_latency_ns << " ns" << std::endl;
        } else {
            std::cout << "! TLB measurement failed, using default values" << std::endl;
        }
    }
    
    if (config.verbosity >= 2) {
        std::cout << "\n=== Settings used ===" << std::endl;
        std::cout << "Mode: " << "Custom configuration" << std::endl;
        
        std::string size_str;
        switch (config.size) {
            case SizeTier::LITE: size_str = "Lite"; break;
            case SizeTier::MEDIUM: size_str = "Medium"; break;
            case SizeTier::FULL: size_str = "Full"; break;
            default: size_str = "Custom";
        }
        std::cout << "Size tier: " << size_str << std::endl;
        
        std::cout << "Read ratios (%): ";
        if (config.ratios_pct.size() > 10) {
            std::cout << config.ratios_pct.size() << " ratios (" 
                     << config.ratios_pct.front() << "% to " 
                     << config.ratios_pct.back() << "% in steps of "
                     << (config.ratios_pct.size() > 1 ? 
                         config.ratios_pct[1] - config.ratios_pct[0] : 2) 
                     << "%)" << std::endl;
        } else {
            for (size_t i = 0; i < config.ratios_pct.size(); ++i) {
                if (i > 0) std::cout << ", ";
                std::cout << config.ratios_pct[i] << "%";
            }
            std::cout << std::endl;
        }
        
        std::cout << "Pause values: ";
        if (config.pauses.empty()) {
            std::cout << "(none)" << std::endl;
        } else {
            std::cout << config.pauses.size() << " values (";
            for (size_t i = 0; i < config.pauses.size(); ++i) {
                if (i > 0) std::cout << ", ";
                std::cout << config.pauses[i];
            }
            std::cout << ")" << std::endl;
        }
        
        std::cout << "Point repetitions: " << config.point_reps << std::endl;
        std::cout << "Profile mode: " << (config.profile_output ? "enabled" : "disabled") << std::endl;
        
        std::cout << "Verbosity level: " << config.verbosity << std::endl;
    }
    
}

CLIParser::CLIParser() {
    specs_ = {
        {"", "version", false, "", "print version and exit"},
        {"h", "help", false, "", "print this help and exit"},
        {"r", "ratio", true, "X[,X...]", "read/write ratios in percent; comma-separated list"},
        // {"s", "size", true, "small|medium|large|full", "benchmark size tier (default: full)"},
        {"", "pause", true, "X[,X...]", "pause values in microseconds; comma-separated list"},
        {"", "profile", false, "", "save bw/lat files to measuring/ directory"},

        {"", "dry-run", false, "", "detect system and print configuration without running benchmark"},
        {"", "likwid", false, "", "force use of likwid for bandwidth measurement"},
        {"", "persistent-trafficgen", false, "", "keep TrafficGen alive across repetitions for the same point"},
        {"v", "verbose", true, "N", "verbosity level (1-3, default: 1)"},
        {"", "repetitions", true, "N", "number of point repetitions (default: 3)"},
        {"", "total-cores", true, "N", "number of cores to run TrafficGen workers on (default: TOTAL_CORES-1)"},
        {"", "cores", true, "LIST", "comma-separated list of specific cores to use (e.g. 0,1,2). Incompatible with --total-cores"},
        {"", "folder", true, "DIR", "root output folder (default: measuring)"},
        {"", "bind", true, "LIST", "comma-separated list of NUMA nodes to bind memory to (e.g. 0,1)"},
        {"", "extra-perf", true, "COUNTERS", "comma-separated list of extra perf counters to measure alongside BW"}
    };
}

bool CLIParser::parse(int argc, char** argv, BenchmarkConfig& config) {
    if (argc == 1) {
        config.ratios_pct.clear();
        for (int ratio = 0; ratio <= 100; ratio += 2) {
            config.ratios_pct.push_back(static_cast<double>(ratio));
        }
        
        config.pauses = {0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 80, 90, 100,
                        120, 140, 160, 180, 200, 220, 260, 300, 340, 380, 450, 550, 600, 700,
                        800, 900, 1000, 1500, 2000, 3000, 5000, 40000, 100000};
        
        config.size = SizeTier::FULL;

        config.verbosity = 1;
        
        return true;
    }
    
    bool total_cores_flag_seen = false;
    bool explicit_cores_flag_seen = false;
    
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        
        if (arg == "--version") {
            config.show_version = true;
        } else if (arg == "--help" || arg == "-h") {
            config.show_help = true;
        } else if (arg == "--ratio" || arg == "-r") {
            if (i + 1 >= argc) { std::cerr << "error: --ratio requires an argument" << std::endl; return false; }
            if (!parse_ratios(argv[++i], config.ratios_pct)) { std::cerr << "error: invalid ratio list '" << argv[i] << "'" << std::endl; return false; }
        } else if (arg.find("--ratio=") == 0) {
            std::string value = arg.substr(8);
            if (!parse_ratios(value.c_str(), config.ratios_pct)) { std::cerr << "error: invalid ratio '" << value << "'" << std::endl; return false; }
        } else if (arg == "--pause") {
            if (i + 1 >= argc) { std::cerr << "error: --pause requires an argument" << std::endl; return false; }
            if (!parse_pauses(argv[++i], config.pauses)) { std::cerr << "error: invalid pause list '" << argv[i] << "'" << std::endl; return false; }
            config.explicit_pauses = true;
        } else if (arg.find("--pause=") == 0) {
            std::string value = arg.substr(8);
            if (!parse_pauses(value.c_str(), config.pauses)) { std::cerr << "error: invalid pause '" << value << "'" << std::endl; return false; }
            config.explicit_pauses = true;
        } else if (arg == "--profile") {
            config.profile_output = true;

        } else if (arg == "--dry-run") { // Original: arg == "--dry-run"
            config.dry_run = true;
        } else if (arg == "--likwid") { // Original: arg == "--likwid"
            config.force_likwid = true;
        } else if (arg == "--persistent-trafficgen" || arg == "--persistent-traffic-gen") {
            config.persistent_traffic_gen = true;
        } else if (arg == "--bind") {
            if (i + 1 >= argc) { std::cerr << "error: --bind requires an argument" << std::endl; return false; }
            std::string value = argv[++i];
            if (!parse_bind(value, config.memory_bind_nodes)) {
                std::cerr << "ERROR: Invalid bind value: " << value << std::endl;
                return false;
            }
        } else if (arg.find("--bind=") == 0) {
            std::string value = arg.substr(7);
            if (!parse_bind(value, config.memory_bind_nodes)) {
                std::cerr << "error: invalid bind mode '" << value << "' (expected NUMA node id or list)" << std::endl;
                return false;
            }
        } else if (arg == "--verbose" || arg == "-v") {
            if (i + 1 >= argc) { std::cerr << "error: --verbose requires an argument" << std::endl; return false; }
            config.verbosity = std::stoi(argv[++i]);
            if (config.verbosity < 1 || config.verbosity > 3) { std::cerr << "error: verbosity must be between 1 and 3" << std::endl; return false; }
        } else if (arg.find("--verbose=") == 0) {
            std::string value = arg.substr(10);
            config.verbosity = std::stoi(value);
            if (config.verbosity < 0 || config.verbosity > 4) { std::cerr << "error: verbosity must be between 0 and 4" << std::endl; return false; }
        } else if (arg.find("-v") == 0 && arg.length() > 2) {
            std::string value = arg.substr(2);
            config.verbosity = std::stoi(value);
            if (config.verbosity < 0 || config.verbosity > 4) { std::cerr << "error: verbosity must be between 0 and 4" << std::endl; return false; }
        } else if (arg == "--repetitions") {
            if (i + 1 >= argc) { std::cerr << "error: --repetitions requires an argument" << std::endl; return false; }
            config.point_reps = std::stoi(argv[++i]);
            if (config.point_reps < 1) { std::cerr << "error: repetitions must be >= 1" << std::endl; return false; }
        } else if (arg.find("--repetitions=") == 0) {
            std::string value = arg.substr(14);
            config.point_reps = std::stoi(value);
            if (config.point_reps < 1) { std::cerr << "error: repetitions must be >= 1" << std::endl; return false; }
        } else if (arg == "--total-cores") {
            if (i + 1 >= argc) { std::cerr << "error: --total-cores requires an argument" << std::endl; return false; }
            int c = std::stoi(argv[++i]);
            if (c < 1 || c > 4096) { std::cerr << "error: total-cores must be between 1 and 4096" << std::endl; return false; }
            config.traffic_gen_cores = c;
            total_cores_flag_seen = true;
        } else if (arg.find("--total-cores=") == 0) {
            std::string value = arg.substr(14);
            int c = std::stoi(value);
            if (c < 1 || c > 4096) { std::cerr << "error: total-cores must be between 1 and 4096" << std::endl; return false; }
            config.traffic_gen_cores = c;
            total_cores_flag_seen = true;
        } else if (arg == "--cores") {
            if (i + 1 >= argc) { std::cerr << "error: --cores requires an argument" << std::endl; return false; }
            std::string value = argv[++i];
            std::stringstream ss(value);
            std::string segment;
            while(std::getline(ss, segment, ',')) {
                config.traffic_gen_explicit_cores.push_back(segment);
            }
            config.traffic_gen_cores = config.traffic_gen_explicit_cores.size();
            explicit_cores_flag_seen = true;
        } else if (arg.find("--cores=") == 0) {
            std::string value = arg.substr(8);
            std::stringstream ss(value);
            std::string segment;
            while(std::getline(ss, segment, ',')) {
                config.traffic_gen_explicit_cores.push_back(segment);
            }
            config.traffic_gen_cores = config.traffic_gen_explicit_cores.size();
            explicit_cores_flag_seen = true;
        } else if (arg == "--folder") {
            if (i + 1 >= argc) { std::cerr << "error: --folder requires an argument" << std::endl; return false; }
            config.output_root = argv[++i];
            if (config.output_root.empty()) { std::cerr << "error: folder must be non-empty" << std::endl; return false; }
        } else if (arg.find("--folder=") == 0) {
            std::string value = arg.substr(9);
            if (value.empty()) { std::cerr << "error: folder must be non-empty" << std::endl; return false; }
            config.output_root = value;
        } else if (arg == "--extra-perf") {
            if (i + 1 >= argc) { std::cerr << "error: --extra-perf requires an argument" << std::endl; return false; }
            std::string value = argv[++i];
            std::stringstream ss(value);
            std::string segment;
            while(std::getline(ss, segment, ',')) {
                if (!segment.empty()) {
                    config.extra_perf_counters.push_back(segment);
                }
            }
        } else if (arg.find("--extra-perf=") == 0) {
            std::string value = arg.substr(13);
            std::stringstream ss(value);
            std::string segment;
            while(std::getline(ss, segment, ',')) {
                if (!segment.empty()) {
                    config.extra_perf_counters.push_back(segment);
                }
            }
        } else {
            std::cerr << "error: unknown option '" << arg << "'" << std::endl;
            return false;
        }
    }
    
    config.generate_multiseq = true;

    if (total_cores_flag_seen && explicit_cores_flag_seen) {
        std::cerr << "ERROR: --total-cores and --cores are mutually exclusive." << std::endl;
        return false;
    }
    
    return true;
}

void CLIParser::print_help(const char* argv0) {
    print_usage(argv0);

    // Basic options (user-facing)
    std::cout << "\nBasic Options:\n";
    std::vector<std::string> basic_opts = {"version", "help", "ratio", "size", "profile", "system", "dry-run", "verbose", "folder"};
    
    for (const auto& spec : specs_) {
        if (std::find(basic_opts.begin(), basic_opts.end(), spec.long_opt) != basic_opts.end()) {
            std::cout << "  ";
            if (!spec.short_opt.empty()) {
                std::cout << "-" << spec.short_opt << ", ";
            } else {
                std::cout << "    ";
            }

            std::string long_opt = "--" + spec.long_opt;
            if (spec.has_arg) {
                long_opt += "=" + spec.metavar;
            }
            std::cout << std::left << std::setw(35) << long_opt << " " << spec.description << std::endl;
        }
    }

    // Advanced options (expert tuning)
    std::cout << "\nAdvanced Options:\n";
    std::cout << "   Warning: These are for fine-tuning; ensure familiarity with the code\n";
    
    for (const auto& spec : specs_) {
        if (std::find(basic_opts.begin(), basic_opts.end(), spec.long_opt) == basic_opts.end()) {
            std::cout << "  ";
            if (!spec.short_opt.empty()) {
                std::cout << "-" << spec.short_opt << ", ";
            } else {
                std::cout << "    ";
            }

            std::string long_opt = "--" + spec.long_opt;
            if (spec.has_arg) {
                long_opt += "=" + spec.metavar;
            }
            std::cout << std::left << std::setw(35) << long_opt << " " << spec.description << std::endl;
        }
    }

    std::cout << "\nExamples:\n";
    std::cout << "  " << argv0 << " --ratio=10,50,90\n";
    std::cout << "  " << argv0 << " --profile\n";
    std::cout << "  " << argv0 << " --dry-run\n";
}

void CLIParser::print_usage(const char* argv0) {
    std::cout << "Usage: " << argv0 << " [options] [ratio]\n\n";
    std::cout << "Mess Benchmark 2.0 - Generic Memory System Characterization\n";
}

bool CLIParser::parse_size(const std::string& s, SizeTier& out) {
    if (s == "lite") {
        out = SizeTier::LITE;
        return true;
    } else if (s == "medium") {
        out = SizeTier::MEDIUM;
        return true;
    } else if (s == "full") {
        out = SizeTier::FULL;
        return true;
    }
    return false;
}

bool CLIParser::parse_bind(const std::string& s, std::vector<int>& out) {
    out.clear();
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, ',')) {
        try {
            out.push_back(std::stoi(item));
        } catch (...) {
            return false;
        }
    }
    return !out.empty();
}

bool CLIParser::parse_ratios(const std::string& s, std::vector<double>& out) {
    out.clear();
    std::stringstream ss(s);
    std::string token;

    while (std::getline(ss, token, ',')) {
        token.erase(token.begin(), std::find_if(token.begin(), token.end(),
            [](int ch) { return !std::isspace(ch); }));
        token.erase(std::find_if(token.rbegin(), token.rend(),
            [](int ch) { return !std::isspace(ch); }).base(), token.end());

        if (token.empty()) continue;

        try {
            double ratio = std::stod(token);
            if (ratio < 0.0 || ratio > 100.0 || !std::isfinite(ratio)) {
                return false;
            }
            out.push_back(ratio);
        } catch (const std::exception&) {
            return false;
        }
    }

    return !out.empty();
}

bool CLIParser::parse_pauses(const std::string& s, std::vector<int>& out) {
    out.clear();
    std::stringstream ss(s);
    std::string token;

    while (std::getline(ss, token, ',')) {
        token.erase(token.begin(), std::find_if(token.begin(), token.end(),
            [](int ch) { return !std::isspace(ch); }));
        token.erase(std::find_if(token.rbegin(), token.rend(),
            [](int ch) { return !std::isspace(ch); }).base(), token.end());

        if (token.empty()) continue;

        try {
            int pause = std::stoi(token);
            if (pause < 0) {
                return false;
            }
            out.push_back(pause);
        } catch (const std::exception&) {
            return false;
        }
    }

    return !out.empty();
}
