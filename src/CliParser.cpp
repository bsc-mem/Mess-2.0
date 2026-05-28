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

#include "CliParser.h"
#include "architecture/BandwidthCounterStrategy.h"
#include <iostream>
#include <sstream>
#include <algorithm>
#include <cstring>
#include <getopt.h>
#include <unistd.h>
#include <cstdlib>
#include <iomanip>
#include <cmath>
#include <limits>

void CLIParser::display_configuration(const BenchmarkConfig& config, bool perf_accessible, int paranoid_level, bool tlb_ok, double tlb_latency_ns) const {
    if (config.verbosity < 1) {
        return;
    }

    if (config.verbosity >= 2) {
        std::cout << "\n=== Configuration ===" << std::endl;
        std::cout << "Performance counter access: " << (perf_accessible ? "OK" : "KO") 
                  << " (paranoid: " << paranoid_level << ")" << std::endl;
        
        if (tlb_ok) {
            std::cout << "TLB hit latency: " << tlb_latency_ns << " ns" << std::endl;
        } else {
            std::cout << "! TLB measurement failed, using default values" << std::endl;
        }
    }else{
        std::cout << "\n";
    }
    
    if (config.verbosity >= 2) {
        std::cout << "\n=== Settings used ===" << std::endl;
        std::cout << "Mode: " << "Custom configuration" << std::endl;
        
        std::cout << "Load ratios (%): ";
        for (size_t i = 0; i < config.ratios_pct.size(); ++i) {
            if (i > 0) std::cout << ", ";
            std::cout << config.ratios_pct[i] << "%";
        }
        std::cout << std::endl;
        
        std::cout << "Pause values: ";
        if (config.pauses.empty()) {
            if (config.adaptive_point_count > 0) {
                std::cout << "(adaptive, point-count=" << config.adaptive_point_count << ")" << std::endl;
            } else {
                std::cout << "(adaptive, tier=" << config.adaptive_tier << ")" << std::endl;
            }
        } else {
            std::cout << config.pauses.size() << " values (";
            for (size_t i = 0; i < config.pauses.size(); ++i) {
                if (i > 0) std::cout << ", ";
                std::cout << config.pauses[i];
            }
            std::cout << ")" << std::endl;
        }
                
        std::cout << "Latency measurement: " << (config.instruction_sampling ? "Instruction Sampling (PEBS/SPE)" : "Pointer Chasing") << std::endl;
        
        std::cout << "Point repetitions: " << config.point_reps << std::endl;
        std::cout << "Profile mode: " << (config.profile_output ? "enabled" : "disabled") << std::endl;
        
        std::cout << "Verbosity level: " << config.verbosity << std::endl;
    }
    
}

CLIParser::CLIParser() {
    specs_ = {
        {"r", "ratio", true, "X[,X...]", "read/write ratios in percent; comma-separated list"},
        {"", "dry-run", false, "", "detect system and print configuration without running benchmark"},
        {"", "profile", false, "", "save bw/lat files to measuring/ directory"},
        {"", "folder", true, "DIR", "root output folder (default: measuring)"},
        {"v", "verbose", true, "N", "verbosity level (0-4, default: 1)"},
        {"", "version", false, "", "print version and exit"},
        {"h", "help", false, "", "print this help and exit"},
        {"", "pause", true, "X[,X...]", "pause values; comma-separated list"},
        {"", "tier", true, "TIER", "adaptive pause discovery preset: lite (15 points), standard (50, default), detailed (200)"},
        {"", "point-count", true, "N", "custom adaptive point budget override; incompatible with --tier and --pause"},
        {"", "measurer", true, "TYPE", "bandwidth measurement tool: auto, perf, likwid, vtune, pcm (default: auto)"},
        {"", "repetitions", true, "N", "number of point repetitions (default: 3)"},
        {"", "total-cores", true, "N", "number of cores to run TrafficGen workers on (default: TOTAL_CORES-1)"},
        {"", "cores", true, "LIST", "comma-separated list of specific cores to use (e.g. 0,1,2). Incompatible with --total-cores"},
        {"", "bind", true, "LIST", "comma-separated list of NUMA nodes to bind memory to (e.g. 0,1)"},
        {"", "persistent-trafficgen", false, "", "keep TrafficGen alive across repetitions for the same point"},
        {"", "inst-lat", false, "", "use hardware instruction sampling (PEBS/SPE) for latency instead of pointer chasing"},
        {"", "add-counters", true, "COUNTERS", "comma-separated list of extra counters to measure alongside BW"}
    };
}

/**
 * @brief Parses command-line arguments and populates the BenchmarkConfig object.
 * 
 * This method iterates through all provided `argv` arguments, matching them against
 * expected flags (e.g., `--size`, `--ratio`, `--bind`). It handles parsing complex lists 
 * (like comma-separated CPU cores for `--bind`) and validates the logical consistency 
 * of the provided arguments (e.g., ensuring mutually exclusive flags are not used together).
 * 
 * If an invalid argument or malformed value is encountered, it outputs an error message 
 * to stderr and returns `false`, halting the execution.
 * 
 * @param argc The number of command-line arguments.
 * @param argv The array of command-line argument strings.
 * @param config The BenchmarkConfig object to populate with parsed values.
 * @return bool True if all arguments were parsed and validated successfully, false otherwise.
 */
bool CLIParser::parse(int argc, char** argv, BenchmarkConfig& config) {
    if (argc == 1) {
        config.ratios_pct.clear();
        for (int ratio = 100; ratio >= 0; ratio -= 2) {
            config.ratios_pct.push_back(static_cast<double>(ratio));
        }
        
        config.verbosity = 1;
        
        return true;
    }
    
    bool total_cores_flag_seen = false;
    bool explicit_cores_flag_seen = false;
    bool repetitions_flag_seen = false;
    bool tier_flag_seen = false;
    bool point_count_flag_seen = false;
    
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
        } else if (arg.find("--pause=") == 0) {
            std::string value = arg.substr(8);
            if (!parse_pauses(value.c_str(), config.pauses)) { std::cerr << "error: invalid pause '" << value << "'" << std::endl; return false; }
        } else if (arg == "--tier") {
            if (i + 1 >= argc) { std::cerr << "error: --tier requires an argument" << std::endl; return false; }
            config.adaptive_tier = argv[++i];
            if (config.adaptive_tier != "lite" && config.adaptive_tier != "standard" && config.adaptive_tier != "detailed") {
                std::cerr << "error: invalid tier '" << config.adaptive_tier << "' (valid: lite, standard, detailed)" << std::endl;
                return false;
            }
            tier_flag_seen = true;
        } else if (arg.find("--tier=") == 0) {
            config.adaptive_tier = arg.substr(7);
            if (config.adaptive_tier != "lite" && config.adaptive_tier != "standard" && config.adaptive_tier != "detailed") {
                std::cerr << "error: invalid tier '" << config.adaptive_tier << "' (valid: lite, standard, detailed)" << std::endl;
                return false;
            }
            tier_flag_seen = true;
        } else if (arg == "--point-count") {
            if (i + 1 >= argc) { std::cerr << "error: --point-count requires an argument" << std::endl; return false; }
            config.adaptive_point_count = std::stoi(argv[++i]);
            if (config.adaptive_point_count < 1) {
                std::cerr << "error: point-count must be >= 1" << std::endl;
                return false;
            }
            point_count_flag_seen = true;
        } else if (arg.find("--point-count=") == 0) {
            std::string value = arg.substr(14);
            config.adaptive_point_count = std::stoi(value);
            if (config.adaptive_point_count < 1) {
                std::cerr << "error: point-count must be >= 1" << std::endl;
                return false;
            }
            point_count_flag_seen = true;
        } else if (arg == "--profile") {
            config.profile_output = true;

        } else if (arg == "--dry-run") {
            config.dry_run = true;
        } else if (arg == "--likwid") {
            std::cerr << "error: --likwid is deprecated. Use --measurer=likwid instead." << std::endl;
            return false;
        } else if (arg == "--measurer") {
            if (i + 1 >= argc) { std::cerr << "error: --measurer requires an argument" << std::endl; return false; }
            config.measurer = argv[++i];
        } else if (arg.find("--measurer=") == 0) {
            config.measurer = arg.substr(11);
        } else if (arg == "--inst-lat") {
            config.instruction_sampling = true;
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
            if (config.verbosity < 0 || config.verbosity > 4) { std::cerr << "error: verbosity must be between 0 and 4" << std::endl; return false; }
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
            repetitions_flag_seen = true;
        } else if (arg.find("--repetitions=") == 0) {
            std::string value = arg.substr(14);
            config.point_reps = std::stoi(value);
            if (config.point_reps < 1) { std::cerr << "error: repetitions must be >= 1" << std::endl; return false; }
            repetitions_flag_seen = true;
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
            if (!expand_core_list(value, config.traffic_gen_explicit_cores)) {
                std::cerr << "error: invalid --cores value '" << value << "' (expected comma-separated cores and/or N-M ranges)" << std::endl;
                return false;
            }
            config.traffic_gen_cores = config.traffic_gen_explicit_cores.size();
            explicit_cores_flag_seen = true;
        } else if (arg.find("--cores=") == 0) {
            std::string value = arg.substr(8);
            if (!expand_core_list(value, config.traffic_gen_explicit_cores)) {
                std::cerr << "error: invalid --cores value '" << value << "' (expected comma-separated cores and/or N-M ranges)" << std::endl;
                return false;
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
        } else if (arg == "--extra-perf" || arg.find("--extra-perf=") == 0) {
            std::cerr << "error: --extra-perf is deprecated. Use --add-counters instead." << std::endl;
            return false;
        } else if (arg == "--add-counters") {
            if (i + 1 >= argc) { std::cerr << "error: --add-counters requires an argument" << std::endl; return false; }
            std::string value = argv[++i];
            std::stringstream ss(value);
            std::string segment;
            while(std::getline(ss, segment, ',')) {
                if (!segment.empty()) {
                    config.add_counters.push_back(segment);
                }
            }
        } else if (arg.find("--add-counters=") == 0) {
            std::string value = arg.substr(15);
            std::stringstream ss(value);
            std::string segment;
            while(std::getline(ss, segment, ',')) {
                if (!segment.empty()) {
                    config.add_counters.push_back(segment);
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

    if (tier_flag_seen && point_count_flag_seen) {
        std::cerr << "warning: --tier selects a built-in adaptive point budget "
                  << "(lite=15, standard=50, detailed=200) and preserves tier-specific defaults "
                  << "such as lite repetitions; --point-count sets a custom adaptive point budget. "
                  << "Use only one of them." << std::endl;
        return false;
    }

    if (!config.pauses.empty() && point_count_flag_seen) {
        std::cerr << "error: --point-count is incompatible with --pause because manual pauses disable adaptive discovery." << std::endl;
        return false;
    }

    if (repetitions_flag_seen && config.point_reps > 1000) {
        std::cerr << "WARNING: High repetitions value (" << config.point_reps << ") may take a very long time. Consider using a lower value." << std::endl;
    }

    if(config.adaptive_tier == "lite" && !repetitions_flag_seen){
        config.point_reps = 1;
    }

    if (string_to_measurer_type(config.measurer) == MeasurerType::VTUNE && !config.add_counters.empty()) {
        std::cerr << "error: --add-counters is not supported with --measurer=vtune." << std::endl;
        return false;
    }
    
    return true;
}

void CLIParser::print_help(const char* argv0) {
    print_usage(argv0);

    std::cout << "\nBasic Options:\n";
    std::vector<std::string> basic_opts = {"ratio", "dry-run", "profile", "folder", "verbose", "tier", "version", "help"};

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
    std::cout << "  " << argv0 << " --profile --ratio=100 --tier=lite\n";
}

void CLIParser::print_usage(const char* argv0) {
    std::cout << "\n\033[1mMess - Memory Stress Benchmark\033[0m\n";
    std::cout << "\nUsage: " << argv0 << " [options]\n";
}

bool CLIParser::expand_core_list(const std::string& s, std::vector<std::string>& out) {
    out.clear();
    std::stringstream ss(s);
    std::string segment;
    while (std::getline(ss, segment, ',')) {
        segment.erase(segment.begin(), std::find_if(segment.begin(), segment.end(),
            [](int ch){ return !std::isspace(ch); }));
        segment.erase(std::find_if(segment.rbegin(), segment.rend(),
            [](int ch){ return !std::isspace(ch); }).base(), segment.end());
        if (segment.empty()) continue;

        auto dash = segment.find('-');
        if (dash == std::string::npos) {
            try {
                int v = std::stoi(segment);
                if (v < 0) return false;
                out.push_back(std::to_string(v));
            } catch (...) { return false; }
        } else {
            try {
                int lo = std::stoi(segment.substr(0, dash));
                int hi = std::stoi(segment.substr(dash + 1));
                if (lo < 0 || hi < lo) return false;
                for (int v = lo; v <= hi; ++v) out.push_back(std::to_string(v));
            } catch (...) { return false; }
        }
    }
    return !out.empty();
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
            size_t consumed = 0;
            long long val = std::stoll(token, &consumed);
            if (consumed != token.size()) return false;
            if (val < 0) return false;
            if (val > std::numeric_limits<int>::max()) {
                std::cerr << "error: pause '" << token << "' exceeds maximum ("
                          << std::numeric_limits<int>::max() << ")" << std::endl;
                return false;
            }
            out.push_back(static_cast<int>(val));
        } catch (const std::exception&) {
            return false;
        }
    }

    return !out.empty();
}
