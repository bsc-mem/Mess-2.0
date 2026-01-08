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

#include <iomanip>
#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <getopt.h>
#include <unistd.h>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <cmath>
#include <cerrno>
#include <csignal>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>

#include "system_detection.h"
#include "architecture/ArchitectureRegistry.h"
#include "architecture/PerformanceCounterStrategy.h"
#include "benchmark_config.h"
#include "codegen.h"
#include "measurement.h"
#include "measurement/bw_measurers/LikwidBandwidthMeasurer.h"
#include "benchmark_executor.h"
#include "results_processor.h"
#include "cli_parser.h"

#include "utils.h"
#include "ptrchase_perf_helper.h"

#define MESS_VERSION "2.0.0"

namespace {

void cleanup_empty_output_dirs(const BenchmarkConfig& config) {
    namespace fs = std::filesystem;
    
    fs::path root = config.output_root.empty() ? fs::path("measuring") : fs::path(config.output_root);

    bool bw_removed = false;
    bool lat_removed = false;
    std::error_code ec;
    
    fs::path bw_dir = root / "bw";
    if (fs::exists(bw_dir, ec) && fs::is_directory(bw_dir, ec) && fs::is_empty(bw_dir, ec)) {
        fs::remove(bw_dir, ec);
        if (!ec) {
            bw_removed = true;
        }
    }
    
    fs::path lat_dir = root / "lat";
    if (fs::exists(lat_dir, ec) && fs::is_directory(lat_dir, ec) && fs::is_empty(lat_dir, ec)) {
        fs::remove(lat_dir, ec);
        if (!ec) {
            lat_removed = true;
        }
    }
    
    if (bw_removed && lat_removed) {
        fs::path plotter = root / "plotter.txt";
        if (fs::exists(plotter, ec) && fs::is_regular_file(plotter, ec)) {
            fs::remove(plotter, ec);
        }
    }
}

    }


BenchmarkExecutor* g_executor = nullptr;
volatile sig_atomic_t g_signal_notified = 0;

void signal_handler(int signal) {
    if (signal == SIGINT || signal == SIGTERM) {
        if (!g_signal_notified) {
            const char msg[] = "\n\nEnding execution please wait...\n";
            write(STDERR_FILENO, msg, sizeof(msg) - 1);
            g_signal_notified = 1;
        }
        
        if (g_executor) {
            g_executor->force_cleanup();
        }
        
        system("pkill -9 -f 'traffic_gen_.*\\.x' 2>/dev/null || true");
        system("pkill -9 '^traffic_gen_' 2>/dev/null || true");
        system("pkill -9 -f 'perf.*ptr_chase' 2>/dev/null || true");
        system("pkill -9 ptr_chase 2>/dev/null || true");
        system("rm -f /tmp/traffic_gen_pid_* /tmp/ptr_chase_perf_* /tmp/ptr_chase_ready_*.flag /tmp/ptr_chase_start_*.flag /tmp/mess_ptrchase_pipe_* /tmp/ptr_chase_*.log 2>/dev/null");
        
        std::signal(signal, SIG_DFL);
        raise(signal);
    }
}

void print_enhanced_header(const SystemDetector& detector) {
    std::cout << "\n";
    std::cout << "╔══════════════════════════════════════════════════════════════╗\n";
    std::cout << "║        ░███     ░███                                         ║\n";  
    std::cout << "║        ░████   ░████                                         ║\n";
    std::cout << "║        ░██░██ ░██░██  ░███████   ░███████   ░███████         ║\n";
    std::cout << "║        ░██ ░████ ░██ ░██    ░██ ░██        ░██               ║\n";
    std::cout << "║        ░██  ░██  ░██ ░█████████  ░███████   ░███████         ║\n";
    std::cout << "║        ░██       ░██ ░██               ░██        ░██        ║\n";
    std::cout << "║        ░██       ░██  ░███████   ░███████   ░███████         ║\n";
    std::cout << "║                                                              ║\n";
    std::cout << "║                   \033[1mMemory Stress Benchmark\033[0m                    ║\n";
    std::cout << "║                                                              ║\n";
    std::cout << "║  ┌─ System Configuration ─────────────────────────────────┐  ║\n";
    std::cout << "║  ├────────────────────────────────────────────────────────┤  ║\n";
    
    std::stringstream ss;
    detector.print(ss, 1);
    
    std::string line;
    while (std::getline(ss, line)) {
        if (!line.empty()) {
            if (line.find("System Information:") == std::string::npos) {
                std::cout << "║  │ " << std::left << std::setw(55) << line << "│  ║\n";
            }
        }
    }
    
    std::cout << "║  └────────────────────────────────────────────────────────┘  ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════════╝\n\n";
}

int main(int argc, char **argv) {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    try {
        TLBMeasurement tlb_measurement;
        bool tlb_ok = false;
        
        auto config = std::make_unique<BenchmarkConfig>();
        CLIParser parser;
        if (!parser.parse(argc, argv, *config)) {
            parser.print_help(argv[0]);
            return EXIT_FAILURE;
        }

        if (!config->profile_output) {
            cleanup_empty_output_dirs(*config);
        }

        if (config->show_version) {
            std::cout << "Mess Benchmark " << MESS_VERSION << std::endl;
            return EXIT_SUCCESS;
        }

        if (config->show_help) {
            parser.print_help(argv[0]);
            return EXIT_SUCCESS;
        }

        SystemDetector detector;
        if (!detector.detect()) {
            std::cerr << "Failed to detect system information" << std::endl;
            return EXIT_FAILURE;
        }

        SystemToolsCache::instance().init();

        print_enhanced_header(detector);

        if (!config->dry_run) {
            std::filesystem::path exe_path = std::filesystem::canonical("/proc/self/exe");
            std::filesystem::path bin_dir = exe_path.parent_path();
            
            std::filesystem::path ptr_chase_path = bin_dir / "ptr_chase";
            std::filesystem::path traffic_gen_path = bin_dir / "traffic_gen_multiseq.x";
            
            if (!std::filesystem::exists(ptr_chase_path)) {
                std::cerr << "ERROR: Kernel binaries not found!" << std::endl;
                std::cerr << "       Expected ptr_chase at: " << ptr_chase_path << std::endl;
                std::cerr << "       Please run generate_code to compile the kernels." << std::endl;
                return EXIT_FAILURE;
            }
            
            if (!std::filesystem::exists(traffic_gen_path)) {
                std::cerr << "ERROR: TrafficGen kernel not found!" << std::endl;
                std::cerr << "       Expected traffic_gen_multiseq.x at: " << traffic_gen_path << std::endl;
                std::cerr << "       Please run generate_code to compile the kernels." << std::endl;
                return EXIT_FAILURE;
            }
        }

        const std::string status_prefix = "Initial status: ";
        std::cout << status_prefix << "\033[33mChecking...\033[0m" << std::string(10, ' ') << "\r" << std::flush;
        
        std::ifstream paranoid_file("/proc/sys/kernel/perf_event_paranoid");
        int paranoid_level = 3;
        if (paranoid_file.is_open()) {
            paranoid_file >> paranoid_level;
            paranoid_file.close();
        }

        bool perf_accessible = false;
        if (paranoid_level <= 1) {
            perf_accessible = run_command_success("perf stat -e cycles:k true 2>/dev/null >/dev/null");
        }
        
        if (perf_accessible) {
            std::cout << status_prefix << "\033[33mMeasuring TLB...\033[0m" << std::string(5, ' ') << "\r" << std::flush;
            tlb_measurement = measure_and_set_tlb_latency();
            tlb_ok = (tlb_measurement.latency_ns > 0);
        } else {
            std::cout << status_prefix << "\033[33mSkipping TLB measurement (performance counters not accessible)\033[0m" << std::string(5, ' ') << "\r" << std::flush;
            tlb_ok = false;
            tlb_measurement.latency_ns = 0;
        }
        
        bool overall_ok = perf_accessible && tlb_ok;
        

        
        std::cout << "\r" << std::string(80, ' ') << "\r";
        std::cout << status_prefix;
        
        if (overall_ok) {
            std::cout << "\033[32mOK\033[0m" << std::endl;
            
            parser.display_configuration(*config, perf_accessible, paranoid_level, tlb_ok, tlb_measurement.latency_ns);
        } else {
            std::cout << "\033[31mKO\033[0m" << std::endl;
            
            if (!perf_accessible) {
                std::cout << "  → Performance counter access denied (paranoid level: " << paranoid_level << ")" << std::endl;
                std::cout << "    Required for benchmark execution. Try: echo 0 | sudo tee /proc/sys/kernel/perf_event_paranoid" << std::endl;
            }
            if (!tlb_ok) {
                std::cout << "  → TLB measurement not available (using default values)" << std::endl;
            } else {
                std::cout << "  → Performance counter access denied (paranoid level: " << paranoid_level << ")" << std::endl;
                std::cout << "    Required for benchmark execution. Try: echo 0 | sudo tee /proc/sys/kernel/perf_event_paranoid" << std::endl;
            }
            
            if (!config->dry_run) {
                std::cerr << "\n\033[1;31mError: System configuration check failed\033[0m" << std::endl;
                std::cerr << "Benchmark cannot run on this system. Please check the configuration and try again.\n" << std::endl;
                return EXIT_FAILURE;
            }
        }

        if (config->dry_run) {
            std::cout << "\n=== Dry Run Mode ===" << std::endl;
            config->print_summary(std::cout, true);
            
            std::cout << "\n=== Memory Architecture & Counter Discovery ===" << std::endl;
            
            std::cout.setstate(std::ios_base::failbit); 
                        
            std::cout.clear(); 
            
            const auto& caps = detector.get_capabilities();
            std::cout << "Memory Configuration:\n";
            std::cout << "  Type: " << caps.memory_type;
            if (!caps.memory_frequency.empty()) {
                std::cout << " @ " << caps.memory_frequency;
            }
            std::cout << "\n";
            std::cout << "  Channels: " << caps.memory_channels << "\n";
            std::cout << "  Total Size: " << (caps.total_memory / (1000*1000*1000)) << " GB\n\n";
            
            int num_cores = config->traffic_gen_cores;
            if (num_cores <= 0) {
                num_cores = detector.get_system_info().sockets[0].core_count - 1;
            }
            if (num_cores < 1) num_cores = 1;

            std::cout << "TrafficGen Configuration:\n";
            std::cout << "  Cores: " << num_cores << "\n";
            std::cout << std::endl;

            std::cout << "Bindings:" << std::endl;


            auto& registry = ArchitectureRegistry::instance();
            auto architecture = registry.getArchitecture(caps);
            if (!architecture) {
                std::cerr << "Error: Unsupported architecture." << std::endl;
                return 1;
            }
            std::cout << "Detected Architecture: " << architecture->getName() << std::endl;

            KernelConfig kernel_config;

            const char* ops_env = std::getenv("MESS_OPS_PER_BLOCK");
            if (ops_env) kernel_config.ops_per_pause_block = std::atoi(ops_env);

            KernelGenerator generator(kernel_config, caps);
            ArchitectureConfig arch_config = generator.setup_architecture_config();

            auto counterStrategy = architecture->createCounterStrategy(caps);
            int src_cpu = 0;
            if (!config->traffic_gen_explicit_cores.empty()) {
                try {
                    src_cpu = std::stoi(config->traffic_gen_explicit_cores[0]);
                } catch (...) {}
            }

            std::vector<int> target_nodes = config->memory_bind_nodes;
            if (target_nodes.empty()) {
                target_nodes.push_back(0); 
            }

            auto bw_counters = PerformanceCounterStrategy::discoverBandwidthCounters(src_cpu, target_nodes, config->force_likwid);
            
            std::cout << "  Memory Binding: " << config->get_bind_name() << std::endl;
            std::cout << std::endl;
            
            bool is_hbm = caps.memory_type.find("HBM") != std::string::npos;
            bool needs_upi = (bw_counters.type == CounterType::UPI_FLITS);
            
            if (is_hbm && needs_upi && !config->force_likwid) {
                std::cout << "Bandwidth Measurement Tool: perf (HBM with remote/cross-socket access)" << std::endl;
                std::cout << "UPI/Interconnect Counters (Remote Bandwidth):" << std::endl;
                if (bw_counters.perf_available) {
                    if (bw_counters.upi.has_rxl_txl) {
                        std::cout << "  RXL Events: " << bw_counters.upi.get_rxl_events_string() << std::endl;
                        std::cout << "  TXL Events: " << bw_counters.upi.get_txl_events_string() << std::endl;
                        if (bw_counters.upi.requires_link_aggregation) {
                            std::cout << "  (Link aggregation required: Yes)" << std::endl;
                        }
                    } else {
                        std::cout << "  No suitable UPI counters found." << std::endl;
                        if (!bw_counters.upi.failure_reason.empty()) {
                            std::cout << "  Reason: " << bw_counters.upi.failure_reason << std::endl;
                        }
                    }
                } else {
                    std::cout << "  perf not available." << std::endl;
                }
            } else if (is_hbm || config->force_likwid) {
                std::string mode = is_hbm ? "HBM Mode" : "MBOX Mode (Forced)";
                std::cout << "Bandwidth Measurement Tool: LIKWID (" << mode << ")" << std::endl;
                
                LikwidBandwidthMeasurer likwid_measurer(*config, detector.get_system_info(), nullptr, nullptr, [](){ return std::vector<int>{}; }, ExecutionMode::MULTISEQUENTIAL);
                std::string likwidCmd = likwid_measurer.find_likwid_binary();
                if (!likwidCmd.empty()) {
                    std::string memType = is_hbm ? "HBM" : "MBOX";
                    std::string eventStr = likwid_measurer.build_event_string(likwidCmd, memType, bw_counters.type);
                    if (!eventStr.empty()) {
                        std::cout << "  Likwid Counters: " << eventStr << std::endl;
                    } else {
                        std::cout << "  Likwid Counters: (Failed to discover counters)" << std::endl;
                    }
                } else {
                    std::cout << "  Likwid Counters: (likwid-perfctr not found)" << std::endl;
                }
            } else if (caps.memory_type == "CXL") {
                std::cout << "Bandwidth Measurement Tool: Intel PCM (CXL Mode)" << std::endl;
                std::cout << "  PCM Counters: (Placeholder)" << std::endl;
            } else {
                std::cout << "Bandwidth Measurement Tool: perf (Standard)" << std::endl;
                if (bw_counters.type == CounterType::UPI_FLITS) {
                    std::cout << "UPI/Interconnect Counters (Remote Bandwidth):" << std::endl;
                    if (bw_counters.perf_available) {
                        if (bw_counters.upi.has_rxl_txl) {
                            std::cout << "  RXL Events: " << bw_counters.upi.get_rxl_events_string() << std::endl;
                            std::cout << "  TXL Events: " << bw_counters.upi.get_txl_events_string() << std::endl;
                            if (bw_counters.upi.requires_link_aggregation) {
                                std::cout << "  (Link aggregation required: Yes)" << std::endl;
                            }
                        } else {
                            std::cout << "  No suitable UPI counters found." << std::endl;
                            if (!bw_counters.upi.failure_reason.empty()) {
                                std::cout << "  Reason: " << bw_counters.upi.failure_reason << std::endl;
                            }
                        }
                    } else {
                        std::cout << "  perf not available." << std::endl;
                    }
                } else {
                    std::cout << "CAS Counters (Local Bandwidth):" << std::endl;
                    if (bw_counters.perf_available) {
                        if (bw_counters.cas.has_read_write) {
                            std::cout << "  Read Events: " << bw_counters.cas.get_read_events_string() << std::endl;
                            std::cout << "  Write Events: " << bw_counters.cas.get_write_events_string() << std::endl;
                            if (bw_counters.cas.requires_channel_aggregation) {
                                std::cout << "  (Aggregation required: Yes)" << std::endl;
                            }
                        } else if (bw_counters.cas.has_combined_counter) {
                            std::cout << "  Combined Events: " << bw_counters.cas.get_all_events_string() << std::endl;
                        } else {
                            std::cout << "  No suitable CAS counters found." << std::endl;
                            if (!bw_counters.cas.failure_reason.empty()) {
                                std::cout << "  Reason: " << bw_counters.cas.failure_reason << std::endl;
                            }
                        }
                    } else {
                        std::cout << "  perf not available." << std::endl;
                    }
                }
            }
            std::cout << std::endl;
            
            std::cout << "Latency Counters: cycles,instructions" << std::endl;
            
            std::cout << "Bandwidth Counters: ";
            if (is_hbm && needs_upi && !config->force_likwid) {
                std::cout << bw_counters.get_all_events_string();
            } else if (is_hbm || config->force_likwid) {
                std::cout << "LIKWID (CAS_COUNT_RD/WR)";
            } else if (caps.memory_type == "CXL") {
                std::cout << "PCM (Placeholder)";
            } else if (bw_counters.perf_available && bw_counters.is_valid()) {
                std::cout << bw_counters.get_all_events_string();
            } else {
                std::cout << "None available";
            }
            std::cout << std::endl;
            
            if (tlb_ok) {
                std::cout << "TLB Hit Latency: " << tlb_measurement.latency_ns << " ns" << std::endl;
                std::cout << "Cache Line Size: " << tlb_measurement.cache_line_size << " bytes" << std::endl;
            }

            uint64_t tlb1_raw = 0, tlb2_raw = 0;
            bool use_tlb1 = false, use_tlb2 = false;
            ptrchase_perf::select_tlb_events_for_ptrchase(detector.get_system_info(), tlb1_raw, tlb2_raw, use_tlb1, use_tlb2);

            std::cout << "TLB Page Walk Counters:" << std::endl;
            if (use_tlb1) {
                std::cout << "  L1 TLB Miss / L2 Hit: 0x" << std::hex << tlb1_raw << std::dec << std::endl;
            }
            if (use_tlb2) {
                std::cout << "  Page Walk: 0x" << std::hex << tlb2_raw << std::dec << std::endl;
            }
            if (!use_tlb1 && !use_tlb2) {
                std::cout << "  None defined for this architecture" << std::endl;
            }
            
            std::cout << "\nDry run complete - the benchmark was not executed." << std::endl;
            return EXIT_SUCCESS;
        }

        if (config->ratios_pct.empty()) {
            config->ratios_pct.clear();
            for (int ratio = 0; ratio <= 100; ratio += 2) {
                config->ratios_pct.push_back(static_cast<double>(ratio));
            }
            config->size = SizeTier::FULL;
            if (config->verbosity >= 2) {
                std::cout << "System benchmark enabled: running all ratios (0-100%) with full size" << std::endl;
            }
        }

        // Verbose info block for normal execution
        if (config->verbosity >= 3) {
            std::cout << "\n=== Memory Architecture & Counter Discovery ===" << std::endl;
            std::cout << "Memory Configuration:\n";
            std::cout << "  Type: " << detector.get_capabilities().memory_type;
            if (!detector.get_capabilities().memory_frequency.empty()) {
                std::cout << " @ " << detector.get_capabilities().memory_frequency;
            }
            std::cout << "\n";
            
            int traffic_cores = config->traffic_gen_cores;
            if (traffic_cores <= 0) {
                traffic_cores = detector.get_system_info().sockets[0].core_count - 1;
            }
            if (traffic_cores < 1) traffic_cores = 1;

            std::cout << "TrafficGen Configuration:\n";
            std::cout << "  Cores: " << traffic_cores << "\n";
            std::cout << std::endl;

            std::cout << "Bindings:" << std::endl;
            std::cout << "Detected Architecture: " << ArchitectureRegistry::instance().getArchitecture(detector.get_capabilities())->getName() << std::endl;
            std::cout << "  Memory Binding: " << config->get_bind_name() << std::endl;
            std::cout << std::endl;
        }

        KernelGenerator kernel_gen(config->kernel, detector.get_capabilities());
        
        std::filesystem::path root = get_project_root();
        std::string output_dir = (root / "src/traffic_gen/src").string();
        
        const auto& capabilities = detector.get_capabilities();
        uint64_t arg1 = 5ULL * 1000 * 1000 * 1000;
        uint64_t arg2 = static_cast<uint64_t>(capabilities.l3_size) * 8;
        uint64_t min_traffic_gen_size = std::max(arg1, arg2) / sizeof(double);

        bool using_avx512 = (config->kernel.isa_mode == ISAMode::AVX512 || 
                            (config->kernel.isa_mode == ISAMode::AUTO && 
                             std::find(capabilities.extensions.begin(), 
                                      capabilities.extensions.end(), 
                                      ISAExtension::AVX512) != capabilities.extensions.end()));
        
        if (using_avx512) {
            min_traffic_gen_size *= 2;
        } else {
            min_traffic_gen_size = static_cast<uint64_t>(min_traffic_gen_size * 1.1);
        }
        
        int num_cores = config->traffic_gen_cores;
        if (num_cores <= 0) {
            num_cores = detector.get_system_info().sockets[0].core_count - 1;
        }
        if (num_cores < 1) num_cores = 1;

        kernel_gen.generate_kernel(output_dir);
        BenchmarkExecutor executor(*config, detector.get_system_info(), capabilities,
                                 tlb_measurement.latency_ns, tlb_measurement.cache_line_size);
        g_executor = &executor;
        
        if (!executor.run()) {
            g_executor = nullptr;
            std::cerr << "\n\nBenchmark execution failed" << std::endl;
            return EXIT_FAILURE;
        }
        g_executor = nullptr;

        ResultsProcessor results_processor(*config, &executor);
        if (!results_processor.process(executor.get_results(), executor.get_total_runtime(), executor.get_iteration_times())) {
            std::cerr << "Results processing failed" << std::endl;
            return EXIT_FAILURE;
        }

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    } catch (...) {
        std::cerr << "Unknown error occurred" << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
