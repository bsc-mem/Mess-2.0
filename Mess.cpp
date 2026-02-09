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
#include <future>
#include <sstream>

#include "system_detection.h"
#include "architecture/ArchitectureRegistry.h"
#include "architecture/BandwidthCounterStrategy.h"
#include "benchmark_config.h"
#include "codegen.h"
#include "measurement.h"
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

struct PerfAccessProbeResult {
    int paranoid_level = 3;
    bool perf_accessible = false;
};

PerfAccessProbeResult probe_perf_access() {
    PerfAccessProbeResult result;
    std::ifstream paranoid_file("/proc/sys/kernel/perf_event_paranoid");
    if (paranoid_file.is_open()) {
        paranoid_file >> result.paranoid_level;
    }

    if (result.paranoid_level <= 1) {
        result.perf_accessible = run_command_success("perf stat -e cycles:k true 2>/dev/null >/dev/null");
    }
    return result;
}

struct KernelBinaryCheckResult {
    bool ok = true;
    std::string error_message;
};

KernelBinaryCheckResult validate_runtime_binaries() {
    KernelBinaryCheckResult result;
    std::error_code ec;
    const std::filesystem::path exe_path = std::filesystem::canonical("/proc/self/exe", ec);
    if (ec) {
        result.ok = false;
        result.error_message = "ERROR: Could not determine executable path at /proc/self/exe.";
        return result;
    }

    const std::filesystem::path bin_dir = exe_path.parent_path();
    const std::filesystem::path ptr_chase_path = bin_dir / "ptr_chase";
    const std::filesystem::path traffic_gen_path = bin_dir / "traffic_gen_multiseq.x";

    if (!std::filesystem::exists(ptr_chase_path)) {
        std::ostringstream oss;
        oss << "ERROR: Kernel binaries not found!" << std::endl;
        oss << "       Expected ptr_chase at: " << ptr_chase_path << std::endl;
        oss << "       Please run generate_code to compile the kernels.";
        result.ok = false;
        result.error_message = oss.str();
        return result;
    }

    if (!std::filesystem::exists(traffic_gen_path)) {
        std::ostringstream oss;
        oss << "ERROR: TrafficGen kernel not found!" << std::endl;
        oss << "       Expected traffic_gen_multiseq.x at: " << traffic_gen_path << std::endl;
        oss << "       Please run generate_code to compile the kernels.";
        result.ok = false;
        result.error_message = oss.str();
    }

    return result;
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

        system("pkill -u $(id -u) -TERM -x traffic_gen_multiseq.x 2>/dev/null || true");
        system("pkill -u $(id -u) -TERM -x traffic_gen_rand.x 2>/dev/null || true");
        system("pkill -u $(id -u) -TERM -x ptr_chase 2>/dev/null || true");
        system("pkill -u $(id -u) -KILL -x traffic_gen_multiseq.x 2>/dev/null || true");
        system("pkill -u $(id -u) -KILL -x traffic_gen_rand.x 2>/dev/null || true");
        system("pkill -u $(id -u) -KILL -x ptr_chase 2>/dev/null || true");
        system("rm -f /tmp/traffic_gen_pid_* /tmp/ptr_chase_perf_* /tmp/ptr_chase_ready_*.flag /tmp/ptr_chase_start_*.flag /tmp/mess_ptrchase_pipe_* /tmp/mess_tgen_ready_* /tmp/ptr_chase_*.log 2>/dev/null");
        
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
        auto detect_future = std::async(std::launch::async, [&detector]() {
            return detector.detect();
        });
        auto cache_future = std::async(std::launch::async, []() {
            SystemToolsCache::instance().init();
        });
        auto perf_probe_future = std::async(std::launch::async, []() {
            return probe_perf_access();
        });

        std::future<KernelBinaryCheckResult> kernel_check_future;
        if (!config->dry_run) {
            kernel_check_future = std::async(std::launch::async, []() {
                return validate_runtime_binaries();
            });
        }

        if (!detect_future.get()) {
            std::cerr << "Failed to detect system information" << std::endl;
            return EXIT_FAILURE;
        }

        print_enhanced_header(detector);

        if (detector.get_capabilities().arch == CPUArchitecture::RISCV64) {
            std::cerr << "\n\033[1;33mWarning: RISC-V support is work-in-progress.\033[0m" << std::endl;
            std::cerr << "Assembly generation, latency measurement and counter detection are available," << std::endl;
            std::cerr << "but bandwidth measurement is not yet implemented for RISC-V platforms." << std::endl;
            std::cerr << "RISC-V does not expose uncore memory controller counters (CAS) via perf;" << std::endl;
            std::cerr << "bandwidth approximation via alternative counters is planned for a future release.\n" << std::endl;
            if (!config->dry_run) {
                std::cerr << "\033[1;31mError: Cannot run the full benchmark on RISC-V yet.\033[0m" << std::endl;
                std::cerr << "Use --dry-run to inspect system detection and counter discovery.\n" << std::endl;
                return EXIT_FAILURE;
            }
        }

        if (kernel_check_future.valid()) {
            KernelBinaryCheckResult kernel_check = kernel_check_future.get();
            if (!kernel_check.ok) {
                std::cerr << kernel_check.error_message << std::endl;
                return EXIT_FAILURE;
            }
        }

        const auto perf_probe = perf_probe_future.get();
        const int paranoid_level = perf_probe.paranoid_level;
        const bool perf_accessible = perf_probe.perf_accessible;

        std::future<TLBMeasurement> tlb_future;
        const std::string status_prefix = "Initial status: ";
        std::cout << status_prefix << "\033[33mChecking...\033[0m" << std::string(10, ' ') << "\r" << std::flush;
        if (perf_accessible) {
            std::cout << status_prefix << "\033[33mMeasuring TLB...\033[0m" << std::string(5, ' ') << "\r" << std::flush;
            tlb_future = std::async(std::launch::async, []() {
                return measure_and_set_tlb_latency();
            });
        } else {
            std::cout << status_prefix << "\033[33mSkipping TLB measurement (performance counters not accessible)\033[0m" << std::string(5, ' ') << "\r" << std::flush;
            tlb_ok = false;
            tlb_measurement.latency_ns = 0.0;
        }

        const auto& caps = detector.get_capabilities();

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

        auto& bw_strategy = BandwidthCounterStrategy::instance();
        bw_strategy.set_measurer_type(string_to_measurer_type(config->measurer));
        bw_strategy.set_extra_counters(config->add_counters);
        bw_strategy.set_memory_type(caps.memory_type);
        auto counter_discovery_future = std::async(std::launch::async, [&bw_strategy, src_cpu, &target_nodes, &caps]() {
            bw_strategy.initialize(src_cpu, target_nodes, caps);
        });

        if (perf_accessible && tlb_future.valid()) {
            tlb_measurement = tlb_future.get();
            tlb_ok = (tlb_measurement.latency_ns > 0);
        }

        counter_discovery_future.get();
        cache_future.get();

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
            if (perf_accessible && !tlb_ok) {
                std::cout << "  → TLB measurement not available (using default values)" << std::endl;
            }

            if (!config->dry_run) {
                std::cerr << "\n\033[1;31mError: System configuration check failed\033[0m" << std::endl;
                std::cerr << "Benchmark cannot run on this system. Please check the configuration and try again.\n" << std::endl;
                return EXIT_FAILURE;
            }
        }

        uint64_t tlb1_raw = 0, tlb2_raw = 0;
        bool use_tlb1 = false, use_tlb2 = false;
        bw_strategy.get_tlb_counters(tlb1_raw, tlb2_raw, use_tlb1, use_tlb2);
        
        if (config->dry_run) {
            std::cout << "\n=== Dry Run Mode ===" << std::endl;
            config->print_summary(std::cout, true);

            auto architecture = ArchitectureRegistry::instance().getArchitecture(caps);

            std::cout << "\n=== Memory Architecture & Counter Discovery ===" << std::endl;
            std::cout << "Detected Architecture: " << (architecture ? architecture->getName() : "Unknown") << std::endl;
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
            std::cout << "  Memory Binding: " << config->get_bind_name() << "\n\n";

            std::cout << "=== Counter Discovery Results ===" << std::endl;
            std::cout << "Measurer: " << bw_strategy.get_measurer_name() << std::endl;
            std::cout << "Latency Counters: cycles,instructions" << std::endl;
            bw_strategy.print_counter_info(std::cout);

            if (tlb_ok) {
                std::cout << "TLB Hit Latency: " << tlb_measurement.latency_ns << " ns" << std::endl;
                std::cout << "Cache Line Size: " << tlb_measurement.cache_line_size << " bytes" << std::endl;
            }

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
            if (config->verbosity >= 2) {
                std::cout << "System benchmark enabled: running all ratios (0-100%)" << std::endl;
            }
        }

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
