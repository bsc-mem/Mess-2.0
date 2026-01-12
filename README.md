<div align="center">
<img src="doc/logos/512_mess_white.png" alt="Mess Benchmark Logo" width="200">
</div>

<div align="center">
<h1>Mess Benchmark</h1>
<p>A Multiplatform benchmark designed to provide holistic, detailed and close-to-hardware view of memory system performance with family of bandwidth-latency curves. This is an update to the original <a href="https://github.com/bsc-mem/Mess-benchmark" target="_blank">Mess Benchmark</a> (<span style="color:#b34700;font-style:italic;">Now deprecated</span>) to allow for ease of use for the user. 
<br>
</p><p>
Visit our website: <a href="https://mess.bsc.es" target="_blank"><strong>mess.bsc.es</strong></a>
</p>
</div>

<div align="center">
<img src="https://img.shields.io/badge/License-BSD%203--Clause-blue.svg" alt="License"> <img src="https://img.shields.io/badge/Version-2.0.0-blue" alt="Version"> <img src="https://img.shields.io/badge/Contributors-6-blue" alt="Contributors">
</div>

## Table of Contents

- [Architecture Support](#architecture-support)
- [Installation](#installation)
  - [Cloning the repo](#cloning-the-repo)
  - [Dependencies](#dependencies)
  - [Compilation](#compilation)
- [Executing](#executing)
  - [Quick start](#quick-start)
  - [Command Line Options](#command-line-options)
  - [Examples](#examples)
- [Mess Profiler](#mess-profiler)
- [Plotting Utilities](#plotting-utilities)
- [Changelog](#changelog)
- [Contributors](#contributors)
- [References](#references)

## Architecture Support

We consider an architecture to be supported when Mess can automatically run on it with the current code. We are working on porting the original <a href="https://github.com/bsc-mem/Mess-benchmark" target="_blank">Mess Benchmark</a> code to all the architectures it supports.

| Architecture | Mess2.0      |
| ------------ | ------------ |
| x86 CPUs     | ✅ Supported |
| ARM CPUs     | ✅ Supported |
| Power CPUs   | ✅ Supported |
| RISC-V CPUs  | ✅ Supported |
| NVIDIA GPUs  | 🚧 Pending   |

GPUs are the only paradigm still under active development for Mess2.0.

**Disclaimer:** Currently pending architectures here are architectures supported by the original [Mess Benchmark](https://github.com/bsc-mem/Mess-benchmark) that we are adapting to Mess2.0.

## Installation

### Cloning the repo

Clone the repository with all submodules:

```bash
git clone --recursive https://github.com/bsc-mem/Mess-2.0
cd mess2.0
```

We need the recursive tag to ensure we download some additional dependencies required for the benchmark.

If you forget `--recursive`, you can initialize submodules later:

```bash
git submodule update --init --recursive
```

### Dependencies

The following dependencies are required to run the benchmark:

| Dependency | Purpose                                                             | Installation                                                                                                           |
| ---------- | ------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------- |
| `perf`     | Hardware performance counters for bandwidth and latency measurement | `sudo apt install linux-tools-common linux-tools-$(uname -r)` (Debian/Ubuntu) or `sudo yum install perf` (RHEL/CentOS) |
| `numactl`  | NUMA memory binding for multi-socket systems                        | `sudo apt install numactl` (Debian/Ubuntu) or `sudo yum install numactl` (RHEL/CentOS)                                 |
| `GCC`      | C/C++ compiler with C++17 support                                   | `sudo apt install build-essential` (Debian/Ubuntu) or `sudo yum install gcc-c++` (RHEL/CentOS)                         |

**Optional dependencies:**

| Dependency | Purpose                                                      | Installation                                                          |
| ---------- | ------------------------------------------------------------ | --------------------------------------------------------------------- |
| `likwid`   | Alternative bandwidth measurement (required for HBM systems) | See [LIKWID installation guide](https://github.com/RRZE-HPC/likwid)   |
| `taskset`  | CPU core pinning for consistent measurements (recommended)   | Usually included with `util-linux`, already installed on most systems |

> **Note:** The benchmark requires access to hardware performance counters. On Linux, you may need to set `perf_event_paranoid` to allow access:
>
> ```bash
> echo 0 | sudo tee /proc/sys/kernel/perf_event_paranoid
> ```

### Compilation

The Mess Benchmark is designed to compile for any architecture automatically by simply running:

```bash
make
```

Once compiled, you need to run:

```bash
make install
```

for it to generate the corresponding binaries for the current architecture. The binaries will be placed in the `build/bin` folder.

To use them easily, you can add this folder to your PATH:

```bash
export PATH=$PATH:$(pwd)/build/bin
```

Or add it permanently to your shell configuration file (e.g., `~/.bashrc` or `~/.zshrc`).

**If there is an issue when compiling**, please let the maintainers know through emails (see [Contributors](#contributors)) or through an issue in the repository!

## Executing

### Quick start

The benchmark has two modes: Plain and Profile.

**Plain mode** (default): Run the full benchmark and display a performance summary.

```bash
./build/bin/mess
```

**Profile mode**: Generate profiling files for plotting curves.

```bash
./build/bin/mess --profile
```

These are the default behaviors, but you can customize further through the flags in the next section.

### Command Line Options

The Mess Benchmark has been designed to be easy to use for any user. As mentioned before, it can be run without any flags and it will execute correctly.

If you'd like to customize some aspects of its run, you can use the following flags:

- `--ratio=N` or `-r N`: Set memory-to-computation ratio (default: ALL)
  - Specifies the read/write ratio as a percentage (0-100)
  - Example: `--ratio=75` for 75% reads, 25% writes
  - **Optional** - uses default (0-100% in 2% steps) if not specified
- `--profile`: Enable profiling output
  - Saves bandwidth and latency measurements to files
  - This flag is needed to ensure the output files are created to plot the curves from.
  - Creates `measuring/` directory with results
  - **Optional** - disabled by default
- `--folder=DIR`: Set root output folder (default: measuring)
  - Specifies the directory where output files are saved
  - **Optional** - uses default if not specified
- `--verbose=N` or `-v N`: Set verbosity level (0-3)
  - 1: Basic progress information (default)
    - Shows progress bar
    - ETA and final run summary
  - 2: Detailed execution information
    - Flags used
    - Steps the benchmark follows, and some information about its execution
  - 3: Debug information
    - Stabilization data with raw measurement values
    - Detailed commands being instanced from within C++
  - **Optional** - uses default if not specified
- `--dry-run`: Detect system and print configuration without running benchmark
  - Shows system information and benchmark settings
  - Useful for checking configuration before actual execution
  - **Optional** - runs benchmark normally if not specified
- `--help` or `-h`: Show usage information and exit
- `--version` or `-V`: Show version information and exit

#### Advanced Options

Caution: This is meant for more fine-tuning of the benchmark; make sure you are familiar with its code.

- `--total-cores=N`: Limit total number of cores used by TrafficGen workers
  - Default: TOTAL_CORES-1 (core 0 is kept free)
  - Must be <= TOTAL_CORES-1; otherwise default is used
  - Workers are spread across the first N eligible cores
- `--cores=LIST`: Explicit list of cores to use (e.g., `0,1,2,3`)
  - Incompatible with `--total-cores`
  - Useful for specific binding scenarios (e.g., avoiding hyperthreads or specific sockets)
- `--bind=NODES`: NUMA memory binding (node number or comma-separated list)
  - `N` (single integer): Bind memory allocations to NUMA node `N` (e.g., `--bind=1`)
  - `N,M,...` (comma-separated list): Bind to multiple NUMA nodes (e.g., `--bind=0,1`)
  - Affects `numactl`/`srun --mem-bind` used for TrafficGen, bandwidth sampling, and ptr_chase
  - **Optional** - if not specified, uses default system allocation
- `--pause=VALUES`: Specify pause values manually
  - Comma-separated list of pause values
  - Example: `--pause=0,10,100,1000`
  - **Optional** - uses default curve points if not specified
- `--repetitions=N`: Number of repetitions per measurement point (default: 3)
  - More repetitions provide more stable results but take longer
  - **Optional** - uses default if not specified
- `--likwid`: Force use of LIKWID for bandwidth measurement
  - Forces the use of `likwid-perfctr` even on standard memory systems (uses MBOX counters)
  - Requires `likwid-perfctr` to be in PATH or `LIKWID_PERFCTR_PATH` set
  - **Optional** - disabled by default
- `--persistent-trafficgen`: Keep TrafficGen alive across repetitions
  - Improves performance by reusing TrafficGen processes
  - Reduces startup overhead for repeated measurements
  - **Optional** - disabled by default
- `--extra-perf=COUNTERS`: Measure additional performance counters
  - Comma-separated list of perf events to measure alongside bandwidth (e.g., `instructions,cycles`)
  - Useful for correlating bandwidth with other metrics
  - **Optional** - none by default
- Kernel code generation tuning: see `include/kernel_generator.h`
  - You can adjust `KernelConfig` (e.g., `ratio_granularity`, `ops_per_pause_block`, `num_simd_registers`)
  - Control ISA selection via `ISAMode` and related helpers
  - Toggle micro-architectural toggles such as `enable_interleaving`, `use_nontemporal_stores`, `single_registers`
  - These values will tune the generated assembly code for TrafficGen's bw saturation.

### Examples

> **First-time users:** If running on a machine for the first time, it's recommended to test with a single ratio and pause value with verbosity set to 3 to debug and verify the benchmark is working correctly:
>
> ```bash
> ./build/bin/mess --profile --ratio=100 --pause=0 --verbose=3
> ```

```bash
# Basic benchmark with default settings (full size, ratio 50)
./build/bin/mess

# Single ratio with profiling (saves to files)
./build/bin/mess --ratio=50 --profile

# Dry run to check system configuration
./build/bin/mess --dry-run --verbose 2

# Quick test with small problem size
./build/bin/mess --ratio=25 --verbose 1

# Custom pause values and repetitions (advanced options)
./build/bin/mess --ratio=75 --pause=0,50,500,5000 --repetitions=5 --profile
```

## Mess Profiler

Mess Profiler is a unified memory bandwidth profiling tool. It automatically detects your system and selects the appropriate measurement backend (`perf`, `likwid`, or Intel PCM), parses the output, and converts it to a consistent format. This means you get the same output columns regardless of the underlying tool used. It also automatically inherits CPU/memory bindings from `numactl` or `taskset`.

### Usage

```bash
./build/bin/mess-profiler [options] [--] <command> [args...]
```

### Measurement Options

| Option                  | Description                                                    |
| ----------------------- | -------------------------------------------------------------- |
| `-s, --interval <time>` | Sampling interval (e.g., `100ms`, `1s`). Default: summary mode |
| `-o, --output <file>`   | Output file. Default: stdout                                   |

### Targeting Options

| Option               | Description                                             |
| -------------------- | ------------------------------------------------------- |
| `-a, --system-wide`  | System-wide profiling (all CPUs/sockets)                |
| `-p, --pid <pid>`    | Profile existing process by PID                         |
| `-C, --cpu <list>`   | Profile only specified CPUs (e.g., `0-7,16-23`)         |
| `-N, --nodes <list>` | Monitor memory traffic to specified NUMA nodes          |
| `--no-inherit`       | Don't inherit binding from parent (`numactl`/`taskset`) |

### Output Options

| Option             | Description                              |
| ------------------ | ---------------------------------------- |
| `-v, --verbose`    | Verbose output (show counter details)    |
| `--csv`            | CSV output (default)                     |
| `--human`          | Human-readable output                    |
| `--dry, --dry-run` | Dry run: show detected counters and exit |

### Examples

```bash
# Basic profiling with 100ms sampling
./build/bin/mess-profiler -s 100ms ./my_app

# Save output to file
./build/bin/mess-profiler -s 50ms -o bandwidth.csv ./my_app

# System-wide profiling
./build/bin/mess-profiler -a -s 1s sleep 10

# Works with numactl - automatically detects binding
numactl -m 0 -C 0-7 ./build/bin/mess-profiler -s 100ms ./my_app

# Explicit CPU and memory node targeting
./build/bin/mess-profiler -C 0-7 -N 0 -s 100ms ./my_app

# Check what counters will be used
./build/bin/mess-profiler --dry-run
```

### Output Format

The profiler outputs CSV with the following columns:

- `Timestamp(s)`: Time since start
- `Bandwidth(GB/s)`: Measured memory bandwidth
- `ReadBytes`: Bytes read from memory
- `WriteBytes`: Bytes written to memory

Use `--dry-run` to see exactly what counters and configuration will be used on your system.

## Plotting Utilities

The `utils/` directory contains Python scripts for visualizing Mess benchmark results. See [utils/README.md](utils/README.md) for detailed documentation.

Key utilities include:

- `plotter.py`: Generate bandwidth-latency curve visualizations from measurement data
- `app_plotter.py`: Overlay application profiler data on memory curves
- `parse_runtimes.py`: Summarize benchmark execution times

## Changelog

See [CHANGELOG.md](CHANGELOG.md) for details on new features and fixes.

## Contributors

For issues or questions, contact the maintainers:

<table>
  <tr>
    <td align="center" width="150" style="vertical-align: top;">
      <sub><b>Victor Xirau Guardans</b><br/>Main Mess 2.0 developer<br/><a href="mailto:victor.xirau@bsc.es">victor.xirau@bsc.es</a></sub>
    </td>
       <td align="center" width="150" style="vertical-align: top;">
      <sub><b>Mariana Carmin</b><br/>Mess 2.0 developer<br/><a href="mailto:mcarmin@bsc.es">mcarmin@bsc.es</a></sub>
    </td>
    <td align="center" width="150" style="vertical-align: top;">
      <sub><b>Pouya Esmaili Dokht</b><br/>Main Mess 1.0 developer<br/><a href="mailto:pouya.esmaili@bsc.es">pouya.esmaili@bsc.es</a></sub>
    </td>
  </tr>
</table>

Or send an email to [mess@bsc.es](mailto:mess@bsc.es?subject=[Mess2.0]%20Request)

## References

1. **[Mess Benchmark](https://github.com/bsc-mem/Mess-benchmark)** - The original implementation of the Mess benchmark.
2. **[Mess Simulator](https://github.com/bsc-mem/Mess-simulator)** - Analytical memory model using bandwidth-latency curves.
3. **[Mess-Paraver](https://github.com/bsc-mem/Mess-Paraver)** - Integration with Paraver for visualization.
4. **[Mess Paper](https://mess.bsc.es)** - Esmaili-Dokht, P., Sgherzi, F., Girelli, V. S., Boixaderas, I., Carmin, M., Monemi, A., Armejach, A., Mercadal, E., Llort, G., Radojković, P., Moreto, M., Giménez, J., Martorell, X., Ayguadé, E., Labarta, J., Confalonieri, E., Dubey, R., & Adlard, J. (2024). A mess of memory system benchmarking, simulation and application profiling. In _Proceedings of the 57th IEEE/ACM International Symposium on Microarchitecture_ (MICRO) (pp. 136-152). IEEE.
