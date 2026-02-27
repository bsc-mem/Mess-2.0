<div align="center">
  <img src="doc/logos/512_mess_white.png" alt="Mess Benchmark Logo" width="180">
  <h1>Mess Benchmark 2.0</h1>
  <p><strong>A multiplatform benchmark designed to provide a holistic, detailed, and close-to-hardware view of memory system performance through bandwidth-latency curves.</strong></p>
  <p>
    This is an update to the original
    <a href="https://github.com/bsc-mem/Mess-benchmark">Mess Benchmark</a>
    (<em>now deprecated</em>), focused on improved usability and portability.
  </p>
  <p>
    <a href="https://mess.bsc.es">Website</a> &nbsp;|&nbsp;
    <a href="https://github.com/bsc-mem/Mess-2.0">GitHub</a> &nbsp;|&nbsp;
    <a href="https://arxiv.org/pdf/2405.10170">Paper</a>
  </p>
  <p>
    <img src="https://img.shields.io/badge/License-BSD%203--Clause-blue.svg" alt="License">
    <img src="https://img.shields.io/badge/Version-2.0.2-blue" alt="Version">
  </p>
</div>

## Table of Contents

- [Documentation](#documentation)
- [Motivation](#motivation)
- [Tools Included](#tools-included)
- [Architecture Support](#architecture-support)
- [Installation](#installation)
  - [Clone](#clone)
  - [Dependencies](#dependencies)
  - [Build](#build)
  - [Verification](#verification)
- [Quick Start](#quick-start)
- [Common Workflows](#common-workflows)
- [Mess Profiler](#mess-profiler)
- [Plotter-Parser](#plotter-parser)
- [Learning Resources](#learning-resources)
- [Troubleshooting](#troubleshooting)
- [Contributors](#contributors)
- [Citation](#citation)
- [References](#references)

---

## Documentation

Project documentation is available in the GitHub wiki:

- [Wiki Home](https://github.com/bsc-mem/Mess-2.0/wiki/Home)
- [Installation](https://github.com/bsc-mem/Mess-2.0/wiki/Installation)
- [Understanding CLI Arguments](https://github.com/bsc-mem/Mess-2.0/wiki/Understanding-CLI-Arguments)
- [Mess Benchmark](https://github.com/bsc-mem/Mess-2.0/wiki/Mess-Benchmark)
- [Mess Profiler](https://github.com/bsc-mem/Mess-2.0/wiki/Mess-Profiler)
- [Plotter-Parser](https://github.com/bsc-mem/Mess-2.0/wiki/Plotter-Parser)
- [Architecture Support](https://github.com/bsc-mem/Mess-2.0/wiki/Architecture-Support)
- [FAQ](https://github.com/bsc-mem/Mess-2.0/wiki/FAQ)

---

## Motivation

Traditional memory benchmarks report isolated metrics such as peak bandwidth or idle latency, which often fail to capture how memory systems behave under realistic workloads. **Mess** (Memory Stress) addresses this limitation by characterizing memory performance through **bandwidth-latency curves** that cover the full range of memory traffic intensity, from unloaded to fully saturated.

This approach reveals critical insights:

- Memory writes degrade performance significantly compared to reads
- Systems typically saturate at 70-90% of theoretical maximum bandwidth
- Latency ranges from 85-130ns when idle to 200-600ns+ under saturation

Mess provides a holistic, close-to-hardware view of memory system behavior, enabling researchers and engineers to understand real-world performance characteristics that standard benchmarks miss.

> **MICRO 2024 Best Paper Runner-Up**: The Mess methodology was published at the 57th IEEE/ACM International Symposium on Microarchitecture.

For a detailed explanation of the benchmark methodology, see the [Memory BSC Tools page](https://memory.bsc.es/tools/mess-benchmark).

---

## Tools Included

Mess 2.0 provides an integrated workflow for memory system characterization, from benchmarking to application profiling:

- **[Mess Benchmark](https://github.com/bsc-mem/Mess-2.0/wiki/Mess-Benchmark)**: Characterizes your memory system by generating bandwidth-latency curves that reveal how it behaves under varying load.
- **[Mess Profiler](https://github.com/bsc-mem/Mess-2.0/wiki/Mess-Profiler)**: Automates counter discovery and runs profiling tools (`perf`, `likwid`, etc.) with the correct configuration, ensuring application measurements align with benchmark data.
- **[Plotter-Parser](https://github.com/bsc-mem/Mess-2.0/wiki/Plotter-Parser)**: Generates publication-quality plots as well as CSV and JSON files containing the parsed bandwidth-latency curves.
- **[Traffic Generator](https://github.com/bsc-mem/Mess-2.0/wiki/Traffic-Generator)**: The low-level engine that generates precise memory traffic patterns at the assembly level. Can also be used independently for custom microbenchmarks.

---

## Architecture Support

Support status follows the wiki ([Architecture-Support](https://github.com/bsc-mem/Mess-2.0/wiki/Architecture-Support)):

| Architecture | Status | SIMD | Notes |
| --- | --- | --- | --- |
| x86-64 CPUs | Supported | AVX2, AVX-512 | Intel and AMD processors |
| ARM CPUs | Supported | NEON, SVE | Includes Neoverse, Graviton, Apple Silicon |
| Power CPUs | Supported | VSX | Power8 and newer |
| RISC-V CPUs | WIP | RVV 1.0 | Assembly + latency + counter detection available; bandwidth measurement pending |
| GPUs | Pending | — | Under active development |

---

## Installation

See full instructions in the [Installation wiki page](https://github.com/bsc-mem/Mess-2.0/wiki/Installation).

### Clone

```bash
git clone --recursive https://github.com/bsc-mem/Mess-2.0.git
cd Mess-2.0
```

**Important**: The `--recursive` flag is required to download submodules that Mess depends on.

If you forget `--recursive`, initialize submodules later:

```bash
git submodule update --init --recursive
```

### Dependencies

Core requirements:

- **C++17 compiler**: GCC 9+ (recommended), Clang 10+, Intel OneAPI (ICX), AOCC
- **numactl**: NUMA memory binding (required)
- **taskset**: Core pinning (preferred, part of `util-linux`)
- **perf**: Recommended counter backend (`linux-tools-common`)
- **Python 3**: Plotting utilities

On Linux, ensure perf access:

```bash
cat /proc/sys/kernel/perf_event_paranoid
echo 0 | sudo tee /proc/sys/kernel/perf_event_paranoid
```

Huge pages are recommended for more accurate measurements:

```bash
echo 1024 | sudo tee /proc/sys/vm/nr_hugepages
```

> Even without huge pages, Mess automatically compensates for page walk latency. See [Huge-Memory-Pages](https://github.com/bsc-mem/Mess-2.0/wiki/Huge-Memory-Pages) for details.

### Build

```bash
make
make install
```

Binaries are generated in `build/bin/`:

- `mess` — Core benchmark
- `mess-profiler` — Memory bandwidth profiler
- `traffic_generator` — Standalone traffic generation tool

Optional PATH setup:

```bash
export PATH=$PATH:$(pwd)/build/bin
```

### Verification

```bash
./build/bin/mess --version
./build/bin/mess --dry-run --verbose=2
```

---

## Quick Start

```bash
./build/bin/mess --dry-run --verbose=2

./build/bin/mess

./build/bin/mess --profile
```

Common options:

| Option | Description | Example |
| --- | --- | --- |
| `--ratio=N[,N...]` | Issued load ratio(s) in % | `--ratio=100,75,50` |
| `--pause=N[,N...]` | Pause bubble values | `--pause=0,10,100,1000` |
| `--profile` | Save measurement files | `--profile` |
| `--verbose=N` | Verbosity level `0-4` | `--verbose=3` |
| `--measurer=TYPE` | Counter backend (`auto/perf/likwid/pcm`) | `--measurer=perf` |
| `--bind=LIST` | NUMA memory-node binding | `--bind=0` |
| `--cores=LIST` | Explicit traffic-generator cores | `--cores=0-15` |
| `--total-cores=N` | Number of traffic-generator cores | `--total-cores=16` |

For the complete option set, use `./build/bin/mess --help` or see [Understanding-CLI-Arguments](https://github.com/bsc-mem/Mess-2.0/wiki/Understanding-CLI-Arguments).

---

## Common Workflows

### Single-Point Sanity Check

```bash
./build/bin/mess --profile --ratio=100 --pause=0 --verbose=3 --repetitions=1
./build/bin/mess --profile --ratio=0 --pause=0 --verbose=3 --repetitions=1
```

### NUMA Comparison

```bash
./build/bin/mess --profile --bind=0 --folder=numa0
./build/bin/mess --profile --bind=1 --folder=numa1
```

### Core-Scaling Sweep

```bash
for c in 2 4 8 16; do
  ./build/bin/mess --profile --total-cores=$c --folder=cores_$c
done
```

---

## Mess Profiler

`mess-profiler` reuses Mess counter discovery to profile applications with consistent output.

```bash
./build/bin/mess-profiler --dry-run

./build/bin/mess-profiler -s 100ms -o app_profile.csv ./my_app
```

Profiler docs: [Mess-Profiler](https://github.com/bsc-mem/Mess-2.0/wiki/Mess-Profiler)

---

## Plotter-Parser

Visualization utilities in `utils/`:

- `plotter.py`: generates memory-curve plots and processed CSV/JSON
- `app_plotter.py`: overlays application profile points on curves
- `parse_runtimes.py`: summarizes run times

```bash
cd utils
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

Plotter docs: [Plotter-Parser](https://github.com/bsc-mem/Mess-2.0/wiki/Plotter-Parser)

---

## Learning Resources

- **Tutorials and Slides**: [mess.bsc.es/tutorials](https://mess.bsc.es/tutorials)
- **Detailed Methodology**: [memory.bsc.es/tools/mess-benchmark](https://memory.bsc.es/tools/mess-benchmark)

---

## Troubleshooting

- Permission errors on counters: set `perf_event_paranoid` to `0`
- Missing counters/backend mismatch: check `./build/bin/mess-profiler --dry-run`
- Unstable measurements: increase `--repetitions` and use `--verbose=3`

More: [FAQ](https://github.com/bsc-mem/Mess-2.0/wiki/FAQ) and [Iterative-Debugging](https://github.com/bsc-mem/Mess-2.0/wiki/Iterative-Debugging)

**Found a bug?** Open an issue on [GitHub](https://github.com/bsc-mem/Mess-2.0/issues) or email [mess@bsc.es](mailto:mess@bsc.es?subject=[Mess2.0]%20Bug%20Report).

---

## Contributors

Mess is developed by the **[Memory Systems Team](https://memory.bsc.es)** at the Barcelona Supercomputing Center (BSC).

<table>
  <tr>
    <td align="center" width="160" style="vertical-align: top;">
      <sub><b>Victor Xirau Guardans</b><br/>Main Mess 2.0 developer<br/><a href="mailto:victor.xirau@bsc.es">victor.xirau@bsc.es</a></sub>
    </td>
    <td align="center" width="160" style="vertical-align: top;">
      <sub><b>Mariana Carmin</b><br/>Mess 2.0 developer<br/><a href="mailto:mcarmin@bsc.es">mcarmin@bsc.es</a></sub>
    </td>
    <td align="center" width="160" style="vertical-align: top;">
      <sub><b>Pau Diaz</b><br/>Mess 2.0 developer<br/><a href="mailto:pau.diazcuesta@bsc.es">pau.diazcuesta@bsc.es</a></sub>
    </td>
    <td align="center" width="160" style="vertical-align: top;">
      <sub><b>Pouya Esmaili Dokht</b><br/>Mess Paper author<br/><a href="mailto:pouya.esmaili@bsc.es">pouya.esmaili@bsc.es</a></sub>
    </td>
  </tr>
</table>

Or email: [mess@bsc.es](mailto:mess@bsc.es?subject=[Mess2.0]%20Request)

---

## Citation

If you use Mess in research, please cite:

```bibtex
@inproceedings{esmaili2024mess,
  title     = {A Mess of Memory System Benchmarking, Simulation and Application Profiling},
  author    = {Esmaili-Dokht, Pouya and Sgherzi, Francesco and Girelli, Valeria Soldera
               and Boixaderas, Isaac and Carmin, Mariana and Monemi, Alireza
               and Armejach, Adria and Mercadal, Estanislao and Llort, German
               and Radojkovi{\'c}, Petar and Moreto, Miquel and Gim{\'e}nez, Judit
               and Martorell, Xavier and Ayguad{\'e}, Eduard and Labarta, Jesus
               and Confalonieri, Emanuele and Dubey, Rishabh and Adlard, Joshua},
  booktitle = {Proceedings of the 57th IEEE/ACM International Symposium on Microarchitecture (MICRO)},
  pages     = {136--152},
  year      = {2024},
  publisher = {IEEE}
}
```

---

## References

1. **[Mess Benchmark](https://github.com/bsc-mem/Mess-benchmark)** — The original implementation of the Mess benchmark.
2. **[Mess Simulator](https://github.com/bsc-mem/Mess-simulator)** — Analytical memory model using bandwidth-latency curves.
3. **[Mess-Paraver](https://github.com/bsc-mem/Mess-Paraver)** — Integration with Paraver for visualization.
4. **[Mess Paper](https://mess.bsc.es)** — Esmaili-Dokht, P., Sgherzi, F., Girelli, V. S., Boixaderas, I., Carmin, M., Monemi, A., Armejach, A., Mercadal, E., Llort, G., Radojković, P., Moreto, M., Giménez, J., Martorell, X., Ayguadé, E., Labarta, J., Confalonieri, E., Dubey, R., & Adlard, J. (2024). A mess of memory system benchmarking, simulation and application profiling. In _Proceedings of the 57th IEEE/ACM International Symposium on Microarchitecture_ (MICRO) (pp. 136-152). IEEE.

---

<p align="center">
  <sub>Mess Benchmark is released under the BSD 3-Clause License</sub>
</p>
