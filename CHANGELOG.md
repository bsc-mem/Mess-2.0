# Changelog

## [2.0.1] - 12-01-2026

### Added

- **Plotting Utilities**: New Python scripts in `utils/` directory for visualizing benchmark results
  - `app_plotter.py`: Overlay application profiler data on memory curves

### Improved

- **mess-profiler**: Enhanced unified memory bandwidth profiling tool
  - Automatic backend detection (`perf`, `likwid`, or Intel PCM)
  - Consistent output format regardless of underlying tool
  - Automatic inheritance of CPU/memory bindings from `numactl` or `taskset`
  - Added `--dry-run` option to preview counter configuration

## [2.0.0] - 18-12-2025

Presenting **Mess 2.0**, a new approach to using Mess that preserves its logical core while completely rewriting the execution engine to be faster, more portable, and easier to use.

Key highlights:

- **Rewritten entirely in C++**: Native performance and type safety.
- **84x Faster**: Massive validation speedup compared to v1.0.
- **Zero Setup Cost**: No complex dependencies, just plug and play.
- **Universal Portability**: Compiles and runs on all supported ISAs (x86, ARM, Power, RISC-V).
- **Fully Configurable**: All parameters adjustable via CLI arguments.
- **Improved Experience**: More user-friendly with enhanced plotting logic.

### Added

- Rebuilt measurement workflow that keeps bandwidth/latency in sync, shrinking full-system runs from weeks to hours.
- Plain/Profile execution modes plus `--system`, `--ratio`, `--bind`, `--dry-run`, and extended verbosity controls for tailored runs.
- Automatic kernel generation driven by `KernelConfig`, enabling ISA-aware TrafficGenerator and pointer-chase code per architecture.
- Multi-architecture support (x86, ARM, Power, RISC-V) with auto-detected NUMA layouts and system characterization.
- Measurement storage outputs (profiling files, per-kernel data) for plotting bandwidth/latency curves.

### Compatibility

- Mess 1.0 curve formats stay supported so existing workflows continue to work.
