# Changelog

## [2.0.2] - 09-02-2026
_February 9th 2026_

### Added

- **x86 Scalar Support**: Added a scalar fallback backend for x86, enabling baseline performance comparisons and broader compatibility with legacy hardware.
- **Improved bandwidth measurer**: Enhanced bandwidth measurement infrastructure with better separation of concerns.

### CLI Changes

- **New `--measurer` Flag**: Replaces deprecated `--likwid` flag with explicit tool selection:
  - `--measurer=auto` (default): Automatically select best available tool
  - `--measurer=perf`: Use Linux perf for standard DDR systems
  - `--measurer=likwid`: Use LIKWID for HBM systems
  
- **New `--add-counters` Flag**: Replaces deprecated `--extra-perf` flag:
  - Same functionality: add extra performance counters to measurements
  - More consistent naming with other CLI options
  - Currently only supports perf counters

### Improved

- **Code Maintainability**: Refactored `BandwidthCounterStrategy` and related components to improve code organization and readability, making the codebase easier to maintain and extend.
- **Bandwidth Stability**: Enhanced stability detection logic to properly handle bandwidth ramp-up phases and skip array initialization artifacts, resulting in more reliable steady-state measurements.

### Fixed

- **ARM SVE Assembly**: Resolved instruction scheduling errors in the ARM SVE backend that affected data validity on SVE-capable systems.
- **General Stability**: Fixed minor bugs in argument parsing and topology detection for cleaner execution flows.

### Deprecated

- `--likwid`: Use `--measurer=likwid` instead
- `--extra-perf`: Use `--add-counters` instead

## [2.0.1] - 12-01-2026

_January 12th 2026_

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

_December 18th 2025_

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
