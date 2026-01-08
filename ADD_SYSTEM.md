# Adding a New System / Architecture

> [!NOTE]
> **Adding a new system should not be necessary**, since Mess 2.0 automatically detects every configuration.
> 
> From time to time, we will encounter some exotic system we didn't account for. In that case, please use this guide.

This guide explains how to add support for a new CPU architecture or ISA to the Mess Benchmark.

## Overview

The benchmark uses a plugin-based architecture system. To add a new system, you need to implement three main interfaces:
1. **Architecture**: The main entry point and factory.
2. **KernelAssembler**: Handles assembly code generation.
3. **PerformanceCounterStrategy**: Handles hardware performance counter discovery.

## Step-by-Step Guide

### 1. Create Architecture Files

Create a new directory in `src/arch/<arch_name>/` and `include/arch/<arch_name>/`.

#### Header Files (`include/arch/<arch_name>/`)

Create `<ArchName>Architecture.h`:
```cpp
#include "architecture/Architecture.h"

class MyArch : public Architecture {
public:
    std::string getName() const override { return "myarch"; }
    bool supports(const CPUCapabilities& caps) const override;
    
    std::unique_ptr<KernelAssembler> createAssembler(const KernelConfig& config) const override;
    std::unique_ptr<PerformanceCounterStrategy> createCounterStrategy(const CPUCapabilities& caps) const override;
    
    std::vector<std::shared_ptr<ISA>> getSupportedISAs() const override;
    std::shared_ptr<ISA> selectBestISA(const CPUCapabilities& caps) const override;
    
    std::string generateNopFile() const override;
};
```

Create `<ArchName>Assembler.h`:
```cpp
#include "architecture/KernelAssembler.h"

class MyAssembler : public KernelAssembler {
public:
    MyAssembler(const KernelConfig& config);
    // Implement pure virtual methods: generateLoad, generateStore, etc.
};
```

### 2. Implement Logic (`src/arch/<arch_name>/`)

Implement the classes defined above.

**Key Implementation Details:**
- **Assembler**: Must generate valid assembly for the target architecture. Use the provided `config` to handle unrolling and register usage.
- **Counters**: Implement `detectCasCounters` and `getTlbMissCounters` to map abstract events to hardware-specific raw event codes.
- **ISA**: Define supported ISAs (e.g., AVX, NEON) and their vector widths.

### 3. Register the Architecture

In your `<ArchName>Architecture.cpp`, add the registration macro at the end:

```cpp
#include "architecture/ArchitectureRegistry.h"

// ... implementation ...

static ArchitectureRegistrar<MyArch> my_arch_registrar;
```

This automatically registers your architecture with the system.

### 4. Update Build System

Add your new source files to `Makefile`:

```makefile
ARCH_SRCS += src/arch/<arch_name>/<ArchName>Architecture.cpp \
             src/arch/<arch_name>/<ArchName>Assembler.cpp
```

*Note: In the main `Makefile`, find the `ARCH_SRCS` variable definition and append your files there.*

### 5. Verify

Run `make clean && make` and test with `./build/bin/mess --dry-run`.

## Adding a Microarchitecture (Variant)

To add a specific variant (e.g., Zen4 for x86, or Neoverse for ARM):

1.  **Modify the Architecture Class**:
    In your `createAssembler` or `createCounterStrategy` methods, check `caps.model_name` or `caps.uarch` (if available) to return a specialized subclass.

    ```cpp
    std::unique_ptr<KernelAssembler> X86Architecture::createAssembler(const KernelConfig& config) const {
        if (config.uarch == "zen4") {
            return std::make_unique<Zen4Assembler>(config);
        }
        return std::make_unique<X86Assembler>(config);
    }
    ```

2.  **Implement Specialized Classes**:
    Inherit from the base architecture assembler/counters and override only what's needed.

    ```cpp
    class Zen4Assembler : public X86Assembler {
        // Override specific instruction generation
    };
    ```
