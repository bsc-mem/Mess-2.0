# Copyright (c) 2026, Barcelona Supercomputing Center
# Contact: mess             [at] bsc [dot] es
#          victor.xirau     [at] bsc [dot] es
#          petar.radojkovic [at] bsc [dot] es
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright notice,
#       this list of conditions and the following disclaimer.
#
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#
#     * Neither the name of the copyright holder nor the names
#       of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# Mess 2.0 Makefile

CXX = g++
CC = gcc

JOBS ?= $(shell nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 1)

ARCH := $(shell uname -m)

ifeq ($(ARCH),riscv64)
    OPTFLAGS = -O3 -march=rv64g
else
    OPTFLAGS = -O3 -march=native
endif

OPTFLAGS += -funroll-loops -ffast-math -fomit-frame-pointer
OPTFLAGS += -finline-functions

GCC_VERSION := $(shell $(CXX) -dumpversion | cut -d. -f1)
GCC_MINOR := $(shell $(CXX) -dumpversion | cut -d. -f2)

ifeq ($(shell [ $(GCC_VERSION) -lt 5 ] 2>/dev/null && echo 1 || echo 0), 1)
CXXSTD = -std=c++11
else ifeq ($(shell [ $(GCC_VERSION) -eq 5 ] 2>/dev/null && echo 1 || echo 0), 1)
CXXSTD = -std=c++14
else ifeq ($(shell [ $(GCC_VERSION) -eq 6 ] 2>/dev/null || [ $(GCC_VERSION) -eq 7 ] 2>/dev/null && echo 1 || echo 0), 1)
CXXSTD = -std=c++14
else ifeq ($(shell [ $(GCC_VERSION) -ge 8 ] 2>/dev/null && echo 1 || echo 0), 1)
CXXSTD = -std=c++17
else
CXXSTD = -std=c++11
endif

CXXFLAGS = $(CXXSTD) -Wall -Wextra -Wunused $(OPTFLAGS) -pthread -I. -I./include -I./src -I./src/libraries -I./include/arch
CFLAGS = $(OPTFLAGS) -I. -I./include -I./src -I./src/libraries -I./include/arch
LDFLAGS = -pthread -lm
LDFLAGS += $(shell [ $(GCC_VERSION) -lt 9 ] 2>/dev/null && echo "-lstdc++fs" || true)
BUILD_DIR = build
BIN_DIR = $(BUILD_DIR)/bin
LIB_DIR = $(BUILD_DIR)/lib

SRCS = Mess.cpp \
       src/SystemDetection.cpp \
       src/BenchmarkConfig.cpp \
       src/CliParser.cpp \
       src/KernelGenerator.cpp \
       src/measurement/MeasurementStorage.cpp \
       src/measurement/LatencyMeasurer.cpp \
       src/measurement/OutlierDetector.cpp \
       src/measurement/MeasurementUtils.cpp \
       src/measurement/BandwidthMeasurerFactory.cpp \
       src/measurement/bw_measurers/PerfBandwidthMeasurer.cpp \
       src/measurement/bw_measurers/LikwidBandwidthMeasurer.cpp \
       src/measurement/bw_measurers/VtuneBandwidthMeasurer.cpp \
       src/measurement/bw_measurers/PcmBandwidthMeasurer.cpp \
       src/process/TrafficGenProcessManager.cpp \
       src/process/PtrchaseProcessManager.cpp \
       src/utils/ProgressTracker.cpp \
       src/utils/PmuSysfs.cpp \
       src/utils/CpuTopology.cpp \
       src/utils/SubprocessCapture.cpp \
       src/utils/TerminalCursor.cpp \
       src/BenchmarkExecutor.cpp \
       src/ResultsProcessor.cpp \
       src/TlbUtils.cpp \
       src/Utils.cpp \
       src/PtrchasePerfHelper.cpp \
       src/CurveTracer.cpp
SRCS += src/measurement/instruction_samplers/IntelPebsSampler.cpp \
        src/measurement/instruction_samplers/ArmSpeSampler.cpp \
        src/measurement/InstructionSamplerFactory.cpp

HEADERS = include/BenchmarkConfig.h \
          include/BenchmarkExecutor.h \
          include/CliParser.h \
          include/Codegen.h \
          include/Measurement.h \
          include/ProcessManager.h \
          include/ResultsProcessor.h \
          include/SystemDetection.h \
          include/Utils.h \
          include/CurveTracer.h

LIBS = src/SystemInfo.o

src/%.o: src/%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

OBJS = $(SRCS:.cpp=.o) GenerateCode.o

TARGETS = $(BIN_DIR)/mess $(BIN_DIR)/generate_code $(BIN_DIR)/mess-profiler

all:
	@echo "Detected GCC version: $(GCC_VERSION).$(GCC_MINOR)"
	@echo "Using C++ standard: $(CXXSTD)"
	@if [ $(GCC_VERSION) -lt 8 ]; then \
		echo ""; \
		printf "\033[0;33mWARNING: GCC $(GCC_VERSION).$(GCC_MINOR) does not fully support C++17!\033[0m\n"; \
		echo "This code requires C++17 features. Please use a newer compiler:"; \
		echo ""; \
		if command -v module >/dev/null 2>&1; then \
			echo "Available compiler modules:"; \
			module avail gcc 2>/dev/null | grep -E "gcc/(1[0-9]|[2-9][0-9])" | head -10 || echo "  Run 'module avail gcc' to see available compilers"; \
		else \
			echo "To check available compilers, try:"; \
			echo "  which gcc g++"; \
			echo "  ls /usr/bin/gcc* /usr/bin/g++* 2>/dev/null"; \
		fi; \
		echo ""; \
		printf "\033[0;31mRequired: GCC 8.0 or later for C++17 support\033[0m\n"; \
		echo ""; \
		echo "Continuing with $(CXXSTD), but compilation may fail..."; \
		echo ""; \
	fi
	$(MAKE) -j$(JOBS) $(TARGETS)
	@echo "Build completed successfully!"
	@echo "Cleaning up all object files..."
	@rm -f $(SRCS:.cpp=.o) $(LIBS) GenerateCode.o

parallel: 
	$(MAKE) -j$(JOBS) all

$(BIN_DIR):
	mkdir -p $(BIN_DIR)

ARCH_SRCS = src/arch/ArchitectureRegistry.cpp \
            src/arch/BandwidthCounterStrategy.cpp \
            src/arch/x86/X86Architecture.cpp \
            src/arch/x86/X86Assembler.cpp \
            src/arch/x86/X86Counters.cpp \
            src/arch/x86/counters/IntelSkylakeCounters.cpp \
            src/arch/x86/counters/IntelSPRCounters.cpp \
            src/arch/x86/counters/IntelEmeraldRapidsCounters.cpp \
            src/arch/x86/counters/IntelGraniteRapidsCounters.cpp \
            src/arch/x86/counters/AmdZenCounters.cpp \
            src/arch/x86/counters/AmdZen4Counters.cpp \
            src/arch/arm/ArmArchitecture.cpp \
            src/arch/arm/ArmISAUtils.cpp \
            src/arch/arm/ArmAssembler.cpp \
            src/arch/arm/ArmCounters.cpp \
            src/arch/arm/counters/A64FXCounters.cpp \
            src/arch/arm/counters/Graviton3Counters.cpp \
            src/arch/arm/counters/NvidiaGraceCounters.cpp \
            src/arch/powerpc/PowerPCArchitecture.cpp \
            src/arch/powerpc/PowerPCAssembler.cpp \
            src/arch/powerpc/PowerPCCounters.cpp \
            src/arch/riscv/RiscvArchitecture.cpp \
            src/arch/riscv/RiscvAssembler.cpp \
            src/arch/riscv/RiscvCounters.cpp \
            src/arch/riscv/counters/SiFiveCounters.cpp
       
ARCH_OBJS = $(ARCH_SRCS:.cpp=.o)

$(LIB_DIR):
	mkdir -p $(LIB_DIR)


$(BIN_DIR)/mess: $(SRCS:.cpp=.o) $(ARCH_OBJS) $(LIBS) | $(BIN_DIR)
	$(CXX) $(CXXFLAGS) -o $@ $(SRCS:.cpp=.o) $(ARCH_OBJS) $(LIBS) $(LDFLAGS)


$(BIN_DIR)/generate_code: src/SystemDetection.o src/KernelGenerator.o GenerateCode.o src/TlbUtils.o src/PtrchasePerfHelper.o src/Utils.o src/utils/PmuSysfs.o src/utils/CpuTopology.o src/utils/SubprocessCapture.o $(ARCH_OBJS) $(LIBS) | $(BIN_DIR)
	$(CXX) $(CXXFLAGS) -o $@ src/SystemDetection.o src/KernelGenerator.o GenerateCode.o src/TlbUtils.o src/PtrchasePerfHelper.o src/Utils.o src/utils/PmuSysfs.o src/utils/CpuTopology.o src/utils/SubprocessCapture.o $(ARCH_OBJS) $(LIBS) $(LDFLAGS)


MESS_PROFILER_OBJS = src/SystemDetection.o \
                     src/BenchmarkConfig.o \
                     src/measurement/MeasurementUtils.o \
                     src/measurement/BandwidthMeasurerFactory.o \
                     src/measurement/bw_measurers/PerfBandwidthMeasurer.o \
                     src/measurement/bw_measurers/LikwidBandwidthMeasurer.o \
                     src/measurement/bw_measurers/VtuneBandwidthMeasurer.o \
                     src/measurement/bw_measurers/PcmBandwidthMeasurer.o \
                     src/measurement/MeasurementStorage.o \
                     src/process/TrafficGenProcessManager.o \
                     src/profiler/ProcessBinding.o \
                     src/profiler/ProfilerConfig.o \
                     src/utils/PmuSysfs.o \
                     src/utils/CpuTopology.o \
                     src/utils/SubprocessCapture.o \
                     src/Utils.o

$(BIN_DIR)/mess-profiler: MessProfiler.cpp $(MESS_PROFILER_OBJS) $(ARCH_OBJS) $(LIBS) | $(BIN_DIR)
	$(CXX) $(CXXFLAGS) -o $@ MessProfiler.cpp $(MESS_PROFILER_OBJS) $(ARCH_OBJS) $(LIBS) $(LDFLAGS)


Mess.o: Mess.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c Mess.cpp -o $@

GenerateCode.o: GenerateCode.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c GenerateCode.cpp -o $@


%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@


clean:
	rm -f $(OBJS) $(ARCH_OBJS) $(LIBS) $(TARGETS)
	rm -rf $(BUILD_DIR)/
	rm -rf src/traffic_gen/build
	rm -rf src/traffic_gen/src/nop.c
	rm -rf src/traffic_gen/src/nop_*.c
	rm -rf src/traffic_gen/src/generated_*.c
	rm -rf src/traffic_gen/src/utils_*.c
	rm -f *.o src/arch/*.o src/arch/*/*.o src/profiler/*.o
	rm -rf build/*


distclean: clean
	rm -f *.log *.csv *.json


install:
	@echo "Running code generation..."
	@./$(BIN_DIR)/generate_code $(DEBUG_FLAGS)


test: $(TARGETS)
	$(BIN_DIR)/mess --help

debug: CXXFLAGS = $(CXXSTD) -Wall -Wextra -g -O0 -pthread
debug: $(TARGETS)

# Dependencies
Mess.o: include/SystemDetection.h include/BenchmarkConfig.h include/CliParser.h include/Codegen.h include/BenchmarkExecutor.h include/ResultsProcessor.h include/Utils.h

GenerateCode.o: include/SystemDetection.h include/Codegen.h include/Utils.h

src/SystemDetection.o: include/SystemDetection.h

src/BenchmarkConfig.o: include/BenchmarkConfig.h

src/CliParser.o: include/CliParser.h include/BenchmarkConfig.h

src/KernelGenerator.o: include/Codegen.h include/SystemDetection.h

src/CurveTracer.o: include/CurveTracer.h

src/BenchmarkExecutor.o: include/BenchmarkExecutor.h include/BenchmarkConfig.h include/Codegen.h include/CurveTracer.h include/Measurement.h include/ProcessManager.h include/Utils.h

src/ResultsProcessor.o: include/ResultsProcessor.h include/BenchmarkConfig.h include/BenchmarkExecutor.h

.PHONY: all clean distclean install test debug
