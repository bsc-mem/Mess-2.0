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

OPTFLAGS = -O3 -march=native
OPTFLAGS += -funroll-loops -ffast-math -fomit-frame-pointer
OPTFLAGS += -finline-functions

CXXFLAGS = -std=c++17 -Wall -Wextra $(OPTFLAGS) -pthread -I. -I./include -I./src -I./src/libraries -I./include/arch
CFLAGS = $(OPTFLAGS) -I. -I./include -I./src -I./src/libraries -I./include/arch
LDFLAGS = -pthread -lm
GCC_VERSION := $(shell $(CXX) -dumpversion | cut -d. -f1)
LDFLAGS += $(shell [ $(GCC_VERSION) -lt 9 ] 2>/dev/null && echo "-lstdc++fs" || true)
BUILD_DIR = build
BIN_DIR = $(BUILD_DIR)/bin
LIB_DIR = $(BUILD_DIR)/lib

SRCS = Mess.cpp \
       src/system_detection.cpp \
       src/benchmark_config.cpp \
       src/cli_parser.cpp \
       src/kernel_generator.cpp \
       src/measurement/measurement_storage.cpp \
       src/measurement/latency_measurer.cpp \
       src/measurement/outlier_detector.cpp \
       src/measurement/bandwidth_measurer.cpp \
       src/measurement/BandwidthMeasurerFactory.cpp \
       src/measurement/bw_measurers/PerfBandwidthMeasurer.cpp \
       src/measurement/bw_measurers/LikwidBandwidthMeasurer.cpp \
       src/measurement/bw_measurers/PcmBandwidthMeasurer.cpp \
       src/process/traffic_gen_process_manager.cpp \
       src/process/ptrchase_process_manager.cpp \
       src/utils/progress_tracker.cpp \
       src/benchmark_executor.cpp \
       src/results_processor.cpp \
       src/tlb_utils.cpp \
       src/utils.cpp \
       src/ptrchase_perf_helper.cpp

HEADERS = include/benchmark_config.h \
          include/benchmark_executor.h \
          include/cli_parser.h \
          include/codegen.h \
          include/measurement.h \
          include/process_manager.h \
          include/results_processor.h \
          include/system_detection.h \
          include/utils.h

LIBS = src/system_info.o

src/%.o: src/%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

OBJS = $(SRCS:.cpp=.o) generate_code.o

TARGETS = $(BIN_DIR)/mess $(BIN_DIR)/generate_code $(BIN_DIR)/mess-profiler

all:
	$(MAKE) -j$(JOBS) $(TARGETS)
	@echo "Build completed successfully!"
	@echo "Cleaning up all object files..."
	@rm -f $(SRCS:.cpp=.o) $(LIBS) generate_code.o

parallel: 
	$(MAKE) -j$(JOBS) all

$(BIN_DIR):
	mkdir -p $(BIN_DIR)

ARCH_SRCS = src/arch/ArchitectureRegistry.cpp \
            src/arch/PerformanceCounterStrategy.cpp \
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
            src/arch/arm/ArmAssembler.cpp \
            src/arch/arm/ArmCounters.cpp \
            src/arch/arm/counters/A64FXCounters.cpp \
            src/arch/arm/counters/Graviton3Counters.cpp \
            src/arch/arm/counters/NvidiaGraceCounters.cpp \
            src/arch/power/PowerArchitecture.cpp \
            src/arch/power/PowerAssembler.cpp \
            src/arch/power/PowerCounters.cpp \
            src/arch/riscv/RiscvArchitecture.cpp \
            src/arch/riscv/RiscvAssembler.cpp \
            src/arch/riscv/RiscvCounters.cpp \
            src/arch/riscv/counters/SiFiveCounters.cpp
       
ARCH_OBJS = $(ARCH_SRCS:.cpp=.o)

$(LIB_DIR):
	mkdir -p $(LIB_DIR)


$(BIN_DIR)/mess: $(SRCS:.cpp=.o) $(ARCH_OBJS) $(LIBS) | $(BIN_DIR)
	$(CXX) $(CXXFLAGS) -o $@ $(SRCS:.cpp=.o) $(ARCH_OBJS) $(LIBS) $(LDFLAGS)


$(BIN_DIR)/generate_code: src/system_detection.o src/kernel_generator.o generate_code.o src/tlb_utils.o src/ptrchase_perf_helper.o src/utils.o $(ARCH_OBJS) $(LIBS) | $(BIN_DIR)
	$(CXX) $(CXXFLAGS) -o $@ src/system_detection.o src/kernel_generator.o generate_code.o src/tlb_utils.o src/ptrchase_perf_helper.o src/utils.o $(ARCH_OBJS) $(LIBS) $(LDFLAGS)


MESS_PROFILER_OBJS = src/system_detection.o \
                     src/benchmark_config.o \
                     src/measurement/bandwidth_measurer.o \
                     src/measurement/BandwidthMeasurerFactory.o \
                     src/measurement/bw_measurers/PerfBandwidthMeasurer.o \
                     src/measurement/bw_measurers/LikwidBandwidthMeasurer.o \
                     src/measurement/bw_measurers/PcmBandwidthMeasurer.o \
                     src/measurement/measurement_storage.o \
                     src/process/traffic_gen_process_manager.o \
                     src/utils.o

$(BIN_DIR)/mess-profiler: mess_profiler.cpp $(MESS_PROFILER_OBJS) $(ARCH_OBJS) $(LIBS) | $(BIN_DIR)
	$(CXX) $(CXXFLAGS) -o $@ mess_profiler.cpp $(MESS_PROFILER_OBJS) $(ARCH_OBJS) $(LIBS) $(LDFLAGS)


Mess.o: Mess.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c Mess.cpp -o $@

generate_code.o: generate_code.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c generate_code.cpp -o $@


%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@


clean:
	rm -f $(OBJS) $(ARCH_OBJS) $(LIBS) $(TARGETS)
	rm -rf $(BUILD_DIR)/ measuring/*
	rm -rf output/*
	rm -rf src/traffic_gen/build
	rm -rf src/traffic_gen/src/nop.c
	rm -rf src/traffic_gen/src/nop_*.c
	rm -rf src/traffic_gen/src/generated_*.c
	rm -rf src/traffic_gen/src/utils_*.c
	rm -f *.o src/arch/*.o src/arch/*/*.o
	rm -rf build/*


distclean: clean
	rm -f *.log *.csv *.json


install:
	@echo "Running code generation..."
	@./$(BIN_DIR)/generate_code $(DEBUG_FLAGS)


test: $(TARGETS)
	$(BIN_DIR)/mess --help

debug: CXXFLAGS = -std=c++17 -Wall -Wextra -g -O0 -pthread
debug: $(TARGETS)

# Dependencies
Mess.o: include/system_detection.h include/benchmark_config.h include/cli_parser.h include/codegen.h include/benchmark_executor.h include/results_processor.h include/utils.h

generate_code.o: include/system_detection.h include/codegen.h include/utils.h

src/system_detection.o: include/system_detection.h

src/benchmark_config.o: include/benchmark_config.h

src/cli_parser.o: include/cli_parser.h include/benchmark_config.h

src/kernel_generator.o: include/codegen.h include/system_detection.h

src/benchmark_executor.o: include/benchmark_executor.h include/benchmark_config.h include/codegen.h include/measurement.h include/process_manager.h include/utils.h

src/results_processor.o: include/results_processor.h include/benchmark_config.h include/benchmark_executor.h

.PHONY: all clean distclean install test debug

