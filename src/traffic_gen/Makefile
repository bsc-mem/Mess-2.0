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
CC = gcc
MPICC = mpicc
CFLAGS = -march=native -Wall -fopenmp
MPICC_FLAGS = -diag-disable=10441
LDFLAGS = -pthread -lrt -fopenmp

SRCDIR=src
BUILDDIR=build

PRG_SUFFIX=.x
TARGET?=traffic_gen$(PRG_SUFFIX)

all: init $(TARGET)

.PHONY: all init clean

init:
	mkdir -p $(BUILDDIR)

$(BUILDDIR)/nop.o: $(SRCDIR)/nop.c
	mkdir -p $(BUILDDIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILDDIR)/utils.o: $(SRCDIR)/utils.c
	mkdir -p $(BUILDDIR)
	$(CC) $(CFLAGS) -Wno-unused-but-set-variable -O0 -c $< -o $@

$(TARGET): $(BUILDDIR)/nop.o $(BUILDDIR)/utils.o
	$(MPICC) $(CFLAGS) $(MPICC_FLAGS) -O0 $(LDFLAGS) $^ -o $@

$(BUILDDIR)/traffic_gen_multiseq_c.o: $(SRCDIR)/traffic_gen.c
	mkdir -p $(BUILDDIR)
	$(CC) $(CFLAGS) -O0 -c $< -o $@

traffic_gen_multiseq$(PRG_SUFFIX): $(BUILDDIR)/nop.o $(BUILDDIR)/utils.o $(BUILDDIR)/traffic_gen_multiseq_c.o
	$(CC) $(CFLAGS) -O0 $(LDFLAGS) $^ -o $@


clean:
	rm -f $(BUILDDIR)/*.o *.x
