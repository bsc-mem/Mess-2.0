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
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <fcntl.h>
#include "utils.h"

#ifndef MAP_HUGE_SHIFT
#define MAP_HUGE_SHIFT 26
#endif

#ifndef MAP_HUGE_2MB
#define MAP_HUGE_2MB (21 << MAP_HUGE_SHIFT)
#endif

#ifndef TrafficGen_ARRAY_SIZE
#define TrafficGen_ARRAY_SIZE 320000000
#endif

#ifndef TrafficGen_TYPE
#define TrafficGen_TYPE double
#endif

static TrafficGen_TYPE* a;
static TrafficGen_TYPE* b;
static ssize_t array_elements;

static void release_array(TrafficGen_TYPE* ptr, size_t bytes, int via_mmap) {
    if (!ptr) return;
    if (via_mmap) {
        munmap(ptr, bytes);
    } else {
        free(ptr);
    }
}

int main(int argc, char* argv[]) {
    int rd_percentage = 50;
    int pause = 0;
    int opt;
    int verbose = 0;
    int workers = 1;
    const char* fifo_path = NULL;

    while ((opt = getopt(argc, argv, ":r:p:w:v:f:")) != -1) {
        switch (opt) {
            case 'r':
                rd_percentage = atoi(optarg);
                if (rd_percentage < 0 || rd_percentage > 100) {
                    return 1;
                }
                break;
            case 'p':
                {
                    char *endptr;
                    long val = strtol(optarg, &endptr, 10);
                    if (*endptr != '\0' || val < 0) {
                        return 1;
                    }
                    pause = (int)val;
                }
                break;
            case 'w':
                workers = atoi(optarg);
                if (workers <= 0) {
                    return 1;
                }
                break;
            case 'v':
                verbose = atoi(optarg);
                break;
            case 'f':
                fifo_path = optarg;
                break;
            default:
                return 1;
        }
    }

    int ratio_granularity = TrafficGen_get_ratio_granularity();
    if (ratio_granularity <= 0) {
        return 1;
    }
    if ((rd_percentage % ratio_granularity) != 0) {
        return 1;
    }

    array_elements = (ssize_t)TrafficGen_ARRAY_SIZE;
    if (workers > 1) {
        array_elements /= workers;
        if (array_elements < 1) array_elements = 1;
    }

    int loop_increment = TrafficGen_get_loop_increment(rd_percentage);
    if (loop_increment <= 0) loop_increment = 400;

    int alignment = loop_increment * 10;
    array_elements = (array_elements / alignment) * alignment;
    if (array_elements == 0) array_elements = alignment;

    size_t array_bytes = array_elements * sizeof(TrafficGen_TYPE);
    int a_via_mmap = 0;
    int b_via_mmap = 0;
    
#if defined(__linux__) && defined(MAP_HUGETLB)
    #define MMAP_FLAGS (MAP_PRIVATE | MAP_ANONYMOUS | MAP_HUGETLB | MAP_HUGE_2MB)
#else
    #define MMAP_FLAGS (MAP_PRIVATE | MAP_ANONYMOUS)
#endif

    a = (TrafficGen_TYPE*)mmap(NULL, array_bytes, PROT_READ | PROT_WRITE, MMAP_FLAGS, -1, 0);
    if (a == MAP_FAILED) {
        if(verbose == 1){
            fprintf(stderr, "Huge page allocation failed for array A, falling back to posix_memalign\n");
        }
        if (posix_memalign((void**)&a, 64, array_bytes) != 0) return 1;
    } else {
        a_via_mmap = 1;
        if(verbose == 1){
            fprintf(stdout, "Huge page allocation success for array A\n");
        }   
    }

    b = (TrafficGen_TYPE*)mmap(NULL, array_bytes, PROT_READ | PROT_WRITE, MMAP_FLAGS, -1, 0);
    if (b == MAP_FAILED) {
        if(verbose == 1){
            fprintf(stderr, "Huge page allocation failed for array B, falling back to posix_memalign\n");
        }
        if (posix_memalign((void**)&b, 64, array_bytes) != 0) {
            release_array(a, array_bytes, a_via_mmap);
            return 1;
        }
    } else {
        b_via_mmap = 1;
        if(verbose == 1){
            fprintf(stdout, "Huge page allocation success for array B\n");
        }
    }



    fprintf(stdout, "TrafficGen started: ratio=%d, pause=%d, workers=%d, elements=%zd\n", 
            rd_percentage, pause, workers, array_elements);
    fflush(stdout);
    fflush(stderr);

#ifdef _OPENMP
#pragma omp parallel for
#endif
    for (ssize_t j = 0; j < array_elements; j++) {
        a[j] = 1.0;
        b[j] = 2.0;
    }

    if (fifo_path) {
        int fifo_fd = open(fifo_path, O_WRONLY);
        if (fifo_fd >= 0) {
            char ready = 'R';
            write(fifo_fd, &ready, 1);
        }
    }

    for (;;) {
        {
            TrafficGen_copy_rw(a, b, &array_elements, &pause, rd_percentage);
        }
    }

    release_array(a, array_bytes, a_via_mmap);
    release_array(b, array_bytes, b_via_mmap);
    return 0;
}
