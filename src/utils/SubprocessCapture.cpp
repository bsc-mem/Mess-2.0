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

#include "utils/SubprocessCapture.h"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <iostream>
#include <thread>

#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

namespace subprocess {

namespace {

bool wait_for_child_with_timeout(pid_t pid, std::chrono::milliseconds timeout, int& status) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (true) {
        pid_t wait_result = waitpid(pid, &status, WNOHANG);
        if (wait_result == pid) return true;
        if (wait_result == -1) return false;
        if (std::chrono::steady_clock::now() >= deadline) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    kill(pid, SIGKILL);
    waitpid(pid, &status, 0);
    return false;
}

std::string collect_child_output(int fd, pid_t pid, std::size_t max_output_bytes, std::chrono::milliseconds timeout, const char* label, int&  exit_status_out, bool& timed_out_out) {
    std::string output;
    output.reserve(std::min<std::size_t>(max_output_bytes, 8192));

    int flags = fcntl(fd, F_GETFL, 0);
    if (flags != -1) {
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    }

    const auto deadline = std::chrono::steady_clock::now() + timeout;
    bool child_exited = false;
    int  status = 0;

    while (true) {
        char buffer[1024];
        ssize_t n = read(fd, buffer, sizeof(buffer));
        if (n > 0) {
            const std::size_t remaining = (output.size() < max_output_bytes)
                ? (max_output_bytes - output.size())
                : 0;
            if (remaining > 0) {
                output.append(buffer,
                              static_cast<std::size_t>(std::min<ssize_t>(
                                  n, static_cast<ssize_t>(remaining))));
            }
            continue;
        }

        if (n == 0) break;

        if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) break;

        pid_t wait_result = waitpid(pid, &status, WNOHANG);
        if (wait_result == pid) {
            child_exited = true;
        } else if (wait_result == -1) {
            child_exited = true;
        }

        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline) {
            std::cerr << "Warning: Timed out while probing " << label
                      << " during counter discovery. Continuing with fallback detection."
                      << std::endl;
            kill(pid, SIGKILL);
            waitpid(pid, &status, 0);
            timed_out_out = true;
            child_exited  = true;
            break;
        }

        struct pollfd pfd;
        pfd.fd     = fd;
        pfd.events = POLLIN | POLLHUP | POLLERR;
        pfd.revents = 0;

        int timeout_ms = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count());
        timeout_ms = std::max(1, std::min(timeout_ms, 200));
        int poll_result = poll(&pfd, 1, timeout_ms);
        if (poll_result < 0 && errno != EINTR) break;

        if (child_exited && poll_result == 0) break;
    }

    close(fd);
    if (!child_exited) {
        wait_for_child_with_timeout(pid, std::chrono::milliseconds(500), status);
    }

    if (WIFEXITED(status)) {
        exit_status_out = WEXITSTATUS(status);
    } else {
        exit_status_out = -1;
    }
    return output;
}

}

Result capture(const std::string& program,
               const std::vector<std::string>& args,
               const Options& opts) {
    Result result;

    int pipefd[2];
    if (pipe(pipefd) < 0) {
        result.spawn_failed = true;
        return result;
    }

    pid_t pid = fork();
    if (pid < 0) {
        close(pipefd[0]);
        close(pipefd[1]);
        result.spawn_failed = true;
        return result;
    }

    if (pid == 0) {
        close(pipefd[0]);
        dup2(pipefd[1], STDOUT_FILENO);
        dup2(pipefd[1], STDERR_FILENO);
        close(pipefd[1]);

        std::vector<const char*> argv;
        argv.reserve(args.size() + 2);
        argv.push_back(program.c_str());
        for (const auto& a : args) argv.push_back(a.c_str());
        argv.push_back(nullptr);
        execvp(program.c_str(), const_cast<char* const*>(argv.data()));
        _exit(127);
    }

    close(pipefd[1]);
    result.output = collect_child_output(pipefd[0], pid, opts.max_output_bytes, opts.timeout, program.c_str(), result.exit_status, result.timed_out);
    return result;
}

Result capture_shell(const std::string& command, const Options& opts) {
    Result result;

    int pipefd[2];
    if (pipe(pipefd) < 0) {
        result.spawn_failed = true;
        return result;
    }

    pid_t pid = fork();
    if (pid < 0) {
        close(pipefd[0]);
        close(pipefd[1]);
        result.spawn_failed = true;
        return result;
    }

    if (pid == 0) {
        close(pipefd[0]);
        dup2(pipefd[1], STDOUT_FILENO);
        dup2(pipefd[1], STDERR_FILENO);
        close(pipefd[1]);
        execl("/bin/sh", "sh", "-lc", command.c_str(), static_cast<char*>(nullptr));
        _exit(127);
    }

    close(pipefd[1]);
    result.output = collect_child_output(pipefd[0], pid, opts.max_output_bytes, opts.timeout, "shell command", result.exit_status, result.timed_out);
    return result;
}

std::string run_shell(const std::string& command, const Options& opts) {
    return capture_shell(command, opts).output;
}

std::string run(const std::string& program,
                const std::vector<std::string>& args,
                const Options& opts) {
    return capture(program, args, opts).output;
}

}
