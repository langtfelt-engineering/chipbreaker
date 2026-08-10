// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Langtfelt
//
// The reference host: load the library, run a job, read the report, handle a
// refusal, free everything.
//
// This is the file to copy. It is deliberately complete rather than
// illustrative -- every allocation is released, every status is handled, and
// the refusal path is exercised rather than mentioned. It builds and runs in
// CI, so it cannot rot into an example that no longer compiles.
//
//   c++ -std=c++17 host.cpp -lchipbreaker -o host
//   ./host program.nc tools.json stock.stl [nominal.stl]
//
// What an integrator should take from it, in order of how much time it saves:
//
//   1. CB_REFUSED is a success. The engine answered; the answer is "no,
//      because". Show cb_result_message() to the user and stop. Do not log it
//      as an error and do not retry -- nothing about the input has changed.
//
//   2. An unchecked gate does not pass. Running without a nominal does not
//      give you a clean part, it gives you a report saying the gouge gate never
//      ran. cb_result_pass() returns 0, correctly.
//
//   3. One ownership rule. This library allocates; you free with the matching
//      _free. Strings from getters belong to their handle.

#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "chipbreaker.h"

namespace {

// Reads a whole file. Returns false if it cannot be opened, so that a missing
// input is reported as a missing input rather than as an empty one -- an empty
// stock mesh would reach the engine and be refused for the wrong reason.
bool read_file(const char *path, std::vector<unsigned char> &out) {
    std::FILE *f = std::fopen(path, "rb");
    if (f == nullptr) {
        return false;
    }
    std::fseek(f, 0, SEEK_END);
    const long size = std::ftell(f);
    std::fseek(f, 0, SEEK_SET);
    if (size < 0) {
        std::fclose(f);
        return false;
    }
    out.resize(static_cast<size_t>(size));
    const size_t got = out.empty() ? 0 : std::fread(out.data(), 1, out.size(), f);
    std::fclose(f);
    return got == out.size();
}

} // namespace

int main(int argc, char **argv) {
    if (argc < 4) {
        std::fprintf(stderr,
                     "usage: %s <program.nc> <tools.json> <stock.stl> [nominal.stl]\n",
                     argv[0]);
        return 2;
    }

    // Check the ABI version before anything else. One call, and it turns a
    // silent disagreement about struct layout into a clear message at start-up.
    if (cb_abi_version() != 1) {
        std::fprintf(stderr,
                     "this host was built for Chipbreaker ABI 1, the library reports %u\n",
                     cb_abi_version());
        return 2;
    }
    std::printf("chipbreaker %s, ABI %u\n", cb_engine_version(), cb_abi_version());
    std::printf("engine self-test %s\n", cb_selftest_digest());

    std::vector<unsigned char> program, tools, stock, nominal;
    if (!read_file(argv[1], program) || !read_file(argv[2], tools) ||
        !read_file(argv[3], stock)) {
        std::fprintf(stderr, "could not read one of the inputs\n");
        return 2;
    }
    const bool have_nominal = argc > 4 && read_file(argv[4], nominal);

    void *job = cb_job_new();
    if (job == nullptr) {
        std::fprintf(stderr, "out of memory\n");
        return 2;
    }

    // Every setter returns a status. Checking each one individually is noise;
    // accumulating and checking once is not, because any failure here means a
    // caller bug rather than an engine answer.
    int bad = 0;
    bad |= cb_job_set_program(job, reinterpret_cast<const char *>(program.data()),
                              program.size()) != CB_STATUS_OK;
    bad |= cb_job_set_tools(job, reinterpret_cast<const char *>(tools.data()),
                            tools.size()) != CB_STATUS_OK;
    bad |= cb_job_set_stock_stl(job, stock.data(), stock.size()) != CB_STATUS_OK;
    if (have_nominal) {
        bad |= cb_job_set_nominal_stl(job, nominal.data(), nominal.size()) != CB_STATUS_OK;
    }
    bad |= cb_job_set_resolution_mm(job, 0.5) != CB_STATUS_OK;
    bad |= cb_job_set_tolerance_mm(job, 0.1) != CB_STATUS_OK;
    // Worth setting in any long-lived process. Exceeding it is a refusal that
    // names a resolution which would fit, rather than an allocation failure
    // somewhere inside the engine.
    bad |= cb_job_set_memory_ceiling_bytes(job, 2ULL * 1024 * 1024 * 1024) != CB_STATUS_OK;
    bad |= cb_job_set_source(job, argv[1], std::string(argv[1]).size()) != CB_STATUS_OK;

    if (bad != 0) {
        std::fprintf(stderr, "a setter rejected its argument; this is a bug in this host\n");
        cb_job_free(job);
        return 2;
    }

    void *result = nullptr;
    const cb_status status = cb_job_run(job, &result);

    // The job is no longer needed. Freeing it here rather than at the end shows
    // that the result does not borrow from it.
    cb_job_free(job);

    int exit_code = 0;
    switch (status) {
    case CB_STATUS_OK:
    case CB_STATUS_REFUSED: {
        // Both are answers, and both carry a document. This is the branch that
        // matters: a host that handled only CB_STATUS_OK here would present the
        // engine's most useful behaviour as a crash.
        size_t len = 0;
        const char *json = cb_result_json(result, &len);

        if (cb_result_refused(result) != 0) {
            std::printf("\nREFUSED\n%s\n", cb_result_message(result, nullptr));
            // Not an error exit. The engine answered the question it was asked.
            exit_code = 0;
        } else if (cb_result_pass(result) != 0) {
            std::printf("\nPASS -- every gate checked and clear\n");
        } else {
            // Either a gate failed or a gate never ran. The report says which,
            // under verdict.gates, and an unchecked gate has certified nothing.
            std::printf("\nDID NOT PASS -- read verdict.gates for which gate and why\n");
            exit_code = 1;
        }

        std::printf("report is %zu bytes of %s\n", len,
                    cb_result_refused(result) != 0 ? "chipbreaker.refusal"
                                                   : "chipbreaker.verification-report");
        if (const char *out = std::getenv("CHIPBREAKER_WRITE_REPORT")) {
            std::FILE *f = std::fopen(out, "wb");
            if (f != nullptr) {
                std::fwrite(json, 1, len, f);
                std::fclose(f);
                std::printf("wrote %s\n", out);
            }
        }
        break;
    }
    case CB_STATUS_INVALID_ARGUMENT:
        std::fprintf(stderr, "this host passed the library something it cannot use\n");
        exit_code = 2;
        break;
    case CB_STATUS_INTERNAL:
        // A refusal has a sentence; this does not. Reaching here is a defect
        // worth reporting.
        std::fprintf(stderr, "the engine failed in a way it has no explanation for\n");
        exit_code = 2;
        break;
    }

    // Safe even when result is null, which it is for the two failure statuses.
    cb_result_free(result);
    return exit_code;
}
