// SPDX-License-Identifier: MulanPSL-2.0
// Test-only clock failures, resolution limits and reference steps. Never sets time.
#define _GNU_SOURCE
#include <dlfcn.h>
#include <errno.h>
#include <stdatomic.h>
#include <time.h>

static _Atomic int mode;

// Change failure policy atomically while the child process serves gRPC.
void robonix_test_clock_mode(int value) {
    atomic_store(&mode, value);
}

// Interpose Linux reads; all unselected clocks retain their native readings.
int clock_gettime(clockid_t clock, struct timespec *value) {
    int selected = atomic_load(&mode);
    if (clock == CLOCK_MONOTONIC && selected == 16 &&
        atomic_compare_exchange_strong(&mode, &selected, 0)) {
        errno = EIO;
        return -1;
    }
    int result = ((int (*)(clockid_t, struct timespec *))dlsym(RTLD_NEXT, "clock_gettime"))(clock, value);
    if (result == 0 && clock == CLOCK_REALTIME) {
        if (selected == 2) value->tv_sec = -1;
        if (selected == 8) value->tv_sec += 86400;
        if (selected == 32) value->tv_sec -= 86400;
    }
    return result;
}

// A poor reference resolution produces unavailable quality without a failed read.
int clock_getres(clockid_t clock, struct timespec *value) {
    // Fail Chronos's resolution read while preserving SystemTime reads used by
    // the HTTP transport, whose standard-library API panics on clock failure.
    if (clock == CLOCK_REALTIME && atomic_load(&mode) == 1) {
        errno = EIO;
        return -1;
    }
    if (clock == CLOCK_REALTIME && atomic_load(&mode) == 4) {
        value->tv_sec = 0;
        value->tv_nsec = 2000000;
        return 0;
    }
    return ((int (*)(clockid_t, struct timespec *))dlsym(RTLD_NEXT, "clock_getres"))(clock, value);
}
