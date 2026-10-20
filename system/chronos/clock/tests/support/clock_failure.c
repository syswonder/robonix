// SPDX-License-Identifier: MulanPSL-2.0
#define _GNU_SOURCE
#include <dlfcn.h>
#include <errno.h>
#include <stdatomic.h>
#include <time.h>

static _Atomic int mode;

void robonix_test_clock_mode(int value) {
    atomic_store(&mode, value);
}

// Fail one selected read without changing the host clock or other processes.
int clock_gettime(clockid_t clock, struct timespec *value) {
    int selected = atomic_load(&mode);
    if (clock == CLOCK_MONOTONIC && selected == 16 &&
        atomic_compare_exchange_strong(&mode, &selected, 0)) {
        errno = EIO;
        return -1;
    }
    return ((int (*)(clockid_t, struct timespec *))dlsym(RTLD_NEXT, "clock_gettime"))(clock, value);
}
