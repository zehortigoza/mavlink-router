/*
 * This file is part of the MAVLink Router project
 *
 * Copyright (C) 2016  Intel Corporation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "util.h"

#include <assert.h>
#include <errno.h>
#include <stdlib.h>

usec_t ts_usec(const struct timespec *ts)
{
    if (ts->tv_sec == (time_t) -1 &&
        ts->tv_nsec == (long) -1)
        return USEC_INFINITY;

    if ((usec_t) ts->tv_sec > (UINT64_MAX - (ts->tv_nsec / NSEC_PER_USEC)) / USEC_PER_SEC)
        return USEC_INFINITY;

    return
        (usec_t) ts->tv_sec * USEC_PER_SEC +
        (usec_t) ts->tv_nsec / NSEC_PER_USEC;
}

usec_t now_usec(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);

    return ts_usec(&ts);
}

void
timespec_add(const struct timespec *t1, const struct timespec *t2, struct timespec *result)
{
    result->tv_nsec = t1->tv_nsec + t2->tv_nsec;
    result->tv_sec = t1->tv_sec + t2->tv_sec;
    if ((unsigned long long)result->tv_nsec >= NSEC_PER_SEC) {
        result->tv_nsec -= NSEC_PER_SEC;
        result->tv_sec++;
    }
}

int safe_atoul(const char *s, unsigned long *ret)
{
    char *x = NULL;
    unsigned long l;

    assert(s);
    assert(ret);

    errno = 0;
    l = strtoul(s, &x, 0);

    if (!x || x == s || *x || errno)
        return errno ? -errno : -EINVAL;

    *ret = l;

    return 0;
}

int safe_atoull(const char *s, unsigned long long *ret)
{
    char *x = NULL;
    unsigned long long l;

    assert(s);
    assert(ret);

    errno = 0;
    l = strtoull(s, &x, 0);

    if (!x || x == s || *x || errno)
        return errno ? -errno : -EINVAL;

    *ret = l;

    return 0;
}

int safe_atoi(const char *s, int *ret)
{
    char *x = NULL;
    long l;

    assert(s);
    assert(ret);

    errno = 0;
    l = strtol(s, &x, 0);

    if (!x || x == s || *x || errno)
        return errno > 0 ? -errno : -EINVAL;

    if ((long) (int) l != l)
        return -ERANGE;

    *ret = (int) l;
    return 0;
}
