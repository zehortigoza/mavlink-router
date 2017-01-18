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

#pragma once

#include <stdbool.h>
#include <inttypes.h>
#include <time.h>

#include "macro.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef uint64_t usec_t;
typedef uint64_t nsec_t;
#define USEC_INFINITY ((usec_t) -1)

#define MSEC_PER_SEC  1000ULL
#define USEC_PER_SEC  ((usec_t) 1000000ULL)
#define USEC_PER_MSEC ((usec_t) 1000ULL)
#define NSEC_PER_SEC  ((nsec_t) 1000000000ULL)
#define NSEC_PER_MSEC ((nsec_t) 1000000ULL)
#define NSEC_PER_USEC ((nsec_t) 1000ULL)

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
#define streq(a,b) (strcmp((a),(b)) == 0)

int safe_atoull(const char *s, unsigned long long *ret);
int safe_atoul(const char *s, unsigned long *ret);
int safe_atoi(const char *s, int *ret);
usec_t now_usec(void);
usec_t ts_usec(const struct timespec *ts);

static inline void now_timespec(struct timespec *ts)
{
    clock_gettime(CLOCK_MONOTONIC, &ts);
}

#ifdef __cplusplus
}
#endif
