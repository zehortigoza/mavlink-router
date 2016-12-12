/*
 * This file is part of the mavroute project
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

#include "comm.h"

#ifdef __cplusplus
extern "C" {
#endif

int ulog_start(Endpoint *master, const char *base_path);
int ulog_stop();

/**
 * @return true if message was consumed by ulog
 **/
bool ulog_handle(struct buffer *buffer);

#ifdef __cplusplus
}
#endif
