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
#include "timeout.h"
#include "ulog.h"
#include "util.h"

#include <assert.h>

ULog::ULog(Timeout_Manager *timeout_manager) : Endpoint{"ULog", false}
{
    assert(timeout_manager);
    _system_id = SYSTEM_ID;
    _timeout_manager = timeout_manager;
}

int ULog::write_msg(const struct buffer *pbuf)
{
    struct buffer out_buf;

    //handle ulog

    _router->router_msg(&out_buf, TARGET_SYSTEM_ID, _system_id);

    _timeout_manager->add_timeout(MSEC_PER_SEC, NULL, NULL);

    return pbuf->len;
}
