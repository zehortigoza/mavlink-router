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

#include <mavlink.h>
#include <string.h>

#include "comm.h"
#include "log.h"
#include "ulog.h"
#include "util.h"

#define CMD_AND_HEADER_ACK 100000

#define TARGET_SYSTEM_ID 1
#define SYSTEM_ID 2

#define ULOG_HEADER_SIZE 16
#define ULOG_MAGIC { 0x55, 0x4C, 0x6F, 0x67, 0x01, 0x12, 0x35 }

#define NO_FIRST_MSG_OFFSET 255

static bool _start = false;
static bool _start_stop_cmd_ack = false;
static bool _header_received = false;

static Endpoint *_master = NULL;

static uint8_t _msg_buffer[2048];
static uint16_t _msg_buffer_len = 0;

static uint16_t _ulog_seq = 0;
static bool _waiting_first_msg_offset = false;

static FILE *_ulog_file = NULL;

struct _packed_ ulog_msg_header {
    uint16_t msg_size;
    uint8_t msg_type;
};

static int _ulog_start_stop_msg_send(bool enable)
{
    mavlink_message_t msg;
    mavlink_command_long_t cmd;

    bzero(&cmd, sizeof(cmd));
    cmd.command = enable ? MAV_CMD_LOGGING_START : MAV_CMD_LOGGING_STOP;
    cmd.target_component = MAV_COMP_ID_ALL;
    cmd.target_system = TARGET_SYSTEM_ID;

    mavlink_msg_command_long_encode(SYSTEM_ID, 0, &msg, &cmd);
    return _master->write_msg(&msg);
}

static int _ulog_ack_send(uint16_t sequence)
{
    mavlink_message_t msg;
    mavlink_logging_ack_t ack;

    ack.sequence = sequence;
    ack.target_component = MAV_COMP_ID_ALL;
    ack.target_system = TARGET_SYSTEM_ID;

    mavlink_msg_logging_ack_encode(SYSTEM_ID, 0, &msg, &ack);
    return _master->write_msg(&msg);
}

int ulog_start(Endpoint *master, const char *base_path)
{
    time_t t = time(NULL);
    struct tm *timeinfo = localtime(&t);
    char path[1024];

    if (_start) {
        return 0;
    }

    snprintf(path, sizeof(path), "%s/%i-%i-%i_%i-%i-%i.ulg",
            (base_path ? base_path : ""), timeinfo->tm_year + 1900,
            timeinfo->tm_mon + 1, timeinfo->tm_mday, timeinfo->tm_hour,
            timeinfo->tm_min, timeinfo->tm_sec);
    _ulog_file = fopen(path, "wb");
    if (!_ulog_file) {
        log_error_errno(errno, "Unable to open ULOG file.");
        return -errno;
    }

    _master = master;
    _start_stop_cmd_ack = false;
    _header_received = false;
    _start = true;

    return 0;
}

int ulog_stop()
{
    if (!_start) {
        return 0;
    }

    if (_ulog_file) {
        fflush(_ulog_file);
        fclose(_ulog_file);
        _ulog_file = NULL;
    }

    _start_stop_cmd_ack = false;
    _start = false;
    _ulog_start_stop_msg_send(_start);

    return 0;
}

static void _ulog_flush()
{
    while (_msg_buffer_len > sizeof(struct ulog_msg_header)) {
        struct ulog_msg_header *header = (struct ulog_msg_header *)_msg_buffer;
        const uint16_t full_msg_size = header->msg_size + sizeof(struct ulog_msg_header);

        if (full_msg_size > _msg_buffer_len) {
            break;
        }

        fwrite(_msg_buffer, 1, full_msg_size, _ulog_file);
        _msg_buffer_len -= full_msg_size;
        memmove(_msg_buffer, &_msg_buffer[full_msg_size], _msg_buffer_len);
    }
}

static void _ulog_data_handle(mavlink_logging_data_t *ulog_data)
{
    bool drops = false;

    /* Check for message drops */
    if (_ulog_seq + 1 != ulog_data->sequence) {
        drops = true;
    }
    _ulog_seq = ulog_data->sequence;

    /* Waiting for ULog header? */
    if (!_header_received) {
        const uint8_t magic[] = ULOG_MAGIC;

        if (!memcmp(magic, ulog_data->data, sizeof(magic))) {
            _header_received = true;
            fwrite(ulog_data->data, 1, ULOG_HEADER_SIZE, _ulog_file);
            ulog_data->length -= ULOG_HEADER_SIZE;
            memmove(ulog_data->data, &ulog_data->data[ULOG_HEADER_SIZE],
                    ulog_data->length);
        }
    }

    /*
     * Do not cause a buffer overflow, it should only happens if a ULog message
     * don't fit in _msg_buffer
     */
    if (_msg_buffer_len + ulog_data->length >= sizeof(_msg_buffer)) {
        log_warning("Buffer full, dropping everything on buffer");
        _msg_buffer_len = 0;
        _waiting_first_msg_offset = true;
        return;
    }

    if (drops) {
        _ulog_flush();

        _msg_buffer_len = 0;
        _waiting_first_msg_offset = true;
    }

    if (_waiting_first_msg_offset) {
        if (ulog_data->first_message_offset == NO_FIRST_MSG_OFFSET) {
            /* no useful information in this message */
            return;
        }

        _waiting_first_msg_offset = false;
        memcpy(_msg_buffer, &ulog_data->data[ulog_data->first_message_offset],
                ulog_data->length - ulog_data->first_message_offset);
        _msg_buffer_len = ulog_data->length - ulog_data->first_message_offset;
        _ulog_flush();
        return;
    }

    memcpy(&_msg_buffer[_msg_buffer_len], ulog_data->data, ulog_data->length);
    _msg_buffer_len += ulog_data->length;

    if (ulog_data->first_message_offset != NO_FIRST_MSG_OFFSET) {
        _ulog_flush();
    }
}

bool ulog_handle(struct buffer *buffer)
{
    const bool mavlink2 = buffer->data[0] == MAVLINK_STX;
    uint32_t msg_id;
    uint8_t *payload;
    bool msg_consumed = false;

    if (mavlink2) {
        struct mavlink_router_mavlink2_header *msg =
                (struct mavlink_router_mavlink2_header *)buffer->data;
        msg_id = msg->msgid;
        payload = buffer->data + sizeof(struct mavlink_router_mavlink2_header);
    } else {
        struct mavlink_router_mavlink1_header *msg =
                (struct mavlink_router_mavlink1_header *)buffer->data;
        msg_id = msg->msgid;
        payload = buffer->data + sizeof(struct mavlink_router_mavlink1_header);
    }

    /* Check if we are interested in this message */
    if (msg_id != MAVLINK_MSG_ID_COMMAND_ACK
        && msg_id != MAVLINK_MSG_ID_LOGGING_DATA_ACKED
        && msg_id != MAVLINK_MSG_ID_LOGGING_DATA) {
        goto end;
    }
    if (!_start
        && (msg_id == MAVLINK_MSG_ID_LOGGING_DATA_ACKED
            || msg_id == MAVLINK_MSG_ID_LOGGING_DATA)) {
        goto end;
    }

    switch (msg_id) {
    case MAVLINK_MSG_ID_COMMAND_ACK: {
        mavlink_command_ack_t *cmd = (mavlink_command_ack_t *)payload;
        /* waiting ack? */
        if (!_start_stop_cmd_ack) {
            const uint16_t waiting_cmd = _start ? MAV_CMD_LOGGING_START
                    : MAV_CMD_LOGGING_STOP;
            if (waiting_cmd == cmd->command
                && cmd->result == MAV_RESULT_ACCEPTED) {
                _start_stop_cmd_ack = true;
                msg_consumed = true;
            }
        }

        break;
    }
    case MAVLINK_MSG_ID_LOGGING_DATA_ACKED: {
        mavlink_logging_data_acked_t *ulog_data_acked = (mavlink_logging_data_acked_t *)payload;
        _ulog_ack_send(ulog_data_acked->sequence);
    }
    case MAVLINK_MSG_ID_LOGGING_DATA: {
        mavlink_logging_data_t *ulog_data = (mavlink_logging_data_t *)payload;
        _ulog_data_handle(ulog_data);
        msg_consumed = true;
        break;
    }
    }

end:
    /*
     * still did not got a cmd ack or ULog header?
     * check timeout and send the start command again
     */
    if (!_start_stop_cmd_ack || (_start && !_header_received)) {
        static usec_t old = 0;
        usec_t now = now_usec();

        if (now - old > CMD_AND_HEADER_ACK) {
            _ulog_start_stop_msg_send(_start);
            old = now;
        }
    }

    return msg_consumed;
}
