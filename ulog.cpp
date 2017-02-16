/*
 * This file is part of the MAVLink Router project
 *
 * Copyright (C) 2017  Intel Corporation. All rights reserved.
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
#include "ulog.h"

#include <assert.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "log.h"
#include "mainloop.h"
#include "util.h"

#define ULOG_HEADER_SIZE 16
#define ULOG_MAGIC                               \
    {                                            \
        0x55, 0x4C, 0x6F, 0x67, 0x01, 0x12, 0x35 \
    }

#define NO_FIRST_MSG_OFFSET 255

struct _packed_ ulog_msg_header {
    uint16_t msg_size;
    uint8_t msg_type;
};

ULog::ULog(const char *logs_dir)
    : Endpoint{"ULog", false}
{
    assert(logs_dir);
    _logs_dir = logs_dir;
}

ULog::~ULog()
{
}

static bool _ulog_timeout_cb(void *data)
{
    ULog *ulog = static_cast<ULog *>(data);
    return ulog->logging_start_timeout();
}

bool ULog::logging_start_timeout()
{
    mavlink_message_t msg;
    mavlink_command_long_t cmd;

    bzero(&cmd, sizeof(cmd));
    cmd.command = MAV_CMD_LOGGING_START;
    cmd.target_component = MAV_COMP_ID_ALL;
    cmd.target_system = _target_system_id;

    mavlink_msg_command_long_encode(_system_id, MAV_COMP_ID_ALL, &msg, &cmd);
    _send_msg(&msg, _target_system_id);

    return true;
}

bool ULog::start()
{
    time_t t = time(NULL);
    struct tm *timeinfo = localtime(&t);
    char *filename;
    int r;

    if (_file != -1) {
        log_warning("ULog already started");
        return false;
    }

    r = asprintf(&filename, "%s/%i-%02i-%02i_%02i-%02i-%02i.ulg", _logs_dir,
                 timeinfo->tm_year + 1900, timeinfo->tm_mon + 1, timeinfo->tm_mday,
                 timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
    if (r < 1) {
        log_error_errno(errno, "Error formatting ULog file name");
        return false;
    }

    _file = open(filename, O_WRONLY | O_CLOEXEC | O_CREAT | O_NONBLOCK | O_TRUNC,
                 S_IRUSR | S_IROTH | S_IRGRP);
    if (_file < 0) {
        log_error_errno(errno, "Unable to open ULog file(%s): (%m)", filename);
        goto open_error;
    }

    _timeout = _mainloop->add_timeout(MSEC_PER_SEC, _ulog_timeout_cb, this);
    if (!_timeout) {
        log_error("Unable to add timeout");
        goto timeout_error;
    }

    _waiting_header = true;
    _waiting_first_msg_offset = false;
    _expected_seq = 0;
    _buffer_len = 0;
    _buffer_partial_len = 0;
    _system_id = SYSTEM_ID;

    log_info("Logging target system_id=%u on %s", _target_system_id, filename);
    free(filename);

    return true;

timeout_error:
    close(_file);
    _file = -1;
open_error:
    free(filename);
    return false;
}

void ULog::stop()
{
    mavlink_message_t msg;
    mavlink_command_long_t cmd;

    if (_file == -1) {
        log_error("ULog not started");
        return;
    }

    bzero(&cmd, sizeof(cmd));
    cmd.command = MAV_CMD_LOGGING_STOP;
    cmd.target_component = MAV_COMP_ID_ALL;
    cmd.target_system = _target_system_id;

    mavlink_msg_command_long_encode(_system_id, MAV_COMP_ID_ALL, &msg, &cmd);
    _send_msg(&msg, _target_system_id);

    if (_timeout) {
        _mainloop->del_timeout(_timeout);
        _timeout = nullptr;
    }

    fsync(_file);
    close(_file);
    _file = -1;
    _system_id = 0;
}

int ULog::write_msg(const struct buffer *buffer)
{
    const bool mavlink2 = buffer->data[0] == MAVLINK_STX;
    uint32_t msg_id;
    uint16_t payload_len;
    uint8_t trimmed_zeros;

    _stat.write.total++;
    _stat.write.bytes += buffer->len;

    if (mavlink2) {
        struct mavlink_router_mavlink2_header *msg
            = (struct mavlink_router_mavlink2_header *)buffer->data;
        msg_id = msg->msgid;
        payload_len = msg->payload_len;
    } else {
        struct mavlink_router_mavlink1_header *msg
            = (struct mavlink_router_mavlink1_header *)buffer->data;
        msg_id = msg->msgid;
        payload_len = msg->payload_len;
    }

    /* Check if we are interested in this msg_id */
    if (msg_id != MAVLINK_MSG_ID_COMMAND_ACK && msg_id != MAVLINK_MSG_ID_LOGGING_DATA_ACKED
        && msg_id != MAVLINK_MSG_ID_LOGGING_DATA) {
        return buffer->len;
    }

    uint8_t *payload;

    if (mavlink2) {
        payload = buffer->data + sizeof(struct mavlink_router_mavlink2_header);
        trimmed_zeros = get_trimmed_zeros(buffer);
    } else {
        payload = buffer->data + sizeof(struct mavlink_router_mavlink1_header);
        trimmed_zeros = 0;
    }

    /* Handle messages */
    switch (msg_id) {
    case MAVLINK_MSG_ID_COMMAND_ACK: {
        mavlink_command_ack_t cmd;

        memcpy(&cmd, payload, payload_len);
        if (trimmed_zeros)
            memset(((uint8_t *)&cmd) + payload_len, 0, trimmed_zeros);

        if (!_timeout || cmd.command != MAV_CMD_LOGGING_START)
            break;

        if (cmd.result == MAV_RESULT_ACCEPTED) {
            _mainloop->del_timeout(_timeout);
            _timeout = NULL;
        } else
            log_error("MAV_CMD_LOGGING_START result(%u) is different than accepted", cmd.result);
        break;
    }
    case MAVLINK_MSG_ID_LOGGING_DATA_ACKED: {
        mavlink_logging_data_acked_t *ulog_data_acked = (mavlink_logging_data_acked_t *)payload;
        mavlink_message_t msg;
        mavlink_logging_ack_t ack;

        ack.sequence = ulog_data_acked->sequence;
        ack.target_component = MAV_COMP_ID_ALL;
        ack.target_system = _target_system_id;
        mavlink_msg_logging_ack_encode(SYSTEM_ID, MAV_COMP_ID_ALL, &msg, &ack);
        _send_msg(&msg, _target_system_id);
        /* no break needed, message will be handled by MAVLINK_MSG_ID_LOGGING_DATA case */
    }
    case MAVLINK_MSG_ID_LOGGING_DATA: {
        if (trimmed_zeros) {
            mavlink_logging_data_t ulog_data;
            memcpy(&ulog_data, payload, payload_len);
            memset(((uint8_t *)&ulog_data) + payload_len, 0, trimmed_zeros);
            _logging_data_process(&ulog_data);
        } else {
            mavlink_logging_data_t *ulog_data = (mavlink_logging_data_t *)payload;
            _logging_data_process(ulog_data);
        }
        break;
    }
    }

    return buffer->len;
}

/*
 * Return true if the message with seq should be handled.
 */
bool ULog::_logging_seq(uint16_t seq, bool *drop)
{
    if (_expected_seq == seq) {
        _expected_seq++;
        *drop = false;
        return true;
    }

    if (seq > _expected_seq) {
        const uint16_t diff = seq - _expected_seq;
        if (diff > (UINT16_MAX / 2)) {
            /* _expected_seq wrapped and a re-transmission of a non-wrapped message happened */
            return false;
        }
    } else {
        const uint16_t diff = _expected_seq - seq;
        if (diff < (UINT16_MAX / 2)) {
            /* re-transmission */
            return false;
        }
    }

    *drop = true;
    _expected_seq = seq + 1;
    return true;
}

void ULog::_logging_data_process(mavlink_logging_data_t *msg)
{
    bool drops = false;

    if (!_logging_seq(msg->sequence, &drops))
        return;

    /* Waiting for ULog header? */
    if (_waiting_header) {
        const uint8_t magic[] = ULOG_MAGIC;

        if (memcmp(magic, msg->data, sizeof(magic))) {
            log_error("Invalid ULog Magic number");
            return;
        }
        if (msg->length < ULOG_HEADER_SIZE) {
            /* This should never happen */
            log_error("ULog header is not completed");
            return;
        }

        const ssize_t r = write(_file, msg->data, ULOG_HEADER_SIZE);
        if (r != ULOG_HEADER_SIZE) {
            log_error_errno(errno, "Unable to write to ULog file: (%m)");
            return;
        }

        memmove(msg->data, &msg->data[ULOG_HEADER_SIZE], msg->length);
        msg->length -= ULOG_HEADER_SIZE;
        _waiting_header = false;
    }

    if (drops) {
        _logging_flush();

        _buffer_len = 0;
        _waiting_first_msg_offset = true;
    }

    /*
     * Do not cause a buffer overflow, it should only happens if a ULog message
     * don't fit in _msg_buffer
     */
    if ((_buffer_len + msg->length) > sizeof(_buffer)) {
        log_warning("Buffer full, dropping everything on buffer");

        _buffer_len = 0;
        _waiting_first_msg_offset = true;
    }

    uint8_t begin = 0;

    if (_waiting_first_msg_offset) {
        if (msg->first_message_offset == NO_FIRST_MSG_OFFSET) {
            /* no useful information in this message */
            return;
        }

        _waiting_first_msg_offset = false;
        begin = msg->first_message_offset;
    }

    if (!msg->length)
        return;

    memcpy(&_buffer[_buffer_len], &msg->data[begin], msg->length - begin);
    _buffer_len += (msg->length - begin);
    _logging_flush();
}

void ULog::_logging_flush()
{
    while (_buffer_partial_len) {
        const ssize_t r = write(_file, _buffer_partial, _buffer_partial_len);
        if (r == -EAGAIN)
            return;
        if (r < 1) {
            log_error_errno(errno, "Unable to write to ULog file: (%m)");
            return;
        }

        _buffer_partial_len -= r;
        memmove(_buffer_partial, &_buffer_partial[r], _buffer_partial_len);
    }

    while (_buffer_len >= sizeof(struct ulog_msg_header) && !_buffer_partial_len) {
        struct ulog_msg_header *header = (struct ulog_msg_header *)_buffer;
        const uint16_t full_msg_size = header->msg_size + sizeof(struct ulog_msg_header);

        if (full_msg_size > _buffer_len) {
            break;
        }

        const ssize_t r = write(_file, _buffer, full_msg_size);
        if (r == full_msg_size) {
            _buffer_len -= full_msg_size;
            memmove(_buffer, &_buffer[full_msg_size], _buffer_len);
            continue;
        }
        if (r == -EAGAIN)
            break;

        if (r < 1) {
            log_error_errno(errno, "Unable to write to ULog file: (%m)");
            break;
        }

        /* Handle partial write */
        _buffer_partial_len = full_msg_size - r;

        if (_buffer_partial_len > sizeof(_buffer_partial)) {
            _buffer_partial_len = 0;
            log_error("Partial buffer is not big enough to store the "
                      "ULog entry(type=%c len=%u), ULog file is now corrupt.",
                      header->msg_type, full_msg_size);
            break;
        }

        memcpy(_buffer_partial, &_buffer[r], _buffer_partial_len);

        _buffer_len -= full_msg_size;
        memmove(_buffer, &_buffer[full_msg_size], _buffer_len);
        break;
    }
}

void ULog::_send_msg(const mavlink_message_t *msg, int target_sysid)
{
    uint8_t data[MAVLINK_MAX_PACKET_LEN];
    struct buffer buffer { 0, data };

    buffer.len = mavlink_msg_to_send_buffer(data, msg);
    _mainloop->route_msg(&buffer, target_sysid, msg->sysid);

    _stat.read.total++;
    _stat.read.handled++;
    _stat.read.handled_bytes += msg->len;
    if (msg->magic == MAVLINK_STX)
        _stat.read.handled_bytes += sizeof(struct mavlink_router_mavlink2_header);
    else
        _stat.read.handled_bytes += sizeof(struct mavlink_router_mavlink1_header);
}
