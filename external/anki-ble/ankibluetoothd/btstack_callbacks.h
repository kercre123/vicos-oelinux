/**
 * File: btstack_callbacks.h
 *
 * Author: seichert
 * Created: 2/16/2018
 *
 * Description: Bluetooth Stack callbacks
 *
 * Copyright: Anki, Inc. 2018
 *
 **/

#pragma once

#include <vector>

typedef void (*ConnectionCallback)(int conn_id, int connected);
typedef void (*RequestReadCallback)(int conn_id, int trans_id, int attr_handle, int offset);
typedef void (*RequestWriteCallback)(int conn_id, int trans_id, int attr_handle, int offset,
                                     bool need_rsp, const std::vector<uint8_t>& value);
typedef void (*IndicationSentCallback)(int conn_id, int status);
typedef void (*CongestionCallback)(int conn_id, bool congested);
