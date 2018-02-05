/**
 * File: include_ev.h
 *
 * Author: seichert
 * Created: 1/25/2018
 *
 * Description: Include wrapper for libev
 *
 * Copyright: Anki, Inc. 2018
 *
 **/

#define EV_STANDALONE 1
#define EV_USE_MONOTONIC 1
#define EV_USE_SELECT 1
#define EV_USE_EPOLL 0
#define EV_USE_POLL 0
#define EV_MULTIPLICITY 1
#define EV_IDLE_ENABLE 1

#include "ev++.h"
