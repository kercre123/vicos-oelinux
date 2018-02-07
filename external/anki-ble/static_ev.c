/**
 * File: static_ev.c
 *
 * Author: seichert
 * Created: 1/25/2018
 *
 * Description: wrapper for statically linking libev
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

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wundef"
#pragma GCC diagnostic ignored "-Wcomment"
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
#pragma GCC diagnostic ignored "-Wunused-value"
#pragma GCC diagnostic ignored "-Wparentheses"

#include "ev.c"

#pragma GCC diagnostic pop

