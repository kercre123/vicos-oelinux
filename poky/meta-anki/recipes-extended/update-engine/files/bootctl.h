#ifndef __BOOT_CONTROL_H
#define __BOOT_CONTROL_H
/*
 * Copyright (c) 2016, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * @brief Boot control module header for anki
 * @author: Daniel Casner <daniel@anki.com>
 *
 * Forked from https://android.googlesource.com/platform/hardware/qcom/bootctrl/+/master/boot_control.cpp
 * 2018-02-27
 */

// Fixed two slots, we aren't making a general purpose system here
#define NUM_SLOTS (2)

enum part_attr_type {
  ATTR_SLOT_ACTIVE = 0,
  ATTR_BOOT_SUCCESSFUL,
  ATTR_UNBOOTABLE,
};

void boot_control_init(const unsigned boot_slot);

unsigned get_current_slot();

int mark_boot_successful();

const char *get_suffix(unsigned slot);

int set_active_boot_slot(unsigned slot);

int set_slot_as_unbootable(unsigned slot);

int is_slot_active(unsigned slot);

int is_slot_bootable(unsigned slot);

int is_slot_marked_successful(unsigned slot);

#endif
