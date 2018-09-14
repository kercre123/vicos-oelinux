/**
 * File: main.cpp
 *
 * Author: daniel
 * Created: 2/28/2018
 *
 * Description: main entry point for bootctl
 *
 * Copyright: Anki, Inc. 2018
 *
 **/

#include <stdio.h>
#include <string.h>

#include "bootctl.h"

static const char* USAGE_FMT = \
"%s <CURRENT BOOT SLOT> <ACTION> [<SLOT>]\n" \
"\n" \
"<CURRENT BOOT SLOT> must be either a, b or f depending on what slot is currently booted.\n" \
"\n" \
"Where <ACTION> is one of:\n" \
"* mark_successful\n" \
"      Marks the current slot as successfully booted of not already so marked\n" \
" * set_unbootable <SLOT>\n" \
"      Sets <SLOT> as unbootable\n" \
" * set_active <SLOT>\n" \
"      Sets <SLOT> as the active boot slot for next reboot\n" \
" * status <SLOT>\n" \
"      Prints the status of the specified slot\n";

/// Validates a slot argument and sets the corresponding index
// @return true if slot is valid, false if invalid
// @param [in]  slot the Slot argument
// @param [out] index Will set the pointer to the corresponding slot
bool boot_slot_index(const char slot, unsigned* index) {
  switch (slot) {
    case 'f':
    case 'a':
      if (index) *index = 0;
      return true;
    case 'b':
      if (index) *index = 1;
      return true;
    default:
      return false;
  }
}


int main(int argc, char *argv[]) {
  char this_boot_slot;
  unsigned this_boot_index;
  unsigned arg_slot;

  if (argc < 3) {
    printf(USAGE_FMT, argv[0]);
    return 1;
  }

  this_boot_slot = argv[1][0];
  if (!boot_slot_index(this_boot_slot, &this_boot_index)) {
    printf("Invalid current boot slot: \"%s\"\n", argv[1]);
    printf(USAGE_FMT, argv[0]);
    return 2;
  }

  boot_control_init(this_boot_index);

  if (!strcmp(argv[2], "mark_successful")) {
    if (this_boot_slot == 'F') return 0; // Never mark F successful, it just is
    if (is_slot_marked_successful(this_boot_index)) return 0; // Already marked, no need to update
    return mark_boot_successful();
  }
  // Else a slot argument is required for all other commands

  if (argc < 4) {
    printf("Target slot argument required\n");
    return 2;
  }

  if (!boot_slot_index(argv[3][0], &arg_slot)) {
    printf("Invalid target slot argument: \"%s\"\n", argv[3]);
    return 2;
  }

  if (!strcmp(argv[2], "set_unbootable")) {
    if (!is_slot_bootable(arg_slot)) return 0; // Already unbootable, no need to update
    return set_slot_as_unbootable(arg_slot);
  }

  if (!strcmp(argv[2], "set_active")) {
    return set_active_boot_slot(arg_slot);
  }

  if (!strcmp(argv[2], "status")) {
    printf("bootable: %d\nsuccessful: %d\nactive: %d\n",
           is_slot_bootable(arg_slot),
           is_slot_marked_successful(arg_slot),
           is_slot_active(arg_slot));
    return 0;
  }

  printf("Invalid command: \"%s\"\n", argv[2]);
  return 2;
}
