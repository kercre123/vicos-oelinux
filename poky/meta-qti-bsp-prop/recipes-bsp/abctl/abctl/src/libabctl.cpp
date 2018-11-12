/*
 * ---------------------------------------------------------------------------
 * Copyright (c) 2018 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 * ---------------------------------------------------------------------------
 *
 */
#include <sys/types.h>
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <inttypes.h>
#include <gpt.h>
#include <assert.h>

#define SLOT_SUFFIX_STRLEN   (sizeof("androidboot.slot_suffix=_") - sizeof(char))
#define KERNEL_CMDLINE       ("/proc/cmdline")
#define CMDLINE_LEN          (2048)

#define PARTITION_ATTRIBUTE_PRIORITY_BIT_POS      (48)
#define PARTITION_ATTRIBUTE_ACTIVE_BIT_POS        (50)
#define PARTITION_ATTRIBUTE_MAX_RETRY_BIT_POS     (51)
#define PARTITION_ATTRIBUTE_SUCCESS_BIT_POS       (54)
#define PARTITION_ATTRIBUTE_UNBOOTABLE_BIT_POS    (55)

#define PARTITION_ATTRIBUTE_PRIORITY_VAL \
            ((uint64_t)(0x3) << PARTITION_ATTRIBUTE_PRIORITY_BIT_POS)
#define PARTITION_ATTRIBUTE_ACTIVE_VAL   \
            ((uint64_t)0x1 << PARTITION_ATTRIBUTE_ACTIVE_BIT_POS)
#define PARTITION_ATTRIBUTE_MAX_RETRY_COUNT_VAL \
            ((uint64_t)0x7 << PARTITION_ATTRIBUTE_MAX_RETRY_BIT_POS)
#define PARTITION_ATTRIBUTE_SUCCESSFUL_VAL \
            ((uint64_t)0x1 << PARTITION_ATTRIBUTE_SUCCESS_BIT_POS)
#define PARTITION_ATTRIBUTE_UNBOOTABLE_VAL \
            ((uint64_t)0x1 << PARTITION_ATTRIBUTE_UNBOOTABLE_BIT_POS)

#define PARTITION_ATTRIBUTE_PRIORITY_CLR   \
            (~((uint64_t)0x3 << PARTITION_ATTRIBUTE_MAX_RETRY_BIT_POS))
#define PARTITION_ATTRIBUTE_ACTIVE_CLR     \
            (~((uint64_t)0x1 << PARTITION_ATTRIBUTE_ACTIVE_BIT_POS))
#define PARTITION_ATTRIBUTE_MAX_RETRY_COUNT_CLR \
            (~((uint64_t)0x7 << PARTITION_ATTRIBUTE_MAX_RETRY_BIT_POS))
#define PARTITION_ATTRIBUTE_SUCCESSFUL_CLR \
            (~((uint64_t)0x1 << PARTITION_ATTRIBUTE_SUCCESS_BIT_POS))
#define PARTITION_ATTRIBUTE_UNBOOTABLE_CLR \
            (~((uint64_t)0x1 << PARTITION_ATTRIBUTE_UNBOOTABLE_BIT_POS))

#define MAX_SLOTS                                 (2)

#define STRLEN_CHAR16(x)     ((sizeof(x)/sizeof(char16_t)) - 1)

#define SBL_PREFIX   (u"sbl1_")
#define RPM_PREFIX   (u"rpm_")
#define TZ_PREFIX    (u"tz_")
#define ABOOT_PREFIX (u"aboot_")
#define BOOT_PREFIX  (u"boot_")

typedef struct {
   char16_t *partition_name_prefix; /* Prefix for partition name*/
   unsigned short prefix_len;       /* Length of the prefix*/
   unsigned short use_attributes;   /* Set '1' if attribute flags are used*/
   guid_t active_slot_guid[16];     /* GUID for indicates active slot*/
   guid_t dormant_slot_guid[16];    /* GUID for non-active slot*/
}  __attribute__((packed)) partition_guid_map_t;

partition_guid_map_t part_guid_map[] = {
   { SBL_PREFIX, STRLEN_CHAR16(SBL_PREFIX), 0,
     {0x2c,0xba,0xa0,0xde,0xdd,0xcb,0x05,0x48,0xb4,0xf9,0xf4,0x28,0x25,0x1c,0x3e,0x98},
     {0xa2,0xa0,0xd0,0xeb,0xe5,0xb9,0x33,0x44,0x87,0xc0,0x68,0xb6,0xb7,0x26,0x99,0xc7}
   },
   { RPM_PREFIX, STRLEN_CHAR16(RPM_PREFIX), 0,
     {0x93,0xf7,0x8d,0x09,0x12,0xd7,0x3d,0x41,0x9d,0x4e,0x89,0xd7,0x11,0x77,0x22,0x28},
     {0xa2,0xa0,0xd0,0xeb,0xe5,0xb9,0x33,0x44,0x87,0xc0,0x68,0xb6,0xb7,0x26,0x99,0xc7}
   },
   { TZ_PREFIX, STRLEN_CHAR16(TZ_PREFIX), 0,
     {0x7f,0xaa,0x53,0xa0,0xb8,0x40,0x1c,0x4b,0xba,0x08,0x2f,0x68,0xac,0x71,0xa4,0xf4},
     {0xa2,0xa0,0xd0,0xeb,0xe5,0xb9,0x33,0x44,0x87,0xc0,0x68,0xb6,0xb7,0x26,0x99,0xc7}
   },
   { ABOOT_PREFIX, STRLEN_CHAR16(ABOOT_PREFIX), 0,
     {0xcd,0xfd,0x0f,0x40,0xe0,0x22,0xe7,0x47,0x9a,0x23,0xf1,0x6e,0xd9,0x38,0x23,0x88},
     {0xa2,0xa0,0xd0,0xeb,0xe5,0xb9,0x33,0x44,0x87,0xc0,0x68,0xb6,0xb7,0x26,0x99,0xc7}
   },
   { BOOT_PREFIX, STRLEN_CHAR16(BOOT_PREFIX), 1, {0x0}, {0x0} }
};

static char16_t* get_partition_name(unsigned int slot)
{
   switch (slot) {
      case 0:
         return u"boot_a";
         break;

      case 1:
         return u"boot_b";
         break;

      default:
         return NULL;
   }
   return NULL;
}

int libabctl_getPartitionIndexForSlot(int slot, char* device)
{
   int ret = -1;

   assert(slot <= MAX_SLOTS);

   const char16_t* boot_partition = slot == 0 ? u"boot_a" : u"boot_b";

   ret = libgpt_getPartitionEntry("/dev/mmcblk0", boot_partition, NULL);

   return ret;
}

int libabctl_getBootSlot()
{
   char *cmdline = NULL;
   int active_slot = -1;
   int fd = -1;
   ssize_t read_bytes = 0;
   char *key_val_pair = NULL;

   do {
      fd = open(KERNEL_CMDLINE, O_RDONLY, 0);
      if ( fd == -1) {
         perror("libabctl: open: KERNEL_CMDLINE ");
         break;
      }

      cmdline = (char *)malloc(CMDLINE_LEN + 1);
      if (!cmdline) {
         perror("libabctl: malloc: ");
         break;
      }

      read_bytes = read(fd, cmdline, CMDLINE_LEN);
      if ( read_bytes == -1) {
         perror("libabctl: read: KERNEL_CMDLINE ");
         free(cmdline);
         close(fd);
         break;
      }
      close(fd);

      key_val_pair = strtok(cmdline, " ");

      while (key_val_pair != NULL) {
         if (!(strncmp(key_val_pair, "androidboot.slot_suffix=_",
                       SLOT_SUFFIX_STRLEN))) {
            switch(key_val_pair[SLOT_SUFFIX_STRLEN]) {
               case 'a': {
                  active_slot = 0;
                  break;
               }

               case 'b': {
                  active_slot = 1;
                  break;
               }

               default:
                  fprintf(stderr,"Unknown slot\n");
            }
         }
         key_val_pair = strtok(NULL, " ");
      }
   } while (0);

   free(cmdline);
   return active_slot;
}

int libabctl_SetBootSuccess()
{
   int ret = libabctl_getBootSlot();
   gpt_partition_entry_t e;
   char16_t *boot_partition = NULL;

   if(ret == -1) {
      perror("libabctl_SetBootSuccess: ");
      goto out;
   }

   boot_partition = get_partition_name(ret);

   ret = libgpt_getPartitionEntry("/dev/mmcblk0", boot_partition, &e);
   if(ret == -1) {
      perror("libabctl_SetBootSuccess: ");
      goto out;
   }

   e.attribute_flags |= PARTITION_ATTRIBUTE_SUCCESSFUL_VAL;
   e.attribute_flags &= PARTITION_ATTRIBUTE_UNBOOTABLE_CLR;

   ret = libgpt_setAttribute("/dev/mmcblk0", boot_partition,
                                             e.attribute_flags);
out:
   return ret;
}

int libabctl_getActiveStatus(unsigned int slot)
{
   int ret = -1;
   gpt_partition_entry_t e;
   const char16_t* boot_partition = (slot == 0 ? u"boot_a" : u"boot_b");

   ret = libgpt_getPartitionEntry("/dev/mmcblk0", boot_partition, &e);
   if(ret == -1) {
      perror("libabctl_getActiveStatus: ");
      goto out;
   }

   ret = (e.attribute_flags & PARTITION_ATTRIBUTE_ACTIVE_BIT_POS)? 1 : 0;

out:
   return ret;
}

int libabctl_setActive(unsigned int slot)
{
   int ret = -1;
   gpt_partition_entry_t e;
   char16_t active_slot_suffix, non_active_slot_suffix;
   int iter = 0;

   assert(slot <= MAX_SLOTS);

   char16_t active_partition[GPT_NAME_MAX] = {0};
   char16_t non_active_partition[GPT_NAME_MAX] = {0};

   /*Determine suffix letter from the slot ID*/
   switch (slot) {
      case 0: {
         active_slot_suffix = 'a';
         non_active_slot_suffix = 'b';
      }
      break;
      case 1: {
         active_slot_suffix = 'b';
         non_active_slot_suffix = 'a';
      }
      break;
      default:
         goto out;
   }

   /*Iterate through all the partitons that are impacted by AB*/
   for( iter = 0; iter < sizeof(part_guid_map)/sizeof(partition_guid_map_t);
        iter++) {

      /*Copy the partition name prefix.*/
      memset(active_partition, 0, GPT_NAME_MAX);
      memcpy(active_partition, part_guid_map[iter].partition_name_prefix,
                               part_guid_map[iter].prefix_len * sizeof(char16_t));
      /*concat the active-suffix to get complete partition name*/
      active_partition[part_guid_map[iter].prefix_len] = active_slot_suffix;

      ret = libgpt_getPartitionEntry("/dev/mmcblk0", active_partition, &e);
      if(ret == -1) {
         perror("libabctl_setActive: failed to get active partition: ");
         goto out;
      }

      if(part_guid_map[iter].use_attributes) {
         e.attribute_flags |= (PARTITION_ATTRIBUTE_ACTIVE_VAL |
                               PARTITION_ATTRIBUTE_PRIORITY_VAL |
                               PARTITION_ATTRIBUTE_MAX_RETRY_COUNT_VAL);
         e.attribute_flags &= PARTITION_ATTRIBUTE_SUCCESSFUL_CLR;
         e.attribute_flags &= PARTITION_ATTRIBUTE_UNBOOTABLE_CLR;
         ret = libgpt_setAttribute("/dev/mmcblk0", active_partition,
                                                   e.attribute_flags);

      } else {
         ret = libgpt_setTypeGUID("/dev/mmcblk0", active_partition,
                                  part_guid_map[iter].active_slot_guid);
      }

      if(ret == -1) {
         perror("libabctl_setActive: failed to update GPT: ");
         goto out;
      }

      /*Copy the partition name prefix.*/
      memset(non_active_partition, 0, sizeof(non_active_partition));
      memcpy(non_active_partition, part_guid_map[iter].partition_name_prefix,
                                   part_guid_map[iter].prefix_len * sizeof(char16_t));
      /*concat the non-active-suffix to get complete partition name*/
      non_active_partition[part_guid_map[iter].prefix_len] = non_active_slot_suffix;

      ret = libgpt_getPartitionEntry("/dev/mmcblk0", non_active_partition, &e);
      if(ret == -1) {
         perror("libabctl_setActive: failed to get non-active partition: ");
         goto out;
      }

      if(part_guid_map[iter].use_attributes) {
         e.attribute_flags &= ~(PARTITION_ATTRIBUTE_ACTIVE_VAL |
                                PARTITION_ATTRIBUTE_PRIORITY_VAL);
         ret = libgpt_setAttribute("/dev/mmcblk0", non_active_partition,
                                                   e.attribute_flags);

      } else {
         ret = libgpt_setTypeGUID("/dev/mmcblk0", non_active_partition,
                                  part_guid_map[iter].dormant_slot_guid);
      }
   }
out:
   return ret;
}

int libabctl_getSuccessStatus(unsigned int slot)
{
   int ret = -1;
   gpt_partition_entry_t e;
   const char16_t* boot_partition = (slot == 0 ? u"boot_a" : u"boot_b");

   ret = libgpt_getPartitionEntry("/dev/mmcblk0", boot_partition, &e);
   if(ret == -1) {
      perror("libabctl_getSuccessStatus: ");
      goto out;
   }

   ret = (e.attribute_flags & PARTITION_ATTRIBUTE_SUCCESS_BIT_POS) ? 1 : 0;

out:
   return ret;
}

int libabctl_setUnbootable(unsigned int slot)
{
   char16_t *partition = get_partition_name(slot);
   int ret = -1;
   gpt_partition_entry_t e;

   if(partition == NULL)
      return -1;

   ret = libgpt_getPartitionEntry("/dev/mmcblk0", partition, &e);
   if(ret == -1) {
      perror("libabctl_setUnbootable: ");
      goto out;
   }

   e.attribute_flags |= PARTITION_ATTRIBUTE_UNBOOTABLE_VAL;
   e.attribute_flags &= ~(PARTITION_ATTRIBUTE_ACTIVE_VAL |
                          PARTITION_ATTRIBUTE_PRIORITY_VAL);

   ret = libgpt_setAttribute("/dev/mmcblk0", partition, e.attribute_flags);

out:
   return ret;
}
