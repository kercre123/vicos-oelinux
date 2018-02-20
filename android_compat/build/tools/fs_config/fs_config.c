/*
 * Copyright (C) 2008 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <inttypes.h>

#include <selinux/selinux.h>
#include <selinux/label.h>

#ifndef HOST_BUILD
#include <selinux/android.h>
#endif

#include "private/android_filesystem_config.h"

// This program takes a list of files and directories (indicated by a
// trailing slash) on the stdin, and prints to stdout each input
// filename along with its desired uid, gid, and mode (in octal).
// The leading slash should be stripped from the input.
//
// After the first 4 columns, optional key=value pairs are emitted
// for each file.  Currently, the following keys are supported:
// * -S: selabel=[selinux_label]
// * -C: capabilities=[hex capabilities value]
//
// Example input:
//
//      system/etc/dbus.conf
//      data/app/
//
// Output:
//
//      system/etc/dbus.conf 1002 1002 440
//      data/app 1000 1000 771
//
//   or if, for example, -S is used:
//
//      system/etc/dbus.conf 1002 1002 440 selabel=u:object_r:system_file:s0
//      data/app 1000 1000 771 selabel=u:object_r:apk_data_file:s0
//
// Note that the output will omit the trailing slash from
// directories.

typedef struct {
        const char* path;
        unsigned uid;
        unsigned gid;
        unsigned mode;
        uint64_t capabilities;
} fs_config_data_t;

static fs_config_data_t* canned_data = NULL;
static int canned_alloc = 0;
static int canned_used = 0;

static struct selabel_handle* get_sehnd(const char* context_file) {
  struct selinux_opt seopts[] = { { SELABEL_OPT_PATH, context_file } };
  struct selabel_handle* sehnd = selabel_open(SELABEL_CTX_FILE, seopts, 1);

  if (!sehnd) {
    perror("error running selabel_open");
    exit(EXIT_FAILURE);
  }
  return sehnd;
}

static void usage() {
  fprintf(stderr, "Usage: fs_config [-p prefix] [-c fs_config_file] [-D product_out_path] [-S context_file] [-C]\n");
}

static int path_compare(const void* a, const void* b) {
  return strcmp(((fs_config_data_t*)a)->path, ((fs_config_data_t*)b)->path);
}

static int load_fs_config(const char* fn, const char* prefix) {
  FILE* f = fopen(fn, "r");
  if (f == NULL) {
    fprintf(stderr, "failed to open %s: %s\n", fn, strerror(errno));
    return -1;
  }

  int prefix_len = strlen(prefix);
  char line[2048];
  while (fgets(line, sizeof(line), f)) {
    while (canned_used >= canned_alloc) {
      canned_alloc = (canned_alloc+1) * 2;
      canned_data = (fs_config_data_t*) realloc(canned_data, canned_alloc * sizeof(fs_config_data_t));
    }
    fs_config_data_t* p = canned_data + canned_used;

    if (prefix) {
      p->path = strdup(prefix);
      const char *append_path = strtok(line, " ");
      int append_path_len = strlen(append_path);
      realloc(p->path, (prefix_len + append_path_len + 1));
      strcat(p->path, append_path);
    } else {
      p->path = strdup(strtok(line, " "));
    }
    p->uid = atoi(strtok(NULL, " "));
    p->gid = atoi(strtok(NULL, " "));
    p->mode = strtol(strtok(NULL, " "), NULL, 8);   // mode is in octal
    p->capabilities = 0;

    char* token = NULL;
    do {
      token = strtok(NULL, " ");
      if (token && strncmp(token, "capabilities=", 13) == 0) {
        p->capabilities = strtoll(token+13, NULL, 0);
        break;
      }
    } while (token);

    fprintf(stderr,"path:%s, uid:%d, gid:%d, mode:%o, capabilities:%x\n",
            p->path, p->uid, p->gid, p->mode, p->capabilities);
    canned_used++;
  }

  fclose(f);

  qsort(canned_data, canned_used, sizeof(fs_config_data_t), path_compare);
  fprintf(stderr,"loaded %d fs_config entries\n", canned_used);

  return 0;
}

static int canned_fs_config(const char* path, int dir, const char* target_out_path,
                            unsigned* uid, unsigned* gid, unsigned* mode, uint64_t* capabilities) {
  fs_config_data_t key;
  key.path = path;   // canned paths lack the leading '/'
  fs_config_data_t* p = (fs_config_data_t*) bsearch(&key, canned_data, canned_used, sizeof(fs_config_data_t), path_compare);
  if (p == NULL) {
    fprintf(stderr, "failed to find [%s] in canned fs_config\n", path);
    return -1;
  }
  *uid = p->uid;
  *gid = p->gid;
  *mode = p->mode;
  *capabilities = p->capabilities;
  fprintf(stderr,"[%s] found in canned fs_config. uid=%d, gid=%d, mode=%o, capabilities=%x\n",
          p->path, *uid, *gid, *mode, *capabilities);
  return 0;
}

int main(int argc, char** argv) {
  char buffer[1024];
  const char* context_file = NULL;
  const char* product_out_path = NULL;
  struct selabel_handle* sehnd = NULL;
  int print_capabilities = 0;
  int opt;
  const char *fs_config_file = NULL;
  const char *fs_config_prefix = NULL;
  int entry_in_canned = 0;

  while((opt = getopt(argc, argv, "c:p:CS:D:")) != -1) {
    switch(opt) {
    case 'C':
      print_capabilities = 1;
      break;
    case 'S':
      context_file = optarg;
      break;
    case 'D':
      product_out_path = optarg;
      break;
    case 'c':
      fs_config_file = optarg;
      break;
    case 'p':
      fs_config_prefix = optarg;
      break;
    default:
      usage();
      exit(EXIT_FAILURE);
    }
  }

  fprintf(stderr,"fs_config_file: %s\n", (fs_config_file)?fs_config_file:"(none)");
  if (fs_config_file) {
    if (load_fs_config(fs_config_file, fs_config_prefix) < 0) {
      fprintf(stderr, "failed to load %s\n", fs_config_file);
      exit(EXIT_FAILURE);
    }
  }

  if (context_file != NULL) {
    sehnd = get_sehnd(context_file);
  }

  while (fgets(buffer, 1023, stdin) != NULL) {
    int is_dir = 0;
    int i;
    for (i = 0; i < 1024 && buffer[i]; ++i) {
      switch (buffer[i]) {
        case '\n':
          buffer[i-is_dir] = '\0';
          i = 1025;
          break;
        case '/':
          is_dir = 1;
          break;
        default:
          is_dir = 0;
          break;
      }
    }

    unsigned uid = 0, gid = 0, mode = 0;
    uint64_t capabilities;
    if (fs_config_file) {
      entry_in_canned = !(canned_fs_config(buffer, is_dir, product_out_path, &uid, &gid, &mode, &capabilities));
    }

    if (!entry_in_canned) {
      fs_config(buffer, is_dir, product_out_path, &uid, &gid, &mode, &capabilities);
    }

    printf("%s %d %d %o", buffer, uid, gid, mode);

    if (sehnd != NULL) {
      size_t buffer_strlen = strnlen(buffer, sizeof(buffer));
      if (buffer_strlen >= sizeof(buffer)) {
        fprintf(stderr, "non null terminated buffer, aborting\n");
        exit(EXIT_FAILURE);
      }
      size_t full_name_size = buffer_strlen + 2;
      char* full_name = (char*) malloc(full_name_size);
      if (full_name == NULL) {
        perror("malloc");
        exit(EXIT_FAILURE);
      }

      full_name[0] = '/';
      strncpy(full_name + 1, buffer, full_name_size - 1);
      full_name[full_name_size - 1] = '\0';

      char* secontext;
      if (selabel_lookup(sehnd, &secontext, full_name, ( mode | (is_dir ? S_IFDIR : S_IFREG)))) {
        secontext = strdup("u:object_r:unlabeled:s0");
      }

      printf(" selabel=%s", secontext);
      free(full_name);
      freecon(secontext);
    }

    if (print_capabilities) {
      printf(" capabilities=0x%" PRIx64, capabilities);
    }

    printf("\n");
  }
  return 0;
}
