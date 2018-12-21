#ifndef __DAS_H
#define __DAS_H

#include <time.h>

#define DAS_DEBUG 0
#define DAS_INFO 1
#define DAS_EVENT 2
#define DAS_WARN 3
#define DAS_ERROR 4

#define DEBUG_LEVEL 1

static inline long long uptime_ms(void)
{
  struct timespec tp;
  if (clock_gettime(CLOCK_BOOTTIME, &tp) == 0) {
    long long retval = tp.tv_sec;  // Assign before operating to ensure the correct type
    retval *= 1000; // miliseconds
    retval += tp.tv_nsec / 1000000;
    return retval;
  }
  return 0;
}

#define DAS_LOG(level, event, fmt, ...) do { if (level >= DEBUG_LEVEL) { \
      const long long log_time = uptime_ms(); \
      printf("\n@rampost.%s\x1f", event); \
      printf(fmt, ##__VA_ARGS__); \
      printf("\x1f\x1f\x1f\x1f\x1f\x1f\x1f\x1f%lld\n", log_time); \
    }} while(0)

#endif
