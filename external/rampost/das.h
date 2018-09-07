#ifndef __DAS_H
#define __DAS_H

#define DAS_DEBUG 0
#define DAS_INFO 1
#define DAS_EVENT 2
#define DAS_WARN 3
#define DAS_ERROR 4

#define DEBUG_LEVEL 1


#define DAS_LOG(level, event, fmt, ...) do { if (level >= DEBUG_LEVEL) { \
  printf("@rampost.%s\t", event); \
  printf(fmt, ##__VA_ARGS__); \
  printf("\n"); \
}} while(0)

#endif
