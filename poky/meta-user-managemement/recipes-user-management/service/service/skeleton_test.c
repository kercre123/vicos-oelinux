#include <stdio.h>
#include <unistd.h>

int main(int argc, char *argv[])
{
  printf("skeleton-test starting\n");

  while (1) {
    sleep(10);
    printf("Skeleton Daemon doing no work :-)\n");
  }
  return 0;
}
