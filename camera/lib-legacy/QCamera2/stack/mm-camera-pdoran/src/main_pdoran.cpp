#include "application.h"

#include <stdio.h>
#include <getopt.h>

#include <iostream>

/**
 * @brief Print the usage of the application
 */
void usage(char* name)
{
  printf("usage: %s\n", name);
  printf("   --help,-h                           this helpful message\n");
}

/**
 * @brief Program entry point
 */
int main(int argc, char* argv[])
{
  Application app;
  Application::Args args;
  
  // Parse arguments using getopt (posix standard)
  int c;
  int optidx = 0;

  struct option longopt[] = {
    {"help",                0, NULL, 'h'},
    {0, 0, 0, 0}
  };

  while ((c = getopt_long(argc, argv, "Cdho:r:Sv:", longopt, &optidx)) != -1) {
    switch (c) {
    case 'h':
      usage(argv[0]);
      return 0;
    default:
      printf("bad arg\n");
      usage(argv[0]);
      return 1;
    }
  }
  
  return app.exec(args);
}