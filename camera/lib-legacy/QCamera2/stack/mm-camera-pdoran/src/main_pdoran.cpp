#include "application.h"

#include <stdio.h>
#include <getopt.h>

#include <iostream>

using anki::Application;

/**
 * @brief Print the usage of the application
 */
void usage(char* name)
{
  printf("usage: %s\n", name);
  printf("   --help,-h                           this helpful message\n");
  printf("   --dump,-d                           Dump frames to the output directory\n");
  printf("   --output,-o <DIR>                   Output directory for frames\n");
  printf("   --camera,-c <CAMERA>                Camera to use: [RAW,YUV]\n");
}

/**
 * @brief Program entry point
 */
int main(int argc, char* argv[])
{
  Application app;
  Application::Args args;
  
  args.dump = false;
  args.output = "./";
  args.camera = "RAW";

  // Parse arguments using getopt (posix standard)
  int c;
  int optidx = 0;

  struct option longopt[] = {
    {"help",                0, NULL, 'h'},
    {"dump",                0, NULL, 'd'},
    {"output",              0, NULL, 'o'},
    {"camera",              0, NULL, 'c'},
    {0, 0, 0, 0}
  };

  while ((c = getopt_long(argc, argv, "hdo:c:", longopt, &optidx)) != -1) {
    switch (c) {
    case 'h':
      usage(argv[0]);
      return 0;
    case 'd':
      args.dump = true;
      break;
    case 'o':
      args.output = std::string(optarg);
      break;
    case 'c':
      args.camera = std::string(optarg);
      break;
    default:
      printf("bad arg\n");
      usage(argv[0]);
      return 1;
    }
  }
  
  return app.exec(args);
}