#include <assert.h>
#include <errno.h>
#include <getopt.h>
#include <limits.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sisci_error.h>
#include <sisci_api.h>

#include "c63.h"

static uint32_t server_node = 0;
static uint32_t worker_nodes[MAX_NUM_WORKERS] = {};


/* getopt */
extern int optind;
extern char *optarg;

static void print_help()
{
  printf("Usage: ./c63worker -s server -r workers\n");
  printf("Commandline options:\n");
  printf("  -r               Node id of server\n");
  printf("  -w               Node id of workers\n");
  printf("\n");

  exit(EXIT_FAILURE);
}


int main(int argc, char **argv)
{
  int c;
  int w = 0; /* worker index */
  sci_desc_t sd;  
  sci_error_t error;

  if (argc == 1) { print_help(); }

  while ((c = getopt(argc, argv, "r:s:")) != -1)
  {
    switch (c)
    {
      case 's':
        server_node = atoi(optarg);
        break;
      case 'r':
        worker_nodes[w++] = atoi(optarg);
        break;
      default:
        print_help();
        break;
    }
  }

  /* Initialize the SISCI library */
  SCIInitialize(0, &error);
  if (error != SCI_ERR_OK) {
    fprintf(stderr,"SCIInitialize failed: %s\n", SCIGetErrorString(error));
    exit(EXIT_FAILURE);
  }

  printf("worker: Hello World!\n");

  SCITerminate();

  return 0;

}
