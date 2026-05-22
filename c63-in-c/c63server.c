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
#include "c63_write.h"
#include "common.h"
#include "me.h"
#include "tables.h"
#include "sci_common.h"

static char *input_file;
FILE *outfile;

static int limit_numframes = 0;
static uint32_t worker_nodes[MAX_NUM_WORKERS] = {};
static uint32_t writer_node = 0;

static uint32_t width;
static uint32_t height;

/* getopt */
extern int optind;
extern char *optarg;

// SCI variables
static sci_remote_segment_t remote_seg;

typedef struct
{
  config_t *config;
  int buf;
} dma_context_t;

static uint8_t *buffer;

/* Read planar YUV frames with 4:2:0 chroma sub-sampling */
static uint8_t *read_yuv(FILE *file, dma_buffer_t *dma, int w, int h, int buf)
{
  size_t total = dma->total_size;
  size_t len = fread(buffer + buf * total, 1, total, file);
  if (ferror(file)) 
  {
    perror("ferror");
    exit(EXIT_FAILURE);
  }

  if (feof(file)) 
  {
    return NULL;
  }
  else if (len != w*h*1.5)
  {
    fprintf(stderr, "Reached end of file, but incorrect bytes read.\n");
    fprintf(stderr, "Wrong input? (height: %d width: %d)\n", height, width);
    return NULL;
  }

  return buffer + buf * total;
}

static void print_help()
{
  printf("Usage: ./c63server -r nodeid [options] input_file\n");
  printf("Commandline options:\n");
  printf("  -r                             Node id of workers (multiple)\n");
  printf("  -o                             Node id of writer node\n");
  printf("  -h                             Height of images to compress\n");
  printf("  -w                             Width of images to compress\n");
  printf("  [-f]                           Limit number of frames to encode\n");
  printf("\n");

  exit(EXIT_FAILURE);
}

static void sci_init(dma_buffer_t *dma)
{
    sci_error_t error;

    uint32_t wy = (uint32_t)(ceil(width/16.0f)*16);
    uint32_t hy = (uint32_t)(ceil(height/16.0f)*16);
    uint32_t wu = (uint32_t)(ceil(width*UX/(YX*8.0f))*8);
    uint32_t hu = (uint32_t)(ceil(height*UY/(YY*8.0f))*8);

    dma->y_size = wy * hy;
    dma->uv_size = wu * hu;
    dma->total_size = dma->y_size + 2 * dma->uv_size;
    size_t aligned_size = ((dma->total_size + 4095) / 4096) * 4096;

    SCIInitialize(SCI_NO_FLAGS, &error);
    sci_check_and_fail(error, "SCIInitialize", "server");

    SCIOpen(&dma->sd, SCI_NO_FLAGS, &error);
    sci_check_and_fail(error, "SCIOpen", "server");

    SCICreateDMAQueue(dma->sd, &dma->dma_queue, ADAPTER_NO, 1, SCI_NO_FLAGS, &error);
    sci_check_and_fail(error, "SCICreateDMAQueue", "server");

    // Segment
    SCICreateSegment(dma->sd, &dma->local_segment, GET_SEGMENTID(READER), 2 * aligned_size, SCI_NO_CALLBACK,
      NULL, SCI_NO_FLAGS, &error);
    sci_check_and_fail(error, "SCICreateSegment", "server");

    SCIPrepareSegment(dma->local_segment, ADAPTER_NO, SCI_NO_FLAGS, &error);
    sci_check_and_fail(error, "SCIPrepareSegment", "server");

    SCISetSegmentAvailable(dma->local_segment, ADAPTER_NO, SCI_NO_FLAGS, &error);
    sci_check_and_fail(error, "SCISetSegmentAvailable", "server");

    dma->segment_map = SCIMapLocalSegment(dma->local_segment, &dma->segment_map, 0, 2 * aligned_size, 
      NULL, SCI_NO_FLAGS, &error);
    sci_check_and_fail(error, "SCIMapLocalSegment", "server");
    buffer = (uint8_t *) dma->segment_map;
}

static void connect_remote_segment(dma_buffer_t *dma, unsigned int worker_id)
{
    sci_error_t error;

    do 
    {
      SCIConnectSegment(dma->sd, &remote_seg, worker_id, GET_SEGMENTID(WORKER_DATA), ADAPTER_NO, SCI_NO_CALLBACK,
          SCI_NO_ARG, SCI_INFINITE_TIMEOUT, SCI_NO_FLAGS, &error);
    } while (error != SCI_ERR_OK);

    fprintf(stderr, "connection done! (server to worker)\n");
}

static sci_callback_action_t dma_completion_callback(void* arg, sci_dma_queue_t dma_queue, sci_error_t status)
{
  dma_context_t *ctx = (dma_context_t *) arg;
  
  config_t *config = ctx->config;
  int buf = ctx->buf;

  config->dma_queue_state[buf] = TRANSFER_COMPLETED;

  return SCI_CALLBACK_CONTINUE;
}

static void send_frame_data(dma_buffer_t *dma, int buf, dma_context_t *dma_ctx)
{
    sci_error_t error;

    size_t offset = buf * dma->total_size;

    dma->config->dma_queue_state[buf] = TRANSFERRING;
    SCIStartDmaTransfer(dma->dma_queue, dma->local_segment, remote_seg, offset, dma->total_size, offset,
        dma_completion_callback, dma_ctx, SCI_FLAG_USE_CALLBACK, &error);
    sci_check_and_fail(error, "SCIStartDMATransfer", "server");
}

static void sci_cleanup(dma_buffer_t *dma)
{
    sci_error_t error;

    // Segments
    SCIDisconnectSegment(remote_seg, SCI_NO_FLAGS, &error);
    SCIUnmapSegment(dma->segment_map, SCI_NO_FLAGS, &error);
    SCIRemoveSegment(dma->local_segment, SCI_NO_FLAGS, &error);

    // Config
    SCIRemoveSegment(dma->control_segment, SCI_NO_FLAGS, &error);

    // Rest
    SCIRemoveDMAQueue(dma->dma_queue, SCI_NO_FLAGS, &error);
    SCIClose(dma->sd, SCI_NO_FLAGS, &error);

    SCITerminate();
}

static void wait_for_worker(config_t *config, int buf)
{
    while (config->dma_queue_state[buf] == BUSY) {fprintf(stderr, "server prune wait for worker\n");}
}

static void sci_init_control(dma_buffer_t *dma)
{
    sci_error_t error;

    size_t size = sizeof(config_t);

    SCICreateSegment(dma->sd, &dma->control_segment, GET_SEGMENTID(READER_WORKER_CTRL), size, SCI_NO_CALLBACK,
        NULL, SCI_NO_FLAGS, &error);
    sci_check_and_fail(error, "SCICreateSegment", "server");

    SCIPrepareSegment(dma->control_segment, ADAPTER_NO, SCI_NO_FLAGS, &error);
    sci_check_and_fail(error, "SCIPrepareSegment", "server");

    SCISetSegmentAvailable(dma->control_segment, ADAPTER_NO, SCI_NO_FLAGS, &error);
    sci_check_and_fail(error, "SCISetSegmentAvailable", "server");

    sci_map_t local_map;

    dma->config = (config_t *) SCIMapLocalSegment(dma->control_segment, &local_map, 0, size,
        NULL, SCI_NO_FLAGS, &error);
    sci_check_and_fail(error, "SCIMapLocalSegment", "server");

    dma->config->initialized = 0;
    dma->config->width = 0;
    dma->config->height = 0;
    dma->config->writer = 0;
    
    dma->config->complete = ONGOING;
    dma->config->ack      = ONGOING;
}


int main(int argc, char **argv)
{
  int c;
  int w = 0; /* worker index */
  uint8_t *image;

  // SCI variables
  dma_buffer_t dma;
  dma_context_t dma_ctx[NUM_SEG];

  if (argc == 1) { print_help(); }

  while ((c = getopt(argc, argv, "r:h:w:o:f:i:")) != -1)
  {
    switch (c)
    {
      case 'r':
        worker_nodes[w++] = atoi(optarg);
        break;
      case 'h':
        height = atoi(optarg);
        break;
      case 'w':
        width = atoi(optarg);
        break;
      case 'o':
        writer_node = atoi(optarg);
        break;
      case 'f':
        limit_numframes = atoi(optarg);
        break;
      default:
        print_help();
        break;
    }
  }

  if (optind >= argc)
  {
    fprintf(stderr, "Error getting program options, try --help.\n");
    exit(EXIT_FAILURE);
  }

//   struct c63_common *cm = init_c63_enc(width, height);

  // SCI init
  fprintf(stderr, "server prune 1\n");
  sci_init(&dma);
  fprintf(stderr, "server prune 2\n");
  // control
  sci_init_control(&dma);
  dma.config->dma_queue_state[0] = dma.config->dma_queue_state[1] = BUSY;
  dma.config->width = width;
  dma.config->height = height;
  dma.config->writer = writer_node;
  dma.config->initialized = 1;

  fprintf(stderr, "server prune 3\n");
  dma_ctx[0].config = dma_ctx[1].config = dma.config;
  dma_ctx[0].buf = 0; dma_ctx[1].buf = 1;
  fprintf(stderr, "server prune 4\n");

  connect_remote_segment(&dma, worker_nodes[0]);

  input_file = argv[optind];

  if (limit_numframes) { printf("Limited to %d frames.\n", limit_numframes); }

  FILE *infile = fopen(input_file, "rb");
  fprintf(stderr, "reached here server\n");

  if (infile == NULL)
  {
    perror("fopen");
    exit(EXIT_FAILURE);
  }

  /* Encode input frames */
  int numframes = 0;

  printf("server: Hello World!\n");
  printf("server: input file %s\n", input_file);
  printf("server: %ux%u\n", width, height);
  
  for (int i = 0; i < MAX_NUM_WORKERS; i++) {
    printf("server: Worker%d has nodeid: %u\n", i, worker_nodes[i]);
  }

  printf("server: Writer nodeid: %u\n", writer_node);
  
  /* TODO: Time to read and encode the video */

  int buf = 0;
  
  while (1)
  {
    int curr_buf = buf;
    buf ^= 1; 

    wait_for_worker(dma.config, curr_buf);

    image = read_yuv(infile, &dma, width, height, curr_buf);
    if (!image) { break; }

    printf("Encoding frame %d, ", numframes);
    send_frame_data(&dma, curr_buf, &dma_ctx[curr_buf]);
    printf("Done!\n");

    ++numframes;

    if (limit_numframes && numframes >= limit_numframes) { break; }
  }

  // send signal to close workers
  dma.config->complete = DONE;
  dma.config->dma_queue_state[0] = dma.config->dma_queue_state[1] = TRANSFER_COMPLETED;
  while (dma.config->complete != ACKNOWLEDGED) {fprintf(stderr, "server prune waiting ack\n");}

  fclose(infile);
  sci_cleanup(&dma);

  //int i, j;
  //for (i = 0; i < 2; ++i)
  //{
  //  printf("int freq[] = {");
  //  for (j = 0; j < ARRAY_SIZE(frequencies[i]); ++j)
  //  {
  //    printf("%d, ", frequencies[i][j]);
  //  }
  //  printf("};\n");
  //}

  return EXIT_SUCCESS;
}
