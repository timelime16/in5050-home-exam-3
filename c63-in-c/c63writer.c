#include <assert.h>
#include <errno.h>
#include <getopt.h>
#include <limits.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include <sisci_error.h>
#include <sisci_api.h>

#include "common.h"
#include "c63.h"
#include "c63_write.h"
#include "tables.h"
#include "sci_common.h"

static char *output_file;
FILE *outfile;

static uint32_t server_node = 0;
static uint32_t worker_nodes[MAX_NUM_WORKERS] = {};
static int width = 0;
static int height = 0;

/* getopt */
extern int optind;
extern char *optarg;

typedef struct
{
    sci_desc_t sd;

    sci_local_segment_t writer_job_segment;

    sci_map_t writer_map;

    writer_job_t *buffer[MAX_NUM_WORKERS];

} writer_t;

static sci_remote_segment_t worker_remote_control_seg[MAX_NUM_WORKERS];
static sci_map_t worker_remote_control_map[MAX_NUM_WORKERS];


static void print_help()
{
  printf("Usage: ./c63worker -s server -r workers\n");
  printf("Commandline options:\n");
  printf("  -r               Node id of server\n");
  printf("  -w               Node id of workers\n");
  printf("  -o               Output file (.c63)\n");
  printf("\n");

  exit(EXIT_FAILURE);
}

struct c63_common* init_c63_enc(int width, int height)
{
  int i;

  /* calloc() sets allocated memory to zero */
  struct c63_common *cm = calloc(1, sizeof(struct c63_common));

  cm->width = width;
  cm->height = height;

  cm->padw[Y_COMPONENT] = cm->ypw = (uint32_t)(ceil(width/16.0f)*16);
  cm->padh[Y_COMPONENT] = cm->yph = (uint32_t)(ceil(height/16.0f)*16);
  cm->padw[U_COMPONENT] = cm->upw = (uint32_t)(ceil(width*UX/(YX*8.0f))*8);
  cm->padh[U_COMPONENT] = cm->uph = (uint32_t)(ceil(height*UY/(YY*8.0f))*8);
  cm->padw[V_COMPONENT] = cm->vpw = (uint32_t)(ceil(width*VX/(YX*8.0f))*8);
  cm->padh[V_COMPONENT] = cm->vph = (uint32_t)(ceil(height*VY/(YY*8.0f))*8);

  cm->mb_cols = cm->ypw / 8;
  cm->mb_rows = cm->yph / 8;

  /* Quality parameters -- Home exam deliveries should have original values,
   i.e., quantization factor should be 25, search range should be 16, and the
   keyframe interval should be 100. */
  cm->qp = 25;                  // Constant quantization factor. Range: [1..50]
  cm->me_search_range = 16;     // Pixels in every direction
  cm->keyframe_interval = 100;  // Distance between keyframes

  /* Initialize quantization tables */
  for (i = 0; i < 64; ++i)
  {
    cm->quanttbl[Y_COMPONENT][i] = yquanttbl_def[i] / (cm->qp / 10.0);
    cm->quanttbl[U_COMPONENT][i] = uvquanttbl_def[i] / (cm->qp / 10.0);
    cm->quanttbl[V_COMPONENT][i] = uvquanttbl_def[i] / (cm->qp / 10.0);
  }

  return cm;
}

void free_c63_enc(struct c63_common* cm)
{
  destroy_frame(cm->curframe);
  free(cm);
}



// SCI helper
static void sci_init(writer_t *writer)
{
  sci_error_t error;

  SCIInitialize(SCI_NO_FLAGS, &error);
  sci_check_and_fail(error, "SCIInitialize", "writer");

  SCIOpen(&writer->sd, SCI_NO_FLAGS, &error);
  sci_check_and_fail(error, "SCIOpen", "writer");
}

static config_t *sci_init_control(writer_t *writer, int i) 
{
  sci_error_t error;
  c63_segment ctrl_seg = WORKER_WRITER_CTRL + i;
  int count = 0;

  do 
  {
    if (count++ == MAX_RETRY)
    {
      sci_check_and_fail(error, "SCIConnectSegment", "writer to worker");
    }
    SCIConnectSegment(writer->sd, &worker_remote_control_seg[i], worker_nodes[i], GET_SEGMENTID(ctrl_seg), ADAPTER_NO,
        SCI_NO_CALLBACK, SCI_NO_ARG, SCI_INFINITE_TIMEOUT, SCI_NO_FLAGS, &error);
  } while (error != SCI_ERR_OK);

  fprintf(stderr, "connection done! ctrl (writer to worker)\n");

  config_t *config = (config_t *) SCIMapRemoteSegment(worker_remote_control_seg[i], &worker_remote_control_map[i], 0, sizeof(config_t),
      NULL, SCI_NO_FLAGS, &error);
  sci_check_and_fail(error, "SCIMapRemoteSegment", "writer");

  // only done once
  if (i == 0)
  {
    while (!config->initialized);
    width = config->width;
    height = config->height;
  }

  return config;
}

static void sci_init_writer(writer_t *writer)
{
    sci_error_t error;

    SCICreateSegment(writer->sd, &writer->writer_job_segment, GET_SEGMENTID(WRITER), 4 * sizeof(writer_job_t), SCI_NO_CALLBACK,
      NULL, SCI_NO_FLAGS, &error);
    sci_check_and_fail(error, "SCICreateSegment", "writer");

    SCIPrepareSegment(writer->writer_job_segment, ADAPTER_NO, SCI_NO_FLAGS, &error);
    sci_check_and_fail(error, "SCIPrepareSegment", "writer");

    SCISetSegmentAvailable(writer->writer_job_segment, ADAPTER_NO, SCI_NO_FLAGS, &error);
    sci_check_and_fail(error, "SCISetSegmentAvailable", "writer");

    writer->buffer = (writer_job_t *[MAX_NUM_WORKERS]) SCIMapLocalSegment(writer->writer_job_segment, &writer->writer_map, 0, 4 * sizeof(writer_job_t), 
      NULL, SCI_NO_FLAGS, &error);
    sci_check_and_fail(error, "SCIMapLocalSegment", "writer");
}

static inline void wait_for_workers(config_t *config[MAX_NUM_WORKERS], int buf) 
{
  int i;
  #pragma unroll
  for (i = 0; i < MAX_NUM_WORKERS; ++i)
  {
    while (config[i]->dma_queue_state[buf] != TRANSFER_COMPLETED);
    config[i]->dma_queue_state[buf] = BUSY;
  }
}

static void sci_cleanup(writer_t *writer)
{
  int i;
  sci_error_t error;

  SCIUnmapSegment(writer->writer_map, SCI_NO_FLAGS, &error);
  SCIRemoveSegment(writer->writer_job_segment, SCI_NO_FLAGS, &error);

  #pragma unroll
  for (i = 0; i < MAX_NUM_WORKERS; ++i)
  {
    SCIUnmapSegment(worker_remote_control_map[i], SCI_NO_FLAGS, & error);
    SCIDisconnectSegment(worker_remote_control_seg[i], SCI_NO_FLAGS, &error);
  }

  SCIClose(writer->sd, SCI_NO_FLAGS, &error);
  SCITerminate();
}



int main(int argc, char **argv)
{ 
  int c;
  int w = 0; /* worker index */

  writer_t writer_ctx;
  config_t *config[MAX_NUM_WORKERS];

  if (argc == 1) { print_help(); }

  while ((c = getopt(argc, argv, "r:s:o:")) != -1)
  {
    switch (c)
    {
      case 's':
        server_node = atoi(optarg);
        break;
      case 'r':
        worker_nodes[w++] = atoi(optarg);
        break;
      case 'o':
        output_file = optarg;
        break;
      default:
        print_help();
        break;
    }
  }
  
  outfile = fopen(output_file, "wb");

  if (outfile == NULL)
  {
    perror("fopen");
    exit(EXIT_FAILURE);
  }


  /* Initialize the SISCI library */
  sci_init(&writer_ctx);

  sci_init_writer(&writer_ctx);

  int i;
  #pragma unroll
  for (i = 0; i < MAX_NUM_WORKERS; ++i)
  {
    config[i] = sci_init_control(&writer_ctx, i);
  }

  struct c63_common *cm = init_c63_enc(width, height);
  cm->e_ctx.fp = outfile;
  cm->curframe = create_frame(cm, NULL);

  printf("writer: Hello World!\n");

  /* FIXME: You should remove this when you have real data to write */
  // fwrite("HELLO\n", 6, 1, outfile);

  size_t dct_size_y = cm->ypw * cm->yph * sizeof(int16_t) / 2;
  size_t dct_size_u = cm->upw * cm->uph * sizeof(int16_t) / 2;
  size_t dct_size_v = cm->vpw * cm->vph * sizeof(int16_t) / 2;
  size_t mb_size_y = cm->mb_cols * cm->mb_rows * sizeof(struct macroblock) / 2;
  size_t mb_size_uv = (cm->mb_cols/2) * (cm->mb_rows/2) * sizeof(struct macroblock) / 2;

  int buf = 0;

  while (1) 
  {
    wait_for_workers(config, buf);

    #pragma unroll
    for (i = 0; i < MAX_NUM_WORKERS; ++i)
    {
      if (config[i]->complete == DONE) { break; }
    }

    cm->curframe->keyframe = writer_ctx.buffer[0][0].keyframe;
    for (i = 0; i < MAX_NUM_WORKERS; ++i)
    {
        memcpy(cm->curframe->residuals->Ydct + i * dct_size_y,   writer_ctx.buffer[i][buf].Ydct,  dct_size_y);
        memcpy(cm->curframe->residuals->Udct + i * dct_size_u,   writer_ctx.buffer[i][buf].Udct,  dct_size_u);
        memcpy(cm->curframe->residuals->Vdct + i * dct_size_v,   writer_ctx.buffer[i][buf].Vdct,  dct_size_v);
        memcpy(cm->curframe->mbs[0]          + i * mb_size_y/sizeof(struct macroblock),  writer_ctx.buffer[i][buf].mbs_Y, mb_size_y);
        memcpy(cm->curframe->mbs[1]          + i * mb_size_uv/sizeof(struct macroblock), writer_ctx.buffer[i][buf].mbs_U, mb_size_uv);
        memcpy(cm->curframe->mbs[2]          + i * mb_size_uv/sizeof(struct macroblock), writer_ctx.buffer[i][buf].mbs_V, mb_size_uv);
    }

    write_frame(cm);

    for (i = 0; i < MAX_NUM_WORKERS; ++i)
    {
      config[i]->dma_queue_state[0] = AVAILABLE;
    }

    buf ^= 1;
  }

  for (i = 0; i < MAX_NUM_WORKERS; ++i)
  {
    config[i]->ack = ACKNOWLEDGED;
  }

  fclose(outfile);
  free_c63_enc(cm);
  sci_cleanup(&writer_ctx);


  return 0;

}
