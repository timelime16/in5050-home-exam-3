#include <assert.h>
#include <errno.h>
#include <getopt.h>
#include <limits.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdatomic.h>
#include <omp.h>

#include <sisci_error.h>
#include <sisci_api.h>

#include "c63.h"
#include "c63_write.h"
#include "common.h"
#include "me.h"
#include "tables.h"
#include "sci_common.h"


/* getopt */
extern int optind;
extern char *optarg;

// SCI variables
static uint32_t server_node = 0;
static uint32_t worker_nodes[MAX_NUM_WORKERS] = {};
static uint32_t writer_node = 0;
static int width = 0;
static int height = 0;

static int worker_order = -1;

static sci_remote_segment_t writer_remote_seg;
static sci_remote_segment_t reader_remote_control_seg;

static sci_map_t writer_remote_map;
static sci_map_t reader_remote_control_map;


static writer_job_t *writer_job_ctx[NUM_SEG];

typedef struct
{
    sci_desc_t sd;

    sci_local_segment_t frame_segment;

    sci_map_t frame_map;

    uint8_t *frame_buffer;

} worker_t;

// DMA data transfer
typedef struct dma_buffer 
{
    sci_desc_t sd;

    sci_local_segment_t local_segment;
    sci_dma_queue_t dma_queue[NUM_SEG];

    sci_map_t segment_map;

    size_t total_size;

    // control
    sci_local_segment_t control_segment;
    sci_map_t control_map;
    config_t *config;
} dma_buffer_t;

typedef struct
{
  config_t *config;
  int buf;
  int done;
} dma_context_t;



static void c63_encode_image(struct c63_common *cm, yuv_t *image)
{
  int start_mb_row, end_mb_row;
  int start_y, end_y;
  int start_u, end_u;
  int start_v, end_v;
  /* Advance to next frame */
  destroy_frame(cm->refframe);
  cm->refframe = cm->curframe;
  cm->curframe = create_frame(cm, image);

  /* Check if keyframe */
  if (cm->framenum == 0 || cm->frames_since_keyframe == cm->keyframe_interval)
  {
    cm->curframe->keyframe = 1;
    cm->frames_since_keyframe = 0;

    fprintf(stderr, " (keyframe) ");
  }
  else { cm->curframe->keyframe = 0; }

  if (!cm->curframe->keyframe)
  {
    start_mb_row = worker_order * cm->mb_rows / 2;
    end_mb_row = start_mb_row + cm->mb_rows / 2;

    /* Motion Estimation */
    c63_motion_estimate(cm, start_mb_row, end_mb_row);

    /* Motion Compensation */
    c63_motion_compensate(cm, start_mb_row, end_mb_row);
  }

  /* DCT and Quantization */
  start_y = worker_order * cm->padh[Y_COMPONENT] / 2;
  end_y = start_y + cm->padh[Y_COMPONENT] / 2;
  dct_quantize(image->Y, cm->curframe->predicted->Y, cm->padw[Y_COMPONENT],
      cm->padh[Y_COMPONENT], cm->curframe->residuals->Ydct,
      cm->quanttbl[Y_COMPONENT], start_y, end_y);

  start_u = worker_order * cm->padh[U_COMPONENT] / 2;
  end_u = start_u + cm->padh[U_COMPONENT] / 2;
  dct_quantize(image->U, cm->curframe->predicted->U, cm->padw[U_COMPONENT],
      cm->padh[U_COMPONENT], cm->curframe->residuals->Udct,
      cm->quanttbl[U_COMPONENT], start_u, end_u);

  start_v = worker_order * cm->padh[V_COMPONENT] / 2;
  end_v = start_v + cm->padh[V_COMPONENT] / 2;
  dct_quantize(image->V, cm->curframe->predicted->V, cm->padw[V_COMPONENT],
      cm->padh[V_COMPONENT], cm->curframe->residuals->Vdct,
      cm->quanttbl[V_COMPONENT], start_v, end_v);

  /* Reconstruct frame for inter-prediction */
  start_y = worker_order * cm->yph / 2;
  end_y = start_y + cm->yph / 2;
  dequantize_idct(cm->curframe->residuals->Ydct, cm->curframe->predicted->Y,
      cm->ypw, cm->yph, cm->curframe->recons->Y, cm->quanttbl[Y_COMPONENT], 
      start_y, end_y);
  start_u = worker_order * cm->uph / 2;
  end_u = start_u + cm->uph / 2;
  dequantize_idct(cm->curframe->residuals->Udct, cm->curframe->predicted->U,
      cm->upw, cm->uph, cm->curframe->recons->U, cm->quanttbl[U_COMPONENT],
      start_u, end_u);
  start_v = worker_order * cm->vph / 2;
  end_v = start_v + cm->vph / 2;
  dequantize_idct(cm->curframe->residuals->Vdct, cm->curframe->predicted->V,
      cm->vpw, cm->vph, cm->curframe->recons->V, cm->quanttbl[V_COMPONENT],
      start_v, end_v);

  /* Function dump_image(), found in common.c, can be used here to check if the
     prediction is correct */

  // write_frame(cm);

  ++cm->framenum;
  ++cm->frames_since_keyframe;
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

static void print_help()
{
  printf("Usage: ./c63worker -s server -r workers\n");
  printf("Commandline options:\n");
  printf("  -r               Node id of server\n");
  printf("  -w               Node id of workers\n");
  printf("\n");

  exit(EXIT_FAILURE);
}



// SCI helpers
static void sci_init(worker_t *worker)
{
  sci_error_t error;

  SCIInitialize(SCI_NO_FLAGS, &error);
  sci_check_and_fail(error, "SCIInitialize", "worker");

  SCIOpen(&worker->sd, SCI_NO_FLAGS, &error);
  sci_check_and_fail(error, "SCIOpen", "worker");
}

static config_t *sci_init_control(worker_t *worker) 
{
  sci_error_t error;
  c63_segment ctrl_seg = READER_WORKER_CTRL + worker_order;
  printf("worker %d connecting to reader seg: %d\n", worker_order, ctrl_seg);
  int count = 0;
  do 
  {
    if (count++ == MAX_RETRY)
    {
      sci_check_and_fail(error, "SCIConnectSegment", "worker to reader");
    }
    SCIConnectSegment(worker->sd, &reader_remote_control_seg, server_node, GET_SEGMENTID(ctrl_seg), ADAPTER_NO,
        SCI_NO_CALLBACK, SCI_NO_ARG, SCI_INFINITE_TIMEOUT, SCI_NO_FLAGS, &error);
  } while (error != SCI_ERR_OK);

  fprintf(stderr, "connection done! ctrl (worker to server)\n");

  config_t *config = (config_t *) SCIMapRemoteSegment(reader_remote_control_seg, &reader_remote_control_map, 0, sizeof(config_t),
      NULL, SCI_NO_FLAGS, &error);
  sci_check_and_fail(error, "SCIMapRemoteSegment", "worker");

  while (!config->initialized);
  width = config->width;
  height = config->height;
  writer_node = config->writer;

  return config;
}

static void sci_init_dma_ctx(dma_buffer_t *dma)
{
  int i;
  sci_error_t error;
  
  SCIOpen(&dma->sd, SCI_NO_FLAGS, &error);
  sci_check_and_fail(error, "SCIOpen", "worker");

  #pragma unroll
  for (i = 0; i < NUM_SEG; ++i)
  {
    SCICreateDMAQueue(dma->sd, &dma->dma_queue[i], ADAPTER_NO, 1, SCI_NO_FLAGS, &error);
    sci_check_and_fail(error, "SCICreateDMAQueue", "worker");
  }

  // Segment
  c63_segment worker_seg = WORKER_ENCODED + worker_order;
  SCICreateSegment(dma->sd, &dma->local_segment, GET_SEGMENTID(worker_seg), 2 * sizeof(writer_job_t), SCI_NO_CALLBACK,
    NULL, SCI_NO_FLAGS, &error);
  sci_check_and_fail(error, "SCICreateSegment", "worker dma ctx");

  SCIPrepareSegment(dma->local_segment, ADAPTER_NO, SCI_NO_FLAGS, &error);
  sci_check_and_fail(error, "SCIPrepareSegment", "worker");

  SCISetSegmentAvailable(dma->local_segment, ADAPTER_NO, SCI_NO_FLAGS, &error);
  sci_check_and_fail(error, "SCISetSegmentAvailable", "worker");

  for (i = 0; i < NUM_SEG; ++i)
  {
    writer_job_ctx[i] = (writer_job_t *) SCIMapLocalSegment(dma->local_segment, &dma->segment_map, i * sizeof(writer_job_t), 
      sizeof(writer_job_t), NULL, SCI_NO_FLAGS, &error);
    sci_check_and_fail(error, "SCIMapLocalSegment", "worker");
  }

  // Control
  c63_segment ctrl_seg = WORKER_WRITER_CTRL + worker_order;
  SCICreateSegment(dma->sd, &dma->control_segment, GET_SEGMENTID(ctrl_seg), sizeof(config_t), SCI_NO_CALLBACK,
      NULL, SCI_NO_FLAGS, &error);
  sci_check_and_fail(error, "SCICreateSegment", "worker writer ctrl");

  SCIPrepareSegment(dma->control_segment, ADAPTER_NO, SCI_NO_FLAGS, &error);
  sci_check_and_fail(error, "SCIPrepareSegment", "worker");

  SCISetSegmentAvailable(dma->control_segment, ADAPTER_NO, SCI_NO_FLAGS, &error);
  sci_check_and_fail(error, "SCISetSegmentAvailable", "worker");

  dma->config = (config_t *) SCIMapLocalSegment(dma->control_segment, &dma->control_map, 0, sizeof(config_t),
      NULL, SCI_NO_FLAGS, &error);
  sci_check_and_fail(error, "SCIMapLocalSegment", "worker");

  dma->config->dma_queue_state[0] = dma->config->dma_queue_state[1] = AVAILABLE;
  dma->config->width = width;
  dma->config->height = height;
  dma->config->writer = writer_node;
  dma->config->complete[0] = dma->config->complete[1] = ONGOING;
  dma->config->ack      = ONGOING;
  dma->config->initialized = 1;
}


static void sci_init_worker(worker_t *worker, size_t total_size)
{
    sci_error_t error;

    size_t segment_size = 2 * total_size;

    c63_segment data_seg = WORKER_DATA + worker_order;

    SCICreateSegment(worker->sd, &worker->frame_segment, GET_SEGMENTID(data_seg), segment_size, SCI_NO_CALLBACK,
      NULL, SCI_NO_FLAGS, &error);
    sci_check_and_fail(error, "SCICreateSegment", "worker reader");

    SCIPrepareSegment(worker->frame_segment, ADAPTER_NO, SCI_NO_FLAGS, &error);
    sci_check_and_fail(error, "SCIPrepareSegment", "worker");

    SCISetSegmentAvailable(worker->frame_segment, ADAPTER_NO, SCI_NO_FLAGS, &error);
    sci_check_and_fail(error, "SCISetSegmentAvailable", "worker");

    worker->frame_buffer = (uint8_t *) SCIMapLocalSegment(worker->frame_segment, &worker->frame_map, 0, segment_size, 
      NULL, SCI_NO_FLAGS, &error);
    sci_check_and_fail(error, "SCIMapLocalSegment", "worker");
}

static void get_worker_order() 
{
  sci_error_t error;
  unsigned int node_id;
  SCIGetLocalNodeId(ADAPTER_NO, &node_id, SCI_NO_FLAGS, &error);
  sci_check_and_fail(error, "SCIGetLocalNodeId", "worker");

  printf("Node Id: %d\n", node_id);

  int i;
  for (i = 0; i < MAX_NUM_WORKERS; ++i) 
  {
    if (worker_nodes[i] == node_id) 
    {
      worker_order = i;
      break;
    }
  }

  printf("Node order: %d\n", worker_order);

  if (worker_order == -1) 
  {
    printf("Worker ID not found: %d\n", node_id);
    SCITerminate();
    exit(EXIT_FAILURE);
  }
}

static void connect_remote_segment(dma_buffer_t *dma)
{
    sci_error_t error;

    int count = 0;
    do 
    {
      if (count++ == MAX_RETRY)
      {
        sci_check_and_fail(error, "SCIConnectSegment", "worker to writer");
      }
      SCIConnectSegment(dma->sd, &writer_remote_seg, writer_node, GET_SEGMENTID(WRITER), ADAPTER_NO, SCI_NO_CALLBACK,
          SCI_NO_ARG, SCI_INFINITE_TIMEOUT, SCI_NO_FLAGS, &error);
    } while (error != SCI_ERR_OK);

    SCIMapRemoteSegment(writer_remote_seg, &writer_remote_map, 0, 4 * sizeof(writer_job_t), NULL, SCI_NO_FLAGS, &error);
    sci_check_and_fail(error, "SCIMapRemoteSegment", "server");

    fprintf(stderr, "connection done! (worker to writer)\n");
}

static sci_callback_action_t dma_completion_callback(void* arg, sci_dma_queue_t dma_queue, sci_error_t status)
{
  dma_context_t *ctx = (dma_context_t *) arg;
  int buf = ctx->buf;
  ctx->done = 0;
  ctx->config->dma_queue_state[buf] = TRANSFER_COMPLETED;
  ctx->done = 1;
  return SCI_CALLBACK_CONTINUE;
}

static void send_encoded_data(dma_buffer_t *dma, dma_context_t *dma_ctx, int buf)
{
    sci_error_t error;

    dma->config->dma_queue_state[buf] = TRANSFERRING;
    size_t local_offset = buf * sizeof(writer_job_t);
    size_t remote_offset = (buf + worker_order) * sizeof(writer_job_t);
    SCIStartDmaTransfer(dma->dma_queue[buf], dma->local_segment, writer_remote_seg, local_offset, sizeof(writer_job_t), remote_offset,
        dma_completion_callback, dma_ctx, SCI_FLAG_USE_CALLBACK, &error);
    sci_check_and_fail(error, "SCIStartDMATransfer", "worker");
}

static inline void wait_for_writer(config_t *config, int buf)
{
    while (config->dma_queue_state[buf] != AVAILABLE);
}

static void sci_cleanup(worker_t *worker, dma_buffer_t *dma)
{
  sci_error_t error;

  SCIUnmapSegment(worker->frame_map, SCI_NO_FLAGS, &error);
  SCIRemoveSegment(worker->frame_segment, SCI_NO_FLAGS, &error);

  SCIUnmapSegment(dma->segment_map, SCI_NO_FLAGS, &error);
  SCIRemoveSegment(dma->local_segment, SCI_NO_FLAGS, &error);
  SCIUnmapSegment(dma->control_map, SCI_NO_FLAGS, &error);
  SCIRemoveSegment(dma->control_segment, SCI_NO_FLAGS, &error);

  int i;
  #pragma unroll
  for (i = 0; i < NUM_SEG; ++i)
  {
    SCIRemoveDMAQueue(dma->dma_queue[i], SCI_NO_FLAGS, &error);
  }

  SCIClose(worker->sd, SCI_NO_FLAGS, &error);
  SCIClose(dma->sd, SCI_NO_FLAGS, &error);
  SCITerminate();
}



int main(int argc, char **argv)
{
  int c;
  int w = 0; /* worker index */

  yuv_t image;

  worker_t worker_ctx;
  config_t *reader_config;
  dma_buffer_t dma;
  dma_context_t dma_ctx[NUM_SEG];

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

  // SCI 
  sci_init(&worker_ctx);

  get_worker_order();

  reader_config = sci_init_control(&worker_ctx);

  size_t y_size = width * height;
  size_t uv_size = y_size / 4;
  size_t total_size = y_size + 2 * uv_size;
  size_t aligned_size = ((total_size + 4095) / 4096) * 4096;
  sci_init_worker(&worker_ctx, aligned_size);

  sci_init_dma_ctx(&dma);
  
  int i;
  #pragma unroll
  for (i = 0; i < NUM_SEG; ++i)
  {
    dma_ctx[i].config = dma.config;
    dma_ctx[i].buf = i;
    dma_ctx[i].done = 0;
  }

  connect_remote_segment(&dma);

  struct c63_common *cm = init_c63_enc(width, height);

  int dct_count_y = (cm->ypw * cm->yph) / 2;
  int dct_count_u = (cm->upw * cm->uph) / 2;
  int dct_count_v = (cm->vpw * cm->vph) / 2;
  int mb_count_y = (cm->mb_cols * cm->mb_rows) / 2;
  int mb_count_uv = ((cm->mb_cols/2) * (cm->mb_rows/2)) / 2;

  size_t dct_size_y = dct_count_y * sizeof(int16_t);
  size_t dct_size_u = dct_count_u * sizeof(int16_t);
  size_t dct_size_v = dct_count_v * sizeof(int16_t);
  size_t mb_size_y = mb_count_y * sizeof(struct macroblock);
  size_t mb_size_uv = mb_count_uv * sizeof(struct macroblock);

  #pragma unroll
  for (i = 0; i < NUM_SEG; ++i)
  {
    reader_config->dma_queue_state[i] = AVAILABLE;
  }

  int both_buf_done = 0;
  int buf = 0;
  
  while (1) 
  {
    while (reader_config->dma_queue_state[buf] != TRANSFER_COMPLETED);

    if (reader_config->complete[buf] == DONE) 
    { 
      if (both_buf_done) { break; }
      else 
      {
        both_buf_done = 1;
        buf ^= 1;
        continue;
      }
    }

    reader_config->dma_queue_state[buf] = BUSY;

    uint8_t *frame = worker_ctx.frame_buffer + buf * total_size;
    image.Y = frame;
    image.U = frame + y_size;
    image.V = frame + y_size + uv_size;

    c63_encode_image(cm, &image);

    // Send to writer
    wait_for_writer(dma.config, buf);

    writer_job_ctx[buf]->keyframe = cm->curframe->keyframe;
    memcpy(writer_job_ctx[buf]->Ydct, cm->curframe->residuals->Ydct + worker_order * dct_count_y, dct_size_y);
    memcpy(writer_job_ctx[buf]->Udct, cm->curframe->residuals->Udct + worker_order * dct_count_u, dct_size_u);
    memcpy(writer_job_ctx[buf]->Vdct, cm->curframe->residuals->Vdct + worker_order * dct_count_v, dct_size_v);
    memcpy(writer_job_ctx[buf]->mbs_Y, cm->curframe->mbs[0] + worker_order * mb_count_y, mb_size_y);
    memcpy(writer_job_ctx[buf]->mbs_U, cm->curframe->mbs[1] + worker_order * mb_count_uv, mb_size_uv);
    memcpy(writer_job_ctx[buf]->mbs_V, cm->curframe->mbs[2] + worker_order * mb_count_uv, mb_size_uv);

    send_encoded_data(&dma, &dma_ctx[buf], buf);

    printf("Worker %d sending encoded data to writer\n", worker_order);

    reader_config->dma_queue_state[buf] = AVAILABLE;

    buf ^= 1;
  }

  printf("worker: Hello World!\n");

  for (i = 0; i < NUM_SEG; ++i)
  {
    while (!dma_ctx[i].done);
  }

  for (i = 0; i < NUM_SEG; ++i)
  {
    while (dma.config->dma_queue_state[i] != AVAILABLE);
  }

  printf("Worker prune 0\n");

  printf("Worker prune 1\n");

  while (dma.config->ack != ACKNOWLEDGED);

  printf("Worker prune 2\n");

  sci_error_t error;
  SCIUnmapSegment(writer_remote_map, SCI_NO_FLAGS, &error);
  SCIDisconnectSegment(writer_remote_seg, SCI_NO_FLAGS, &error);

  reader_config->ack = ACKNOWLEDGED;
  SCIUnmapSegment(reader_remote_control_map, SCI_NO_FLAGS, &error);
  SCIDisconnectSegment(reader_remote_control_seg, SCI_NO_FLAGS, &error);

  printf("Worker prune 3\n");

  free_c63_enc(cm);

  printf("Worker prune 4\n");
  sci_cleanup(&worker_ctx, &dma);

  printf("Worker prune 5\n");

  return 0;

}
