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

static sci_remote_segment_t writer_remote_seg;
static sci_remote_segment_t reader_remote_control_seg;

static writer_job_t *writer_job_ctx;

typedef struct
{
    sci_desc_t sd;

    sci_local_segment_t frame_segment;

    void *frame_map;

    uint8_t *frame_buffer;

} worker_t;



static void c63_encode_image(struct c63_common *cm, yuv_t *image)
{
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
    /* Motion Estimation */
    c63_motion_estimate(cm);

    /* Motion Compensation */
    c63_motion_compensate(cm);
  }

  /* DCT and Quantization */
  dct_quantize(image->Y, cm->curframe->predicted->Y, cm->padw[Y_COMPONENT],
      cm->padh[Y_COMPONENT], cm->curframe->residuals->Ydct,
      cm->quanttbl[Y_COMPONENT]);

  dct_quantize(image->U, cm->curframe->predicted->U, cm->padw[U_COMPONENT],
      cm->padh[U_COMPONENT], cm->curframe->residuals->Udct,
      cm->quanttbl[U_COMPONENT]);

  dct_quantize(image->V, cm->curframe->predicted->V, cm->padw[V_COMPONENT],
      cm->padh[V_COMPONENT], cm->curframe->residuals->Vdct,
      cm->quanttbl[V_COMPONENT]);

  /* Reconstruct frame for inter-prediction */
  dequantize_idct(cm->curframe->residuals->Ydct, cm->curframe->predicted->Y,
      cm->ypw, cm->yph, cm->curframe->recons->Y, cm->quanttbl[Y_COMPONENT]);
  dequantize_idct(cm->curframe->residuals->Udct, cm->curframe->predicted->U,
      cm->upw, cm->uph, cm->curframe->recons->U, cm->quanttbl[U_COMPONENT]);
  dequantize_idct(cm->curframe->residuals->Vdct, cm->curframe->predicted->V,
      cm->vpw, cm->vph, cm->curframe->recons->V, cm->quanttbl[V_COMPONENT]);

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

  do 
  {
    SCIConnectSegment(worker->sd, &reader_remote_control_seg, server_node, GET_SEGMENTID(READER_WORKER_CTRL), ADAPTER_NO,
        SCI_NO_CALLBACK, SCI_NO_ARG, SCI_INFINITE_TIMEOUT, SCI_NO_FLAGS, &error);
    fprintf(stderr, "retrying connection (worker to server)\n");
  } while (error != SCI_ERR_OK);

  config_t *config = (config_t *) SCIMapRemoteSegment(reader_remote_control_seg, NULL, 0, sizeof(config_t),
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
  sci_error_t error;
  
  SCIOpen(&dma->sd, SCI_NO_FLAGS, &error);
  sci_check_and_fail(error, "SCIOpen", "worker");

  SCICreateDMAQueue(dma->sd, &dma->dma_queue, ADAPTER_NO, 1, SCI_NO_FLAGS, &error);
  sci_check_and_fail(error, "SCICreateDMAQueue", "worker");

  // Segment
  SCICreateSegment(dma->sd, &dma->local_segment, GET_SEGMENTID(WORKER), sizeof(writer_job_t), SCI_NO_CALLBACK,
    NULL, SCI_NO_FLAGS, &error);
  sci_check_and_fail(error, "SCICreateSegment", "worker");

  SCIPrepareSegment(dma->local_segment, ADAPTER_NO, SCI_NO_FLAGS, &error);
  sci_check_and_fail(error, "SCIPrepareSegment", "worker");

  SCISetSegmentAvailable(dma->local_segment, ADAPTER_NO, SCI_NO_FLAGS, &error);
  sci_check_and_fail(error, "SCISetSegmentAvailable", "worker");

  dma->segment_map = SCIMapLocalSegment(dma->local_segment, &dma->segment_map, 0, sizeof(writer_job_t), 
    NULL, SCI_NO_FLAGS, &error);
  sci_check_and_fail(error, "SCIMapLocalSegment", "worker");

  writer_job_ctx = (writer_job_t *) dma->segment_map;

  // Control
  SCICreateSegment(dma->sd, &dma->control_segment, GET_SEGMENTID(WORKER_WRITER_CTRL), sizeof(config_t), SCI_NO_CALLBACK,
      NULL, SCI_NO_FLAGS, &error);
  sci_check_and_fail(error, "SCICreateSegment", "worker");

  SCIPrepareSegment(dma->control_segment, ADAPTER_NO, SCI_NO_FLAGS, &error);
  sci_check_and_fail(error, "SCIPrepareSegment", "worker");

  SCISetSegmentAvailable(dma->control_segment, ADAPTER_NO, SCI_NO_FLAGS, &error);
  sci_check_and_fail(error, "SCISetSegmentAvailable", "worker");

  dma->config = (config_t *) SCIMapLocalSegment(dma->control_segment, NULL, 0, sizeof(config_t),
      NULL, SCI_NO_FLAGS, &error);
  sci_check_and_fail(error, "SCIMapLocalSegment", "worker");

  dma->config->initialized = 0;
  dma->config->width = 0;
  dma->config->height = 0;
  dma->config->writer = 0;
  
  dma->config->complete = ONGOING;
  dma->config->ack      = ONGOING;
}


static void sci_init_worker(worker_t *worker, size_t total_size)
{
    sci_error_t error;

    size_t segment_size = 2 * total_size;

    SCICreateSegment(worker->sd, &worker->frame_segment, GET_SEGMENTID(WORKER), segment_size, SCI_NO_CALLBACK,
      NULL, SCI_NO_FLAGS, &error);
    sci_check_and_fail(error, "SCICreateSegment", "worker");

    SCIPrepareSegment(worker->frame_segment, ADAPTER_NO, SCI_NO_FLAGS, &error);
    sci_check_and_fail(error, "SCIPrepareSegment", "worker");

    SCISetSegmentAvailable(worker->frame_segment, ADAPTER_NO, SCI_NO_FLAGS, &error);
    sci_check_and_fail(error, "SCISetSegmentAvailable", "worker");

    worker->frame_map = SCIMapLocalSegment(worker->frame_segment, NULL, 0, segment_size, 
      NULL, SCI_NO_FLAGS, &error);
    sci_check_and_fail(error, "SCIMapLocalSegment", "worker");

    worker->frame_buffer = (uint8_t *) worker->frame_map;
}

static void connect_remote_segment(sci_desc_t *sd)
{
    sci_error_t error;

    do 
    {
      SCIConnectSegment(*sd, &writer_remote_seg, writer_node, GET_SEGMENTID(WRITER), ADAPTER_NO, SCI_NO_CALLBACK,
          SCI_NO_ARG, SCI_INFINITE_TIMEOUT, SCI_NO_FLAGS, &error);
      fprintf(stderr, "retrying connection (worker to writer)\n");
    } while (error != SCI_ERR_OK);
}

static sci_callback_action_t dma_completion_callback(void* arg, sci_dma_queue_t dma_queue, sci_error_t status)
{
  dma_buffer_t *dma = (dma_buffer_t *) arg;
  dma->config->dma_queue_state[0] = TRANSFER_COMPLETED;
  return SCI_CALLBACK_CONTINUE;
}

static void send_encoded_data(dma_buffer_t *dma)
{
    sci_error_t error;

    dma->config->dma_queue_state[0] = TRANSFERRING;
    SCIStartDmaTransfer(dma->dma_queue, dma->local_segment, writer_remote_seg, 0, sizeof(writer_job_t), 0,
        dma_completion_callback, dma, SCI_FLAG_USE_CALLBACK, &error);
    sci_check_and_fail(error, "SCIStartDMATransfer", "worker");
}

static inline void wait_for_writer(config_t *config)
{
    while (config->dma_queue_state[0] == BUSY);
}

static void sci_cleanup(worker_t *worker, dma_buffer_t *dma)
{
  sci_error_t error;

  SCIUnmapSegment(worker->frame_map, SCI_NO_FLAGS, &error);
  SCIRemoveSegment(worker->frame_segment, SCI_NO_FLAGS, &error);
  SCIDisconnectSegment(reader_remote_control_seg, SCI_NO_FLAGS, &error);

  SCIUnmapSegment(dma->segment_map, SCI_NO_FLAGS, &error);
  SCIRemoveSegment(dma->local_segment, SCI_NO_FLAGS, &error);
  SCIRemoveSegment(dma->control_segment, SCI_NO_FLAGS, &error);
  SCIDisconnectSegment(writer_remote_seg, SCI_NO_FLAGS, &error);
  SCIRemoveDMAQueue(dma->dma_queue, SCI_NO_FLAGS, &error);

  SCIClose(worker->sd, SCI_NO_FLAGS, &error);
  SCIClose(dma->sd, SCI_NO_FLAGS, &error);
  SCITerminate();
}


int main(int argc, char **argv)
{
    //TODO: REMOVE
  fprintf(stderr, "reader = %d, worker = %d, writer = %d, rwctrl = %d, wwctrl = %d", GET_SEGMENTID(READER), GET_SEGMENTID(WORKER), GET_SEGMENTID(WRITER), GET_SEGMENTID(READER_WORKER_CTRL), GET_SEGMENTID(WORKER_WRITER_CTRL));

  int c;
  int w = 0; /* worker index */

  yuv_t image;

  worker_t worker_ctx;
  config_t *reader_config;
  dma_buffer_t dma;

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
  reader_config = sci_init_control(&worker_ctx);
  size_t y_size = width * height;
  size_t uv_size = y_size / 4;
  size_t total_size = y_size + 2 * uv_size;
  size_t aligned_size = ((total_size + 4095) / 4096) * 4096;
  sci_init_worker(&worker_ctx, aligned_size);
  sci_init_dma_ctx(&dma);
  connect_remote_segment(&dma.sd);
  
  dma.config->dma_queue_state[0] = dma.config->dma_queue_state[1] = BUSY;
  dma.config->width = width;
  dma.config->height = height;
  dma.config->writer = writer_node;
  dma.config->initialized = 1;

  struct c63_common *cm = init_c63_enc(width, height);

  image.Y = (uint8_t *) malloc(y_size * sizeof(uint8_t));
  image.U = (uint8_t *) malloc(uv_size * sizeof(uint8_t));
  image.V = (uint8_t *) malloc(uv_size * sizeof(uint8_t));

  int buf = 0;

  while (1) 
  {
    while (reader_config->dma_queue_state[buf] != TRANSFER_COMPLETED);
    reader_config->dma_queue_state[buf] = BUSY;

    if (reader_config->complete == DONE) { break; }

    uint8_t *frame = worker_ctx.frame_buffer + buf * total_size;

    memcpy(image.Y, frame, y_size);
    memcpy(image.U, frame + y_size, uv_size);
    memcpy(image.V, frame + y_size + uv_size, uv_size);

    c63_encode_image(cm, &image);

    // Send to writer
    wait_for_writer(dma.config);

    writer_job_ctx->keyframe = cm->curframe->keyframe;
    memcpy(writer_job_ctx->Ydct, cm->curframe->residuals->Ydct, y_size * sizeof(int16_t));
    memcpy(writer_job_ctx->Udct, cm->curframe->residuals->Udct, uv_size * sizeof(int16_t));
    memcpy(writer_job_ctx->Vdct, cm->curframe->residuals->Vdct, uv_size * sizeof(int16_t));
    memcpy(writer_job_ctx->mbs_Y, cm->curframe->mbs[0], cm->mb_cols * cm->mb_rows * sizeof(struct macroblock));
    memcpy(writer_job_ctx->mbs_U, cm->curframe->mbs[1], (cm->mb_cols/2) * (cm->mb_rows/2) * sizeof(struct macroblock));
    memcpy(writer_job_ctx->mbs_V, cm->curframe->mbs[2], (cm->mb_cols/2) * (cm->mb_rows/2) * sizeof(struct macroblock));

    send_encoded_data(&dma);

    reader_config->dma_queue_state[buf] = AVAILABLE;

    buf ^= 1;
  }

  printf("worker: Hello World!\n");

  dma.config->complete = DONE;
  dma.config->dma_queue_state[0] = dma.config->dma_queue_state[1] = TRANSFER_COMPLETED;
  while (dma.config->complete != ACKNOWLEDGED);

  sci_cleanup(&worker_ctx, &dma);
  free_c63_enc(cm);
  free(image.Y);
  free(image.U);
  free(image.V);

  return 0;

}
