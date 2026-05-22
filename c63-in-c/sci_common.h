#ifndef SCI_COMMON_H
#define SCI_COMMON_H

#include <stdio.h>
#include <stdlib.h>

#include <sisci_error.h>
#include <sisci_api.h>

#include "c63.h"

#define SCI_NO_FLAGS    0
#define SCI_NO_CALLBACK 0
#define SCI_NO_ARG      0

#define NUM_SEG 2

#define ADAPTER_NO 0

#define MAX_YPW  1920
#define MAX_YPH  1088  
#define MAX_UPW  960
#define MAX_UPH  544
#define MAX_VPW  960
#define MAX_VPH  544
#define MAX_MB_COLS  (MAX_YPW / 8)
#define MAX_MB_ROWS  (MAX_YPH / 8)

typedef enum 
{
    READER,
    WORKER,
    WRITER,
    READER_WORKER_CTRL,
    WORKER_WRITER_CTRL,
} c63_segment;

typedef enum 
{
    AVAILABLE,
    TRANSFERRING,
    TRANSFER_COMPLETED,
    BUSY,
} c63_state;

typedef enum 
{
    ONGOING, 
    DONE,
    ACKNOWLEDGED,
} c63_process_state;

typedef struct {
    int keyframe;

    int16_t Ydct[MAX_YPW * MAX_YPH];
    int16_t Udct[MAX_UPW * MAX_UPH];
    int16_t Vdct[MAX_VPW * MAX_VPH];

    struct macroblock mbs_Y[MAX_MB_COLS * MAX_MB_ROWS];
    struct macroblock mbs_U[(MAX_MB_COLS/2) * (MAX_MB_ROWS/2)];
    struct macroblock mbs_V[(MAX_MB_COLS/2) * (MAX_MB_ROWS/2)];
} writer_job_t;

// PIO sync
typedef struct 
{
    volatile c63_state dma_queue_state[NUM_SEG];

    volatile int width;
    volatile int height;
    volatile int writer;

    volatile c63_process_state complete;
    volatile c63_process_state ack;

    volatile int initialized;
} config_t;

// DMA data transfer
typedef struct dma_buffer 
{
    sci_desc_t sd;

    sci_local_segment_t local_segment;
    sci_dma_queue_t dma_queue;

    sci_map_t segment_map;

    size_t y_size;
    size_t uv_size;
    size_t total_size;

    // control
    sci_local_segment_t control_segment;
    config_t *config;
} dma_buffer_t;

static void sci_check_and_fail(sci_error_t err, const char *ctx)
{
    if (err != SCI_ERR_OK)
    {
        fprintf(stderr, "SCI error in %s: %s\n", ctx, SCIGetErrorString(err));
        SCITerminate();
        exit(EXIT_FAILURE);
    }
}

#endif