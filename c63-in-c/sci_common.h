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

#define MAX_YPW  (1920 / 2)
#define MAX_YPH  (1088 / 2)
#define MAX_UPW  (960 / 2)
#define MAX_UPH  (544 / 2)
#define MAX_VPW  (960 / 2)
#define MAX_VPH  (544 / 2)
#define MAX_MB_COLS  ((MAX_YPW / 8) / 2)
#define MAX_MB_ROWS  ((MAX_YPH / 8) / 2)

typedef enum 
{
    READER,
    WORKER_DATA,
    WORKER_DATA_2,
    WORKER_ENCODED,
    WRITER,
    READER_WORKER_CTRL,
    READER_WORKER_CTRL_2,
    WORKER_WRITER_CTRL,
    WORKER_WRITER_CTRL_2,
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

static void sci_check_and_fail(sci_error_t err, const char *ctx, const char *loc)
{
    if (err != SCI_ERR_OK)
    {
        fprintf(stderr, "SCI error in %s (%s): %s\n", ctx, loc, SCIGetErrorString(err));
        SCITerminate();
        exit(EXIT_FAILURE);
    }
}

#endif