#include <assert.h>
#include <errno.h>
#include <getopt.h>
#include <limits.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <sisci_error.h>
#include <sisci_api.h>

#include "c63.h"
#include "c63_write.h"
#include "common.h"
#include "me.h"
#include "tables.h"

/* ── Shared segment ID constants (must match c63worker.c / c63writer.c) ── */
#define ADAPTER_NO           0

#define SEG_SRV_RAW_BASE     0x100   /* + worker_index → raw YUV frame       */
#define SEG_SRV_SIG_BASE     0x200   /* + worker_index → ack word from worker*/
#define SEG_WRK_SIG_BASE     0x300   /* + worker_index → cmd word to worker  */
/* (writer segment IDs are only needed by worker / writer, listed for docs)  */

/* PIO signal values */
#define SIG_IDLE    0u
#define SIG_READY   1u   /* server → worker: frame ready in DMA segment      */
#define SIG_ACK     2u   /* worker → server: frame consumed                  */
#define SIG_DONE    3u   /* server → worker: no more frames, shut down       */

/* ── Globals ─────────────────────────────────────────────────────────────── */
static char    *input_file;
static int      limit_numframes = 0;
static uint32_t worker_nodes[MAX_NUM_WORKERS];
static int      num_workers     = 0;
static uint32_t writer_node     = 0;
static uint32_t width           = 0;
static uint32_t height          = 0;

extern int   optind;
extern char *optarg;

/* ── SISCI state for one worker connection ──────────────────────────────── */
typedef struct {
    /* Local segment that holds the raw YUV pixels (DMA source) */
    sci_local_segment_t  raw_seg;
    sci_map_t            raw_map;
    void                *raw_ptr;      /* locally mapped pointer for memcpy  */
    size_t               raw_size;

    /* Local signal segment (worker writes ack here via PIO) */
    sci_local_segment_t  srv_sig_seg;
    sci_map_t            srv_sig_map;
    volatile uint32_t   *srv_sig_ptr;  /* [0] = ack word                     */

    /* Remote signal segment on the worker (server writes cmd here via PIO) */
    sci_remote_segment_t wrk_sig_seg;
    sci_map_t            wrk_sig_map;
    volatile uint32_t   *wrk_sig_ptr;  /* [0] = command word, [1] = frame_no */
} worker_conn_t;

/* ── Helpers ─────────────────────────────────────────────────────────────── */
static void die_sci(sci_error_t err, const char *ctx)
{
    if (err != SCI_ERR_OK) {
        fprintf(stderr, "SISCI error in %s: %s\n", ctx, SCIGetErrorString(err));
        exit(EXIT_FAILURE);
    }
}

/* ── Encoder init (for padding / quantisation table setup) ───────────────── */
static struct c63_common *init_c63_enc(int w, int h)
{
    struct c63_common *cm = calloc(1, sizeof(*cm));
    cm->width  = w;
    cm->height = h;

    cm->padw[Y_COMPONENT] = cm->ypw = (uint32_t)(ceil(w / 16.0f) * 16);
    cm->padh[Y_COMPONENT] = cm->yph = (uint32_t)(ceil(h / 16.0f) * 16);
    cm->padw[U_COMPONENT] = cm->upw = (uint32_t)(ceil(w * UX / (YX * 8.0f)) * 8);
    cm->padh[U_COMPONENT] = cm->uph = (uint32_t)(ceil(h * UY / (YY * 8.0f)) * 8);
    cm->padw[V_COMPONENT] = cm->vpw = (uint32_t)(ceil(w * VX / (YX * 8.0f)) * 8);
    cm->padh[V_COMPONENT] = cm->vph = (uint32_t)(ceil(h * VY / (YY * 8.0f)) * 8);

    cm->mb_cols            = cm->ypw / 8;
    cm->mb_rows            = cm->yph / 8;
    cm->qp                 = 25;
    cm->me_search_range    = 16;
    cm->keyframe_interval  = 100;

    for (int i = 0; i < 64; ++i) {
        cm->quanttbl[Y_COMPONENT][i] = yquanttbl_def[i]  / (cm->qp / 10.0);
        cm->quanttbl[U_COMPONENT][i] = uvquanttbl_def[i] / (cm->qp / 10.0);
        cm->quanttbl[V_COMPONENT][i] = uvquanttbl_def[i] / (cm->qp / 10.0);
    }
    return cm;
}

/* ── YUV file reader ─────────────────────────────────────────────────────── */
static yuv_t *read_yuv(FILE *file, struct c63_common *cm)
{
    yuv_t *image = malloc(sizeof(*image));
    size_t len   = 0;

    image->Y = calloc(1, cm->padw[Y_COMPONENT] * cm->padh[Y_COMPONENT]);
    len += fread(image->Y, 1, width * height, file);

    image->U = calloc(1, cm->padw[U_COMPONENT] * cm->padh[U_COMPONENT]);
    len += fread(image->U, 1, (width * height) / 4, file);

    image->V = calloc(1, cm->padw[V_COMPONENT] * cm->padh[V_COMPONENT]);
    len += fread(image->V, 1, (width * height) / 4, file);

    if (ferror(file)) { perror("ferror"); exit(EXIT_FAILURE); }

    if (feof(file) || len != (size_t)(width * height * 3 / 2)) {
        free(image->Y); free(image->U); free(image->V); free(image);
        return NULL;
    }
    return image;
}

/* ── SISCI setup ─────────────────────────────────────────────────────────── */

/*
 * Create and expose the raw-frame segment for one worker.
 * The local buffer is memory-mapped so we can memcpy() pixels into it.
 */
static void setup_raw_segment(sci_desc_t sd, worker_conn_t *wc,
                               int worker_idx, size_t raw_size)
{
    sci_error_t err;
    unsigned int seg_id = SEG_SRV_RAW_BASE + worker_idx;

    wc->raw_size = raw_size;

    SCICreateSegment(sd, &wc->raw_seg, seg_id, raw_size,
                     NULL, NULL, 0, &err);
    die_sci(err, "SCICreateSegment (raw)");

    SCIPrepareSegment(wc->raw_seg, ADAPTER_NO, 0, &err);
    die_sci(err, "SCIPrepareSegment (raw)");

    /* Map locally so we can write pixels into it with memcpy */
    wc->raw_ptr = SCIMapLocalSegment(wc->raw_seg, &wc->raw_map,
                                     0, raw_size, NULL, 0, &err);
    die_sci(err, "SCIMapLocalSegment (raw)");

    SCISetSegmentAvailable(wc->raw_seg, ADAPTER_NO, 0, &err);
    die_sci(err, "SCISetSegmentAvailable (raw)");
}

/*
 * Create and expose the server-side signal segment for one worker.
 * The worker will PIO-write its ack into slot [0] of this segment.
 */
static void setup_srv_signal_segment(sci_desc_t sd, worker_conn_t *wc,
                                      int worker_idx)
{
    sci_error_t  err;
    unsigned int seg_id = SEG_SRV_SIG_BASE + worker_idx;
    size_t       size   = sizeof(uint32_t) * 4; /* generous alignment pad */

    SCICreateSegment(sd, &wc->srv_sig_seg, seg_id, size,
                     NULL, NULL, 0, &err);
    die_sci(err, "SCICreateSegment (srv_sig)");

    SCIPrepareSegment(wc->srv_sig_seg, ADAPTER_NO, 0, &err);
    die_sci(err, "SCIPrepareSegment (srv_sig)");

    wc->srv_sig_ptr = (volatile uint32_t *)
        SCIMapLocalSegment(wc->srv_sig_seg, &wc->srv_sig_map,
                           0, size, NULL, 0, &err);
    die_sci(err, "SCIMapLocalSegment (srv_sig)");

    wc->srv_sig_ptr[0] = SIG_IDLE;

    SCISetSegmentAvailable(wc->srv_sig_seg, ADAPTER_NO, 0, &err);
    die_sci(err, "SCISetSegmentAvailable (srv_sig)");
}

/*
 * Connect to the worker's signal segment and map it (PIO write target).
 * Retries until the worker has exposed it.
 */
static void connect_to_worker_signal(sci_desc_t sd, worker_conn_t *wc,
                                      int worker_idx, uint32_t worker_node)
{
    sci_error_t  err;
    unsigned int seg_id = SEG_WRK_SIG_BASE + worker_idx;
    size_t       size   = sizeof(uint32_t) * 4;

    printf("server: connecting to worker %d signal segment...\n", worker_idx);
    do {
        SCIConnectSegment(sd, &wc->wrk_sig_seg, worker_node, seg_id,
                          ADAPTER_NO, NULL, NULL, SCI_INFINITE_TIMEOUT, 0, &err);
    } while (err != SCI_ERR_OK);

    wc->wrk_sig_ptr = (volatile uint32_t *)
        SCIMapRemoteSegment(wc->wrk_sig_seg, &wc->wrk_sig_map,
                            0, size, NULL, 0, &err);
    die_sci(err, "SCIMapRemoteSegment (wrk_sig)");

    printf("server: connected to worker %d.\n", worker_idx);
}

/* ── PIO helpers ─────────────────────────────────────────────────────────── */

/* Write a command word + frame number into the worker's signal segment. */
static inline void pio_signal_worker(worker_conn_t *wc,
                                      uint32_t cmd, uint32_t frame_no)
{
    wc->wrk_sig_ptr[1] = frame_no;   /* write frame number first             */
    SCIStoreBarrier(wc->wrk_sig_map, 0);
    wc->wrk_sig_ptr[0] = cmd;        /* then the command (worker polls [0])  */
    SCIStoreBarrier(wc->wrk_sig_map, 0);
}

/* Spin until the worker writes SIG_ACK into our local signal segment. */
static inline void wait_for_worker_ack(worker_conn_t *wc)
{
    while (wc->srv_sig_ptr[0] != SIG_ACK)
        ; /* tight spin — PIO round-trip is sub-microsecond on Dolphin PCIe  */
    wc->srv_sig_ptr[0] = SIG_IDLE;  /* reset for next round                  */
}

/* ── Help / print ─────────────────────────────────────────────────────────── */
static void print_help()
{
    printf("Usage: ./c63server -w <width> -h <height>\n");
    printf("         -r <worker_node> [-r <worker_node> ...]\n");
    printf("         -o <writer_node> [-f <max_frames>] <input.yuv>\n");
    exit(EXIT_FAILURE);
}

/* ══════════════════════════════════════════════════════════════════════════ */
int main(int argc, char **argv)
{
    int c;
    sci_desc_t  sd;
    sci_error_t error;

    if (argc == 1) { print_help(); }

    while ((c = getopt(argc, argv, "r:h:w:o:f:")) != -1) {
        switch (c) {
            case 'r': worker_nodes[num_workers++] = atoi(optarg); break;
            case 'h': height = atoi(optarg);           break;
            case 'w': width  = atoi(optarg);           break;
            case 'o': writer_node = atoi(optarg);      break;
            case 'f': limit_numframes = atoi(optarg);  break;
            default:  print_help();                    break;
        }
    }

    if (optind >= argc) {
        fprintf(stderr, "Error: missing input file.\n");
        exit(EXIT_FAILURE);
    }
    if (num_workers == 0) {
        fprintf(stderr, "Error: at least one -r worker node required.\n");
        exit(EXIT_FAILURE);
    }
    if (width == 0 || height == 0) {
        fprintf(stderr, "Error: -w and -h are required.\n");
        exit(EXIT_FAILURE);
    }

    input_file = argv[optind];

    /* ── Init SISCI ─────────────────────────────────────────────────────── */
    SCIInitialize(0, &error);
    die_sci(error, "SCIInitialize");

    SCIOpen(&sd, 0, &error);
    die_sci(error, "SCIOpen");

    /* ── Build encoder context (for padding sizes) ───────────────────────── */
    struct c63_common *cm = init_c63_enc(width, height);

    /* ── Compute raw frame size ──────────────────────────────────────────── */
    size_t y_size   = (size_t)cm->padw[Y_COMPONENT] * cm->padh[Y_COMPONENT];
    size_t u_size   = (size_t)cm->padw[U_COMPONENT] * cm->padh[U_COMPONENT];
    size_t v_size   = (size_t)cm->padw[V_COMPONENT] * cm->padh[V_COMPONENT];
    size_t raw_size = y_size + u_size + v_size;

    /* ── Set up one SISCI connection per worker ──────────────────────────── */
    worker_conn_t wconn[MAX_NUM_WORKERS];
    memset(wconn, 0, sizeof(wconn));

    for (int w = 0; w < num_workers; w++) {
        setup_raw_segment(sd, &wconn[w], w, raw_size);
        setup_srv_signal_segment(sd, &wconn[w], w);
    }

    /* Workers must expose their signal segments before we connect */
    for (int w = 0; w < num_workers; w++) {
        connect_to_worker_signal(sd, &wconn[w], w, worker_nodes[w]);
    }

    /* ── Open input file ─────────────────────────────────────────────────── */
    FILE *infile = fopen(input_file, "rb");
    if (!infile) { perror("fopen"); exit(EXIT_FAILURE); }

    if (limit_numframes)
        printf("server: limited to %d frames.\n", limit_numframes);

    printf("server: %ux%u, %d worker(s), writer node %u\n",
           width, height, num_workers, writer_node);
    for (int w = 0; w < num_workers; w++)
        printf("server: worker[%d] node id = %u\n", w, worker_nodes[w]);

    /* ── Main encode loop ─────────────────────────────────────────────────── */
    int numframes  = 0;
    int worker_idx = 0;   /* round-robin assignment */

    while (1) {
        yuv_t *image = read_yuv(infile, cm);
        if (!image) break;
        if (limit_numframes && numframes >= limit_numframes) {
            free(image->Y); free(image->U); free(image->V); free(image);
            break;
        }

        worker_conn_t *wc = &wconn[worker_idx];

        /*
         * Copy the raw YUV pixels into the DMA-able local segment.
         * The worker will DMA-read this segment into its own local memory.
         * Layout: Y plane | U plane | V plane (all padded)
         */
        uint8_t *dst = (uint8_t *)wc->raw_ptr;
        memcpy(dst,                   image->Y, y_size);
        memcpy(dst + y_size,          image->U, u_size);
        memcpy(dst + y_size + u_size, image->V, v_size);

        free(image->Y); free(image->U); free(image->V); free(image);

        /*
         * PIO: signal the worker that frame numframes is ready.
         * The worker polls its local signal segment for SIG_READY.
         */
        pio_signal_worker(wc, SIG_READY, (uint32_t)numframes);

        /*
         * PIO: wait for the worker to acknowledge it has consumed the frame
         * (worker writes SIG_ACK into our srv_sig segment).
         */
        wait_for_worker_ack(wc);

        printf("server: dispatched frame %d → worker %d\n",
               numframes, worker_idx);

        numframes++;
        worker_idx = (worker_idx + 1) % num_workers;
    }

    /* ── Shutdown: tell every worker there are no more frames ─────────────── */
    for (int w = 0; w < num_workers; w++) {
        pio_signal_worker(&wconn[w], SIG_DONE, (uint32_t)numframes);
    }

    printf("server: all %d frames dispatched. Shutting down.\n", numframes);

    /* ── Teardown SISCI resources ─────────────────────────────────────────── */
    for (int w = 0; w < num_workers; w++) {
        worker_conn_t *wc = &wconn[w];

        SCISetSegmentUnavailable(wc->raw_seg,     ADAPTER_NO, 0, &error);
        SCISetSegmentUnavailable(wc->srv_sig_seg, ADAPTER_NO, 0, &error);

        SCIUnmapSegment(wc->raw_map,     0, &error);
        SCIUnmapSegment(wc->srv_sig_map, 0, &error);
        SCIUnmapSegment(wc->wrk_sig_map, 0, &error);

        SCIDisconnectSegment(wc->wrk_sig_seg, 0, &error);

        SCIRemoveSegment(wc->raw_seg,     0, &error);
        SCIRemoveSegment(wc->srv_sig_seg, 0, &error);
    }

    fclose(infile);
    free(cm);

    SCIClose(sd, 0, &error);
    SCITerminate();

    return EXIT_SUCCESS;
}