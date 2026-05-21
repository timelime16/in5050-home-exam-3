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
#include "common.h"
#include "me.h"
#include "tables.h"

/* ── Shared segment ID constants (must match c63server.c / c63writer.c) ── */
#define ADAPTER_NO           0

#define SEG_SRV_RAW_BASE     0x100   /* server raw frame segment             */
#define SEG_SRV_SIG_BASE     0x200   /* server ack segment (we write here)   */
#define SEG_WRK_SIG_BASE     0x300   /* our own signal segment (server reads)*/
#define SEG_WRK_ENC_BASE     0x400   /* our encoded frame segment (DMA src)  */
#define SEG_WRT_ENC_BASE     0x500   /* writer encoded frame segment (dst)   */
#define SEG_WRT_SIG_BASE     0x600   /* writer signal segment (we write here)*/

#define SIG_IDLE    0u
#define SIG_READY   1u
#define SIG_ACK     2u
#define SIG_DONE    3u

/* Encoded frame header size (frame_number + keyframe, both uint32_t) */
#define ENC_HEADER_SIZE  (2 * sizeof(uint32_t))

/* ── Globals ─────────────────────────────────────────────────────────────── */
static uint32_t server_node  = 0;
static uint32_t writer_node  = 0;
static int      worker_index = 0;
static uint32_t width        = 0;
static uint32_t height       = 0;

extern int   optind;
extern char *optarg;

/* ── SISCI state ─────────────────────────────────────────────────────────── */
typedef struct {
    /* Local signal segment (server and writer write commands here) */
    sci_local_segment_t  wrk_sig_seg;
    sci_map_t            wrk_sig_map;
    volatile uint32_t   *wrk_sig_ptr; /* [0]=cmd, [1]=frame_no, [2]=wrt_ack */

    /* Local raw segment (DMA destination from server) */
    sci_local_segment_t  raw_seg;
    sci_map_t            raw_map;
    void                *raw_ptr;
    size_t               raw_size;

    /* Local encoded segment (DMA source to writer) */
    sci_local_segment_t  enc_seg;
    sci_map_t            enc_map;
    void                *enc_ptr;
    size_t               enc_size;

    /* Remote: server raw segment (DMA read source) */
    sci_remote_segment_t srv_raw_seg;

    /* Remote: server signal segment (we PIO-write ack here) */
    sci_remote_segment_t srv_sig_seg;
    sci_map_t            srv_sig_map;
    volatile uint32_t   *srv_sig_ptr;

    /* Remote: writer encoded segment (DMA write destination) */
    sci_remote_segment_t wrt_enc_seg;

    /* Remote: writer signal segment (we PIO-write notification here) */
    sci_remote_segment_t wrt_sig_seg;
    sci_map_t            wrt_sig_map;
    volatile uint32_t   *wrt_sig_ptr; /* [0]=cmd, [1]=worker_idx, [2]=frame_no */

    /* DMA queues */
    sci_dma_queue_t dma_read_q;   /* for reading raw frames from server      */
    sci_dma_queue_t dma_write_q;  /* for writing encoded frames to writer    */
} worker_state_t;

/* ── Helpers ─────────────────────────────────────────────────────────────── */
static void die_sci(sci_error_t err, const char *ctx)
{
    if (err != SCI_ERR_OK) {
        fprintf(stderr, "SISCI error in %s: %s\n", ctx, SCIGetErrorString(err));
        exit(EXIT_FAILURE);
    }
}

/* ── Encoder init ─────────────────────────────────────────────────────────── */
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

    cm->curframe = create_frame(cm, NULL);
    cm->refframe = create_frame(cm, NULL);
    return cm;
}

/* ── Encoded segment size calculator ─────────────────────────────────────── */
static size_t calc_enc_size(struct c63_common *cm)
{
    size_t dct_y  = (size_t)cm->ypw * cm->yph * sizeof(int16_t);
    size_t dct_uv = (size_t)cm->upw * cm->uph * sizeof(int16_t);
    size_t mb_y   = (size_t)cm->mb_rows * cm->mb_cols * sizeof(struct macroblock);
    size_t mb_uv  = (size_t)(cm->mb_rows / 2) * (cm->mb_cols / 2)
                    * sizeof(struct macroblock);
    return ENC_HEADER_SIZE + dct_y + dct_uv + dct_uv + mb_y + mb_uv + mb_uv;
}

/* ── SISCI setup ─────────────────────────────────────────────────────────── */

static void setup_local_signal_segment(sci_desc_t sd, worker_state_t *ws)
{
    sci_error_t  err;
    unsigned int seg_id = SEG_WRK_SIG_BASE + worker_index;
    size_t       size   = sizeof(uint32_t) * 8;  /* slots [0..7] */

    SCICreateSegment(sd, &ws->wrk_sig_seg, seg_id, size,
                     NULL, NULL, 0, &err);
    die_sci(err, "SCICreateSegment (wrk_sig)");

    SCIPrepareSegment(ws->wrk_sig_seg, ADAPTER_NO, 0, &err);
    die_sci(err, "SCIPrepareSegment (wrk_sig)");

    ws->wrk_sig_ptr = (volatile uint32_t *)
        SCIMapLocalSegment(ws->wrk_sig_seg, &ws->wrk_sig_map,
                           0, size, NULL, 0, &err);
    die_sci(err, "SCIMapLocalSegment (wrk_sig)");

    ws->wrk_sig_ptr[0] = SIG_IDLE;   /* command word         */
    ws->wrk_sig_ptr[1] = 0;          /* frame number         */
    ws->wrk_sig_ptr[2] = SIG_IDLE;   /* writer ack word      */

    SCISetSegmentAvailable(ws->wrk_sig_seg, ADAPTER_NO, 0, &err);
    die_sci(err, "SCISetSegmentAvailable (wrk_sig)");
}

static void setup_local_raw_segment(sci_desc_t sd, worker_state_t *ws,
                                     size_t raw_size)
{
    sci_error_t  err;
    unsigned int seg_id = SEG_WRK_SIG_BASE + 0x50 + worker_index; /* unique */

    ws->raw_size = raw_size;

    SCICreateSegment(sd, &ws->raw_seg, seg_id, raw_size,
                     NULL, NULL, 0, &err);
    die_sci(err, "SCICreateSegment (raw local)");

    SCIPrepareSegment(ws->raw_seg, ADAPTER_NO, 0, &err);
    die_sci(err, "SCIPrepareSegment (raw local)");

    ws->raw_ptr = SCIMapLocalSegment(ws->raw_seg, &ws->raw_map,
                                     0, raw_size, NULL, 0, &err);
    die_sci(err, "SCIMapLocalSegment (raw local)");

    /* No need to make available to other nodes — DMA destination only */
}

static void setup_local_enc_segment(sci_desc_t sd, worker_state_t *ws,
                                     size_t enc_size)
{
    sci_error_t  err;
    unsigned int seg_id = SEG_WRK_ENC_BASE + worker_index;

    ws->enc_size = enc_size;

    SCICreateSegment(sd, &ws->enc_seg, seg_id, enc_size,
                     NULL, NULL, 0, &err);
    die_sci(err, "SCICreateSegment (enc)");

    SCIPrepareSegment(ws->enc_seg, ADAPTER_NO, 0, &err);
    die_sci(err, "SCIPrepareSegment (enc)");

    ws->enc_ptr = SCIMapLocalSegment(ws->enc_seg, &ws->enc_map,
                                     0, enc_size, NULL, 0, &err);
    die_sci(err, "SCIMapLocalSegment (enc)");

    /* No need to expose to other nodes — worker is the DMA initiator */
}

static void connect_to_server(sci_desc_t sd, worker_state_t *ws)
{
    sci_error_t err;

    /* Server raw segment (DMA read source) */
    printf("worker[%d]: connecting to server raw segment...\n", worker_index);
    do {
        SCIConnectSegment(sd, &ws->srv_raw_seg, server_node,
                          SEG_SRV_RAW_BASE + worker_index, ADAPTER_NO,
                          NULL, NULL, SCI_INFINITE_TIMEOUT, 0, &err);
    } while (err != SCI_ERR_OK);

    /* Server signal segment (PIO write for ack) */
    do {
        SCIConnectSegment(sd, &ws->srv_sig_seg, server_node,
                          SEG_SRV_SIG_BASE + worker_index, ADAPTER_NO,
                          NULL, NULL, SCI_INFINITE_TIMEOUT, 0, &err);
    } while (err != SCI_ERR_OK);

    ws->srv_sig_ptr = (volatile uint32_t *)
        SCIMapRemoteSegment(ws->srv_sig_seg, &ws->srv_sig_map,
                            0, sizeof(uint32_t) * 4, NULL, 0, &err);
    die_sci(err, "SCIMapRemoteSegment (srv_sig)");

    printf("worker[%d]: connected to server.\n", worker_index);
}

static void connect_to_writer(sci_desc_t sd, worker_state_t *ws,
                               size_t enc_size)
{
    sci_error_t err;

    /* Writer encoded segment (DMA write destination) */
    printf("worker[%d]: connecting to writer encoded segment...\n", worker_index);
    do {
        SCIConnectSegment(sd, &ws->wrt_enc_seg, writer_node,
                          SEG_WRT_ENC_BASE + worker_index, ADAPTER_NO,
                          NULL, NULL, SCI_INFINITE_TIMEOUT, 0, &err);
    } while (err != SCI_ERR_OK);

    /* Writer signal segment (PIO write for notification) */
    do {
        SCIConnectSegment(sd, &ws->wrt_sig_seg, writer_node,
                          SEG_WRT_SIG_BASE, ADAPTER_NO,
                          NULL, NULL, SCI_INFINITE_TIMEOUT, 0, &err);
    } while (err != SCI_ERR_OK);

    ws->wrt_sig_ptr = (volatile uint32_t *)
        SCIMapRemoteSegment(ws->wrt_sig_seg, &ws->wrt_sig_map,
                            0, sizeof(uint32_t) * 8, NULL, 0, &err);
    die_sci(err, "SCIMapRemoteSegment (wrt_sig)");

    printf("worker[%d]: connected to writer.\n", worker_index);
}

static void create_dma_queues(sci_desc_t sd, worker_state_t *ws)
{
    sci_error_t err;

    SCICreateDMAQueue(sd, &ws->dma_read_q,  ADAPTER_NO, 1, 0, &err);
    die_sci(err, "SCICreateDMAQueue (read)");

    SCICreateDMAQueue(sd, &ws->dma_write_q, ADAPTER_NO, 1, 0, &err);
    die_sci(err, "SCICreateDMAQueue (write)");
}

/* ── DMA helpers ─────────────────────────────────────────────────────────── */

/*
 * DMA-read the raw frame from the server's segment into our local raw segment.
 * Blocks until the transfer completes.
 */
static void dma_read_raw_frame(worker_state_t *ws)
{
    sci_error_t err;

    SCIStartDmaTransfer(ws->dma_read_q,
                        ws->raw_seg,          /* local destination            */
                        ws->srv_raw_seg,      /* remote source                */
                        0,                    /* local offset                 */
                        ws->raw_size,         /* bytes                        */
                        0,                    /* remote offset                */
                        NULL, NULL,
                        SCI_FLAG_DMA_READ,    /* direction: remote → local    */
                        &err);
    die_sci(err, "SCIStartDmaTransfer (read raw)");

    sci_dma_queue_state_t state =
        SCIWaitForDMAQueue(ws->dma_read_q, SCI_INFINITE_TIMEOUT, 0, &err);
    die_sci(err, "SCIWaitForDMAQueue (read raw)");

    if (state != SCI_DMAQUEUE_DONE) {
        fprintf(stderr, "worker[%d]: DMA read failed (state=%d)\n",
                worker_index, state);
        exit(EXIT_FAILURE);
    }
}

/*
 * DMA-write our local encoded segment to the writer's remote segment.
 * Blocks until the transfer completes.
 */
static void dma_write_enc_frame(worker_state_t *ws)
{
    sci_error_t err;

    SCIStartDmaTransfer(ws->dma_write_q,
                        ws->enc_seg,          /* local source                 */
                        ws->wrt_enc_seg,      /* remote destination           */
                        0,                    /* local offset                 */
                        ws->enc_size,         /* bytes                        */
                        0,                    /* remote offset                */
                        NULL, NULL,
                        0,                    /* direction: local → remote    */
                        &err);
    die_sci(err, "SCIStartDmaTransfer (write enc)");

    sci_dma_queue_state_t state =
        SCIWaitForDMAQueue(ws->dma_write_q, SCI_INFINITE_TIMEOUT, 0, &err);
    die_sci(err, "SCIWaitForDMAQueue (write enc)");

    if (state != SCI_DMAQUEUE_DONE) {
        fprintf(stderr, "worker[%d]: DMA write failed (state=%d)\n",
                worker_index, state);
        exit(EXIT_FAILURE);
    }
}

/* ── PIO helpers ─────────────────────────────────────────────────────────── */

/* Spin until the server writes SIG_READY (or SIG_DONE) into our signal seg */
static uint32_t wait_for_server_signal(worker_state_t *ws)
{
    while (ws->wrk_sig_ptr[0] == SIG_IDLE)
        ;
    uint32_t cmd = ws->wrk_sig_ptr[0];
    /* Do NOT clear here — we read frame_no next */
    return cmd;
}

/* Ack to server: write SIG_ACK into the server's signal segment via PIO */
static void pio_ack_server(worker_state_t *ws)
{
    ws->srv_sig_ptr[0] = SIG_ACK;
    SCIStoreBarrier(ws->srv_sig_map, 0);
    ws->wrk_sig_ptr[0] = SIG_IDLE;   /* reset our own command word           */
}

/* Notify writer: frame encoded and DMA'd, come pick it up */
static void pio_notify_writer(worker_state_t *ws, uint32_t frame_no)
{
    /* slot[1] = worker index so writer knows which enc segment to look at */
    ws->wrt_sig_ptr[2] = frame_no;
    ws->wrt_sig_ptr[1] = (uint32_t)worker_index;
    SCIStoreBarrier(ws->wrt_sig_map, 0);
    ws->wrt_sig_ptr[0] = SIG_READY;
    SCIStoreBarrier(ws->wrt_sig_map, 0);
}

/* Wait for writer to ack it has consumed the encoded frame */
static void wait_for_writer_ack(worker_state_t *ws)
{
    while (ws->wrk_sig_ptr[2] != SIG_ACK)
        ;
    ws->wrk_sig_ptr[2] = SIG_IDLE;
}

/* ── Encode one frame ─────────────────────────────────────────────────────── */
static void encode_frame(struct c63_common *cm, const uint8_t *raw_ptr,
                          int frame_no)
{
    size_t y_sz = (size_t)cm->padw[Y_COMPONENT] * cm->padh[Y_COMPONENT];
    size_t u_sz = (size_t)cm->padw[U_COMPONENT] * cm->padh[U_COMPONENT];

    /* Build a temporary yuv_t pointing into the DMA buffer */
    yuv_t image;
    image.Y = (uint8_t *)raw_ptr;
    image.U = (uint8_t *)raw_ptr + y_sz;
    image.V = (uint8_t *)raw_ptr + y_sz + u_sz;

    /* Advance reference frame */
    destroy_frame(cm->refframe);
    cm->refframe = cm->curframe;
    cm->curframe  = create_frame(cm, &image);

    /* Keyframe logic */
    if (cm->framenum == 0 || cm->frames_since_keyframe == cm->keyframe_interval) {
        cm->curframe->keyframe    = 1;
        cm->frames_since_keyframe = 0;
    } else {
        cm->curframe->keyframe = 0;
    }

    if (!cm->curframe->keyframe) {
        c63_motion_estimate(cm);
        c63_motion_compensate(cm);
    }

    dct_quantize(image.Y, cm->curframe->predicted->Y,
                 cm->padw[Y_COMPONENT], cm->padh[Y_COMPONENT],
                 cm->curframe->residuals->Ydct, cm->quanttbl[Y_COMPONENT]);
    dct_quantize(image.U, cm->curframe->predicted->U,
                 cm->padw[U_COMPONENT], cm->padh[U_COMPONENT],
                 cm->curframe->residuals->Udct, cm->quanttbl[U_COMPONENT]);
    dct_quantize(image.V, cm->curframe->predicted->V,
                 cm->padw[V_COMPONENT], cm->padh[V_COMPONENT],
                 cm->curframe->residuals->Vdct, cm->quanttbl[V_COMPONENT]);

    dequantize_idct(cm->curframe->residuals->Ydct, cm->curframe->predicted->Y,
                    cm->ypw, cm->yph, cm->curframe->recons->Y,
                    cm->quanttbl[Y_COMPONENT]);
    dequantize_idct(cm->curframe->residuals->Udct, cm->curframe->predicted->U,
                    cm->upw, cm->uph, cm->curframe->recons->U,
                    cm->quanttbl[U_COMPONENT]);
    dequantize_idct(cm->curframe->residuals->Vdct, cm->curframe->predicted->V,
                    cm->vpw, cm->vph, cm->curframe->recons->V,
                    cm->quanttbl[V_COMPONENT]);

    ++cm->framenum;
    ++cm->frames_since_keyframe;
}

/*
 * Pack the encoded data into the flat DMA-able encoded segment.
 * Layout must match what c63writer.c unpacks.
 */
static void pack_encoded_frame(worker_state_t *ws, struct c63_common *cm,
                                int frame_no)
{
    uint8_t *p = (uint8_t *)ws->enc_ptr;

    size_t dct_y  = (size_t)cm->ypw * cm->yph * sizeof(int16_t);
    size_t dct_uv = (size_t)cm->upw * cm->uph * sizeof(int16_t);
    size_t mb_y   = (size_t)cm->mb_rows * cm->mb_cols * sizeof(struct macroblock);
    size_t mb_uv  = (size_t)(cm->mb_rows / 2) * (cm->mb_cols / 2)
                    * sizeof(struct macroblock);

    uint32_t fn  = (uint32_t)frame_no;
    uint32_t kf  = (uint32_t)cm->curframe->keyframe;
    memcpy(p, &fn, sizeof(fn)); p += sizeof(fn);
    memcpy(p, &kf, sizeof(kf)); p += sizeof(kf);

    memcpy(p, cm->curframe->residuals->Ydct, dct_y);  p += dct_y;
    memcpy(p, cm->curframe->residuals->Udct, dct_uv); p += dct_uv;
    memcpy(p, cm->curframe->residuals->Vdct, dct_uv); p += dct_uv;

    memcpy(p, cm->curframe->mbs[Y_COMPONENT], mb_y);  p += mb_y;
    memcpy(p, cm->curframe->mbs[U_COMPONENT], mb_uv); p += mb_uv;
    memcpy(p, cm->curframe->mbs[V_COMPONENT], mb_uv);
}

/* ── Help ─────────────────────────────────────────────────────────────────── */
static void print_help()
{
    printf("Usage: ./c63worker -s <server_node> -i <worker_index>\n");
    printf("         -o <writer_node> -W <width> -H <height>\n");
    exit(EXIT_FAILURE);
}

/* ══════════════════════════════════════════════════════════════════════════ */
int main(int argc, char **argv)
{
    int c;
    sci_desc_t  sd;
    sci_error_t error;

    if (argc == 1) { print_help(); }

    while ((c = getopt(argc, argv, "s:i:o:W:H:")) != -1) {
        switch (c) {
            case 's': server_node  = atoi(optarg); break;
            case 'i': worker_index = atoi(optarg); break;
            case 'o': writer_node  = atoi(optarg); break;
            case 'W': width        = atoi(optarg); break;
            case 'H': height       = atoi(optarg); break;
            default:  print_help();                break;
        }
    }

    if (server_node == 0 || writer_node == 0 || width == 0 || height == 0) {
        fprintf(stderr, "Error: -s, -o, -W, -H are all required.\n");
        print_help();
    }

    printf("worker[%d]: server=%u writer=%u %ux%u\n",
           worker_index, server_node, writer_node, width, height);

    /* ── Init SISCI ─────────────────────────────────────────────────────── */
    SCIInitialize(0, &error);
    die_sci(error, "SCIInitialize");

    SCIOpen(&sd, 0, &error);
    die_sci(error, "SCIOpen");

    /* ── Build encoder context ───────────────────────────────────────────── */
    struct c63_common *cm = init_c63_enc(width, height);

    size_t y_size   = (size_t)cm->padw[Y_COMPONENT] * cm->padh[Y_COMPONENT];
    size_t u_size   = (size_t)cm->padw[U_COMPONENT] * cm->padh[U_COMPONENT];
    size_t v_size   = (size_t)cm->padw[V_COMPONENT] * cm->padh[V_COMPONENT];
    size_t raw_size = y_size + u_size + v_size;
    size_t enc_size = calc_enc_size(cm);

    /* ── Set up local segments ───────────────────────────────────────────── */
    worker_state_t ws;
    memset(&ws, 0, sizeof(ws));

    setup_local_signal_segment(sd, &ws);     /* expose before server connects */
    setup_local_raw_segment(sd, &ws, raw_size);
    setup_local_enc_segment(sd, &ws, enc_size);

    /* ── Connect to remote nodes ─────────────────────────────────────────── */
    connect_to_server(sd, &ws);
    connect_to_writer(sd, &ws, enc_size);

    /* ── Create DMA queues ───────────────────────────────────────────────── */
    create_dma_queues(sd, &ws);

    printf("worker[%d]: ready.\n", worker_index);

    /* ════════════════════════════════════════════════════════════════════════
     * Main processing loop
     * ════════════════════════════════════════════════════════════════════════ */
    while (1) {
        /* 1. PIO: wait for server to signal a frame is ready (or done) */
        uint32_t cmd = wait_for_server_signal(&ws);
        uint32_t frame_no = ws.wrk_sig_ptr[1];

        if (cmd == SIG_DONE) {
            printf("worker[%d]: received SIG_DONE after %u frames.\n",
                   worker_index, frame_no);
            break;
        }

        if (cmd != SIG_READY) {
            fprintf(stderr, "worker[%d]: unexpected signal %u\n",
                    worker_index, cmd);
            continue;
        }

        /* 2. DMA: read raw frame from server into our local raw segment */
        dma_read_raw_frame(&ws);

        /* 3. PIO: ack to server that we have the frame */
        pio_ack_server(&ws);

        /* 4. CPU encode */
        encode_frame(cm, (const uint8_t *)ws.raw_ptr, (int)frame_no);

        /* 5. Pack encoded data into flat DMA-able buffer */
        pack_encoded_frame(&ws, cm, (int)frame_no);

        /* 6. DMA: push encoded data to writer's segment */
        dma_write_enc_frame(&ws);

        /* 7. PIO: notify writer that encoded frame frame_no is ready */
        pio_notify_writer(&ws, frame_no);

        /* 8. PIO: wait for writer to ack before overwriting enc segment */
        wait_for_writer_ack(&ws);

        printf("worker[%d]: encoded and transmitted frame %u\n",
               worker_index, frame_no);
    }

    /* ── Cleanup ─────────────────────────────────────────────────────────── */
    SCIRemoveDMAQueue(ws.dma_read_q,  0, &error);
    SCIRemoveDMAQueue(ws.dma_write_q, 0, &error);

    SCISetSegmentUnavailable(ws.wrk_sig_seg, ADAPTER_NO, 0, &error);

    SCIUnmapSegment(ws.raw_map,     0, &error);
    SCIUnmapSegment(ws.enc_map,     0, &error);
    SCIUnmapSegment(ws.wrk_sig_map, 0, &error);
    SCIUnmapSegment(ws.srv_sig_map, 0, &error);
    SCIUnmapSegment(ws.wrt_sig_map, 0, &error);

    SCIDisconnectSegment(ws.srv_raw_seg, 0, &error);
    SCIDisconnectSegment(ws.srv_sig_seg, 0, &error);
    SCIDisconnectSegment(ws.wrt_enc_seg, 0, &error);
    SCIDisconnectSegment(ws.wrt_sig_seg, 0, &error);

    SCIRemoveSegment(ws.raw_seg,     0, &error);
    SCIRemoveSegment(ws.enc_seg,     0, &error);
    SCIRemoveSegment(ws.wrk_sig_seg, 0, &error);

    destroy_frame(cm->curframe);
    destroy_frame(cm->refframe);
    free(cm);

    SCIClose(sd, 0, &error);
    SCITerminate();

    return EXIT_SUCCESS;
}