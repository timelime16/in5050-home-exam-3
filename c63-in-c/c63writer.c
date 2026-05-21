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
#include "tables.h"

/* ── Shared segment ID constants (must match c63server.c / c63worker.c) ── */
#define ADAPTER_NO       0

#define SEG_WRK_SIG_BASE 0x300   /* worker signal segment (we write ack here)*/
#define SEG_WRT_ENC_BASE 0x500   /* our encoded frame segments (per worker)  */
#define SEG_WRT_SIG_BASE 0x600   /* our shared signal segment (workers notify)*/

#define SIG_IDLE  0u
#define SIG_READY 1u
#define SIG_ACK   2u
#define SIG_DONE  3u

#define ENC_HEADER_SIZE (2 * sizeof(uint32_t))

/* ── Globals ─────────────────────────────────────────────────────────────── */
static char    *output_file;
FILE           *outfile;
static uint32_t worker_nodes[MAX_NUM_WORKERS];
static int      num_workers = 0;
static uint32_t width       = 0;
static uint32_t height      = 0;

extern int   optind;
extern char *optarg;

/* ── Per-worker SISCI state ──────────────────────────────────────────────── */
typedef struct {
    /* Local encoded segment (worker DMA-writes encoded data here) */
    sci_local_segment_t  enc_seg;
    sci_map_t            enc_map;
    void                *enc_ptr;
    size_t               enc_size;

    /* Remote worker signal segment (we PIO-write ack here) */
    sci_remote_segment_t wrk_sig_seg;
    sci_map_t            wrk_sig_map;
    volatile uint32_t   *wrk_sig_ptr;   /* slot[2] = ack to worker          */
} wrt_worker_conn_t;

/* ── Hold-off buffer for out-of-order frame reassembly ───────────────────── */
typedef struct {
    int      valid;
    int      frame_no;
    int      keyframe;
    int16_t *Ydct, *Udct, *Vdct;
    struct macroblock *mb_Y, *mb_U, *mb_V;
} frame_hold_t;

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

    cm->mb_cols           = cm->ypw / 8;
    cm->mb_rows           = cm->yph / 8;
    cm->qp                = 25;
    cm->me_search_range   = 16;
    cm->keyframe_interval = 100;

    for (int i = 0; i < 64; ++i) {
        cm->quanttbl[Y_COMPONENT][i] = yquanttbl_def[i]  / (cm->qp / 10.0);
        cm->quanttbl[U_COMPONENT][i] = uvquanttbl_def[i] / (cm->qp / 10.0);
        cm->quanttbl[V_COMPONENT][i] = uvquanttbl_def[i] / (cm->qp / 10.0);
    }

    cm->curframe = create_frame(cm, NULL);
    cm->refframe = create_frame(cm, NULL);
    return cm;
}

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

/* Our shared inbound signal segment — all workers write notifications here */
static sci_local_segment_t  wrt_sig_seg;
static sci_map_t            wrt_sig_map;
static volatile uint32_t   *wrt_sig_ptr;   /* [0]=cmd [1]=worker_idx [2]=frame_no */

static void setup_signal_segment(sci_desc_t sd)
{
    sci_error_t err;
    size_t      size = sizeof(uint32_t) * 8;

    SCICreateSegment(sd, &wrt_sig_seg, SEG_WRT_SIG_BASE, size,
                     NULL, NULL, 0, &err);
    die_sci(err, "SCICreateSegment (wrt_sig)");

    SCIPrepareSegment(wrt_sig_seg, ADAPTER_NO, 0, &err);
    die_sci(err, "SCIPrepareSegment (wrt_sig)");

    wrt_sig_ptr = (volatile uint32_t *)
        SCIMapLocalSegment(wrt_sig_seg, &wrt_sig_map, 0, size, NULL, 0, &err);
    die_sci(err, "SCIMapLocalSegment (wrt_sig)");

    wrt_sig_ptr[0] = SIG_IDLE;
    wrt_sig_ptr[1] = 0;
    wrt_sig_ptr[2] = 0;

    SCISetSegmentAvailable(wrt_sig_seg, ADAPTER_NO, 0, &err);
    die_sci(err, "SCISetSegmentAvailable (wrt_sig)");
}

static void setup_enc_segments(sci_desc_t sd, wrt_worker_conn_t *wconn,
                                size_t enc_size)
{
    sci_error_t err;
    for (int w = 0; w < num_workers; w++) {
        unsigned int seg_id = SEG_WRT_ENC_BASE + w;

        wconn[w].enc_size = enc_size;

        SCICreateSegment(sd, &wconn[w].enc_seg, seg_id, enc_size,
                         NULL, NULL, 0, &err);
        die_sci(err, "SCICreateSegment (enc)");

        SCIPrepareSegment(wconn[w].enc_seg, ADAPTER_NO, 0, &err);
        die_sci(err, "SCIPrepareSegment (enc)");

        wconn[w].enc_ptr = SCIMapLocalSegment(
                               wconn[w].enc_seg, &wconn[w].enc_map,
                               0, enc_size, NULL, 0, &err);
        die_sci(err, "SCIMapLocalSegment (enc)");

        SCISetSegmentAvailable(wconn[w].enc_seg, ADAPTER_NO, 0, &err);
        die_sci(err, "SCISetSegmentAvailable (enc)");
    }
}

static void connect_to_workers(sci_desc_t sd, wrt_worker_conn_t *wconn)
{
    sci_error_t err;
    for (int w = 0; w < num_workers; w++) {
        printf("writer: connecting to worker[%d] signal segment...\n", w);
        do {
            SCIConnectSegment(sd, &wconn[w].wrk_sig_seg, worker_nodes[w],
                              SEG_WRK_SIG_BASE + w, ADAPTER_NO,
                              NULL, NULL, SCI_INFINITE_TIMEOUT, 0, &err);
        } while (err != SCI_ERR_OK);

        wconn[w].wrk_sig_ptr = (volatile uint32_t *)
            SCIMapRemoteSegment(wconn[w].wrk_sig_seg, &wconn[w].wrk_sig_map,
                                0, sizeof(uint32_t) * 8, NULL, 0, &err);
        die_sci(err, "SCIMapRemoteSegment (wrk_sig)");

        printf("writer: connected to worker[%d].\n", w);
    }
}

/* ── Unpack encoded frame from flat buffer ───────────────────────────────── */
static void unpack_encoded_frame(const void *enc_ptr,
                                  struct c63_common *cm,
                                  int *frame_no_out, int *keyframe_out)
{
    const uint8_t *p = (const uint8_t *)enc_ptr;

    size_t dct_y  = (size_t)cm->ypw * cm->yph * sizeof(int16_t);
    size_t dct_uv = (size_t)cm->upw * cm->uph * sizeof(int16_t);
    size_t mb_y   = (size_t)cm->mb_rows * cm->mb_cols * sizeof(struct macroblock);
    size_t mb_uv  = (size_t)(cm->mb_rows / 2) * (cm->mb_cols / 2)
                    * sizeof(struct macroblock);

    uint32_t fn, kf;
    memcpy(&fn, p, sizeof(fn)); p += sizeof(fn);
    memcpy(&kf, p, sizeof(kf)); p += sizeof(kf);
    *frame_no_out  = (int)fn;
    *keyframe_out  = (int)kf;

    cm->curframe->keyframe = (int)kf;

    memcpy(cm->curframe->residuals->Ydct, p, dct_y);  p += dct_y;
    memcpy(cm->curframe->residuals->Udct, p, dct_uv); p += dct_uv;
    memcpy(cm->curframe->residuals->Vdct, p, dct_uv); p += dct_uv;

    memcpy(cm->curframe->mbs[Y_COMPONENT], p, mb_y);  p += mb_y;
    memcpy(cm->curframe->mbs[U_COMPONENT], p, mb_uv); p += mb_uv;
    memcpy(cm->curframe->mbs[V_COMPONENT], p, mb_uv);
}

/* ── PIO helpers ─────────────────────────────────────────────────────────── */

/* Spin until any worker writes SIG_READY into our signal segment */
static void wait_for_any_worker(int *worker_idx_out, uint32_t *frame_no_out)
{
    while (wrt_sig_ptr[0] != SIG_READY)
        ;
    *worker_idx_out = (int)wrt_sig_ptr[1];
    *frame_no_out   = wrt_sig_ptr[2];
    wrt_sig_ptr[0]  = SIG_IDLE;   /* reset immediately after reading         */
}

/* PIO-write SIG_ACK into worker's signal slot[2] so it can reuse enc seg */
static void pio_ack_worker(wrt_worker_conn_t *wc)
{
    wc->wrk_sig_ptr[2] = SIG_ACK;
    SCIStoreBarrier(wc->wrk_sig_map, 0);
}

/* ── Help ─────────────────────────────────────────────────────────────────── */
static void print_help()
{
    printf("Usage: ./c63writer -W <width> -H <height>\n");
    printf("         -r <worker_node> [-r <worker_node> ...]\n");
    printf("         -o <output.c63>\n");
    exit(EXIT_FAILURE);
}

/* ══════════════════════════════════════════════════════════════════════════ */
int main(int argc, char **argv)
{
    int c;
    sci_desc_t  sd;
    sci_error_t error;

    if (argc == 1) { print_help(); }

    while ((c = getopt(argc, argv, "r:W:H:o:")) != -1) {
        switch (c) {
            case 'r': worker_nodes[num_workers++] = atoi(optarg); break;
            case 'W': width       = atoi(optarg); break;
            case 'H': height      = atoi(optarg); break;
            case 'o': output_file = optarg;       break;
            default:  print_help();               break;
        }
    }

    if (!output_file) {
        fprintf(stderr, "Error: -o output file is required.\n");
        print_help();
    }
    if (num_workers == 0) {
        fprintf(stderr, "Error: at least one -r worker node is required.\n");
        print_help();
    }
    if (width == 0 || height == 0) {
        fprintf(stderr, "Error: -W and -H are required.\n");
        print_help();
    }

    /* ── Open output file ────────────────────────────────────────────────── */
    outfile = fopen(output_file, "wb");
    if (!outfile) { perror("fopen"); exit(EXIT_FAILURE); }

    /* ── Init SISCI ─────────────────────────────────────────────────────── */
    SCIInitialize(0, &error);
    die_sci(error, "SCIInitialize");

    SCIOpen(&sd, 0, &error);
    die_sci(error, "SCIOpen");

    /* ── Build encoder context ───────────────────────────────────────────── */
    struct c63_common *cm = init_c63_enc(width, height);
    size_t enc_size = calc_enc_size(cm);

    /* ── Set up segments ────────────────────────────────────────────────── */
    wrt_worker_conn_t wconn[MAX_NUM_WORKERS];
    memset(wconn, 0, sizeof(wconn));

    setup_signal_segment(sd);
    setup_enc_segments(sd, wconn, enc_size);

    /* ── Connect to workers ──────────────────────────────────────────────── */
    connect_to_workers(sd, wconn);

    printf("writer: ready. Output → %s\n", output_file);

    /*
     * ─── Reorder buffer ─────────────────────────────────────────────────
     * Workers encode out of order (worker 0 handles even frames, worker 1
     * handles odd frames, or any round-robin split). We buffer received
     * frames and flush them to disk only when we have the next sequential
     * frame number.
     */
    int total_hold_slots = num_workers * 2 + 2;
    frame_hold_t *hold = calloc(total_hold_slots, sizeof(frame_hold_t));

    /* Sizes for hold buffer allocations */
    size_t dct_y  = (size_t)cm->ypw * cm->yph * sizeof(int16_t);
    size_t dct_uv = (size_t)cm->upw * cm->uph * sizeof(int16_t);
    size_t mb_y   = (size_t)cm->mb_rows * cm->mb_cols * sizeof(struct macroblock);
    size_t mb_uv  = (size_t)(cm->mb_rows / 2) * (cm->mb_cols / 2)
                    * sizeof(struct macroblock);

    for (int i = 0; i < total_hold_slots; i++) {
        hold[i].Ydct = malloc(dct_y);
        hold[i].Udct = malloc(dct_uv);
        hold[i].Vdct = malloc(dct_uv);
        hold[i].mb_Y = malloc(mb_y);
        hold[i].mb_U = malloc(mb_uv);
        hold[i].mb_V = malloc(mb_uv);
    }

    int next_write_frame = 0;   /* next frame number to write to disk        */
    int frames_written   = 0;
    int workers_done     = 0;   /* not used for shutdown here — rely on EOF  */

    /* ════════════════════════════════════════════════════════════════════════
     * Main receive-and-write loop
     * We run until we have written all expected frames.  Since we don't know
     * the total up front, we loop until the workers have sent SIG_DONE (not
     * implemented here — see note below) or we receive all frames.
     * ════════════════════════════════════════════════════════════════════════ */
    while (1) {
        /* 1. PIO: wait for any worker to signal an encoded frame is ready */
        int      w;
        uint32_t frame_no;
        wait_for_any_worker(&w, &frame_no);

        if (w < 0 || w >= num_workers) {
            fprintf(stderr, "writer: bad worker index %d in signal\n", w);
            pio_ack_worker(&wconn[w]);
            continue;
        }

        /* 2. Unpack encoded data from the worker's enc segment */
        int recv_frame_no, recv_keyframe;
        unpack_encoded_frame(wconn[w].enc_ptr, cm,
                             &recv_frame_no, &recv_keyframe);

        /* 3. PIO: ack worker immediately so it can reuse its enc segment */
        pio_ack_worker(&wconn[w]);

        /*
         * 4. Store in hold buffer.
         *    Find a free slot (we guarantee there are always enough because
         *    the pipeline depth is bounded by num_workers).
         */
        int slot = -1;
        for (int i = 0; i < total_hold_slots; i++) {
            if (!hold[i].valid) { slot = i; break; }
        }
        if (slot == -1) {
            fprintf(stderr, "writer: hold buffer full! frame %d dropped.\n",
                    recv_frame_no);
            continue;
        }

        hold[slot].valid     = 1;
        hold[slot].frame_no  = recv_frame_no;
        hold[slot].keyframe  = recv_keyframe;
        memcpy(hold[slot].Ydct, cm->curframe->residuals->Ydct, dct_y);
        memcpy(hold[slot].Udct, cm->curframe->residuals->Udct, dct_uv);
        memcpy(hold[slot].Vdct, cm->curframe->residuals->Vdct, dct_uv);
        memcpy(hold[slot].mb_Y, cm->curframe->mbs[Y_COMPONENT], mb_y);
        memcpy(hold[slot].mb_U, cm->curframe->mbs[U_COMPONENT], mb_uv);
        memcpy(hold[slot].mb_V, cm->curframe->mbs[V_COMPONENT], mb_uv);

        /*
         * 5. Flush as many sequential frames as possible to disk.
         *    We must write frame next_write_frame, then next_write_frame+1,
         *    etc., to keep the bitstream valid.
         */
        int flushed = 1;
        while (flushed) {
            flushed = 0;
            for (int i = 0; i < total_hold_slots; i++) {
                if (!hold[i].valid) continue;
                if (hold[i].frame_no != next_write_frame) continue;

                /* Copy back into cm so write_frame() sees the data */
                cm->curframe->keyframe = hold[i].keyframe;
                cm->framenum = hold[i].frame_no;
                memcpy(cm->curframe->residuals->Ydct, hold[i].Ydct, dct_y);
                memcpy(cm->curframe->residuals->Udct, hold[i].Udct, dct_uv);
                memcpy(cm->curframe->residuals->Vdct, hold[i].Vdct, dct_uv);
                memcpy(cm->curframe->mbs[Y_COMPONENT], hold[i].mb_Y, mb_y);
                memcpy(cm->curframe->mbs[U_COMPONENT], hold[i].mb_U, mb_uv);
                memcpy(cm->curframe->mbs[V_COMPONENT], hold[i].mb_V, mb_uv);

                write_frame(cm);

                hold[i].valid = 0;
                next_write_frame++;
                frames_written++;
                flushed = 1;

                printf("writer: wrote frame %d\n", hold[i].frame_no);
                break;   /* restart scan from top so we don't skip */
            }
        }

        /*
         * Termination: no clean "done" signal from workers in this design.
         * The outer loop will block in wait_for_any_worker() indefinitely
         * once all frames are processed. To shut down cleanly, one option
         * is to have the server send a total frame count alongside the
         * dimensions, then break here once frames_written == total.
         * For now, the operator sends SIGINT or uses limit_numframes.
         * The loop is intentionally left open-ended so it can be extended.
         */
    }

    /* ── Cleanup (reached only if loop exits via break/signal) ─────────── */
    SCISetSegmentUnavailable(wrt_sig_seg, ADAPTER_NO, 0, &error);
    for (int w = 0; w < num_workers; w++) {
        SCISetSegmentUnavailable(wconn[w].enc_seg, ADAPTER_NO, 0, &error);
    }

    SCIUnmapSegment(wrt_sig_map, 0, &error);
    for (int w = 0; w < num_workers; w++) {
        SCIUnmapSegment(wconn[w].enc_map,     0, &error);
        SCIUnmapSegment(wconn[w].wrk_sig_map, 0, &error);
        SCIDisconnectSegment(wconn[w].wrk_sig_seg, 0, &error);
        SCIRemoveSegment(wconn[w].enc_seg, 0, &error);
    }

    SCIRemoveSegment(wrt_sig_seg, 0, &error);

    for (int i = 0; i < total_hold_slots; i++) {
        free(hold[i].Ydct); free(hold[i].Udct); free(hold[i].Vdct);
        free(hold[i].mb_Y); free(hold[i].mb_U); free(hold[i].mb_V);
    }
    free(hold);

    destroy_frame(cm->curframe);
    destroy_frame(cm->refframe);
    free(cm);

    fclose(outfile);

    SCIClose(sd, 0, &error);
    SCITerminate();

    printf("writer: done. %d frames written to %s\n",
           frames_written, output_file);
    return EXIT_SUCCESS;
}