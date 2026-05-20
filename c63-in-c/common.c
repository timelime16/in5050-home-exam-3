#include <assert.h>
#include <errno.h>
#include <getopt.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <arm_neon.h>

#include "common.h"
#include "dsp.h"
#include "tables.h"

void dequantize_idct_row_neon(int16_t *in_data, uint8_t *prediction, int w, int h,
    int y, uint8_t *out_data, uint8_t *quantization)
{
  int x;

  float16x8_t dct0, dct1, dct2, dct3, dct4, dct5, dct6, dct7; // dctlookup

  dct0 = vld1q_f16(dctlookup_f16_T[0]);
  dct1 = vld1q_f16(dctlookup_f16_T[1]);
  dct2 = vld1q_f16(dctlookup_f16_T[2]);
  dct3 = vld1q_f16(dctlookup_f16_T[3]);
  dct4 = vld1q_f16(dctlookup_f16_T[4]);
  dct5 = vld1q_f16(dctlookup_f16_T[5]);
  dct6 = vld1q_f16(dctlookup_f16_T[6]);
  dct7 = vld1q_f16(dctlookup_f16_T[7]);

  float16x8_t dct[8] = {dct0, dct1, dct2, dct3, dct4, dct5, dct6, dct7};

  /* Perform the dequantization and iDCT */
  for(x = 0; x < w; x += 8)
  { 
    dequant_idct_block_8x8_neon(in_data+(x*8), out_data + x, prediction, quantization, x, w, dct);
  }
}

void dequantize_idct_neon(int16_t *in_data, uint8_t *prediction, uint32_t width,
    uint32_t height, uint8_t *out_data, uint8_t *quantization)
{
  int y;
  
  #pragma omp parallel for schedule(static)
  for (y = 0; y < height; y += 8)
  {
    dequantize_idct_row_neon(in_data+y*width, prediction+y*width, width, height, y,
        out_data+y*width, quantization);
  }
}

static inline float16x8_t load_row(uint8_t *in_data, uint8_t *prediction, int w, int x, int i) 
{
  // block[i*8+j] = ((int16_t)in_data[i*w+j+x] - prediction[i*w+j+x])
  int16x8_t input, p;
  input = vreinterpretq_s16_u16(vmovl_u8(vld1_u8(in_data + x + i*w)));
  p = vreinterpretq_s16_u16(vmovl_u8(vld1_u8(prediction + x + i*w)));
  return vcvtq_f16_s16(vsubq_s16(input, p));
}

void dct_quantize_row(uint8_t *in_data, uint8_t *prediction, int w, int h,
    int16_t *out_data, uint8_t *quantization)
{
  int x;

  float16x8_t b0, b1, b2, b3, b4, b5, b6, b7;
  float16x8_t dct0, dct1, dct2, dct3, dct4, dct5, dct6, dct7; // dctlookup

  dct0 = vld1q_f16(dctlookup_f16[0]);
  dct1 = vld1q_f16(dctlookup_f16[1]);
  dct2 = vld1q_f16(dctlookup_f16[2]);
  dct3 = vld1q_f16(dctlookup_f16[3]);
  dct4 = vld1q_f16(dctlookup_f16[4]);
  dct5 = vld1q_f16(dctlookup_f16[5]);
  dct6 = vld1q_f16(dctlookup_f16[6]);
  dct7 = vld1q_f16(dctlookup_f16[7]);

  float16x8_t dct[8] = {dct0, dct1, dct2, dct3, dct4, dct5, dct6, dct7};

  /* Perform the DCT and quantization */
  for(x = 0; x < w; x += 8)
  {
    // /* Store MBs linear in memory, i.e. the 64 coefficients are stored
    //    continous. This allows us to ignore stride in DCT/iDCT and other
    //    functions. */

    b0 = load_row(in_data, prediction, w, x, 0);
    b1 = load_row(in_data, prediction, w, x, 1);
    b2 = load_row(in_data, prediction, w, x, 2);
    b3 = load_row(in_data, prediction, w, x, 3);
    b4 = load_row(in_data, prediction, w, x, 4);
    b5 = load_row(in_data, prediction, w, x, 5);
    b6 = load_row(in_data, prediction, w, x, 6);
    b7 = load_row(in_data, prediction, w, x, 7);

    float16x8_t block[8] = {b0, b1, b2, b3, b4, b5, b6, b7};

    dct_quant_block_8x8_neon(block, dct, out_data + x*8, quantization);
  }
}

void dct_quantize(uint8_t *in_data, uint8_t *prediction, uint32_t width,
    uint32_t height, int16_t *out_data, uint8_t *quantization)
{
  int y;

  for (y = 0; y < height; y += 8)
  {
    dct_quantize_row(in_data+y*width, prediction+y*width, width, height,
        out_data+y*width, quantization);
  }
}

void destroy_frame(struct frame *f)
{
  /* First frame doesn't have a reconstructed frame to destroy */
  if (!f) { return; }

  free(f->recons->Y);
  free(f->recons->U);
  free(f->recons->V);
  free(f->recons);

  free(f->residuals->Ydct);
  free(f->residuals->Udct);
  free(f->residuals->Vdct);
  free(f->residuals);

  free(f->predicted->Y);
  free(f->predicted->U);
  free(f->predicted->V);
  free(f->predicted);

  free(f->mbs[Y_COMPONENT]);
  free(f->mbs[U_COMPONENT]);
  free(f->mbs[V_COMPONENT]);

  free(f);
}

struct frame* create_frame(struct c63_common *cm, yuv_t *image)
{
  struct frame *f = malloc(sizeof(struct frame));

  f->orig = image;

  f->recons = malloc(sizeof(yuv_t));
  f->recons->Y = malloc(cm->ypw * cm->yph);
  f->recons->U = malloc(cm->upw * cm->uph);
  f->recons->V = malloc(cm->vpw * cm->vph);

  f->predicted = malloc(sizeof(yuv_t));
  f->predicted->Y = calloc(cm->ypw * cm->yph, sizeof(uint8_t));
  f->predicted->U = calloc(cm->upw * cm->uph, sizeof(uint8_t));
  f->predicted->V = calloc(cm->vpw * cm->vph, sizeof(uint8_t));

  f->residuals = malloc(sizeof(dct_t));
  f->residuals->Ydct = calloc(cm->ypw * cm->yph, sizeof(int16_t));
  f->residuals->Udct = calloc(cm->upw * cm->uph, sizeof(int16_t));
  f->residuals->Vdct = calloc(cm->vpw * cm->vph, sizeof(int16_t));

  f->mbs[Y_COMPONENT] =
    calloc(cm->mb_rows * cm->mb_cols, sizeof(struct macroblock));
  f->mbs[U_COMPONENT] =
    calloc(cm->mb_rows/2 * cm->mb_cols/2, sizeof(struct macroblock));
  f->mbs[V_COMPONENT] =
    calloc(cm->mb_rows/2 * cm->mb_cols/2, sizeof(struct macroblock));

  return f;
}

void dump_image(yuv_t *image, int w, int h, FILE *fp)
{
  fwrite(image->Y, 1, w*h, fp);
  fwrite(image->U, 1, w*h/4, fp);
  fwrite(image->V, 1, w*h/4, fp);
}


// Keep original for dec
void dequantize_idct_row(int16_t *in_data, uint8_t *prediction, int w, int h,
    int y, uint8_t *out_data, uint8_t *quantization)
{
  int x;

  int16_t block[8*8];

  /* Perform the dequantization and iDCT */
  for(x = 0; x < w; x += 8)
  {
    int i, j;

    dequant_idct_block_8x8(in_data+(x*8), block, quantization);

    for (i = 0; i < 8; ++i)
    {
      for (j = 0; j < 8; ++j)
      {
        /* Add prediction block. Note: DCT is not precise -
           Clamp to legal values */
        int16_t tmp = block[i*8+j] + (int16_t)prediction[i*w+j+x];

        if (tmp < 0) { tmp = 0; }
        else if (tmp > 255) { tmp = 255; }

        out_data[i*w+j+x] = tmp;
      }
    }
  }
}

void dequantize_idct(int16_t *in_data, uint8_t *prediction, uint32_t width,
    uint32_t height, uint8_t *out_data, uint8_t *quantization)
{
  int y;

  #pragma omp parallel for schedule(static)
  for (y = 0; y < height; y += 8)
  {
    dequantize_idct_row(in_data+y*width, prediction+y*width, width, height, y,
        out_data+y*width, quantization);
  }
}
