#include <inttypes.h>
#include <math.h>
#include <stdlib.h>
#include <arm_neon.h>

#include "dsp.h"
#include "tables.h"

// neon instructions
static inline float16x8_t row_mat_mul(float16x8_t row, float16x8_t *dct) 
{
  float16x8_t buf1, buf2;

  buf1 = vdupq_n_f16(0.0f);
  buf2 = vdupq_n_f16(0.0f);

  buf1 = vfmaq_laneq_f16(buf1, dct[0], row, 0);
  buf2 = vfmaq_laneq_f16(buf2, dct[1], row, 1);

  buf1 = vfmaq_laneq_f16(buf1, dct[2], row, 2);
  buf2 = vfmaq_laneq_f16(buf2, dct[3], row, 3);

  buf1 = vfmaq_laneq_f16(buf1, dct[4], row, 4);
  buf2 = vfmaq_laneq_f16(buf2, dct[5], row, 5);

  buf1 = vfmaq_laneq_f16(buf1, dct[6], row, 6);
  buf2 = vfmaq_laneq_f16(buf2, dct[7], row, 7);

  return vaddq_f16(buf1, buf2);
}

static inline void transpose_block_neon(float16x8_t *block) 
{
  float16x8x2_t tmp0, tmp1, tmp2, tmp3; // tanspose - 16bit
  float32x4x2_t tmp4, tmp5, tmp6, tmp7; // transpose - 32 bit


  tmp0 = vtrnq_f16(block[0], block[1]);
  tmp1 = vtrnq_f16(block[2], block[3]);
  tmp2 = vtrnq_f16(block[4], block[5]);
  tmp3 = vtrnq_f16(block[6], block[7]);
  tmp4 = vtrnq_f32(vreinterpretq_f32_f16(tmp0.val[0]), vreinterpretq_f32_f16(tmp1.val[0]));
  tmp5 = vtrnq_f32(vreinterpretq_f32_f16(tmp0.val[1]), vreinterpretq_f32_f16(tmp1.val[1]));
  tmp6 = vtrnq_f32(vreinterpretq_f32_f16(tmp2.val[0]), vreinterpretq_f32_f16(tmp3.val[0]));
  tmp7 = vtrnq_f32(vreinterpretq_f32_f16(tmp2.val[1]), vreinterpretq_f32_f16(tmp3.val[1]));
  block[0] = vreinterpretq_f16_f32(vcombine_f32(vget_low_f32(tmp4.val[0]), vget_low_f32(tmp6.val[0])));
  block[1] = vreinterpretq_f16_f32(vcombine_f32(vget_low_f32(tmp4.val[1]), vget_low_f32(tmp6.val[1])));
  block[2] = vreinterpretq_f16_f32(vcombine_f32(vget_low_f32(tmp5.val[0]), vget_low_f32(tmp7.val[0])));
  block[3] = vreinterpretq_f16_f32(vcombine_f32(vget_low_f32(tmp5.val[1]), vget_low_f32(tmp7.val[1])));
  block[4] = vreinterpretq_f16_f32(vcombine_f32(vget_high_f32(tmp4.val[0]), vget_high_f32(tmp6.val[0])));
  block[5] = vreinterpretq_f16_f32(vcombine_f32(vget_high_f32(tmp4.val[1]), vget_high_f32(tmp6.val[1])));
  block[6] = vreinterpretq_f16_f32(vcombine_f32(vget_high_f32(tmp5.val[0]), vget_high_f32(tmp7.val[0])));
  block[7] = vreinterpretq_f16_f32(vcombine_f32(vget_high_f32(tmp5.val[1]), vget_high_f32(tmp7.val[1])));
}

static inline void scale_block_neon(float16x8_t *block)
{
  int i;
  float16x8_t scale_factors_row_0 = {ISQRT2 * ISQRT2, ISQRT2, ISQRT2, ISQRT2, ISQRT2, ISQRT2, ISQRT2, ISQRT2};
  float16x8_t scale_factors_norm = {ISQRT2, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
  
  block[0] = vmulq_f16(block[0], scale_factors_row_0);
  #pragma unroll
  for (i = 1; i < 8; ++i) 
  {
    block[i] = vmulq_f16(block[i], scale_factors_norm);
  }
}

static inline void zigzag_gather(float16x8_t *block) 
{
  int i;
  float16_t tmp[64], tmp2[64];

  #pragma unroll
  for (i = 0; i < 8; ++i)
  {
    vst1q_f16(tmp + i*8, block[i]);
  }

  #pragma unroll
  for (i = 0; i < 64; ++i)
  {
    uint8_t u = zigzag_U[i];
    uint8_t v = zigzag_V[i];
    tmp2[i] = tmp[v*8+u];
  }

  #pragma unroll
  for (i = 0; i < 8; ++i)
  {
    block[i] = vld1q_f16(tmp + i*8);
  }
}

static inline void quantize_block_neon(float16x8_t *block, uint8_t *quant_tbl) 
{
  int i;
  float16x8_t q, rp;

  #pragma unroll
  for (i = 0; i < 8; ++i)
  {
    q = vcvtq_f16_u16(vmovl_u8(vld1_u8(quant_tbl + i*8)));
    rp = vrecpeq_f16(q);
    rp = vmulq_f16(vrecpsq_f16(q, rp), rp);
    block[i] = vmulq_f16(vmulq_n_f16(block[i], 0.25f), rp);
  }
}

void dct_quant_block_8x8_neon(float16x8_t *block, float16x8_t *dct, int16_t *out_data, uint8_t *quant_tbl) 
{
  int i;

  // Matrix multiplication
  #pragma unroll
  for (i = 0; i < 8; ++i) 
  {
    block[i] = row_mat_mul(block[i], dct);
  }
  // Transpose
  transpose_block_neon(block);

  // Matrix multiplication
  #pragma unroll
  for (i = 0; i < 8; ++i) 
  {
    block[i] = row_mat_mul(block[i], dct);
  }
  // Transpose
  transpose_block_neon(block);

  // scale block
  scale_block_neon(block);

  // Zigzag gathering
  zigzag_gather(block);

  // Quantize
  quantize_block_neon(block, quant_tbl);

  // Store back to memory
  #pragma unroll 
  for (i = 0; i < 8; ++i)
  {
    vst1q_s16(out_data + i*8, vcvtq_s16_f16(vrndnq_f16(block[i])));
  }
}


static void dequantize_block_neon(float16_t *in_data, float16_t *out_data,
    uint8_t *quant_tbl)
{
  int zigzag;

  for (zigzag = 0; zigzag < 64; ++zigzag)
  {
    uint8_t u = zigzag_U[zigzag];
    uint8_t v = zigzag_V[zigzag];

    float dct = in_data[zigzag];

    /* Zig-zag and de-quantize */
    out_data[v*8+u] = (float16_t) round((dct * quant_tbl[zigzag]) / 4.0);
  }
}

static inline float16x8_t foo(float16x8_t row, float16x8x4_t mat1, float16x8x4_t mat2) 
{
  float16x8_t buf1, buf2;

  buf1 = vdupq_n_f16(0.0f);
  buf2 = vdupq_n_f16(0.0f);

  buf1 = vfmaq_laneq_f16(buf1, mat1.val[0], row, 0);
  buf2 = vfmaq_laneq_f16(buf2, mat2.val[0], row, 4);

  buf1 = vfmaq_laneq_f16(buf1, mat1.val[1], row, 1);
  buf2 = vfmaq_laneq_f16(buf2, mat2.val[1], row, 5);

  buf1 = vfmaq_laneq_f16(buf1, mat1.val[2], row, 2);
  buf2 = vfmaq_laneq_f16(buf2, mat2.val[2], row, 6);

  buf1 = vfmaq_laneq_f16(buf1, mat1.val[3], row, 3);
  buf2 = vfmaq_laneq_f16(buf2, mat2.val[3], row, 7);

  return vaddq_f16(buf1, buf2);
}

static inline void dequant_block_neon(float16x8_t *block, uint8_t *quant_tbl)
{
  // out_data[v*8+u] = (float16_t) round((dct * quant_tbl[zigzag]) / 4.0);
  int i;
  float16x8_t q;

  #pragma unroll
  for (i = 0; i < 8; ++i)
  {
    q = vcvtq_f16_u16(vmovl_u8(vld1_u8(quant_tbl + i*8)));
    block[i] = vmulq_n_f16(vmulq_f16(block[i], q), 0.25f);
  }
}

static inline void scatter_neon(float16x8_t *block)
{
  int i;
  float16_t tmp[64], tmp2[64];

  #pragma unroll
  for (i = 0; i < 8; ++i)
  {
    vst1q_f16(tmp + i*8, block[i]);
  }

  #pragma unroll
  for (i = 0; i < 64; ++i)
  {
    uint8_t u = zigzag_U[i];
    uint8_t v = zigzag_V[i];
    tmp2[v*8+u] = tmp[i];
  }

  #pragma unroll
  for (i = 0; i < 8; ++i)
  {
    block[i] = vld1q_f16(tmp + i*8);
  }
}

static inline void dequant_store_neon(float16x8_t *block, uint8_t *out_data, uint8_t *prediction, int x, int w)
{
  int i;
  int16x8_t p;

  #pragma unroll
  for (i = 0; i < 8; ++i) 
  {
    p = vreinterpretq_s16_u16(vmovl_u8(vld1_u8(prediction + x + i*w)));
    p = vaddq_s16(vcvtq_s16_f16(block[i]), p);
    vst1_u8(out_data + i*w, vqmovun_s16(p));
  }
}

void dequant_idct_block_8x8_neon(int16_t *in_data, uint8_t *out_data, uint8_t *prediction, uint8_t *quant_tbl, 
  int x, int w, float16x8_t *dct)
{
  int i;
  float16x8_t r0, r1, r2, r3, r4, r5, r6, r7;

  r0 = vcvtq_f16_s16(vld1q_s16(in_data));
  r1 = vcvtq_f16_s16(vld1q_s16(in_data + 8));
  r2 = vcvtq_f16_s16(vld1q_s16(in_data + 2*8));
  r3 = vcvtq_f16_s16(vld1q_s16(in_data + 3*8));
  r4 = vcvtq_f16_s16(vld1q_s16(in_data + 4*8));
  r5 = vcvtq_f16_s16(vld1q_s16(in_data + 5*8));
  r6 = vcvtq_f16_s16(vld1q_s16(in_data + 6*8));
  r7 = vcvtq_f16_s16(vld1q_s16(in_data + 7*8));

  // dequant
  float16x8_t block[8] = {r0, r1, r2, r3, r4, r5, r6, r7};
  dequant_block_neon(block, quant_tbl);

  // scatter
  scatter_neon(block);

  // scale
  scale_block_neon(block);

  // Matrix multiplication
  #pragma unroll
  for (i = 0; i < 8; ++i) 
  {
    block[i] = row_mat_mul(block[i], dct);
  }
  // Transpose
  transpose_block_neon(block);

  // Matrix multiplication
  #pragma unroll
  for (i = 0; i < 8; ++i) 
  {
    block[i] = row_mat_mul(block[i], dct);
  }
  // Transpose
  transpose_block_neon(block);

  // Clean up and store
  dequant_store_neon(block, out_data, prediction, x, w);
  // int16_t tmp = block[i*8+j] + (int16_t)prediction[i*w+j+x];
  // int16x8_t out0, out1, out2, out3, out4, out5, out6, out7;
  // out0 = vaddq_s16(vcvtq_s16_f16(r0), p0);
  // out1 = vaddq_s16(vcvtq_s16_f16(r1), p1);
  // out2 = vaddq_s16(vcvtq_s16_f16(r2), p2);
  // out3 = vaddq_s16(vcvtq_s16_f16(r3), p3);
  // out4 = vaddq_s16(vcvtq_s16_f16(r4), p4);
  // out5 = vaddq_s16(vcvtq_s16_f16(r5), p5);
  // out6 = vaddq_s16(vcvtq_s16_f16(r6), p6);
  // out7 = vaddq_s16(vcvtq_s16_f16(r7), p7);

  // // if (tmp < 0) { tmp = 0; }
  // // else if (tmp > 255) { tmp = 255; }

  // // out_data[i*w+j+x] = tmp;
  // // vqmovun_s16 -> automatically does min/max
  // vst1_u8(out_data, vqmovun_s16(out0));
  // vst1_u8(out_data + w, vqmovun_s16(out1));
  // vst1_u8(out_data + 2*w, vqmovun_s16(out2));
  // vst1_u8(out_data + 3*w, vqmovun_s16(out3));
  // vst1_u8(out_data + 4*w, vqmovun_s16(out4));
  // vst1_u8(out_data + 5*w, vqmovun_s16(out5));
  // vst1_u8(out_data + 6*w, vqmovun_s16(out6));
  // vst1_u8(out_data + 7*w, vqmovun_s16(out7));
}



// Keep original for dec
static void transpose_block(float *in_data, float *out_data)
{
  int i, j;

  for (i = 0; i < 8; ++i)
  {
    for (j = 0; j < 8; ++j)
    {
      out_data[i*8+j] = in_data[j*8+i];
    }
  }
}

static void scale_block(float *in_data, float *out_data)
{
  int u, v;

  for (v = 0; v < 8; ++v)
  {
    for (u = 0; u < 8; ++u)
    {
      float a1 = !u ? ISQRT2 : 1.0f;
      float a2 = !v ? ISQRT2 : 1.0f;

      /* Scale according to normalizing function */
      out_data[v*8+u] = in_data[v*8+u] * a1 * a2;
    }
  }
}

static void idct_1d(float *in_data, float *out_data)
{
  int i, j;

  for (i = 0; i < 8; ++i)
  {
    float idct = 0;

    for (j = 0; j < 8; ++j)
    {
      idct += in_data[j] * dctlookup[i][j];
    }

    out_data[i] = idct;
  }
}

static void dequantize_block(float *in_data, float *out_data,
    uint8_t *quant_tbl)
{
  int zigzag;

  for (zigzag = 0; zigzag < 64; ++zigzag)
  {
    uint8_t u = zigzag_U[zigzag];
    uint8_t v = zigzag_V[zigzag];

    float dct = in_data[zigzag];

    /* Zig-zag and de-quantize */
    out_data[v*8+u] = (float) round((dct * quant_tbl[zigzag]) / 4.0);
  }
}

void dequant_idct_block_8x8(int16_t *in_data, int16_t *out_data,
    uint8_t *quant_tbl)
{
  float mb[8*8] __attribute((aligned(16)));
  float mb2[8*8] __attribute((aligned(16)));

  int i, v;

  for (i = 0; i < 64; ++i) { mb[i] = in_data[i]; }

  dequantize_block(mb, mb2, quant_tbl);
  scale_block(mb2, mb);

  /* Two 1D inverse DCT operations with transpose */
  for (v = 0; v < 8; ++v) { idct_1d(mb+v*8, mb2+v*8); }
  transpose_block(mb2, mb);
  for (v = 0; v < 8; ++v) { idct_1d(mb+v*8, mb2+v*8); }
  transpose_block(mb2, mb);

  for (i = 0; i < 64; ++i) { out_data[i] = mb[i]; }
}
