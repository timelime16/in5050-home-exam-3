#ifndef C63_DSP_H_
#define C63_DSP_H_

#define ISQRT2 0.70710678118654f

#include <inttypes.h>

void dct_quant_block_8x8_neon(float16x8_t *block, float16x8_t *dct, int16_t *out_data, uint8_t *quant_tbl);

void dequant_idct_block_8x8(int16_t *in_data, int16_t *out_data,
    uint8_t *quant_tbl);

void dequant_idct_block_8x8_neon(int16_t *in_data, uint8_t *out_data, uint8_t *prediction, uint8_t *quant_tbl, 
  int x, int w, float16x8_t *dct);

#endif  /* C63_DSP_H_ */
