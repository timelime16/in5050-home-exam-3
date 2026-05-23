#ifndef C63_ME_H_
#define C63_ME_H_

#include "c63.h"

// Declaration
void c63_motion_estimate(struct c63_common *cm, int start_mb_row, int end_mb_row);

void c63_motion_compensate(struct c63_common *cm, int start_mb_row, int end_mb_row);

#endif  /* C63_ME_H_ */
