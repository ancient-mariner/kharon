#if !defined(SOFTIRON_H)
#define SOFTIRON_H
#include "lin_alg.h"

void apply_softiron_correction(
      /* in     */ const matrix_type *r,
      /* in     */ const vector_type *scale,
      /* in     */ const vector_type *offset,
      /* in     */ const vector_type *mag,
      /*    out */       vector_type *corrected_mag
      );

#endif   // SOFTIRON_H
