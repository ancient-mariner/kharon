#if !defined(ACCUMULATOR_H)
#define ACCUMULATOR_H
//#include <xmmintrin.h>
#include "pin_types.h"


// accumulator is used when remapping image, to bring together weighted
//    values of source pixels projecting to destination pixel locations
// stores all values in the world pixel type, plus weight
union vy_accumulator_element {
   struct {
      uint32_t v;
      uint32_t y;
      uint32_t z; // border flag
      uint32_t w; // total input
   };
//   __m128 element;
};
typedef union vy_accumulator_element vy_accumulator_element_type;


union vy_accumulator_pixel {
   struct {
      // v and y channels, plus border channel (z). 'a' unused
      uint32_t v, y;
   };
   uint64_t vy;
};
typedef union vy_accumulator_pixel vy_accumulator_pixel_type;

union real_pix_value {
   struct {
      float x, y;
   };
   uint32_t all;
};
typedef union real_coordinate real_coordinate_type;

//union vy_accumulator_pixel {
//   // v and y channels, plus border channel (z)
//   struct {
//      uint32_t v;
//      uint32_t y;
//      uint32_t z;
//      uint32_t one;  // this should always be 1.0
//   };
//   __m128 element;
//};
//typedef union vy_accumulator_pixel vy_accumulator_pixel_type;


struct vy_accumulator {
   vy_accumulator_element_type *accum;
   image_size_type size;
   // half of field-of-view represented by accumulator on each axis
   degree_type half_width;
   degree_type half_height;
};
typedef struct vy_accumulator vy_accumulator_type;


#endif   // ACCUMULATOR_H
