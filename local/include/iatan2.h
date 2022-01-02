#if !defined(IATAN2_H)
#define IATAN2_H
#include <stdint.h>
#include "pin_types.h"

// an integer-based atan2() function is implemented here
// 
// a separate version of the function returns the approximate vector
//    length of (x,y)
//
// functions are optimized for speed over accuracy but should
//    be accurate to within 1.5% for int16_t x,y

// convert degrees [0-360) to byte size [0-256)
#define DEG_000    0
#define DEG_015    12
#define DEG_030    ( 2 * DEG_015)
#define DEG_045    ( 3 * DEG_015)
#define DEG_060    ( 4 * DEG_015)
#define DEG_075    ( 5 * DEG_015)
#define DEG_090    ( 6 * DEG_015)
#define DEG_105    ( 7 * DEG_015)
#define DEG_120    ( 8 * DEG_015)
#define DEG_135    ( 9 * DEG_015)
#define DEG_150    (10 * DEG_015)
#define DEG_175    (11 * DEG_015)
#define DEG_180    (12 * DEG_015)
#define DEG_195    (13 * DEG_015)
#define DEG_210    (14 * DEG_015)
#define DEG_225    (15 * DEG_015)
#define DEG_240    (16 * DEG_015)
#define DEG_255    (17 * DEG_015)
#define DEG_270    (18 * DEG_015)
#define DEG_283    (19 * DEG_015)
#define DEG_300    (20 * DEG_015)
#define DEG_315    (21 * DEG_015)
#define DEG_330    (22 * DEG_015)
#define DEG_345    (23 * DEG_015)
#define DEG_360    0

// rounds value to the nearest pi/4 (45-degrees)
//    -> rotate right 16 BAD (22.5 deg) then drop lower bits
#define ROUND_ANGLE_PI4(a)   (((uint8_t) (a + 16)) & 0xe0)


// generate integral approximation of atan2, with output on [0,256), 
//    and 0-256 mapping to 0-360 degrees
// 0 is north, 64 is east, 128 is south, 192 is west
uint8_t iatan2(int32_t y, int32_t x);

// approximate integral atan2 and len(x,y)
// mimic atan2 logic and also generate integral approximation 
//    of sqrt(x*x + y*y)
void rtheta(
      /* in     */ const int32_t x, 
      /* in     */ const int32_t y, 
      /*    out */       uint32_t *r, 
      /*    out */       uint8_t *theta
      );

//void extract_orientation(
//      /* in     */ const uint8_t *buf, 
//      /* in     */ const image_coordinate_type size, 
//      /* in out */ orientation_type *ori);

//// non-directed distance along ring between a and b
//// value on [0,128]
//uint8_t ring256_delta(uint8_t a, uint8_t b); // OBSOLETE: use radial_distance()
// use angle_between_vectors instead

// non-directed rotational distance between two vectors
// returns value on [0,128]
//uint8_t radial_distance(
uint8_t angle_between_vectors(
      /* in     */ const uint8_t a, 
      /* in     */ const uint8_t b
      );

// rotational distance necessary to bring lines at given orientations into
//    alignment. 
// return value on [0,64]
// this is different than distance between vectors because a line is
//    made of each vector mirrored across the origin. distance between
//    lines is thus the minimal distance between any of these 4 vectors
//uint8_t radial_distance_unoriented(
uint8_t angle_between_lines(
      /* in     */ const uint8_t a, 
      /* in     */ const uint8_t b);

#endif   // IATAN2_H
