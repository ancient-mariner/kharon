#include <math.h>
// common functions between submodules

// valgrind --tool=callgrind /pinet/local/bin_intel/bob

//static uint8_t float_to_uint8(
//      /* in      */ const double f
//      )
//{
//   // include +0.5 in floor operation, as floating point roundoff
//   //    can/had interfered with calculation accuracy
//   if (f <= 0.0f)
//      return 0;
//   else if (f >= 254.5f)
//      return 255;
//   else
//      return (uint8_t) (f + 0.5f);
//}


// computes the world lat/lon for the given unit vector
// must be unit vector
// input vector assumed to be in world space, which is right-hand
//    system with Y up, Z forward and X left
// lon is longitude [-180,180]
// lat is latitude [-90,90]
static inline void vector_to_latlon(
      /* in      */ const vector_type *vec,
      /*     out */       degree_type *lon,
      /*     out */       degree_type *lat
      )
{
   lon->degrees = R2D * atan2(-vec->v[0], vec->v[2]);
   if (lon->degrees < 0.0) {
      lon->degrees += 360.0;
   }
   lat->degrees = R2D * asin(0.99999 * vec->v[1]);
}


static inline void vector_to_latlon32(
      /* in      */ const vector_type *vec,
      /*     out */       sphere_coordinate32_type *coord
      )
{
   double lon_deg = R2D * atan2(-vec->v[0], vec->v[2]);
   CVT_DEG_TO_BAM32(lon_deg, coord->lon);
//   coord->lon.angle32 = (uint32_t) (lon_deg * DEG_TO_BAM32);
   double lat_deg = R2D * asin(0.99999 * vec->v[1]);
   CVT_DEG_TO_BAM32(lat_deg, coord->lat);
//   coord->lat.angle32 = (uint32_t) (lat_deg * DEG_TO_BAM32);
}


// acquired image is "flat" in the sense that a photo of a square is
//    represented as a square in the image, even though the spherical
//    projection of the square results in the height at the center
//    of the image being more degrees than the height at an outer edge.
//    this distortion needs to be removed, which is done by
//    reverse projecting flattened image onto sphere
// assume that image is uniformly projected on a planar sensor with
//    uniform pixel sizes. sensor width and height are in degrees.
//    find Z value that allows using x and y pixel offsets from 
//    center w/o scaling
// function projects pixel to unit sphere and rotates about image center
//static void project_pixel_to_sphere(
//      /* in      */ const int32_t x_pos, 
//      /* in      */ const int32_t y_pos, 
//      /*     out */       vector_type *vec
//      )
//{
//   // image width is opp pixels and theta degrees. tan(theta)=opp/hyp.
//   // tan(31.1) = 410 / z  ->  z = 410 / tan(31.1) = 410/.603 = 679.66
//   // tan(24.4) = 308 / z  ->  z = 308 / tan(24.4) = 308/.455 = 679.98
//#if CAM_ROWS != 616 || CAM_COLS != 820u
//#error "project to sphere based on 820x616 image"
//#endif   // VY ROWS and COLS
//   const double z = 679.32f;
//   const double scale = 1.0f / sqrtf((double) (x_pos*x_pos + y_pos*y_pos) + z*z);
//   vec->v[0] = (double) x_pos * scale;
//   vec->v[1] = (double) y_pos * scale;
//   vec->v[2] = z * scale;
//   // for image size 820x616
//}
