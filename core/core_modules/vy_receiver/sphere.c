#if !defined(SPHERE_C)
#define SPHERE_C
#include <string.h>
// generates table of rectilinear projection mapping onto unit sphere

// when multiple image streams have the same resolution and FOV, re-use
//    the same maps instead of creating a new one for each stream

// number of cached mappings
// number required is num pyramid levels * number of supported resolutions
#define MAX_CACHED_MAPS    32
static uint32_t num_allocated_maps_ = 0;
static vector_type * map_cache_[MAX_CACHED_MAPS];
static char map_name_[MAX_CACHED_MAPS][STR_LEN];

static const vector_type* get_projection_map(
      /* in     */ const image_size_type size,
      /* in     */ const degree_type fov_horiz,
      /* in     */ const degree_type fov_vert
      )
{
   // check cache to see if this configuration is already allocated
   // if so, return it
   char name[64];
   sprintf(name, "%d %d %.2f %.2f", size.x, size.y, 
         (double) fov_horiz.degrees, (double) fov_vert.degrees);
   vector_type *map = NULL;
   for (uint32_t i=0; i<num_allocated_maps_; i++) {
      if (strcmp(name, map_name_[i]) == 0) {
         map = map_cache_[i];
         break;
      }
   }
   if (map == NULL) {
      if (num_allocated_maps_ >= MAX_CACHED_MAPS) {
         fprintf(stderr, "Exceeded number of supported image resolutions\n");
         fprintf(stderr, "Failed on '%s'\n", name);
         hard_exit(__FILE__, __LINE__);
      }
      uint32_t n_pix = (uint32_t) (size.x * size.y);
      map = malloc(n_pix * sizeof *map);
      strcpy(map_name_[num_allocated_maps_], name);
      map_cache_[num_allocated_maps_] = map;
      ++num_allocated_maps_;
      const double d_h = (double) (size.x/2) / 
            tan(D2R * fov_horiz.degrees / 2.0);
      const double d_v = (double) (size.y/2) / 
            tan(D2R * fov_vert.degrees / 2.0);
      const double d = 0.5 * (d_h + d_v);
// print this out as a debugging measure, so error can more easily
//    be detected when new image format is used
printf("Rectilinear projection remapping for '%s' estimates circle radius as %.1f,%.1f on horiz,vert axes, so sphere is estimated at %.1f\n", name, d_h, d_v, d);
      // get mathematical center to measure pixel distances from it
      // distance is based on center of pixel so width is one pixel less
      //    than number of pixels (outer 1/2 pixel on each side isn't 
      //    part of measurement)
      const double h_center = 0.5 * (double) (size.x - 1);
      const double v_center = 0.5 * (double) (size.y - 1);
      uint32_t idx = 0;
      for (uint32_t y=0; y<size.rows; y++) {
         const double dy = v_center - (double) y;
         for (uint32_t x=0; x<size.cols; x++) {
            const double dx = h_center - (double) x;
            vector_type *vec = &map[idx];
            idx++;
            // map vector to surface of unit sphere
            double len = sqrt(dx*dx + dy*dy + d*d);
            // positive x in image is moving rightward on sphere
            // positive x axis is out left side of sphere. need to invert
            //    as image x is opposite sphere x axis
            vec->v[0] = dx / len;
            vec->v[1] = dy / len;
            vec->v[2] = d / len;
//if ((x%10==0) && (y%10==0)) {
//   printf("%d,%d -> %f,%f,%f\n", x, y, (double) vec->v[0], (double) vec->v[1], (double) vec->v[2]);
//}
         }
      }
   }
   return map;
}


#endif
