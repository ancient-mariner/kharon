#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <math.h>
#include "world_map.h"

static world_coordinate_type latlon_;
static char map_dir_[BUF_LEN];

static uint32_t parse_command_line(int argc, const char **argv)
{
   if (argc != 4) {
      printf("Usage: %s <map-dir> <lon> <lat>\n", argv[0]);
      return 1;
   }
   errno = 0;
   uint32_t errs = 0;
   latlon_.lon = strtof(argv[2], NULL);
   if (errno != 0) {
      fprintf(stderr, "Error parsing longitude: '%s' not recognized", argv[2]);
      errs++;
   }
   latlon_.lat = strtof(argv[3], NULL);
   if (errno != 0) {
      fprintf(stderr, "Error parsing latitude: '%s' not recognized", argv[3]);
      errs++;
   }
   // bounds check/correct latlon
   if ((latlon_.lat < -90.0) || (latlon_.lat > 90.0)) {
      fprintf(stderr, "Invalide latitude: %.4f\n", (double) latlon_.lat);
      errs++;
   }
   while (latlon_.lon < -180.0) {
      latlon_.lon += 360.0;
   }
   while (latlon_.lon > 180.0) {
      latlon_.lon -= 360.0;
   }
   //
   terminate_folder_path(argv[1], map_dir_);
   return errs;
}


int main(int argc, const char **argv)
{
   // parse command line
   uint32_t errs = 0;
   errs += parse_command_line(argc, argv);
   // verify map directory is present
   char fname[BUF_LEN];
   sprintf(fname, "%s%s", map_dir_, MAP_LEVEL_1_FILE_NAME);
   if (file_exists(fname) == 0) {
      fprintf(stderr, "Unable to locate map file '%s'\n", fname);
      errs++;
   }
   // bail out if setup is not correct
   if (errs != 0) {
      return 1;
   }
   /////////////////////////////////////////////////////////////////////
   map_level3_type map;
   printf("Building composite map at lon=%.4f  lat=%.4f\n", 
         (double) latlon_.lon, (double) latlon_.lat);
   build_60x60_map(map_dir_, latlon_, &map);
   // build output file
   sprintf(fname, "%.4f%c_%.4f%c.pgm", 
         fabs(latlon_.lon), latlon_.lon < 0.0 ? 'W' : 'E',
         fabs(latlon_.lat), latlon_.lat < 0.0 ? 'S' : 'N');
   FILE *ofp = fopen(fname, "w");
   if (!ofp) {
      fprintf(stderr, "Unable to create '%s'\n", fname);
      return 1;
   }
   fprintf(ofp, "P5\n%d %d\n255\n", MAP_LEVEL3_SIZE, MAP_LEVEL3_SIZE);
   uint8_t buf[MAP_LEVEL3_SIZE];
   for (uint32_t y=0; y<MAP_LEVEL3_SIZE; y++) {
      for (uint32_t x=0; x<MAP_LEVEL3_SIZE; x++) {
         uint32_t idx = (uint32_t) (x + y * MAP_LEVEL3_SIZE);
         // land and unknown are black
         int val = 255 - map.grid[idx].min_depth;
         if (val == 255) {
            val = 0;
         }
//if (val != 0) {
//   printf("%d,%d (%d)  -> %d\n", x, y, idx, val);
//}
         buf[x] = (uint8_t) val;
      }
      fwrite(buf, 1, MAP_LEVEL3_SIZE, ofp);
   }
   printf("Writing file '%s'\n", fname);
   fclose(ofp);
   return 0;
}

