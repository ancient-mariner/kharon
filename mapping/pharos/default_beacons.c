#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <assert.h>
#include "pin_types.h"
#include "beacon.h"
#include "lin_alg.h"

static char map_folder_[STR_LEN];
static char declination_file_[STR_LEN];
static FILE *outfile_ = NULL;

static void parse_command_line(
      /* in     */ const int argc,
      /* in     */ const char **argv
      )
{
   if (argc != 3) {
      printf("Usage: %s <map folder> <beacon txt file>\n", argv[0]);
      goto err;
   }
   strncpy(map_folder_, argv[1], STR_LEN);
   size_t len = strlen(map_folder_);
   assert(len > 1);
   snprintf(declination_file_, STR_LEN, "%s%s%s", map_folder_,
         map_folder_[len-1] == '/' ? "" : "/", DECLINATION_FILE);
   outfile_ = fopen(argv[2], "w");
   if (!outfile_) {
      fprintf(stderr, "Unable to create output file '%s'\n", argv[2]);
      goto err;
   }
   return;
err:
   exit(1);
}


// 
static void generate_beacons(void)
{
   // generate default beacons for each band
   map_level1_type *map1 = load_map_level1(map_folder_, NULL);
   akn_position_type pos;
   pos.akn_y = DEFAULT_LAT_OFFSET_DEG;
   while (pos.akn_y < DEFAULT_LAT_SOUTHERN_BOUNDS_AKN) {
      // circumference correction for latitude
      double circum_scale = sin(D2R * pos.akn_y);   
      double circum_met = circum_scale * 360.0 * DEG_TO_METER;
      // how many beacons and how far between them, in this row
      double steps = ceil(circum_met / DEFAULT_LON_STEP_MET);
      double step_size_met = circum_met / steps;
printf("akn-lat %.2f  circum %.2fm  scale %.4f  steps %.2f, step size %.2fm\n", -(pos.akn_y-90.0), circum_met, circum_scale, steps, step_size_met);
      for (double i=0.0; i<steps; i+=1.0) {
         // position of this beacon (akn base)
         double x_met = i * step_size_met;
         pos.akn_x = x_met * METER_TO_DEG / circum_scale;
         ///////////////////////////////////////
         // check depth -- create beacon if grid square has at least
         //    some water in it
         map_grid_num_type grid_num = convert_akn_to_grid(pos, NULL);
         uint32_t grid_idx = (uint32_t) (grid_num.akn_x + grid_num.akn_y * 360);
         map_level1_square_type *square = &map1->grid[grid_idx];
         if (square->low >= 0) {
            // lowest point in square is above sea level, so no
            //    point putting beacon here
//printf("Skipping %.4f,%.4f due depth, high=%d low=%d\n", pos.akn_x, pos.akn_y, square->low, square->high);
            continue;
         }
//printf("Keeping %.4f,%.4f with positive depth (low=%d)\n", pos.akn_x, pos.akn_y, square->low);
         ///////////////////////////////////////
         // check magnetic inclination. if too near a magnetic pole, 
         //    don't create a beacon
         world_coordinate_type latlon = convert_akn_to_world(pos);
         double declination, inclination;
         load_declination(declination_file_, latlon, 
               &declination, &inclination);
         if (fabs(inclination) > BEACON_INCLINATION_LIMIT) {
//printf("Skipping %.4f,%.4f due inclination %.3f\n", pos.akn_x, pos.akn_y, inclination);
            continue;
         }
//printf("Keeping %.4f,%.4f with inclination %.3f\n", pos.akn_x, pos.akn_y, inclination);
         ///////////////////////////////////////
         // position is tentatively valid. output it
printf("Keeping %.4f,%.4f with low depth %d and inclination %.3f\n", latlon.lon, latlon.lat, square->low, inclination);
         fprintf(outfile_, "%.6f %.6f\n", pos.akn_x, pos.akn_y);
      }
      pos.akn_y += DEFAULT_LAT_STEP_DEG;
   }
}


int main(int argc, const char **argv)
{
   parse_command_line(argc, argv);
   generate_beacons();
   /////////////////////////////
   if (outfile_) {
      fclose(outfile_);
   }
   return 0;
}

