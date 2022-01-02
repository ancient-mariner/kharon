////////////////////////////////////////////////////////////////////////
// output

void write_depth_map(
      /* in     */ const path_map_type *path_map,
      /* in     */ const char *fname
      )
{
   printf("Writing depth map to '%s'\n", fname);
   FILE *fp = fopen(fname, "w");
   if (!fp) {
      fprintf(stderr, "Cannot open '%s' for writing\n", fname);
      exit(1);
   }
   fprintf(fp, "P6\n%d %d\n255\n", path_map->size.x, path_map->size.y);
   uint8_t row[3 * path_map->size.x];
//printf("  %d beacons\n", path_map->num_beacons);
//for (uint32_t i=0; i<path_map->num_beacons; i++) {
//   image_coordinate_type pos = path_map->beacon_ref[i].pos_in_map;
//   printf("    %d,%d\n", pos.x, pos.y);
//}
   uint8_t r, g, b;
   assert(path_map->size.x == 720);
   assert(path_map->size.y == 720);
//for (uint32_t i=0; i<path_map->num_beacons; i++) {
//   const map_beacon_reference_type *ref = &path_map->beacon_ref[i];
//   printf("%d\tidx %6d\t%.4f,%.4f\n", i, ref->index, ref->coords.akn_x, ref->coords.akn_y);
//}
   for (uint32_t y=0; y<path_map->size.y; y++) {
      for (uint32_t x=0; x<path_map->size.x; x++) {
         uint32_t map_idx = x + y * path_map->size.x;
         int16_t val = 
               (int16_t) (-path_map->feature_nodes[map_idx].depth_meters);
         int beacon_found = 0;
         r = 0;
         g = 0;
         b = 0;
         if (path_map != NULL) {
            if ((x >= 359) && (x <= 361) && (y >= 359) && (y <= 361)) {
               beacon_found = 1;
            }
            // if beacon is in this location, indicate on map
            for (uint32_t i=0; i<path_map->num_beacons; i++) {
               image_coordinate_type pos =
                     path_map->beacon_ref[i].pos_in_map;
               // displaying a single pixel is too difficult to see
               //    so increase size to 3x3
               int32_t dx = (int32_t) pos.x - (int32_t) x;
               int32_t dy = (int32_t) pos.y - (int32_t) y;
               if ((abs(dx) <= 1) && (abs(dy) <= 1)) {
//               if ((pos.x == x) && (pos.y == y)) {
                  r = 255;
if ((dx == 0) && (dy == 0)) {
   const map_beacon_reference_type *ref = &path_map->beacon_ref[i];
   printf("dm   %4d    %d,%d    (%.4f,%.4f)   %.1f\n", ref->index, 
         x, y, ref->coords.akn_x, ref->coords.akn_y, (double) ref->path_weight);
}
//printf("beacon %d. depth %d\n", i, val);
                  beacon_found = 1;
                  break;
               }
            }
         }
         if (beacon_found == 0) {
            // no beacon -- colorize depth
            if (val < 0) {
               if (val > -250) {
                  r = 0;
                  g = (uint8_t) (255.0 + 255.0 * (double) val / 255.0);
                  b = 255;
               } else if (val > -500) {
                  double val2 = (double) (val + 250);
                  r = (uint8_t) (-255.0 * val2 / 250.0);
                  g = 0;
                  b = 255;
               } else {
//printf("Deep  %d,%d   %d\n", x, y, val);
                  b = 255;
               }
            } else {
               g = 192;
            }
         }
//if (beacon_found == 1) {
//   printf("  %d,%d   %d,%d,%d\n", x, y, r, g, b);
//}
         row[3*x] = r;
         row[3*x+1] = g;
         row[3*x+2] = b;
      }
      fwrite(row, 3, path_map->size.x, fp);
   }
   fclose(fp);
}


void write_path_map(
      /* in     */ const path_map_type *path_map,
      /* in     */ const char *fname
      )
{
   printf("Writing path map to '%s'\n", fname);
   FILE *fp = fopen(fname, "w");
   if (!fp) {
      fprintf(stderr, "Cannot open '%s' for writing\n", fname);
      exit(1);
   }
   uint8_t row[3 * path_map->size.x];
   // get lowest path weight in map, and plot map weight colors relative
   //    to that
   float min_weight = -1.0f;
   for (uint32_t y=0; y<path_map->size.y; y++) {
      for (uint32_t x=0; x<path_map->size.x; x++) {
         uint32_t map_idx = x + y * path_map->size.x;
         float weight = path_map->nodes[map_idx].weight;
         if ((weight >= 0.0f) && 
               ((min_weight < 0.0f) || (weight < min_weight))) {
            min_weight = weight;
         }
      }
   }
   fprintf(fp, "P6\n%d %d\n255\n", path_map->size.x, path_map->size.y);
   assert(path_map->size.x > 0);
   for (uint32_t y=0; y<path_map->size.y; y++) {
      for (uint32_t x=0; x<path_map->size.x; x++) {
         uint32_t map_idx = x + y * path_map->size.x;
         // colorize weight
         uint8_t r, g, b;
         double weight = path_map->nodes[map_idx].weight - min_weight;
         if (weight < 0.0) {
            r = 0;
            g = 255;
            b = 0;
         } else {
            int32_t val = (int32_t) weight;
            if (val < 256) {
               r = (uint8_t) val;
               g = 0;
               b = (uint8_t) (255 - val);
            } else if (val < 512) {
               val -= 256;
               r = 255;
               g = (uint8_t) val;
               b = 0;
            } else if (val < 768) {
               val -= 512;
               r = (uint8_t) (255 - val / 2);
               g = (uint8_t) (255 - val / 2);
               b = (uint8_t) val;
            } else if (val < 1024) { 
               val -= 768;
               r = (uint8_t) (128 - val / 2);
               g = 128;
               b = 255;
            } else {
               val -= 768;
               r = 128;
               g = 128;
               b = (uint8_t) (255 - val);
            }
         }
         row[3*x] = r;
         row[3*x+1] = g;
         row[3*x+2] = b;
      }
      fwrite(row, 3, path_map->size.x, fp);
   }
   fclose(fp);
}


void write_direction_map(
      /* in     */ const path_map_type *path_map,
      /* in     */ const char *fname
      )
{
   printf("Writing direction map to '%s'\n", fname);
   FILE *fp = fopen(fname, "w");
   if (!fp) {
      fprintf(stderr, "Cannot open '%s' for writing\n", fname);
      exit(1);
   }
   fprintf(fp, "P6\n%d %d\n255\n", path_map->size.x, path_map->size.y);
   uint8_t row[3 * path_map->size.x];
   for (uint32_t y=0; y<path_map->size.y; y++) {
      for (uint32_t x=0; x<path_map->size.x; x++) {
         uint32_t map_idx = x + y * path_map->size.x;
         path_map_node_type *node = &path_map->nodes[map_idx];
         // colorize direction
         uint8_t r, g, b;
         if (node->weight < 0.0f) {
            r = 0;
            g = 255;
            b = 0;
         } else {
//printf("%d,%d %.1f  (%.1f)\n", x, y, (double) node->true_course.angle16 * BAM16_TO_DEG, node->weight);
            double map = (double) node->true_course.angle16 / 32768.0 - 1.0;
            r = (uint8_t) (255.0 * fabs(map));
            g = 128;
            map = (double) ((uint16_t) 
                  (16384 + node->true_course.angle16)) / 32768.0 - 1.0;
            b = (uint8_t) (255.0 * fabs(map));
            // encoding    red 255 going north, 0 going south  (blue 127)
            //             blue 255 going west, 0 going east   (red 127)
         }
         row[3*x] = r;
         row[3*x+1] = g;
         row[3*x+2] = b;
      }
      fwrite(row, 3, path_map->size.x, fp);
   }
   fclose(fp);
}


void write_active_direction_map(
      /* in     */ const path_map_type *path_map,
      /* in     */ const char *fname
      )
{
   printf("Writing active direction map to '%s'\n", fname);
   FILE *fp = fopen(fname, "w");
   if (!fp) {
      fprintf(stderr, "Cannot open '%s' for writing\n", fname);
      exit(1);
   }
   fprintf(fp, "P6\n%d %d\n255\n", path_map->size.x, path_map->size.y);
   uint8_t row[3 * path_map->size.x];
   for (uint32_t y=0; y<path_map->size.y; y++) {
      for (uint32_t x=0; x<path_map->size.x; x++) {
         uint32_t map_idx = x + y * path_map->size.x;
         path_map_node_type *node = &path_map->nodes[map_idx];
         // colorize direction
         uint8_t r, g, b;
         if (node->weight < 0.0f) {
            r = 0;
            g = 255;
            b = 0;
         } else {
            double map = (double) node->active_course.angle16 / 32768.0 - 1.0;
            r = (uint8_t) (255.0 * fabs(map));
            g = 128;
            map = (double) ((uint16_t) 
                  (16384 + node->active_course.angle16)) / 32768.0 - 1.0;
            b = (uint8_t) (255.0 * fabs(map));
            // encoding    red 255 going north, 0 going south  (blue 127)
            //             blue 255 going west, 0 going east   (red 127)
         }
         row[3*x] = r;
         row[3*x+1] = g;
         row[3*x+2] = b;
      }
      fwrite(row, 3, path_map->size.x, fp);
   }
   fclose(fp);
}


// output
////////////////////////////////////////////////////////////////////////
