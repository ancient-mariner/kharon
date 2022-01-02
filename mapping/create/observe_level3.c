#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "world_map.h"

// make sure file has expected extension
static void verify_map3_name(
      /* in    */ const char *fname
      )
{
   size_t len = strlen(fname);
   if (len <= 5) {
      fprintf(stderr, "'%s' is not a valid level3 file\n", fname);
      exit(1);
   }
   if (strcmp(".map3", &fname[len-5]) != 0) {
      fprintf(stderr, "'%s' is not a valid level3 file\n", fname);
      exit(1);
   }
}


int main(int argc, const char **argv)
{
   if (argc != 2) {
      printf("Usage: %s <level-3 map>\n", argv[0]);
      return 1;
   }
   const char *fname = argv[1];
   verify_map3_name(fname);
   FILE *fp = fopen(fname, "r");
   if (!fp) {
      fprintf(stderr, "Unable to open '%s'\n", fname);
      return 1;
   }
   map_level3_type map;
   fread(&map, sizeof map, 1, fp);
   fclose(fp);
   char name[BUF_LEN];
   sprintf(name, "%s.pgm", fname);
   FILE *ofp = fopen(name, "w");
   if (!ofp) {
      fprintf(stderr, "Unable to create '%s'\n", name);
      return 1;
   }
   fprintf(ofp, "P5\n%d %d\n255\n", MAP_LEVEL3_SIZE, MAP_LEVEL3_SIZE);
   uint8_t buf[MAP_LEVEL3_SIZE];
   for (uint32_t y=0; y<MAP_LEVEL3_SIZE; y++) {
      for (uint32_t x=0; x<MAP_LEVEL3_SIZE; x++) {
         uint32_t idx = (uint32_t) (x + y * MAP_LEVEL3_SIZE);
         int val = 255 - map.grid[idx].min_depth;
         if (val == 255) {
            val = 0;
         }
if (val != 0) {
   printf("%d,%d  -> %d\n", x, y, val);
}
         buf[x] = (uint8_t) val;
         //printf("%d,%d     %d\n", x, y, map.grid[idx].min_depth);
      }
      fwrite(buf, 1, MAP_LEVEL3_SIZE, ofp);
   }
   fclose(ofp);
   return 0;
}

