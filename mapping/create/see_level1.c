#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "world_map.h"

int main(int argc, const char **argv)
{
   if (argc != 2) {
      printf("Usage: %s <level-1 map>\n", argv[0]);
   }
   const char *fname = argv[1];
   FILE *fp = fopen(fname, "r");
   if (!fp) {
      fprintf(stderr, "Unable to open '%s'\n", fname);
      return 1;
   }
   map_level1_type map;
   fread(&map, sizeof map, 1, fp);
   fclose(fp);
   char name[BUF_LEN];
   sprintf(name, "%s.pgm", fname);
   printf("Writing '%s'\n", name);
   FILE *ofp = fopen(name, "w");
   if (!ofp) {
      fprintf(stderr, "Unable to create '%s'\n", name);
      return 1;
   }
   const uint32_t width = 360;
   const uint32_t height = 180;
   fprintf(ofp, "P5\n%d %d\n255\n", width, height);
   uint8_t buf[width];
   for (uint32_t y=0; y<height; y++) {
      for (uint32_t x=0; x<width; x++) {
         uint32_t idx = (uint32_t) (x + y * width);
         int depth = map.grid[idx].low;
         uint8_t code = 0;
         if (depth < 0) {
            code = encode_submap_depth((uint16_t) (-depth));
         } 
         buf[x] = code;
      }
      fwrite(buf, 1, width, ofp);
   }
   fclose(ofp);
   return 0;
}

