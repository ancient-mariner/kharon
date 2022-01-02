#include <stdio.h>
#include <stdint.h>
#include "world_map.h"

static void check_depth_encode(void)
{
   printf("===========================================================\n");
   printf("Depth encoding check\n");
   for (int i=0; i<13; i++) {
      int code = encode_submap_depth((uint16_t) i);
      printf("%5d    %d\n", i, code);
   }
   printf("-------------------------\n");
   for (int i=100; i<630; i+=5) {
      int code = encode_submap_depth((uint16_t) i);
      printf("%5d    %d\n", i, code);
   }
   printf("-------------------------\n");
   for (int i=600; i<11500; i+=50) {
      int code = encode_submap_depth((uint16_t) i);
      printf("%5d    %d\n", i, code);
   }
}

static void check_depth_decode(void)
{
   printf("===========================================================\n");
   printf("Depth decoding check\n");
   for (int i=0; i<256; i++) {
      int depth = decode_submap_depth((uint8_t) i);
      printf("%5d    %d\n", i, depth);
   }
}

int main(void)
{
   check_depth_encode();
   check_depth_decode();
   return 0;
}

