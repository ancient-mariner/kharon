/***********************************************************************
* This file is part of kharon <https://github.com/ancient-mariner/kharon>.
* Copyright (C) 2019-2022 Keith Godfrey
*
* kharon is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, version 3.
*
* kharon is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with kharon.  If not, see <http://www.gnu.org/licenses/>.
***********************************************************************/
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

