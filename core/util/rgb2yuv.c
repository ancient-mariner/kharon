#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "image.h"

int main(int argc, char **argv)
{
   if (argc != 2) {
      printf("Changes colorspace of image from RGB to YUV\n");
      printf("Usage: %s <rgb.pnm>\n", argv[0]);
      return 1;
   }
   /////////////////////////////////////////////
   const char *infile = argv[1];
   char outfile[256];
   strcpy(outfile, infile);
   size_t len = strlen(outfile) - 4;
   assert(len > 4);
   sprintf(&outfile[len], "_yuv.pnm");
   /////////////////////////////////////////////
   // read file
   image_type *img = create_image(infile);
   if (!img) {
      printf("Failed to read input file '%s'\n", infile);
      return -1;
   }
   /////////////////////////////////////////////
   // convert pixels (in place)
   uint32_t n_pix = (uint32_t) (img->size.x * img->size.y);
   for (uint32_t i=0; i<n_pix; i++) {
      pix_type pix = img->rgb[i];
      pix_type out;
      uint32_t y = (uint32_t) 
            ((  66*pix.r + 129*pix.g +  25*pix.b + 128) >> 8) + 16;
      uint32_t u = (uint32_t) 
            (( -38*pix.r -  74*pix.g + 112*pix.b + 128) >> 8) + 128;
      uint32_t v = (uint32_t) 
            (( 112*pix.r -  94*pix.g -  18*pix.b + 128) >> 8) + 128;
      assert(y <= 255);
      assert(u <= 255);
      assert(v <= 255);
      out.y = (uint8_t) y;
      out.u = (uint8_t) u;
      out.v = (uint8_t) v;
      img->rgb[i] = out;
   }
   /////////////////////////////////////////////
   // write file
   write_pnm_file(outfile, img->size, img->rgb);
   /////////////////////////////////////////////
   return 0;
}

