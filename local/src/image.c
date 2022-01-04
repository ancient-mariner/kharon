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
#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "image.h"
#include "mem.h"

double clamp(double x);

////////////////////////////////////////////////////////////////////////
// file IO

// like getline, but filters out lines starting with '#'
static int get_next_line(FILE *fp, char *buf, int buf_len)
{
   int rc = -1;
   do {
      if (fgets(buf, buf_len, fp) == NULL) {
         perror("Error reading line from file");
         goto err;
      }
   } while (buf[0] == '#');
   rc = 0;
err:
   return rc;
}

int write_pnm_file(
      /* in     */ const char *filename,
      /* in     */ const image_size_type size,
      /* in     */ const pix_type * restrict rgb
      )
{
   FILE *fp = NULL;
   int rc = -1;
   rgb_pix_type rgb_row[size.cols];
   uint32_t idx = 0;
   printf("Writing %s\n", filename);
   if ((fp = fopen(filename, "wb")) == NULL) {
      perror("Unable to open output file for writing PNM");
      goto err;
   }
   fprintf(fp, "P6\n%d %d\n255\n", size.cols, size.rows);
   for (uint32_t r=0; r<size.rows; r++)
   {
      for (uint32_t c=0; c<size.cols; c++) {
         rgb_pix_type *out = &rgb_row[c];
         pix_type in = rgb[idx++];
         out->r = in.r;
         out->g = in.g;
         out->b = in.b;
      }
      fwrite(&rgb_row, sizeof rgb_row, 1, fp);
   }
//   for (uint32_t i=0; i<n_pix; i++) {
//      pix_type p = rgb[i];
//      rgb_pix_type pix = { .r=p.r, .g=p.g, .b=p.b };
//      fwrite(&pix, sizeof pix, 1, fp);
//   }
   rc = 0;
err:
   if (fp)
      fclose(fp);
   return rc;
}

int write_pnm_file_by_channel(const char *filename, image_size_type size, uint8_t * restrict r, uint8_t * restrict g, uint8_t * restrict b)
{
   FILE *fp = NULL;
   int rc = -1;
   uint32_t n_pix = (uint32_t) (size.cols * size.rows);
   printf("Writing %s\n", filename);
   if ((fp = fopen(filename, "wb")) == NULL) {
      perror("Unable to open output file for writing PNM");
      goto err;
   }
   fprintf(fp, "P6\n%d %d\n255\n", size.cols, size.rows);
   if (r && g && b) {
      for (uint32_t i=0; i<n_pix; i++)
         fprintf(fp, "%c%c%c", r[i], g[i], b[i]);
   } else if (!r && g && b) {
      for (uint32_t i=0; i<n_pix; i++)
         fprintf(fp, "%c%c%c", 0, g[i], b[i]);
   } else if (r && !g && b) {
      for (uint32_t i=0; i<n_pix; i++)
         fprintf(fp, "%c%c%c", r[i], 0, b[i]);
   } else if (r && g && !b) {
      for (uint32_t i=0; i<n_pix; i++)
         fprintf(fp, "%c%c%c", r[i], g[i], 0);
   } else if (!r && !g && b) {
      for (uint32_t i=0; i<n_pix; i++)
         fprintf(fp, "%c%c%c", 0, 0, b[i]);
   } else if (r && !g && !b) {
      for (uint32_t i=0; i<n_pix; i++)
         fprintf(fp, "%c%c%c", r[i], 0, 0);
   } else if (!r && g && !b) {
      for (uint32_t i=0; i<n_pix; i++)
         fprintf(fp, "%c%c%c", 0, g[i], 0);
   }
   rc = 0;
err:
   if (fp)
      fclose(fp);
   return rc;
}



////////////////////////////////////////////////////////////////////////
// image

void free_image(image_type* img)
{
   if (img == NULL)
      return;
   struct memory_pool *pool = get_default_memory_pool();
   cache_free(pool, img->gray);
   cache_free(pool, img->rgb);
   free(img);
}

// allocates storage for image
image_type* raw_create_image(image_size_type size)
{
   size_t n_pix = (size_t) (size.rows * size.cols);
   image_type *img = (image_type*) malloc(sizeof(*img));
   img->size = size;
   // provide default values so struct can be safely and cleanly deleted
   img->gray= NULL;
   img->rgb = NULL;
   // allocate memory
   struct memory_pool *pool = get_default_memory_pool();
   img->gray = cache_malloc(pool, n_pix);
   img->rgb = cache_malloc(pool, n_pix * sizeof(img->rgb[0]));
   /////////////////////
   // TODO verify allocations
   if (!img) {
      goto err;
   }
   goto done;
err:
   fprintf(stderr, "Malloc error in raw_create_image (%d pix)\n",
         (int32_t) n_pix);
   free_image(img);
   img = NULL;
done:
   return img;
}


image_type * create_image_pgm(const char *filename)
{
   FILE *fp;
   const int BUF_SZ = 256;
   char buf[BUF_SZ];
   char * tok;
   size_t n_pix = 0;
   uint32_t bpp;
   image_size_type size;
   image_type *img = NULL;
   ///////////////////////////////
   // open file and get image size
   if ((fp = fopen(filename, "rb")) == NULL) {
      perror("Error opening PNM file");
      goto err;
   }
   // check magic number
   if (get_next_line(fp, buf, BUF_SZ) != 0) {
      fprintf(stderr, "Error parsing PNM header (magic number)\n");
      goto err;
   }
   if (strncmp(buf, "P5", 2) != 0) {
      fprintf(stderr, "Unrecognized file format for file '%s'\n", filename);
      goto err;
   }
   // image size
   if (get_next_line(fp, buf, BUF_SZ) != 0) {
      fprintf(stderr, "Error parsing PNM header (image size)\n");
      goto err;
   }
   tok = strtok(buf, " \n\r\t");
   if (tok == NULL) {
      fprintf(stderr, "Error parsing PNM header (image width)\n");
      goto err;
   }
   size.cols = (uint16_t) atoi(tok);
   tok = strtok(NULL, " \n\r\t");
   if (tok == NULL) {
      fprintf(stderr, "Error parsing PNM header (image height)\n");
      goto err;
   }
   size.rows = (uint16_t) atoi(tok);
   // bits per pixel channel
   if (get_next_line(fp, buf, BUF_SZ) != 0) {
      fprintf(stderr, "Error parsing PNM header (bpp)\n");
      goto err;
   }
   bpp = (uint32_t) atoi(buf);
   if (bpp != 255) {
      fprintf(stderr, "Unrecognized image depth (%d)\n", bpp);
      goto err;
   }
   ////////////////////////////////////////////
   // read image data
   img = raw_create_image(size);
   n_pix = (size_t) (size.rows * size.cols);
   size_t bytes = n_pix;
   if (fread(img->gray, bytes, 1, fp) != 1) {
      fprintf(stderr, "Error reading grayscale data from input file\n");
      goto err;
   }
   for (uint32_t i=0; i<n_pix; i++) {
      uint8_t gray = img->gray[i];
      img->rgb[i].r = gray;
      img->rgb[i].g = gray;
      img->rgb[i].b = gray;
   }
   //
   goto done;
err:
   free_image(img);
   img = NULL;
done:
   // clean up
   if (fp)
      fclose(fp);
   return img;
}

image_type * create_image_pnm(const char *filename)
{
   FILE *fp;
   const int BUF_SZ = 256;
   char buf[BUF_SZ];
   char * tok;
   size_t n_pix = 0;
   uint32_t bpp;
   image_size_type size;
   image_type *img = NULL;
   ///////////////////////////////
   // open file and get image size
   if ((fp = fopen(filename, "rb")) == NULL) {
      perror("Error opening PNM file");
      goto err;
   }
   // check magic number
   if (get_next_line(fp, buf, BUF_SZ) != 0) {
      fprintf(stderr, "Error parsing PNM header (magic number)\n");
      goto err;
   }
   if (strncmp(buf, "P6", 2) != 0) {
      if (strncmp(buf, "P5", 2) == 0) {
         fclose(fp);
         return create_image_pgm(filename);
      }
      fprintf(stderr, "Unrecognized file format for file '%s'\n", filename);
      goto err;
   }
   // image size
   if (get_next_line(fp, buf, BUF_SZ) != 0) {
      fprintf(stderr, "Error parsing PNM header (image size)\n");
      goto err;
   }
   tok = strtok(buf, " \n\r\t");
   if (tok == NULL) {
      fprintf(stderr, "Error parsing PNM header (image width)\n");
      goto err;
   }
   size.cols = (uint16_t) atoi(tok);
   tok = strtok(NULL, " \n\r\t");
   if (tok == NULL) {
      fprintf(stderr, "Error parsing PNM header (image height)\n");
      goto err;
   }
   size.rows = (uint16_t) atoi(tok);
   // bits per pixel channel
   if (get_next_line(fp, buf, BUF_SZ) != 0) {
      fprintf(stderr, "Error parsing PNM header (bpp)\n");
      goto err;
   }
   bpp = (uint32_t) atoi(buf);
   if (bpp != 255) {
      fprintf(stderr, "Unrecognized image depth (%d)\n", bpp);
      goto err;
   }
   ////////////////////////////////////////////
   // read image data
   img = raw_create_image(size);
   n_pix = (size_t) (size.rows * size.cols);
   for (uint32_t i=0; i<n_pix; i++) {
      fread(buf, 3, 1, fp);
      img->rgb[i].r = (uint8_t) buf[0];
      img->rgb[i].g = (uint8_t) buf[1];
      img->rgb[i].b = (uint8_t) buf[2];
   }
   for (uint32_t i=0; i<n_pix; i++) {
      pix_type p = img->rgb[i];
      img->gray[i] = (uint8_t) (0.3*p.r + 0.5*p.g + 0.2*p.g + 0.5/255.0);
   }
   //
   goto done;
err:
   free_image(img);
   img = NULL;
done:
   // clean up
   if (fp)
      fclose(fp);
   return img;
}

double clamp(double x)
{
   if (x < 0.0)
      return 0.0;
   else if (x >= 1.0)
      return 1.0;
   return x;
}

uint8_t clamp255(double x)
{
   if (x <= 0.0)
      return 0;
   else if (x >= 255.0)
      return 255;
   return (uint8_t) x;
}

//uint8_t clamp255x(double x)
//{
//   if (x <= 0.0f)
//      return 0;
//   else if (x >= 1.0f)
//      return 255;
//   return (uint8_t) (x * 255.0f);
//}

//static inline void yuv_to_rgb(pix_type *pix)
//{
//   uint8_t r = clamp255((double) pix->y +
//            1.402f * ((double) ((pix->v-128))));
//   uint8_t g = clamp255((double) pix->y -
//            0.344f * ((double) (pix->u-128)) -
//            0.714f * ((double) (pix->v-128)));
//   uint8_t b = clamp255((double) pix->y +
//            1.772f * ((double) ((pix->u-128))));
//   pix->r = r;
//   pix->g = g;
//   pix->b = b;
//}

static inline void yuv_to_rgb(pix_type *pix)
{
   const double fy = (double) pix->y;
   const double u = (double) (pix->u - 128);
   const double v = (double) (pix->v - 128);
   pix->r = clamp255(fy + 1.402 * v);
   pix->g = clamp255(fy - 0.344 * u - 0.714 * v);
   pix->b = clamp255(fy + 1.772 * u);
}

void convert_yuv_to_rgb(image_type *restrict img)
{
   uint32_t n_pix = (uint32_t) (img->size.x * img->size.y);
   for (uint32_t i=0; i<n_pix; i++) {
      yuv_to_rgb(&img->rgb[i]);
//      const pix_type pix = img->rgb[i];
//      const double v128 = (double) (pix.v - 128);
//      const double u128 = (double) (pix.u - 128);
//      const double py = (double) pix.y;
//      const uint8_t r = clamp255(py + 1.402f * v128);
//      const uint8_t g = clamp255(py - 0.344f * u128 - 0.714f * v128);
//      const uint8_t b = clamp255(py + 1.772f * u128);
//      const pix_type rgb = { .r=r, .g=g, .b=b };
//      img->rgb[i] = rgb;
   }
}

image_type * create_image(const char *filename)
{
   size_t len = strlen(filename);
   if (len < 5) {
      fprintf(stderr, "Unreadable file extension (%s)\n", filename);
      return NULL;
   }
   image_type *img = NULL;
   if (strcmp(&filename[len-4], ".pnm") == 0)
      img = create_image_pnm(filename);
   else if (strcmp(&filename[len-4], ".pgm") == 0)
      img = create_image_pgm(filename);
   else {
      fprintf(stderr, "Unrecognized file extension (%s) for %s\n", &filename[len-4], filename);
      return NULL;
   }
   return img;
}

#if defined(IMAGE_TEST)

int main(int argc, char** argv)
{
   (void) argc;
   (void) argv;
   uint16_t width = 200;
   image_size_type sz = { .x=width, .y=width };
   image_type *img = raw_create_image(sz);
   uint32_t n_pix = (uint32_t) (img->size.x * img->size.y);
   memset(img->rgb, 0, n_pix * sizeof img->rgb[0]);
   for (uint32_t i=0; i<sz.cols; i++) {
      uint32_t idx = i + sz.cols * i;
      img->rgb[idx].r = 255;
   }
   for (uint32_t i=0; i<sz.cols; i++) {
      uint32_t idx = sz.cols - i - 1 + sz.cols * i;
      img->rgb[idx].g = 255;
   }
   write_pnm_file("x.pnm", img->size, img->rgb);
   free_image(img);
   return 0;
}

#endif   // IMAGE_TEST

