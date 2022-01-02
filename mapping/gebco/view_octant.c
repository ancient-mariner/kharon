#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// takes gebco ascii octant file and generates image

#define EXPECTED_COLS   21600u
#define EXPECTED_ROWS   21600u

#define BIN_SIZE     8

#define N_PIX_COLS      (EXPECTED_COLS / BIN_SIZE)
#define N_PIX_ROWS      (EXPECTED_ROWS / BIN_SIZE)

static int data_row_[N_PIX_COLS] = { 0 };
static uint8_t color_data_[3*N_PIX_COLS] = { 0 };


// write depth (and altitude) data to a pnm file
// pixel colors are based on depth (altitude)
static void write_image_row(FILE *ofp)
{
   for (uint32_t i=0; i<N_PIX_COLS; i++) {
      int val = data_row_[i] / (BIN_SIZE * BIN_SIZE);
      // colorize value
      uint8_t r, g, b;
      if (val < 0) {
         if (val > -4000) {
            r = 0;
            g = (uint8_t) (255.0 + 255.0 * (double) val / 4000.0);
            b = 255;
         } else if (val > -8000) {
            double val2 = (double) (val + 4000);
            r = (uint8_t) (-255.0 * val2 / 4000.0);
            g = 0;
            b = 255;
         } else {
            r = 255;
            g = 0;
            b = 255;
         }
      } else {
         if (val < 1000) {
            r = (uint8_t) (255.0 * (double) val / 1000.0);
            g = 192;
            b = 0;
         } else if (val < 3000) {
            double val2 = (double) (val - 1000);
            r = (uint8_t) (255.0 - 96.0 * val2 / 2000.0);
            g = (uint8_t) (192.0 - 64.0 * val2 / 2000.0);
            b = 0;
         } else if (val < 6000) {
            double val2 = (double) (val - 3000);
            r = 160;
            g = (uint8_t) (128.0 + 32.0 * val2 / 3000.0);
            b = (uint8_t) (160.0 * val2 / 3000.0);
         } else {
            r = 160;
            g = 160;
            b = 160;
         }
      }
      color_data_[3*i] = r;
      color_data_[3*i+1] = g;
      color_data_[3*i+2] = b;
   }
   fwrite(color_data_, 3, N_PIX_COLS, ofp);
}


int main(int argc, char **argv) 
{
   int rc = -1;
   if (argc != 2) {
      printf("Generates image from GEBCO data\n");
      printf("Usage: %s <filename>\n", argv[0]);
      return 1;
   }
   /////////////////////////////////////////////////////////////////////
   // each token should be 6 or less chars, so 10 should be plenty
   uint32_t max_len = EXPECTED_COLS * 10;
   FILE *ofp = NULL;
   char *buf = malloc(max_len);
   char *fname = argv[1];
   FILE *fp = fopen(fname, "r");
   if (!fp) {
      fprintf(stderr, "Unable to open '%s'\n", fname);
      goto err;
   }
   char img_name[256];
   sprintf(img_name, "%s.pnm", fname);
   ofp = fopen(img_name, "w");
   if (!ofp) {
      fprintf(stderr, "Unable to open '%s' for writing\n", img_name);
      goto err;
   }
   fprintf(ofp, "P6\n%d %d\n255\n", N_PIX_COLS, N_PIX_ROWS);
   /////////////////////////////////////////////////////////////////////
   // read header
   // ignore first 6 lines
   for (uint32_t i=0; i<6; i++) {
      fgets(buf, (int) max_len, fp);
   }
   uint32_t n_cols = EXPECTED_COLS;
   uint32_t n_rows = EXPECTED_ROWS;
   uint32_t row_ctr = 0;
   for (uint32_t y=0; y<n_rows; y++) {
      fgets(buf, (int) max_len, fp);
      char *line = buf;
      for (uint32_t x=0; x<n_cols; x++) {
         char *tok = strtok(line, " ");
         line = NULL;
         int val = atoi(tok);
         int bin = (int) (x / BIN_SIZE);
         data_row_[bin] += val;
      }
      if (++row_ctr == BIN_SIZE) {
         write_image_row(ofp);
         memset(data_row_, 0, N_PIX_COLS * sizeof data_row_[0]);
         row_ctr = 0;
      }
   }
   rc = 0;
   goto end;
err:
   rc = -1;
end:
   if (fp) {
      fclose(fp);
      fp = NULL;
   }
   if (ofp) {
      fclose(ofp);
      ofp = NULL;
   }
   if (buf) {
      free(buf);
      buf = NULL;
   }
   return rc;
}

