#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

int main(int argc, char **argv) {
   int rc = 1;
   FILE *ifp = NULL;
   FILE *ofp = NULL;
   if (argc != 5) {
      printf("Usage: %s <infile> <cols> <rows> <outfile>\n", argv[0]);
      return 1;
   }
   const char *infile = argv[1];
   int32_t cols = atoi(argv[2]);
   int32_t rows = atoi(argv[3]);
   const char *outfile = argv[4];
   printf("Converting %s -> %s (%dx%d)\n", infile, outfile, cols, rows);
   // 
   uint32_t ycols = (uint32_t) (((cols + 31) / 32) * 32);
   uint32_t yrows = (uint32_t) (((rows + 15) / 16) * 16);
   uint32_t n_ypix = (uint32_t) (ycols * yrows);
   uint32_t n_tot_ypix = (uint32_t) (3 * ycols * yrows / 2);
//printf("read size %d x %d = %d (%d)\n", ycols, yrows, n_ypix, n_tot_ypix);
   //
   ifp = fopen(infile, "r");
   if (!ifp) {
      printf("Unable to open input file %s\n", infile);
      return 1;
   }
   fseek(ifp, 0, SEEK_END);
   int64_t len = ftell(ifp); 
   if (len != n_tot_ypix) {
      printf("Size mismatch. Expected input of %d bytes, found %ld\n",
            n_tot_ypix, len);
      return 1;
   }
   fseek(ifp, 0, SEEK_SET);
   int8_t y[n_ypix];
   int8_t u[n_ypix/4];
   int8_t v[n_ypix/4];
   fread(y, n_ypix, 1, ifp);
   fread(u, n_ypix/4, 1, ifp);
   fread(v, n_ypix/4, 1, ifp);
   fclose(ifp);
   ifp = NULL;
   //
   ofp = fopen(outfile, "w");
   if (!ofp) {
      printf("Unable to open output file %s\n", outfile);
      goto done;
   }
   // write pgm header
   fprintf(ofp, "P5\n%d %d\n255\n", cols, 3*rows/2);
   // copy y to output
   uint32_t offset = 0;
   for (int32_t i=0; i<rows; i++) {
      fwrite(&y[offset], (size_t) cols, 1, ofp);
      offset = (uint32_t) (offset + ycols);
   }
   // copy v and u to output
   offset = 0;
   for (int32_t i=0; i<rows/2; i++) {
      fwrite(&u[offset], (size_t) (cols/2), 1, ofp);
      fwrite(&v[offset], (size_t) (cols/2), 1, ofp);
      offset += ycols/2;
   }
   if (ofp) {
      fclose(ofp);
   }
   ofp = NULL;
   //
   rc = 0;
   goto done;
done:
   if (ifp) {
      fclose(ifp);
   }
   if (ofp) {
      fclose(ofp);
   }
   return rc;
}


