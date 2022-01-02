#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// takes gebco 2020 ascii octant file and generates image
// 15-second resolution

#define IMAGE_PATH   "2020_gebco_ascii/"

#define ASCII_COLS   21600
#define ASCII_ROWS   21600

#define IMAGE_COLS   (10 * 60 * 4)
#define IMAGE_ROWS   (10 * 60 * 4)

static int data_row_[IMAGE_COLS] = { 0 };
static uint8_t color_data_[3*IMAGE_COLS] = { 0 };

struct world_coord {
   double lat;
   double lon;
};
typedef struct world_coord world_coord_type;


static void write_image_row(FILE *ofp)
{
   for (uint32_t i=0; i<IMAGE_COLS; i++) {
      int val = data_row_[i];
      uint8_t r, g, b;
      // colorize value
      if (val < 0) {
//         if (val > -12) {
//            r = 64;
//            g = 255;
//            b = 220;
//         } else if (val > -250) {
//         if (val > -250) {
//            r = 0;
//            g = (uint8_t) (255.0 + 255.0 * (double) val / 250.0);
//            b = 255;
//         } else if (val > -500) {
//            double val2 = (double) (val + 250);
//            r = (uint8_t) (-255.0 * val2 / 250.0);
//            g = 0;
//            b = 255;
//         } else {
//            r = 255;
//            g = 0;
//            b = 255;
//         }
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
   fwrite(color_data_, 3, IMAGE_COLS, ofp);
}


static const char *get_gebco_filename(world_coord_type pos)
{
   if (pos.lat >= 0) {
      // northern hemisphere
      if (pos.lon >= 90.0) {
         // asia
         return IMAGE_PATH "gebco_2020_n90.0_s0.0_w90.0_e180.0.asc";
      } else if (pos.lon <= -90.0) {
         // pacific
         return IMAGE_PATH "gebco_2020_n90.0_s0.0_w-180.0_e-90.0.asc";
      } else if (pos.lon < 0.0) {
         // atlantic
         return IMAGE_PATH "gebco_2020_n90.0_s0.0_w-90.0_e0.0.asc";
      } else {
         // europe
         return IMAGE_PATH "gebco_2020_n90.0_s0.0_w0.0_e90.0.asc";
      }
   } else {
      // northern hemisphere
      if (pos.lon >= 90.0) {
         // australia
         return IMAGE_PATH "gebco_2020_n0.0_s-90.0_w90.0_e180.0.asc";
      } else if (pos.lon <= -90.0) {
         // pacific
         return IMAGE_PATH "gebco_2020_n0.0_s-90.0_w-180.0_e-90.0.asc";
      } else if (pos.lon < 0.0) {
         // south america
         return IMAGE_PATH "gebco_2020_n0.0_s-90.0_w-90.0_e0.0.asc";
      } else {
         // indian ocean
         return IMAGE_PATH "gebco_2020_n0.0_s-90.0_w0.0_e90.0.asc";
      }
   }
}

static uint32_t get_top_offset(world_coord_type pos)
{
   uint32_t top;
   if (pos.lat >= 0) {
      uint32_t bottom = (uint32_t) (pos.lat / 10.0);
      if (bottom > 8) {
         bottom = 8;
      }
      top = (uint32_t) (8 - bottom);
   } else {
      top = (uint32_t) (-pos.lat / 10.0);
   }
   top *= IMAGE_ROWS;
   return top;
}

// see where 10 degree block for this longitude resides w/in its
//    octant
static uint32_t get_left_offset(world_coord_type pos)
{
   uint32_t left = 0;
   double adj_lon = pos.lon;   // assume europe
   if (pos.lon >= 90.0) {
      // asia
      adj_lon = pos.lon - 90.0;
   } else if (pos.lon <= -90.0) {
      // pacific
      adj_lon = pos.lon + 180.0;
   } else if (pos.lon < 0.0) {
      // atlantic
      adj_lon = pos.lon + 90.0;
   }
   // get 10-degree block (0-8)
   left = (uint32_t) (adj_lon / 10.0);
   if (left > 8) {
      left = 8;
   }
   left *= IMAGE_COLS;
   return left;
}


int main(int argc, char **argv) 
{
   int rc = -1;
   if (argc != 3) {
      printf("Generates full-res image from of 10x10 degree square from "
            "gebco ascii files\n");
      printf("Takes as input a geographical location in that square\n");
      printf("Usage: %s <lon> <lat>\n", argv[0]);
      return 1;
   }
   world_coord_type pos;
   pos.lon = atof(argv[1]);
   pos.lat = atof(argv[2]);
   if (fabs(pos.lat) >= 90.0) {
      fprintf(stderr, "Latitude %f isn't valid\n", pos.lat);
      return 1;
   }
   if (pos.lon > 180.0) {
      pos.lon -= 360.0;
   }
   if (fabs(pos.lon) >= 180.0) {
      fprintf(stderr, "Longitude %s isn't supported\n", argv[1]);
      return 1;
   }
   printf("Pulling data for N%.1f degrees latitude, E%.1f degrees longitude\n",
         pos.lat, pos.lon);
   const char *fname = get_gebco_filename(pos);
   uint32_t left_offset = get_left_offset(pos);
   uint32_t top_offset = get_top_offset(pos);
printf("left offset: %d\n", left_offset);
printf("top offset: %d\n", top_offset);
printf("loading %s\n", fname);
//return 0;
   /////////////////////////////////////////////////////////////////////
   // each token should be 6 or less chars, so 10 should be plenty
   uint32_t max_len = ASCII_COLS * 10;
   FILE *ofp = NULL;
   char *buf = malloc(max_len);
   FILE *fp = fopen(fname, "r");
   if (!fp) {
      fprintf(stderr, "Unable to open '%s'\n", fname);
      goto err;
   }
   char img_name[256];
   sprintf(img_name, "%s_N%.1f_E%.1f.pnm", fname, pos.lat, pos.lon);
printf("Writing to '%s'\n", img_name);
   ofp = fopen(img_name, "w");
   if (!ofp) {
      fprintf(stderr, "Unable to open '%s' for writing\n", img_name);
      goto err;
   }
   fprintf(ofp, "P6\n%d %d\n255\n", IMAGE_COLS, IMAGE_ROWS);
   /////////////////////////////////////////////////////////////////////
   // read header
   // ignore first 6 lines
   for (uint32_t i=0; i<6; i++) {
      fgets(buf, (int) max_len, fp);
   }
   for (uint32_t y=0; y<ASCII_ROWS; y++) {
      fgets(buf, (int) max_len, fp);
      if (y >= top_offset) {
         if (y >= (top_offset + IMAGE_ROWS)) {
            break;
         }
         char *line = buf;
         for (uint32_t x=0; x<ASCII_COLS; x++) {
            char *tok = strtok(line, " ");
            line = NULL;
            if (x >= left_offset) {
               if (x >= (left_offset + IMAGE_COLS)) {
                  break;
               }
               data_row_[x-left_offset] = atoi(tok);
            }
         }
         write_image_row(ofp);
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

