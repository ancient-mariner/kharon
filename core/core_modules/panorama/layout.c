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

// builds image from highest, lowest and average distribution values
void write_array_image(
      /* in     */ const char *name_root,
      /* in     */ const pan_color_grid_type *grid
      );
void write_array_image(
      /* in     */ const char *name_root,
      /* in     */ const pan_color_grid_type *grid
      )
{
   image_size_type size = grid->size;
   uint32_t n_pix = (uint32_t) (size.x * size.y);
   const int32_t mult = 256 / COLOR_GRID_DIST_SIZE;
   /////////////////////////////////////////////
   // open output file
   char buf[STR_LEN];
   for (uint32_t lev=0; lev<NUM_PYRAMID_LEVELS; lev++) {
      sprintf(buf, "%s.%d.pgm", name_root, lev);
      FILE *fp = fopen(buf, "w");
      if (fp == NULL) {
         fprintf(stderr, "Unable to open %s for writing\n", buf);
         continue;
      }
      fprintf(fp, "P5\n%d %d\n255\n", 2*size.x, 3*size.y);
      uint8_t pix_buf_low[n_pix * 2];
      uint8_t pix_buf_high[n_pix * 2];
      uint8_t pix_buf_avg[n_pix * 2];
      /////////////////////////////////////////////
      // write content
      for (uint32_t y=0; y<size.y; y++) {
         for (uint32_t x=0; x<size.x; x++) {
            uint32_t idx = (uint32_t) (x + y * size.x);
            pan_color_grid_unit_type *unit = &grid->grid[idx];
            // find low, high and avg of distribution
            int32_t low_v = -1;
            int32_t high_v = 0;
            float avg_v = 0.0f;
            int32_t low_y = -1;
            int32_t high_y = 0;
            float avg_y = 0.0f;
            for (uint32_t i=0; i<COLOR_GRID_DIST_SIZE; i++) {
               float vchan = unit->color_v[lev][i];
               if (vchan > 0.0f) {
                  if (low_v < 0) {     // is 1st pix?
                     low_v = (int32_t) i;
                     high_v = (int32_t) i;
                  } else {
                     high_v = (int32_t) i;
                  }
                  avg_v += (float) i * vchan;
               }
               float ychan = unit->color_y[lev][i];
               if (ychan > 0.0f) {
                  if (low_y < 0) {     // is 1st pix?
                     low_y = (int32_t) i;
                     high_y = (int32_t) i;
                  } else {
                     high_y = (int32_t) i;
                  }
                  avg_y += (float) i * ychan;
               }
            }
            pix_buf_low[idx] = (uint8_t) (low_v * mult);
            pix_buf_low[n_pix+idx] = (uint8_t) (low_y * mult);
            //
            pix_buf_high[idx] = (uint8_t) (high_v * mult);
            pix_buf_high[n_pix+idx] = (uint8_t) (high_y * mult);
            //
            avg_v /= unit->num_samples[lev];
            avg_y /= unit->num_samples[lev];
            pix_buf_avg[idx] = (uint8_t) (avg_v * (float) mult);
            pix_buf_avg[n_pix+idx] = (uint8_t) (avg_y * (float) mult);
         }
      }
      fwrite(pix_buf_low, 1, 2*n_pix, fp);
      fwrite(pix_buf_high, 1, 2*n_pix, fp);
      fwrite(pix_buf_avg, 1, 2*n_pix, fp);
      //
      fclose(fp);
   }
}


// initialize color grid, setting size values and allocating memory for
//    grid array
static void init_color_grid(
      /* in out */       pan_color_grid_type *grid
      )
{
   // calculate location of grid top and get number of rows
   float above_deg = WORLD_HEIGHT_ABOVE_HORIZ_DEGS + GRID_BELOW_HORIZON_OFFSET;
   float below_deg = WORLD_HEIGHT_BELOW_HORIZ_DEGS - GRID_BELOW_HORIZON_OFFSET;
   // find integeral number of grid rows above and below offset line
   const uint32_t num_rows_above = (uint32_t)
         (above_deg / COLOR_GRID_UNIT_HEIGHT_DEG + 0.9999f);
   const uint32_t num_rows_below = (uint32_t)
         (below_deg / COLOR_GRID_UNIT_HEIGHT_DEG + 0.9999f);
   const uint32_t num_rows = num_rows_above + num_rows_below;
   grid->size.rows = (uint16_t) num_rows;
   const uint32_t num_cols = NUM_COLOR_GRIDS_HORIZ;
   grid->size.cols = (uint16_t) num_cols;
   assert((uint32_t) ((float) num_cols * COLOR_GRID_UNIT_WIDTH_DEG) == 360);
   // get top offset (virtual top of grid). offset is the number of pixels
   //    between actual top of panoramic view and where top of first grid
   //    row lies
   float gap_deg = COLOR_GRID_UNIT_HEIGHT_DEG * (float) num_rows_above -
         above_deg;
   for (uint32_t lev=0; lev<NUM_PYRAMID_LEVELS; lev++) {
      grid->top_buffer[lev] = (uint32_t) (gap_deg * PIX_PER_DEG[lev]);
   }
   // allocate grid array
   uint32_t n_units = (uint32_t) (num_rows * num_cols);
   grid->grid = calloc(n_units, sizeof grid->grid[0]);
}


// update each color grid unit with color from pixels in area that it covers
static void push_to_color_grid(
      /* in out */       pan_color_grid_type *grid,
      /* in out */       frame_page_type *page
      )
{
   panorama_output_type *out = page->frame;
   static const uint32_t BIN_WIDTH = 256 / COLOR_GRID_DIST_SIZE;
printf("TAU: %.4f\n", 1.0 - COLOR_GRID_TAU);
   //
   for (uint32_t lev=0; lev<NUM_PYRAMID_LEVELS; lev++) {
      // make shorthand copies to access values
      image_size_type world_size = out->frame_size[lev];
      image_size_type grid_size = grid->size;
      uint32_t buffer = grid->top_buffer[lev];
      overlap_pixel_type *pixels = out->world_frame[lev];
      const float dpp = DEG_PER_PIX[lev];
      //////////////////////////////////////////
      // decay existing grid values before adding new
      uint32_t n_units = (uint32_t) (grid_size.x * grid_size.y);
      for (uint32_t i=0; i<n_units; i++) {
         pan_color_grid_unit_type *unit = &grid->grid[i];
         float *color_y = unit->color_y[lev];
         float *color_v = unit->color_v[lev];
         for (uint32_t j=0; j<COLOR_GRID_DIST_SIZE; j++) {
            color_y[j] *= 1.0f - (float) COLOR_GRID_TAU;
         }
         for (uint32_t j=0; j<COLOR_GRID_DIST_SIZE; j++) {
            color_v[j] *= 1.0f - (float) COLOR_GRID_TAU;
         }
      }
      //////////////////////////////////////////
      // loop through all panorama pixels
      for (uint32_t y=0; y<world_size.y; y++) {
         // get base indices for desired row
         uint32_t row_idx = y * world_size.cols;
         uint32_t grid_row = (uint32_t) (dpp * (float) (y + buffer) /
               COLOR_GRID_UNIT_HEIGHT_DEG);
         uint32_t grid_row_idx = grid_row * NUM_COLOR_GRIDS_HORIZ;
         ///////////////
         for (uint32_t x=0; x<world_size.x; x++) {
            uint32_t pix_idx = (uint32_t) (x + row_idx);
            overlap_pixel_type *pix = &pixels[pix_idx];
            if (pix->fg.border != 0) {
               // no content here
               continue;
            }
            // get indices for pixel and grid unit
            uint32_t grid_col = (uint32_t) ((float) x * dpp /
                  COLOR_GRID_UNIT_WIDTH_DEG);
            uint32_t grid_idx = (uint32_t) (grid_col + grid_row_idx);
            pan_color_grid_unit_type *unit = &grid->grid[grid_idx];
            float *color_y = unit->color_y[lev];
            float *color_v = unit->color_v[lev];
if (pix->fg.color.y == 0) {
   printf("Pixel %d,%d has y=%d  v=%d -- expected border to be set\n", x, y, pix->fg.color.y, pix->fg.color.v);
}
            // update distribution
            const uint32_t y_bin = (uint32_t) (pix->fg.color.y / BIN_WIDTH);
            color_y[y_bin] += 1.0f;
            const uint32_t v_bin = (uint32_t) (pix->fg.color.v / BIN_WIDTH);
            color_v[v_bin] += 1.0f;
if ((y == 100) && ((x & 7) == 0)) {
   printf("PIX %d,%d  idx %d -> GRID %d,%d idx %d  yv: %d,%d  %d,%d  %.1f,%.1f\n", x, y, pix_idx, grid_col, grid_row, grid_idx, pix->fg.color.y, pix->fg.color.v, y_bin, v_bin, color_y[y_bin], color_v[v_bin]);
}
         }
      }
      //////////////////////////////////////////
      // sum total content of distribution for each grid square
      for (uint32_t i=0; i<n_units; i++) {
         pan_color_grid_unit_type *unit = &grid->grid[i];
         float *color_y = unit->color_y[lev];
         float total = 0;
         for (uint32_t j=0; j<COLOR_GRID_DIST_SIZE; j++) {
            total += color_y[j];
if ((lev == 0) && (i == 1230)) {
   printf("%d Y    %2d    %.1f\n", i, j, (double) color_y[j]);
}
         }
if (lev == 0) {
   printf("Total: %f\n", (double) total);
}
         unit->num_samples[lev] = total;
      }
   }
}


// used when pulling color distribution data from master into out_unit
static void append_color_data(
      /* in     */ const pan_color_grid_unit_type *master_unit,
      /* in out */       pan_color_grid_unit_type *out_unit
      )
{
   for (uint32_t lev=0; lev<NUM_PYRAMID_LEVELS; lev++) {
      for (uint32_t i=0; i<COLOR_GRID_DIST_SIZE; i++) {
         out_unit->color_y[lev][i] += master_unit->color_y[lev][i];
         out_unit->color_v[lev][i] += master_unit->color_v[lev][i];
      }
      out_unit->num_samples[lev] += master_unit->num_samples[lev];
   }
}


static void build_output_color_dist(
      /* in     */ const panorama_type *pan,
      /* in out */       pan_color_grid_type *out_grid
      )
{
   const pan_color_grid_type *master = &pan->master_color_grid;
   image_size_type size = master->size;
   // copy values from pan's master color grid to output grid
   // for each grid unit, copy values from the surround of that unit
   //    in master
   for (uint32_t r=0; r<size.rows; r++) {
      uint32_t row_idx = (uint32_t) (r * size.cols);
      for (uint32_t c=0; c<size.cols; c++) {
         pan_color_grid_unit_type *unit = &out_grid->grid[row_idx+c];
         // get column index in this row for left and right neighbors
         uint32_t left_c = (uint32_t) (c - 1);
         if (left_c >= size.cols) {
            left_c = (uint32_t) (size.cols-1);
         }
         uint32_t right_c = (uint32_t) (c + 1);
         if (right_c >= size.cols) {
            right_c = 0;
         }
         // initialize output -- copy from unit to west
         memcpy(unit, &master->grid[row_idx+left_c], sizeof *unit);
         // copy data from east
         append_color_data(&master->grid[row_idx+right_c], unit);
         if (r > 0) {
            // copy from above row
            uint32_t top_row_idx = row_idx - size.cols;
            append_color_data(&master->grid[top_row_idx+left_c], unit);
            append_color_data(&master->grid[top_row_idx+c], unit);
            append_color_data(&master->grid[top_row_idx+right_c], unit);
         }
         if (r < (uint32_t) (size.rows-1)) {
            // copy from row below
            uint32_t bot_row_idx = row_idx + size.cols;
            append_color_data(&master->grid[bot_row_idx+left_c], unit);
            append_color_data(&master->grid[bot_row_idx+c], unit);
            append_color_data(&master->grid[bot_row_idx+right_c], unit);
         }
      }
   }
}

