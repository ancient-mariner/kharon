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
#define CAM_WIDTH_PIX_Y      1640
#define CAM_WIDTH_PIX_Y_YUV  1664
#define CAM_HEIGHT_PIX_Y     1232

// accumulator
static accumulator_element_type * accum_y_ = NULL;

// map of image pixels to accumulator elements
static accumulator_coord_type * accum_map_y_ = NULL;


// update accumulator element with intensity value
static void update_element_y(
      /* in     */ const uint8_t y,
      /* in     */ const uint8_t w,
      /* in out */       accumulator_element_type *element
      )
{
   element->val = element->val + (uint32_t) (w * y);
   element->w = (uint16_t) (element->w + w);
}

// push top or bottom row of image to accumulator, setting border flag
//    for nw,ne,sw,se elements
static void push_y_border_row(
      /* in     */ const uint8_t *buf,
      /* in     */ const uint32_t row_num
      )
{
   // get pointer to this data row
   uint32_t idx = row_num * CAM_WIDTH_PIX_Y_YUV;
   accumulator_coord_type *map_row =  &accum_map_y_[idx];
   const uint8_t *row = &buf[idx];
   // push pixels of this row to the accumulator
   for (uint32_t i=0; i<CAM_WIDTH_PIX_Y; i++) {
      // get pixel value and its accumulator mapping
      const uint8_t val = row[i];
      const accumulator_coord_type *coord = &map_row[i];
      // get accumulator elements
      const uint32_t top_idx = (uint32_t) (coord->x + coord->y * ACCUM_WIDTH);
      accumulator_element_type *top = &accum_y_[top_idx];
      accumulator_element_type *bot = &accum_y_[top_idx + ACCUM_WIDTH];
      // set border and push values, left edge
      top->border = 255;
      update_element_y(val, coord->nw, top);
      bot->border = 255;
      update_element_y(val, coord->sw, bot);
      // set border and push values, right edge
      top++;
      bot++;
      top->border = 255;
      update_element_y(val, coord->ne, top);
      bot->border = 255;
      update_element_y(val, coord->se, bot);
   }
}

// push non-border row of image to accumulator, setting border flag
//    for nw,ne,sw,se elements
static void push_y_row(
      /* in     */ const uint8_t *buf,
      /* in     */ const uint32_t row_num
      )
{
   // get pointer to this data row
   uint32_t idx = row_num * CAM_WIDTH_PIX_Y_YUV;
   accumulator_coord_type *coord =  &accum_map_y_[idx];
   const uint8_t *row = &buf[idx];
   // push pixel value for first col, setting border flag
   {
      // get pixel value and its accumulator mapping
      const uint8_t val = *row;
      const uint32_t top_idx = (uint32_t) (coord->x + coord->y * ACCUM_WIDTH);
      accumulator_element_type *top = &accum_y_[top_idx];
      accumulator_element_type *bot = &accum_y_[top_idx + ACCUM_WIDTH];
      // set border and push values, left edge
      top->border = 255;
      update_element_y(val, coord->nw, top);
      bot->border = 255;
      update_element_y(val, coord->sw, bot);
      // set border and push values, right edge
      top++;
      bot++;
      top->border = 255;
      update_element_y(val, coord->ne, top);
      bot->border = 255;
      update_element_y(val, coord->se, bot);
      // advance to next pixel
      coord++;
      row++;
   }
   // push pixels of this row to the accumulator
   for (uint32_t i=1; i<CAM_WIDTH_PIX_Y-1; i++) {
      // get pixel value and its accumulator mapping
      const uint8_t val = *row;
      const uint32_t top_idx = (uint32_t) (coord->x + coord->y * ACCUM_WIDTH);
      accumulator_element_type *top = &accum_y_[top_idx];
      accumulator_element_type *bot = &accum_y_[top_idx + ACCUM_WIDTH];
      // push values, left edge
      update_element_y(val, coord->nw, top);
      update_element_y(val, coord->sw, bot);
      // push values, right edge
      update_element_y(val, coord->ne, &top[1]);
      update_element_y(val, coord->se, &bot[1]);
      // advance to next pixel
      coord++;
      row++;
   }
   // push pixel value for first col, setting border flag
   {
      // get pixel value and its accumulator mapping
      const uint8_t val = *row;
      const uint32_t top_idx = (uint32_t) (coord->x + coord->y * ACCUM_WIDTH);
      accumulator_element_type *top = &accum_y_[top_idx];
      accumulator_element_type *bot = &accum_y_[top_idx + ACCUM_WIDTH];
      // set border and push values, left edge
      top->border = 255;
      update_element_y(val, coord->nw, top);
      bot->border = 255;
      update_element_y(val, coord->sw, bot);
      // set border and push values, right edge
      top++;
      bot++;
      top->border = 255;
      update_element_y(val, coord->ne, top);
      bot->border = 255;
      update_element_y(val, coord->se, bot);
   }
}

// push color image to accumulator
static void push_image_y_to_accumulator(
      /* in     */ const uint8_t *buf
      )
{
   push_y_border_row(buf, 0);
   for (uint32_t row=1; row<CAM_HEIGHT_PIX_Y-1; row++) {
      push_y_row(buf, row);
   }
   push_y_border_row(buf, CAM_HEIGHT_PIX_Y-1);
}


//
//// push intensity image to accumulator
//static void push_image_y_to_accumulator(
//      /* in     */ const uint8_t *buf
//      )
//{
//   for (uint32_t r=0; r<CAM_HEIGHT_PIX_Y; r++) {
//      if ((r == 0) || (r == CAM_HEIGHT_PIX_Y-1)) {
//         // signal border for this row
//         uint32_t idx_acc = r * ACCUM_WIDTH;
//         for (uint32_t c=0; c<CAM_WIDTH_PIX_Y; c++) {
//            // NW(SW) is immediately before NE(SE)
//            // SW is one row past NW
//            accumulator_element_type *acc =  &accum_y_[idx_acc];
//            acc->border = 255;
//            acc++;
//            acc->border = 255;
//            acc =  &accum_y_[idx_acc+ACCUM_WIDTH];
//            acc->border = 255;
//            acc++;
//            acc->border = 255;
//            idx_acc++;
//         }
//      } else {
//         // set border flag. we only need to do this for one row
//         //    as the second row will be handled on the next pass
//         uint32_t idx_acc = r * ACCUM_WIDTH;
//         accumulator_element_type *acc =  &accum_y_[idx_acc];
//         acc->border = 255;
//         acc++;
//         acc->border = 255;
//      }
//      // base indices for read position of y pixel in image, and
//      //    northwest accumulator element for that pixel
//      uint32_t idx_img = r * CAM_WIDTH_PIX_Y_YUV;
//      uint32_t idx_acc = r * ACCUM_WIDTH;
//      // set border for first pixel in row
//      for (uint32_t c=0; c<CAM_WIDTH_PIX_Y; c++) {
//         // get intensity value (y) for this pixel and its
//         //    accumulator mapping
//         const uint8_t intensity = buf[idx_img];
//         accumulator_coord_type *coord = &accum_map_y_[idx_img];
//         idx_img++;
//         //
//         // push intensity value to cornering accumulators
//         // NW(SW) is immediately before NE(SE)
//         // SW is one row past NW
//         accumulator_element_type *acc =  &accum_y_[idx_acc];
//         update_element_y(intensity, coord->nw, acc++);
//         update_element_y(intensity, coord->ne, acc);
//         acc =  &accum_y_[idx_acc+ACCUM_WIDTH];
//         update_element_y(intensity, coord->sw, acc++);
//         update_element_y(intensity, coord->se, acc);
//         idx_acc++;
//      }
//      {
//         // set border flag for last pixel in row
//         // index already set. back it up one as it was incremented
//         //    at end of previous loop
//         idx_acc--;
//         // set border flag. we only need to do this for one row
//         //    as the second row will be handled on the next pass
//         accumulator_element_type *acc =  &accum_y_[idx_acc];
//         acc->border = 255;
//         acc++;
//         acc->border = 255;
//      }
//   }
//}

// intensity channel
////////////////////////////////////////////////////////////////////////
