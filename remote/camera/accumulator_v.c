////////////////////////////////////////////////////////////////////////
// color channel
#define CAM_WIDTH_PIX_V       820
#define CAM_WIDTH_PIX_V_YUV   832
#define CAM_HEIGHT_PIX_V      616

// accumulator
static accumulator_element_type * accum_v_ = NULL;

// map of image pixels to accumulator elements
static accumulator_coord_type * accum_map_v_ = NULL;


// update accumulator element with color value
static void update_element_v(
      /* in     */ const uint8_t v,
      /* in     */ const uint8_t w,
      /* in out */       accumulator_element_type *element
      )
{
   element->val = element->val + (uint32_t) (w * v);
   element->w = (uint16_t) (element->w + w);
}

// push top or bottom row of image to accumulator, setting border flag 
//    for nw,ne,sw,se elements
static void push_v_border_row(
      /* in     */ const uint8_t *buf,
      /* in     */ const uint32_t row_num
      )
{
   // get pointer to this data row
   uint32_t idx = row_num * CAM_WIDTH_PIX_V_YUV;
   accumulator_coord_type *map_row =  &accum_map_v_[idx];
   const uint8_t *row = &buf[idx];
   // push pixels of this row to the accumulator
   for (uint32_t i=0; i<CAM_WIDTH_PIX_V; i++) {
      // get pixel value and its accumulator mapping
      const uint8_t val = row[i];
      const accumulator_coord_type *coord = &map_row[i];
      // get accumulator elements
      const uint32_t top_idx = (uint32_t) (coord->x + coord->y * ACCUM_WIDTH);
      accumulator_element_type *top = &accum_v_[top_idx];
      accumulator_element_type *bot = &accum_v_[top_idx + ACCUM_WIDTH];
      // set border and push values, left edge
      top->border = 255;
      update_element_v(val, coord->nw, top);
      bot->border = 255;
      update_element_v(val, coord->sw, bot);
      // set border and push values, right edge
      top++;
      bot++;
      top->border = 255;
      update_element_v(val, coord->ne, top);
      bot->border = 255;
      update_element_v(val, coord->se, bot);
   }
}

// push non-border row of image to accumulator, setting border flag 
//    for nw,ne,sw,se elements
static void push_v_row(
      /* in     */ const uint8_t *buf,
      /* in     */ const uint32_t row_num
      )
{
   // get pointer to this data row
   uint32_t idx = row_num * CAM_WIDTH_PIX_V_YUV;
   accumulator_coord_type *coord =  &accum_map_v_[idx];
   const uint8_t *row = &buf[idx];
   // push pixel value for first col, setting border flag
   {
      // get pixel value and its accumulator mapping
      const uint8_t val = *row;
      const uint32_t top_idx = (uint32_t) (coord->x + coord->y * ACCUM_WIDTH);
      accumulator_element_type *top = &accum_v_[top_idx];
      accumulator_element_type *bot = &accum_v_[top_idx + ACCUM_WIDTH];
      // set border and push values, left edge
      top->border = 255;
      update_element_v(val, coord->nw, top);
      bot->border = 255;
      update_element_v(val, coord->sw, bot);
      // set border and push values, right edge
      top++;
      bot++;
      top->border = 255;
      update_element_v(val, coord->ne, top);
      bot->border = 255;
      update_element_v(val, coord->se, bot);
      // advance to next pixel
      coord++;
      row++;
   }
   // push pixels of this row to the accumulator
   for (uint32_t i=1; i<CAM_WIDTH_PIX_V-1; i++) {
      // get pixel value and its accumulator mapping
      const uint8_t val = *row;
      const uint32_t top_idx = (uint32_t) (coord->x + coord->y * ACCUM_WIDTH);
      accumulator_element_type *top = &accum_v_[top_idx];
      accumulator_element_type *bot = &accum_v_[top_idx + ACCUM_WIDTH];
      // push values, left edge
      update_element_v(val, coord->nw, top);
      update_element_v(val, coord->sw, bot);
      // push values, right edge
      update_element_v(val, coord->ne, &top[1]);
      update_element_v(val, coord->se, &bot[1]);
      // advance to next pixel
      coord++;
      row++;
   }
   // push pixel value for first col, setting border flag
   {
      // get pixel value and its accumulator mapping
      const uint8_t val = *row;
      const uint32_t top_idx = (uint32_t) (coord->x + coord->y * ACCUM_WIDTH);
      accumulator_element_type *top = &accum_v_[top_idx];
      accumulator_element_type *bot = &accum_v_[top_idx + ACCUM_WIDTH];
      // set border and push values, left edge
      top->border = 255;
      update_element_v(val, coord->nw, top);
      bot->border = 255;
      update_element_v(val, coord->sw, bot);
      // set border and push values, right edge
      top++;
      bot++;
      top->border = 255;
      update_element_v(val, coord->ne, top);
      bot->border = 255;
      update_element_v(val, coord->se, bot);
   }
}

// push color image to accumulator
static void push_image_v_to_accumulator(
      /* in     */ const uint8_t *buf
      )
{
   push_v_border_row(buf, 0);
   for (uint32_t row=1; row<CAM_HEIGHT_PIX_V-1; row++) {
      push_v_row(buf, row);
   }
   push_v_border_row(buf, CAM_HEIGHT_PIX_V-1);
}

// color channel
////////////////////////////////////////////////////////////////////////
