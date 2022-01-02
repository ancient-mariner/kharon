#include "core_modules/vy_receiver.h"
#include <stdio.h>
#include "dev_info.h"

////////////////////////////////////////////////////////////////////////
// accumulator for resizing phantom

// accumulator element size is one pixel at lowest pyramid level
// accumulator size, in degrees (this represents max target size)
#define ACCUM_WIDTH_DEG    MAX_PHANTOM_ARC_WIDTH_DEG
// number of accumulator elements
#define ACCUM_WIDTH        ((uint32_t) ((ACCUM_WIDTH_DEG * PIX_PER_DEG[0])))
#define ACCUM_HEIGHT       ACCUM_WIDTH
#define N_ACCUM_ELEMENTS   (ACCUM_WIDTH * ACCUM_HEIGHT)

#define ACCUM_HALF_WIDTH   (ACCUM_WIDTH / 2)
#define ACCUM_HALF_HEIGHT  (ACCUM_HEIGHT / 2)

// the old 'sphere_coordinate' before it was deprecated for general use
// keep it around here because it's not worth the effort to switch to
//    BAMs
union latlon_coordinate {
   struct { double latitude, longitude; };
   struct { double lat, lon; };
   struct { double y_deg, x_deg; };
   uint64_t all;
};
typedef union latlon_coordinate latlon_coordinate_type;


struct accumulator_element {
   // weight is total opaqueness of input pixels. cnt is # of pixels
   // if weight == tot then pixel is fully opaque
   double weight, v, y;
   double max_weight;  // max possible weight
};
typedef struct accumulator_element accumulator_element_type;


// thread has one accumulator that's stored statically, plus
//    temporary storage
static __thread accumulator_element_type *accum_;
static __thread accumulator_element_type *tmp_accum_;

// initialize accumator to generate resampled image
// accumulator elements have same resolution as lowest pyramid level
static void accum_init(void)
{
   size_t sz = N_ACCUM_ELEMENTS * sizeof *accum_;
   if (accum_ == NULL) {
      accum_ = malloc(sz);
      tmp_accum_ = malloc(sz);
   }
   memset(accum_, 0, sz);
}

// add value from one accumulator bin to another, applying specified 
//    relative weight
static void accum_add_element(
      /* in     */ const accumulator_element_type *src,
      /* in     */ const double rel_wt,
      /* in out */       accumulator_element_type *element
      )
{
   element->y += rel_wt * src->y;
   element->v += rel_wt * src->v;
   element->weight += rel_wt * src->weight;
   element->max_weight += rel_wt * src->max_weight;
}

// add value from one pixel to accumulator bin
static void accum_add_pix(
      /* in     */ const pix_type *pix,
      /* in out */       accumulator_element_type *element
      )
{
   double weight = 255.0 - (double) pix->b;
   element->y += (double) pix->r * weight;
   element->v += (double) pix->g * weight;
   element->weight += weight;
   element->max_weight += 255.0;
}


// perform rad=1 Gaus blur on accumulator
static void accum_blur(void)
{
//#warning "blur disabled"
//return;
   // normalize all elements so max weight is 255
   // this seems the only practical way to blur and maintail alpha
   for (uint32_t i=0; i<N_ACCUM_ELEMENTS; i++) {
      accumulator_element_type *element = &accum_[i];
      if (element->max_weight > 0.0) {
         double scale = 255.0 / element->max_weight;
         element->y *= scale;
         element->v *= scale;
         element->weight *= scale;
         element->max_weight = 255.0;
      } else {
         element->max_weight = 255.0;
         assert(element->weight == 0.0);
      }
   }
   /////////////////////////////////////////////////////////////////////
   // horizontal blur
   for (uint32_t y=0; y<ACCUM_HEIGHT; y++) {
      uint32_t row_idx = (uint32_t) (y * ACCUM_WIDTH);
      // left col
      {
         uint32_t idx = row_idx;
         accum_add_element(&accum_[idx  ], 0.66, &tmp_accum_[idx]);
         accum_add_element(&accum_[idx+1], 0.33, &tmp_accum_[idx]);
      }
      // middle cols
      for (uint32_t x=1; x<ACCUM_WIDTH-1; x++) {
         uint32_t idx = (uint32_t) (x + row_idx);
         accum_add_element(&accum_[idx-1], 0.25, &tmp_accum_[idx]);
         accum_add_element(&accum_[idx  ], 0.5, &tmp_accum_[idx]);
         accum_add_element(&accum_[idx+1], 0.25, &tmp_accum_[idx]);
      }
      // right col
      {
         uint32_t idx = row_idx + ACCUM_WIDTH - 1;
         accum_add_element(&accum_[idx-1], 0.33, &tmp_accum_[idx]);
         accum_add_element(&accum_[idx  ], 0.66, &tmp_accum_[idx]);
      }
   }
   /////////////////////////////////////////////////////////////////////
   // vertical blur
   for (uint32_t x=0; x<ACCUM_WIDTH; x++) {
      // top row
      {
         uint32_t idx = x;
         accum_add_element(&tmp_accum_[idx], 0.66, &accum_[idx]);
         accum_add_element(&tmp_accum_[idx+ACCUM_WIDTH], 0.33, 
               &accum_[idx]);
      }
      // middle rows
      for (uint32_t y=1; y<ACCUM_HEIGHT-1; y++) {
         uint32_t idx = x + ACCUM_WIDTH * y;
         accum_add_element(&tmp_accum_[idx-ACCUM_WIDTH], 0.25, 
               &accum_[idx]);
         accum_add_element(&tmp_accum_[idx], 0.5, &accum_[idx]);
         accum_add_element(&tmp_accum_[idx+ACCUM_WIDTH], 0.25, 
               &accum_[idx]);
      }
      // bottom row
      {
         uint32_t idx = x + ACCUM_WIDTH * (ACCUM_HEIGHT - 1);
         accum_add_element(&tmp_accum_[idx-ACCUM_WIDTH], 0.33, 
               &accum_[idx]);
         accum_add_element(&tmp_accum_[idx], 0.66, &accum_[idx]);
      }
   }
}


// reduce resolution of accumulator so it aligns with next pyramid level
// content moved toward accum center and invalid pixels (fully transparent)
//    fill in the rest
static void accum_downsample(void)
{
   size_t sz = N_ACCUM_ELEMENTS * sizeof *accum_;
   accum_blur();
   // copy accum to tmp, clear accum, then selectively copy tmp back to center
   memcpy(tmp_accum_, accum_, sz);
   memset(accum_, 0, sz);
   for (uint32_t y=0; y<ACCUM_HALF_HEIGHT; y++) {
      // read from every-other row
      uint32_t read_row_idx = (uint32_t) (2*y * ACCUM_WIDTH);
      // write to quadrant centered in accum
      uint32_t write_row_idx = (uint32_t) (ACCUM_HALF_WIDTH/2 +
            (y + ACCUM_HALF_HEIGHT/2) * ACCUM_WIDTH );
      for (uint32_t x=0; x<ACCUM_HALF_WIDTH; x++) {
         // read from every other pixel
         uint32_t read_idx = (uint32_t) (read_row_idx + 2*x);
         // write to center of accum. row_idx already offset for this
         uint32_t write_idx = (uint32_t) (write_row_idx + x);
         accum_[write_idx] = tmp_accum_[read_idx];
      }
   }
}

// accumulator for resizing phantom
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// project phantom to panorama

// write phantom to panorama view
// for pyramid levels higher than base it's necessary that accum already
//    be downsampled
static void push_accumulator_to_panorama_level(
      /* in     */ const latlon_coordinate_type phantom_center,
      /* in out */       panorama_output_type *output,
      /* in     */ const uint32_t level
      )
{
   uint32_t img_left = (uint32_t) (-ACCUM_HALF_WIDTH + (uint32_t) 
         ((double) phantom_center.x_deg * PIX_PER_DEG[level]));
   if (img_left > WORLD_WIDTH_PIX[level]) {
      img_left += WORLD_WIDTH_PIX[level];
      assert(img_left < WORLD_WIDTH_PIX[level]);
   }
   uint32_t img_top = (uint32_t) (
         WORLD_HEIGHT_PIX[level]/2 - ACCUM_HALF_HEIGHT + (uint32_t) 
         ((double) -phantom_center.y_deg * PIX_PER_DEG[level]));
//printf("  img left=%d [right=%d]\n", img_left, img_left+ACCUM_WIDTH);
//printf("  img top=%d [bottom=%d]\n", img_top, img_top+ACCUM_HEIGHT);
   for (uint32_t y=0; y<ACCUM_HEIGHT; y++) {
      uint32_t img_row = (uint32_t) (img_top + y);
      if (img_row >= WORLD_HEIGHT_PIX[level]) {
         // because row is unsigned, it could mean that this row
         //    is off top or bottom of image. continuing loop is
         //    safest course of action
         continue;
      }
      uint32_t img_row_idx = (uint32_t) (img_row * WORLD_WIDTH_PIX[level]);
      uint32_t accum_row_idx = (uint32_t) (y * ACCUM_WIDTH);
      for (uint32_t x=0; x<ACCUM_WIDTH; x++) {
         // phantom data
         uint32_t accum_idx = (uint32_t) (accum_row_idx + x);
         accumulator_element_type *element = &accum_[accum_idx];
         if (element->weight == 0.0) {
//if (x == ACCUM_WIDTH/2) {
//   printf(" row %d    no foreground weight\n", y);
//}
            // no content in this element
            continue;
         }
         // image data
         uint32_t img_col = (uint32_t) (img_left + x);
         if (img_col >= WORLD_WIDTH_PIX[level]) {
            img_col -= WORLD_WIDTH_PIX[level];
         }
         uint32_t img_idx = img_col + img_row_idx;
         overlap_pixel_type *pix = &output->world_frame[level][img_idx];
         if (pix->fg.radius == 0xffff) {
//if (x == ACCUM_WIDTH/2) {
//   printf(" row %d    no foreground data\n", y);
//}
            // no foreground pixel here so nothing to write to
            continue;
         }
         // project phantom onto image
         double opaque = element->weight / element->max_weight;
         assert(opaque <= 1.0);
         double pix_y = element->y / element->weight;
         double pix_v = element->v / element->weight;
         double out_y = 
               pix_y * opaque + (double) pix->fg.color.y * (1.0 - opaque);
         double out_v = 
               pix_v * opaque + (double) pix->fg.color.v * (1.0 - opaque);
//if (x == ACCUM_WIDTH/2) {
//   printf(" row %d    opaque %.3f  y %.1f  v %.1f\n", y, (double) opaque, (double) out_y, (double) out_v);
//}
         assert(out_y <= 255.0);
         assert(out_v <= 255.0);

         pix->fg.color.y = (uint8_t) (out_y);
         pix->fg.color.v = (uint8_t) (out_v);
         // push to background too
         if (pix->bg.radius != 0xffff) {
            out_y = out_y * opaque + (double) pix->bg.color.y * (1.0 - opaque);
            out_v = out_v * opaque + (double) pix->bg.color.v * (1.0 - opaque);
            assert(out_y <= 255.0);
            assert(out_v <= 255.0);
            pix->bg.color.y = (uint8_t) (out_y);
            pix->bg.color.v = (uint8_t) (out_v);
         }
      }
   }
}


// write accumularot data to panorama view
// accumulator blurred and downsampled for higher pyramid levels
static void push_accumulator_to_panorama(
      /* in     */ const latlon_coordinate_type phantom_center,
      /* in out */       panorama_output_type *output
      )
{
   for (uint lev=0; lev<NUM_PYRAMID_LEVELS; lev++) {
      if (lev > 0) {
         // for each level above base, blur the accumulator
         accum_downsample();
      }
      push_accumulator_to_panorama_level(phantom_center, output, lev);
   }
}


// write phantom to accumulator
// phantom is written to center of accumulator
static void push_phantom_to_accumulator(
      /* in     */ const phantom_image_type *phantom,
      /* in     */ const pixel_offset_type phantom_offset
      )
{
   accum_init();
   const uint32_t phantom_height_pix = phantom->image->size.height;
   const uint32_t phantom_half_height_pix = phantom_height_pix / 2;
   const uint32_t phantom_width_pix = phantom->image->size.width;
   const uint32_t phantom_half_width_pix = phantom_width_pix / 2;
   // size scaling to convert from phantom pixels to accum elements
   // sanity check:
   //    phantom_pix = 1000
   //    phantom_deg = 2.5
   //    -> phantom ppd = 400
   //    acc ppd = 10
   // phantom pix / accum pix = 40
   // scale = 1/40
   //    2.5 / (1000 * 0.1) = 1/40
   const double scale = phantom->arc_width.degrees / 
         ((double) phantom_width_pix * DEG_PER_PIX[0]);
//printf("Scale: %.4f   phantom pix offset %d,%d\n", (double) scale, phantom_offset.dx, phantom_offset.dy);
//printf("Accum height=%d  width=%d  half height=%d  half_width=%d\n", ACCUM_HEIGHT, ACCUM_WIDTH, ACCUM_HALF_HEIGHT, ACCUM_HALF_WIDTH);
   // push each phantom pixel into the appropriate accumulator bin
   // NOTE to make this GPU-friendly, invert paradigm so each accum
   //    bin pulls from all input phantom pixels. that will allow 
   //    accumulation to happen w/o possible race conditions
   // a,b are coordinates in phantom space
   // x,y are coordiantes in accumulator space
   for (uint32_t b=0; b<phantom_height_pix; b++) {
      // phantom resolution should be much higher than accum res, so
      //    using an int (versus double) shouldn't create too much of an
      //    artifact
      int32_t y0 = (int32_t) (scale * (double) ((int32_t) b - 
            (int32_t) phantom_half_height_pix - phantom_offset.dy));
      if (y0 < -((int32_t) ACCUM_HALF_HEIGHT)) {
         continue;
      } else if (y0 >= (int32_t) ACCUM_HALF_HEIGHT) {
         break;
      }
      uint32_t y = (uint32_t) (y0 + (int32_t) ACCUM_HALF_HEIGHT);
      uint32_t accum_row_idx = (uint32_t) (y * ACCUM_HEIGHT);
      uint32_t phantom_row_idx = (uint32_t) (b * phantom_width_pix);
      for (uint32_t a=0; a<phantom_width_pix; a++) {
         int32_t x0 = (int32_t) (scale * (double) ((int32_t) a - 
               (int32_t) phantom_half_width_pix - phantom_offset.dx));
         if (x0 < -((int32_t) ACCUM_HALF_WIDTH)) {
            continue;
         } else if (x0 >= (int32_t) ACCUM_HALF_WIDTH) {
            break;
         }
         uint32_t x = (uint32_t) (x0 + (int32_t) ACCUM_HALF_WIDTH);
         uint32_t accum_idx = (uint32_t) (accum_row_idx + x);
         uint32_t phantom_idx = (uint32_t) (phantom_row_idx + a);
         pix_type *pix = &phantom->image->rgb[phantom_idx];
//if (a == phantom_width_pix/2) {
//   printf("  phantom row %d  rgb %d,%d,%d\n", a, pix->r, pix->g, pix->b);
//}
         accum_add_pix(pix, &accum_[accum_idx]);
      }
   }
//for (uint32_t i=0; i<ACCUM_HEIGHT; i++) {
//   uint32_t idx = (uint32_t) (ACCUM_WIDTH/2 + i * ACCUM_WIDTH);
//   accumulator_element_type *element = &accum_[idx];
//   printf(" accum row %d  v=%.1f  y=%.1f  wt=%.1f  tot=%.1f\n", i, (double) element->v, (double) element->y, (double) element->weight, (double) element->max_weight);
//}
}


// calculates and returns offset of phantom pixel center from
//    center of pixel in world space. this offset is what generates
//    sub-pixel-resolution changes in how the phantom is observed
// offset is how far right/down phantom center is
static pixel_offset_type get_phantom_offset(
      /* in     */ const latlon_coordinate_type phantom_center,
      /* in     */ const phantom_image_type *phantom
      )
{
   // find target of projection in world space, in fractions of a pixel
   // x is positive, y is negative (if below the horizon)
   double horiz_pix_target = phantom_center.x_deg * PIX_PER_DEG[0];
   double vert_pix_target = phantom_center.y_deg * PIX_PER_DEG[0];
//printf(" . target pix: %.2f,%.2f\n", (double) horiz_pix_target, (double) vert_pix_target);
   // fraction of a pixel that phantom is offset from world pix, in
   //    world pix
   double horiz_pix_offset = horiz_pix_target - 
         (double) ((int) horiz_pix_target);
   double vert_pix_offset = vert_pix_target - 
         (double) ((int) vert_pix_target);
//printf(" . pix offset: %.2f,%.2f\n", (double) horiz_pix_offset, (double) vert_pix_offset);
   // degree offset of phantom center from pixel center, in degrees
   double horiz_deg_offset = horiz_pix_offset * DEG_PER_PIX[0];
   double vert_deg_offset = vert_pix_offset * DEG_PER_PIX[0];
//printf(" . deg offset: %.4f,%.4f\n", (double) horiz_deg_offset, (double) vert_deg_offset);
   // offset in phantom pixels
   double phantom_ppd = 
         (double) phantom->image->size.x / phantom->arc_width.degrees;
   double phantom_horiz_pix_offset = horiz_deg_offset * phantom_ppd;
   double phantom_vert_pix_offset = vert_deg_offset * phantom_ppd;
//printf(" . phantom pix offset: %.2f,%.2f\n", (double) phantom_horiz_pix_offset, (double) phantom_vert_pix_offset);
   //
   pixel_offset_type offset = { 
      .dx = (int16_t) phantom_horiz_pix_offset,
      .dy = (int16_t) phantom_vert_pix_offset,
      .dt = 0
   };
   return offset;
}


// project phantom image onto panorama view
static void project_phantom(
      /* in out */       panorama_output_type *output,
      /* in     */ const phantom_image_type *phantom,
      /* in     */ const latlon_coordinate_type phantom_center
      )
{
   if (phantom->arc_width.degrees >= MAX_PHANTOM_ARC_WIDTH_DEG) {
      fprintf(stderr, "Requested phantom size of %.2f degrees is greater "
            "than maximum size of %.2f\n", (double) phantom->arc_width.degrees,
            (double) MAX_PHANTOM_ARC_WIDTH_DEG);
      hard_exit(__func__, __LINE__);
   }
//printf("Projecting phantom image at lat=%.1f, lon=%.1f\n", (double) phantom_center.lat, (double) phantom_center.lon);
   // get offset of phantom center, in phantom pixels, from center of
   //    world pixel
   pixel_offset_type phantom_offset = get_phantom_offset(phantom_center,
         phantom);
   push_phantom_to_accumulator(phantom, phantom_offset);
//   // blur accumulator same amount that input images are blurred
//   for (uint32_t i=0; i<VY_BLUR_RADIUS; i++) {
//      accum_blur();
//   }
   push_accumulator_to_panorama(phantom_center, output);
}

// project phantom to panorama
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// load and manage phantom(s)

static void convert_bearing_to_xy(
      /* in out */       phantom_image_type *phantom,
      /* in     */ const degree_type bearing,
      /* in     */ const meter_type range
      )
{
   double c, s;
   double theta = bearing.degrees * D2R;
   sincos(theta, &s, &c);
   phantom->pos_x.meters = range.meters * s;
   phantom->pos_y.meters = range.meters * c;
}


static void convert_track_to_dxdy(
      /* in out */       phantom_image_type *phantom
      )
{
   double c, s;
   double theta = phantom->track.course.degrees * D2R;
   sincos(theta, &s, &c);
   phantom->dx.mps = phantom->track.speed.meters_per_second * s;
   phantom->dy.mps = phantom->track.speed.meters_per_second * c;
}


// updates phantom position and returns its relative visual position
// visual position is the center of the image, while calculation of 
//    placement is based on image bottom (ie, phantom's wedge)
static latlon_coordinate_type update_phantom_size_and_position(
      /* in out */       panorama_class_type *pan,
      /* in out */       phantom_image_type *phantom,
      /* in     */ const double t
      )
{
   double dt_sec = (double) (t - phantom->last_update_sec);
   phantom->last_update_sec = t;
//printf("Phantom xy %.2f,%.2f   dx,dy %.2f,%.2f\n", (double) phantom->pos_x.meters, (double) phantom->pos_y.meters, (double) phantom->dx.meters_per_second, (double) phantom->dy.meters_per_second);
   phantom->pos_x.meters += phantom->dx.meters_per_second * dt_sec;
   phantom->pos_y.meters += phantom->dy.meters_per_second * dt_sec;
   // phantom position is updated. now calculate where it is in visual field
   // TODO when tracking logic is converted to spherical logic (from
   //    planar) convert here as well
   latlon_coordinate_type pos;
   double range_meters = sqrt(
         phantom->pos_x.meters * phantom->pos_x.meters + 
         phantom->pos_y.meters * phantom->pos_y.meters);
   if (range_meters < 5.0) {
      // impact. there's no meaningful data to display
      goto impact;
   }
   //
   phantom->arc_width.degrees = R2D * 
         atan(phantom->length.meters / range_meters);
   //
   pos.x_deg = R2D * atan2(phantom->pos_x.meters, phantom->pos_y.meters);
   if (pos.x_deg < 0.0) {
      pos.x_deg += 360.0;
   }
   // y position represents where center of phantom will be drawn. this 
   //    needs to be adjusted for phantom height, so phantom bottom (wedge)
   //    is at desired range
   pos.y_deg = -R2D * atan(pan->camera_height.meters / range_meters);
   double height_degrees = phantom->arc_width.degrees * 
         ((double) ACCUM_HEIGHT / (double) ACCUM_WIDTH);
   pos.y_deg += height_degrees / 2.0;
   /////////////////////////////////////////////
   if (pos.y_deg < -20.0) {
      goto impact;
   }
//printf("Arc: %.1f, range: %.1f\n", (double) phantom->arc_width.degrees, (double) range_meters);
   return pos;
impact:  
   // too close to display. consider it an impact. relocate phantom
   //    to 500m to north and set on horizon
   phantom->pos_x.meters = 0.0;
   phantom->pos_y.meters = 500.0;
   phantom->arc_width.degrees = R2D * 
         atan(phantom->length.meters / phantom->pos_y.meters);
   pos.x_deg = 0.0;
   pos.y_deg = 0.0;
   return pos;
}


// create phantom record as defined in config file
static void project_phantom_images(
      /* in out */       panorama_class_type *pan,
      /* in out */       panorama_output_type *output,
      /* in     */ const double t
      )
{
#if ENABLE_PHANTOM_IMAGES == 0
   return;
#endif   // ENABLE_PHANTOM_IMAGES
   // initialize timers if this is the first pass
   if (pan->phantom_init_flag == 0) {
      pan->phantom_init_flag = 1;
      for (uint32_t i=0; i<pan->num_phantoms; i++) {
         phantom_image_type *phantom = pan->phantoms[i];
         for (uint32_t j=0; j<phantom->num_tracks; j++) {
            phantom->track_start[j] += t;
         }
         phantom->last_update_sec = t;
      }
   }
   /////////////////////////////////////////////////////////////
   // project each phantom
   for (uint32_t i=0; i<pan->num_phantoms; i++) {
      phantom_image_type *phantom = pan->phantoms[i];
      //////////////////////////////////////////
      // see if it's time to advance to the next track
      uint32_t active_track = phantom->active_track;
      if (active_track < (phantom->num_tracks-1)) {
         if (t > phantom->track_start[active_track]) {
            // time is after next track start time -- advance to next track
            phantom->track = phantom->track_queue[++active_track];
            phantom->active_track = active_track;
            convert_track_to_dxdy(phantom);
         }
      }
      //////////////////////////////////////////
      // based on track, update relative size and position
      latlon_coordinate_type phantom_center =
            update_phantom_size_and_position(pan, phantom, t);
      project_phantom(output, phantom, phantom_center);
   }
}


// create phantom record as defined in config file
static void load_phantom_config(
      /* in out */       panorama_class_type *pan,
      /* in     */ const char *config_name
      )
{
   // create new phantom
   if (pan->num_phantoms >= MAX_PHANTOMS) {
      fprintf(stderr, "Too many phantoms declared for panorama. Max is %d\n",
            MAX_PHANTOMS);
      hard_exit(__func__, __LINE__);
   }
   phantom_image_type *phantom = malloc(sizeof *phantom);
   pan->phantoms[pan->num_phantoms++] = phantom;
   /////////////////////////////////////////////
   // init phantom from config
   FILE *fp = fopen(config_name, "r");
   if (!fp) {
      fprintf(stderr, "Unable to open phantom config '%s'\n", config_name);
      hard_exit(__func__, __LINE__);
   }
   char buf[STR_LEN];
   uint32_t num_tracks = 0;
   uint32_t num_frames = 0;
   // put phantom in invalid state
   phantom->length.meters = -1.0f;
   // read config
   while (get_next_line(fp, buf, STR_LEN) != NULL) {
      char *tok = strtok(buf, " \t");
      if (strcmp(tok, "init") == 0) {
         ///////////////////////////////////////////////////////////////
         // init data
         double val[3];
         for (uint32_t i=0; i<3; i++) {
            tok = strtok(NULL, " \t");
            if (tok == NULL) {
               fprintf(stderr, "Error reading 'init' value %d in "
                     "phantom config '%s'\n", i, config_name);
               goto err;
            }
            errno = 0;
            val[i] = strtof(tok, NULL);
            if (errno != 0) {
               perror("Error parsing 'init' value");
               fprintf(stderr, "Error parsing term %d ('%s') "
                     "from phantom config '%s'\n", i, tok, config_name);
               goto err;
            }
         }
         meter_type range = { .meters = val[0] };
         degree_type bearing = { .degrees = val[1] };
         convert_bearing_to_xy(phantom, bearing, range);
         phantom->length.meters = val[2];
      } else if (strcmp(tok, "set") == 0) {
         ///////////////////////////////////////////////////////////////
         // course set
         if (num_tracks >= MAX_PHANTOM_COURSE_CHANGES) {
            fprintf(stderr, "Too many course changes specified in phantom "
                  "config '%s'\n", config_name);
            goto err;
         }
         double val[3];
         for (uint32_t i=0; i<3; i++) {
            tok = strtok(NULL, " \t");
            if (tok == NULL) {
               fprintf(stderr, "Error reading 'set' value %d in "
                     "phantom config '%s'\n", i, config_name);
               goto err;
            }
            errno = 0;
            val[i] = strtof(tok, NULL);
            if (errno != 0) {
               perror("Error parsing 'set' value");
               fprintf(stderr, "Error parsing term %d ('%s') "
                     "from phantom config '%s'\n", i, tok, config_name);
               goto err;
            }
         }
         ground_track_type track;
         phantom->track_start[num_tracks] = val[0];
         track.course.degrees = val[1];
         track.speed.mps = val[2];
         phantom->track_queue[num_tracks] = track;
         num_tracks++;
      } else if (strcmp(tok, "frame") == 0) {
         ///////////////////////////////////////////////////////////////
         // image frame
         if (num_frames >= MAX_PHANTOM_FRAMES) {
            fprintf(stderr, "Too many frames specified in phantom "
                  "config '%s'\n", config_name);
            goto err;
         }
         tok = strtok(NULL, " \t");
         image_type *img = create_image(tok);
         if (img == NULL) {
            fprintf(stderr, "Failed to open image file '%s' while processing "
                  "phantom config '%s'\n", tok, config_name);
            goto err;
         }
         phantom->frames[num_frames] = img;
         num_frames++;
      } else {
         fprintf(stderr, "Parse error in phantom config '%s'" 
               " -- don't recognize token '%s'\n", config_name, tok);
         goto err;
      }
   }
   fclose(fp);
   fp = NULL;
   // complete setup of initial state
   phantom->num_tracks = num_tracks;
   phantom->active_track = 0;
   phantom->track = phantom->track_queue[0];
   convert_track_to_dxdy(phantom);
   phantom->num_frames = num_frames;
   phantom->active_frame = 0;
   phantom->image = phantom->frames[0];
   // make sure enough data was read
   if (phantom->length.meters < 0.0) {
      fprintf(stderr, "'init' field missing from phantom config '%s'\n", 
            config_name);
      goto err;
   }
   if (num_tracks == 0) {
      fprintf(stderr, "'set' field(s) missing from phantom config '%s'\n", 
            config_name);
      goto err;
   }
   if (num_frames == 0) {
      fprintf(stderr, "frame field(s) missing from phantom config '%s'\n", 
            config_name);
      goto err;
   }
   return;
   /////////////////////////////////////////////
err:
   if (fp) {
      fclose(fp);
   }
   hard_exit(__func__, __LINE__);
}

void insert_phantom_image(
      /* in out */       datap_desc_type *pan_dp,
      /* in     */ const char *config_name
      )
{
   // sanity check
   if (pan_dp == NULL) {
      fprintf(stderr, "NULL source provided to insert_phantom_image\n");
      hard_exit(__func__, __LINE__);
   } 
   /////////////////////////////////////////////////////////////////////
   panorama_class_type *pan = (panorama_class_type*) pan_dp->local;
   if (strcmp(pan_dp->td->class_name, PANORAMA_CLASS_NAME) != 0) {
      fprintf(stderr, "Phantom images must be set on panorama module, "
            "not %s\n", pan_dp->td->class_name);
      hard_exit(__func__, __LINE__);
   }
   load_phantom_config(pan, config_name);
}


