
static void clear_world_buffer(
      /* in out */       panorama_output_type *output
      )
{
   const pixel_cam_info_type empty_pix = {
      .color = { .v=128, .y=0 },
      .radius = 0xffff, // set to max value as default
      .cam_num = 255,   // cam num=255 is invalid
      .border = 255,    // 255 is invalid pixel (anything > 0 actually)
   };
   const overlap_pixel_type empty = { .fg=empty_pix, .bg=empty_pix };
   overlap_pixel_type *elements = output->pyramid_;
   // buffer is 1.5x the size of world at lowest pyramid level
   //    as all pyrs can be stored there
   // format:
   //    11111111...112222...2333...34...4
   uint32_t n_pix = (uint32_t) 
         (3 * WORLD_HEIGHT_PIX[0] * WORLD_WIDTH_PIX[0] / 2);
   for (uint32_t i=0; i<n_pix; i++) {
      elements[i] = empty;
   }
}

// saves 'color' panorama. foreground pix in red. bg pixel in green or
//    blue depending on cam num
static void write_panorama_image(
      /* in     */ const panorama_class_type *pan,
      /* in     */ const panorama_output_type *output,
      /* in     */ const double t,
      /* in     */ const uint32_t level
      )
{
   const image_size_type sz = {
         .width = (uint16_t) WORLD_WIDTH_PIX[level],
         .height = (uint16_t) WORLD_HEIGHT_PIX[level]
   };
   uint32_t N_BYTES = 3u * sz.cols;
   uint8_t line[N_BYTES];   // 1 each for r, g, b
   char path[STR_LEN];
   snprintf(path, STR_LEN, "%s/%.3f_%d.pnm", pan->data_folder, t, level);
   FILE *fp = fopen(path, "w");
//printf("Saving pan file %s\n", path);
   if (!fp) {
      log_err(pan->log, "Unable to create output file '%s'", path);
      goto end;
   }
   log_info(pan->log, "Writing %s", path);
   fprintf(fp, "P6\n%d %d\n255\n", sz.cols, sz.rows);
   overlap_pixel_type *elements = output->world_frame[level];
   for (uint32_t y=0; y<sz.rows; y++) {
      memset(line, 0, N_BYTES);
      for (uint32_t x=0; x<N_BYTES; x+=3) {
         if (elements->fg.radius != 0xffff) {
            line[x] = elements->fg.color.y;
            if (pan->output_type == 0) {
               line[x+1] = elements->fg.color.y;
               line[x+2] = elements->fg.color.y;
            } else {
               if (elements->bg.radius != 0xffff) {
                  if (elements->fg.cam_num > elements->bg.cam_num) {
                     line[x+1] = elements->bg.color.y;
                  } else if (elements->bg.cam_num < 255) {
                     line[x+2] = elements->bg.color.y;
                  }
               }
            }
         }
         elements++;
      }
      fwrite(line, N_BYTES, 1, fp);
   }
end:
   if (fp)
      fclose(fp);
}


// flag written-to areas in panorama coverage record
static void mark_coverage(
      /* in     */ const optical_up_output_type *frame,
      /* in out */       panorama_output_type *output
      )
{
   ///////////////////////////////////////////////////////////////////
   // flag based on image width w/o rotation (eg, 62.2 deg for pi cam)
   // start flags at first even degree boundary after 1.5 degrees inset,
   //    and mark until last even boundary before 1.5 degree inset on right
   const double half_width_deg = 0.5 * VY_FOV_HORIZ_DEG;
   double lon_deg = (double) frame->world_center.lon.angle32 * BAM32_TO_DEG;
   int32_t left_coverage = (int32_t) (lon_deg - half_width_deg + 1.5);
   int32_t right_coverage = (int32_t) (lon_deg + half_width_deg - 1.5);
   int32_t idx = left_coverage+1;
   for (int32_t i=idx; i<right_coverage; i++) {
      // left and right may extend below 0 or above 360 so adjust as
      //    necessary
      if (idx < 0) {
         idx += 360;
      } 
      output->coverage.radial[idx++] = 1;
      if (idx >= 360) {
         idx -= 360;
      }
   }
}


// project individual image to panorama and handle resolving overlap
//    between images
static void project_frame_to_panorama(
      /* in     */ const optical_up_output_type *frame,
      /* in     */ const image_size_type in_sz,
      /* in out */       panorama_output_type *output,
      /* in     */ const image_size_type out_sz,
      /* in     */ const uint32_t level
      )
{
   //////////////////
   // source and sink
   const pixel_cam_info_type *pixels = frame->frame[level];
   overlap_pixel_type *world = output->world_frame[level];
   ////////////////////////////////////
   // point of projection in world view
   const double ppd = PIX_PER_DEG[level];
   // image center, in pixel space of world
   // image center aligns with bow of ship (positive Z axis)
   double lon_deg = (double) frame->world_center.lon.angle32 * BAM32_TO_DEG;
   int32_t center_x = (int32_t) (ppd * lon_deg);
   // cover for negative longitude, which is adjusted to positive, and
   //    for round-off errors
   if (center_x < 0)
      center_x += out_sz.width;
   else if (center_x >= out_sz.width)
      center_x -= out_sz.width;
//printf("World center %f,%f\n", (double) frame->world_center.longitude, (double) frame->world_center.latitude);
   double lat_deg = (double) frame->world_center.lat.sangle32 * BAM32_TO_DEG;
   const int32_t center_y = (int32_t) (ppd * 
         (WORLD_HEIGHT_ABOVE_HORIZ_DEGS + lat_deg));
//   const int32_t center_y = out_sz.rows/2 +
//         (int32_t) (ppd * frame->world_center.latitude);
   // top-left of image
   uint32_t origin_x = (uint32_t) (center_x - in_sz.cols/2);
   // unsigned, so negative is a very large value
   if (origin_x >= WORLD_WIDTH_PIX[level]) {
      origin_x += WORLD_WIDTH_PIX[level];
   }
   uint32_t origin_y = (uint32_t) (center_y - in_sz.rows/2);
//printf(" Pan-%d projection center %d,%d\n", level, center_x, center_y);
   /////////
   mark_coverage(frame, output);
   ///////////////////////////////////////////////////////////////////
   // loop over src frame pixels. push them their appropriate location
   //    in the world view
   uint32_t in_r, in_c;
   uint32_t out_r, out_c;
   for (in_r=0, out_r=origin_y; in_r<in_sz.rows; in_r++, out_r++) {
      // nothing to copy if destination row is outside of image
      if (out_r >= out_sz.rows)
         continue;   // out_r is unsigned so this check covers <0 also
      // precompute things needed in inner loop
      const uint32_t in_row_offset = in_r * in_sz.cols;
      const uint32_t out_row_offset = out_r * out_sz.cols;
      //////////////////////////////////////////////////////////////////
      for (in_c=0, out_c=origin_x; in_c<in_sz.cols; in_c++, out_c++) {
         // wrap around at 360 degrees
         if (out_c >= out_sz.cols)
            out_c = 0;
         // input
         uint32_t in_idx = in_row_offset + in_c;
         pixel_cam_info_type src = pixels[in_idx];
         ///////////////////////////////////////////////////////////////
         // 
         const uint32_t out_idx = out_row_offset + out_c;
         overlap_pixel_type *sink = &world[out_idx];
         // if pixel has content, push to panorama view. if radius is
         //    less than existing pixel, or there is no existing
         //    pixel (which is same comparison as no existing has
         //    radius=0xffff) then push existing to background and
         //    use new pixel. lower radius means pixel is closer to
         //    image center than other frame, and foreground pixels
         //    are those that are closest to their parent image centers
         if (src.radius != 0xffff) {
            // valid pixel. see if belongs in fg or bg
            // foreground pixel
            if (src.radius < sink->fg.radius) {
               // img pix has lower radius than world view. save its
               //    value in fg and move existing fg to bg
               sink->bg = sink->fg;
               sink->fg = src;
            } else {
               // background pix
               sink->bg = src;
            }
         }
      }
   }
}

//
//void get_camera_position(
//      /* in     */ const panorama_class_type *pan,
//      /*    out */       meter_type *height,
//      /*    out */       meter_type *forward_position,
//      /*    out */       meter_type *starboard_position
//      )
//{
//   height->meters = pan->camera_height.meters;
//   forward_position->meters = pan->camera_forward_position.meters;
//   starboard_position->meters = pan->camera_starboard_position.meters;
//}

