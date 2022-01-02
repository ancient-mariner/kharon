#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>

#include "timekeeper.h"

#define ENABLE_PHANTOM_IMAGES    1
#include "tracking/panorama.h"

////////////////////////////////////////////////////////////////////////
// the following boilerplate is to import panorama.c

#define hard_exit(a,b)  { fprintf(stderr, "%s : %d", a, b); exit(1); }

////////////////////////////////////////////////
// create dummy functions to resolve link errors
void dp_wait(datap_desc_type *foo) { ; }
void * dp_get_object_at(const datap_desc_type *foo, const uint32_t bar) { return NULL; };
struct datap_desc *dp_create() { return NULL; }
void dp_execute(datap_desc_type *foo) { ; }
void dp_signal_data_available(datap_desc_type *foo) { ; }
image_size_type upright_get_image_size(uint32_t level);
image_size_type upright_get_image_size(uint32_t level) { image_size_type sz = { .x=256, .y=256 }; return sz; }
////////////////////////////////////////////////

#include "../panorama.c"

// for initialization, use panorma class code
datap_desc_type *dp_ = NULL;
thread_desc_type *td_ = NULL;
panorama_class_type *panorama_ = NULL;

////////////////////////////////////////////////////////////////////////

static void create_panorama(void)
{
   if (dp_ == NULL) {
      dp_ = calloc(1, sizeof *dp_);
      td_ = calloc(1, sizeof *td_);
      panorama_ = calloc(1, sizeof *panorama_);
      dp_->td = td_;
      td_->dp = dp_;
      dp_->local = panorama_;
      strcpy(td_->class_name, PANORAMA_CLASS_NAME);
      strcpy(td_->obj_name, "Test panorma");
      // initialize panorama
      panorama_->camera_height.meters = 4.0f;
      panorama_->camera_forward_position.meters = 0.0f;
      panorama_->camera_starboard_position.meters = 0.0f;
      panorama_->output_type = 0;
      panorama_->data_folder = malloc(256);
      sprintf(panorama_->data_folder, "image_output");
      // pretend there's a producer to get around sanity check (but
      //    don't try to access producer!)
      dp_->num_attached_producers = 1;
      panorama_pre_run(dp_);
   }
}

static void init_panorama_output(
      /*    out */       panorama_output_type *out
      )
{
   // put coverage everywhere, with red sky, green water, and alternating
   //    bands with bg coverage
   memset(out->coverage.radial, 1, 
         NUM_PAN_COVERAGE_RADIALS * sizeof *out->coverage.radial);
   pixel_cam_info_type sky_fg = {
      .radius=1, .border=0, .cam_num=1, .color.y=180, .color.v=160 
   };
   pixel_cam_info_type sky_bg = {
      .radius=1, .border=0, .cam_num=2, .color.y=170, .color.v=150
   };
   pixel_cam_info_type sea_fg = {
      .radius=1, .border=0, .cam_num=1, .color.y=110, .color.v=110
   };
   pixel_cam_info_type sea_bg = {
      .radius=1, .border=0, .cam_num=2, .color.y=100, .color.v=86 
   };
   pixel_cam_info_type fg, bg;
   for (uint32_t lev=0; lev<NUM_PYRAMID_LEVELS; lev++) {
      for (uint32_t y=0; y<WORLD_HEIGHT_PIX[lev]; y++) {
         uint32_t row_idx = (uint32_t) (y * WORLD_WIDTH_PIX[lev]);
         if (y == 0) {
            fg = sky_fg;
            bg = sky_bg;
         }
         if (y == WORLD_HEIGHT_PIX[lev]/2) {
            fg = sea_fg;
            bg = sea_bg;
         }
         for (uint32_t x=0; x<WORLD_WIDTH_PIX[lev]; x++) {
            uint32_t idx = (uint32_t) (row_idx + x);
            overlap_pixel_type *pix = &out->world_frame[lev][idx];
            pix->fg = fg;
            pix->bg = bg;
         }
      }
   }
}

////////////////////////////////////////////////////////////////////////

static uint32_t test_init(void)
{
   printf("test init\n");
   uint32_t errs = 0;
   // if panorama initializes w/o crash, test passes
   create_panorama();
   init_panorama_output((panorama_output_type*) dp_->void_queue);
   insert_phantom_image(dp_, "phantom_config_1.txt");
   if (panorama_->num_phantoms != 1) {
      fprintf(stderr, "Unexpected number of phantoms. Expected 1, got %d\n",
            panorama_->num_phantoms);
      errs++;
   }
   /////////////////////////////////////////////////////////////
   if (errs == 0) {
      printf("    passed\n");
   } else {
      printf("    %d errors\n", errs);
   }
   return errs;
}


// write excerpt of pan as text. content is meant to be loaded and 
//    compared in subsequent unit test
void save_test_data(
      /* in     */ const char *outfile,
      /* in     */ const uint32_t left,
      /* in     */ const uint32_t right,
      /* in     */ const uint32_t top,
      /* in     */ const uint32_t bottom,
      /* in     */ const panorama_output_type *out
      );
void save_test_data(
      /* in     */ const char *outfile,
      /* in     */ const uint32_t left,
      /* in     */ const uint32_t right,
      /* in     */ const uint32_t top,
      /* in     */ const uint32_t bottom,
      /* in     */ const panorama_output_type *out
      )
{
   FILE *fp = fopen(outfile, "w");
   if (!fp) {
      fprintf(stderr, "Unable to save test data to '%s'\n", outfile);
      hard_exit(__func__, __LINE__);
   }
   for (uint32_t y=top; y<=bottom; y++) {
      for (uint32_t x=left; x<=right; x++) {
         uint32_t idx = (uint32_t) (x + y * WORLD_WIDTH_PIX[0]);
         overlap_pixel_type *pix = &out->world_frame[0][idx];
         uint32_t fgy = pix->fg.color.y;
         uint32_t fgv = pix->fg.color.v;
         uint32_t bgy = pix->bg.color.y;
         uint32_t bgv = pix->bg.color.v;
         fprintf(fp, "%d\t%d\t%d\t%d\t%d\t%d\n", x, y, fgy, fgv, bgy, bgv);
      }
   }
   fclose(fp);
}

static uint32_t compare_test_data(
      /* in     */ const char *infile,
      /* in     */ const uint32_t left,
      /* in     */ const uint32_t right,
      /* in     */ const uint32_t top,
      /* in     */ const uint32_t bottom,
      /* in     */ const panorama_output_type *out
      )
{
   char line[STR_LEN];
   FILE *fp = fopen(infile, "r");
   uint32_t errs = 0;
   if (!fp) {
      fprintf(stderr, "Unable to save test data to '%s'\n", infile);
      errs++;
      goto end;
   }
   for (uint32_t y=top; y<=bottom; y++) {
      for (uint32_t x=left; x<=right; x++) {
         uint32_t idx = (uint32_t) (x + y * WORLD_WIDTH_PIX[0]);
         overlap_pixel_type *pix = &out->world_frame[0][idx];
         uint32_t fgy = pix->fg.color.y;
         uint32_t fgv = pix->fg.color.v;
         uint32_t bgy = pix->bg.color.y;
         uint32_t bgv = pix->bg.color.v;
         // 
         if (get_next_line(fp, line, STR_LEN) == NULL) {
            fprintf(stderr, "Data stream from '%s' ended prematurely\n",
                  infile);
            errs++;
            goto end;
         }
         uint32_t vals[6];
         char * tok = strtok(line, " \t");
         for (uint32_t i=0; i<6; i++) {
            if (tok == NULL) {
               fprintf(stderr, "Parse error at line for %d,%d in '%s'\n",
                     x, y, infile);
               errs++;
               goto end;
            }
            vals[i] = (uint32_t) atoi(tok);
            tok = strtok(NULL, " \t");
         }
         if ((x != vals[0]) || (y != vals[1]) ||
               (fgy != vals[2]) || (fgv != vals[3]) ||
               (bgy != vals[4]) || (bgv != vals[5])) {
            fprintf(stderr, "Unexpected value for location %d,%d (%d,%d) "
                  "in '%s'\n", x, y, vals[0], vals[1], infile);
            fprintf(stderr, "  Read %d,%d,%d,%d (fg-y, fg-v, bg-y, bg-v), "
                  "expected %d,%d,%d,%d\n", 
                  vals[2], vals[3], vals[4], vals[5], fgy, fgv, bgy, bgv);
            errs++;
         }
      }
   }
end:
   if (fp) {
      fclose(fp);
   }
   return errs;
}

static uint32_t test_project_phantom(void)
{
   printf("test project_phantom\n");
   uint32_t errs = 0;
   /////////////////////////////////////////////////////////////
   // project image to panorama
   // note that projections are done manually and this bypasses pos update
   phantom_image_type *phantom = panorama_->phantoms[0];
   phantom->arc_width.degrees = 2.0f;
   //panorama_->phantoms[0]->arc_width.degrees = 2.0f;
   panorama_output_type *out = (panorama_output_type*) dp_->void_queue;
   for (uint32_t i=0; i<11; i++) {
      float offset_x = (float) i * 2.51f;
      float offset_y = (float) i * 0.01f;
      world_coordinate_type pos = 
            { .x_deg=1.5f+offset_x, .y_deg=-1.5f+offset_y };
      {
         pixel_offset_type phantom_offset = get_phantom_offset(pos, phantom);
         push_phantom_to_accumulator(phantom, phantom_offset);
         push_accumulator_to_panorama(pos, out);
      }
      pos.y_deg = -pos.y_deg;
      {
         pixel_offset_type phantom_offset = get_phantom_offset(pos, phantom);
         push_phantom_to_accumulator(phantom, phantom_offset);
         push_accumulator_to_panorama(pos, out);
      }
      // project w/ blur
      pos.y_deg = -4.5f + offset_y;
      project_phantom(out, phantom, pos);
   }
   const char testfile[] = "data_project_phantom.txt";
   write_panorama_image(panorama_, out, 1.0, 0);
   write_panorama_image(panorama_, out, 1.0, 1);
   //save_test_data(testfile, 0, 280, 330, 420, out);
   errs += compare_test_data(testfile, 0, 280, 330, 420, out);
   /////////////////////////////////////////////////////////////
   if (errs == 0) {
      printf("    passed\n");
   } else {
      printf("    %d errors\n", errs);
   }
   return errs;
}

static uint32_t test_project_phantom_images(void)
{
   printf("test project_phantom_images\n");
   uint32_t errs = 0;
   /////////////////////////////////////////////////////////////
   // insert 2nd phantom. project all, using position estimatation
   insert_phantom_image(dp_, "phantom_config_2.txt");
   if (panorama_->num_phantoms != 2) {
      fprintf(stderr, "Unexpected number of phantoms. Expected 1, got %d\n",
            panorama_->num_phantoms);
      errs++;
   }
   panorama_output_type *out = (panorama_output_type*) dp_->void_queue;
   init_panorama_output(out);
   for (uint32_t i=0; i<13; i++) {
      double t = 2.0 + (double) i;
      project_phantom_images(panorama_, out, t);
      write_panorama_image(panorama_, out, t, 0);
      write_panorama_image(panorama_, out, t, 1);
   }
   const char testfile[] = "data_project_phantom_images.txt";
   //save_test_data(testfile, 0, 325, 350, 400, out);
   errs += compare_test_data(testfile, 0, 325, 350, 400, out);
   /////////////////////////////////////////////////////////////
   if (errs == 0) {
      printf("    passed\n");
   } else {
      printf("    %d errors\n", errs);
   }
   return errs;
}

//static uint32_t test_assign_pixel_to_section(void)
//{
//   uint32_t errs = 0;
//   printf("testing assign_pixel_to_section\n");
//   /////////////////////////////////////////////////////////////
//   pixel_summary_type a, b, c, d;
//   a.edge_desc.salience = 100;
//   a.edge_desc.flags = PIX_FLAG_LOCAL_PEAK;
//   b.edge_desc.salience = 110;
//   b.edge_desc.flags = PIX_FLAG_LOCAL_PEAK;
//   c.edge_desc.salience = 120;
//   c.edge_desc.flags = PIX_FLAG_LOCAL_PEAK;
//   d.edge_desc.salience = 130;
//   d.edge_desc.flags = PIX_FLAG_LOCAL_PEAK;
//   target_section_type sect;
//   sect.num_pixels = 0;
//   uint32_t expected_num;
//   ////////////
//   // add b. make sure there's one pixel
//   assign_pixel_to_section(&b, &sect, 0);
//   expected_num = 1;
//   if (sect.num_pixels != expected_num) {
//      fprintf(stderr, "Unexpected number of pixels. Found %d, got %d\n",
//            sect.num_pixels, expected_num);
//      errs++;
//   }
//   // add c. make sure there are two pixels, and c is in slot 0
//   assign_pixel_to_section(&c, &sect, 0);
//   expected_num = 2;
//   if (sect.num_pixels != expected_num) {
//      fprintf(stderr, "Unexpected number of pixels. Found %d, got %d\n",
//            sect.num_pixels, expected_num);
//      errs++;
//   }
//   if (sect.pixel_kps[0] != &c) {
//      fprintf(stderr, "Expected c as most salient pix (1). Found pix w/ "
//            "salience of %d\n", sect.pixel_kps[0]->edge_desc.salience);
//      errs++;
//   }
//   // add a. make sure there are three pixels, and c is in slot 0
//   assign_pixel_to_section(&a, &sect, 0);
//   expected_num = 3;
//   if (sect.num_pixels != expected_num) {
//      fprintf(stderr, "Unexpected number of pixels. Found %d, got %d\n",
//            sect.num_pixels, expected_num);
//      errs++;
//   }
//   if (sect.pixel_kps[0] != &c) {
//      fprintf(stderr, "Expected c as most salient pix (2). Found pix w/ "
//            "salience of %d\n", sect.pixel_kps[0]->edge_desc.salience);
//      errs++;
//   }
//   // add d. make sure there are four pixels, and d is in slot 0
//   assign_pixel_to_section(&d, &sect, 0);
//   expected_num = 4;
//   if (sect.num_pixels != expected_num) {
//      fprintf(stderr, "Unexpected number of pixels. Found %d, got %d\n",
//            sect.num_pixels, expected_num);
//      errs++;
//   }
//   if (sect.pixel_kps[0] != &d) {
//      fprintf(stderr, "Expected d as most salient pix. Found pix w/ "
//            "salience of %d\n", sect.pixel_kps[0]->edge_desc.salience);
//      errs++;
//   }
//   /////////////////////////////////////////////////////////////
//   if (errs == 0) {
//      printf("    passed\n");
//   } else {
//      printf("    %d errors\n", errs);
//   }
//   return errs;
//}

int main(int argc, char** argv)
{
   uint32_t errs = 0;
   set_world_height(25.0f, 25.0f);
   set_ppd(10.0f);
   errs += test_init();
   errs += test_project_phantom();
   errs += test_project_phantom_images();
   //////////////////
   printf("\n");
   if (errs == 0) {
      printf("--------------------\n");
      printf("--  Tests passed  --\n");
      printf("--------------------\n");
   } else {
      printf("**********************************\n");
      printf("**** ONE OR MORE TESTS FAILED ****\n");
      printf("**********************************\n");
      fprintf(stderr, "%s failed\n", argv[0]);
   }
   return (int) errs;
}


