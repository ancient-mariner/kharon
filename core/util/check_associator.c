#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include "image.h"
#include "dev_info.h"

// examines content of log directory. places targets and associator
//    output on panorama view
// requires panorama data to be logged and routing to be active

// this should be executed from a script that prepares the environment

#define  PAN_DIR           "panorama/"
#define  TRACKER_LOG0      "chk_tracker-0.txt"
#define  TRACKER_LOG1      "chk_tracker-1.txt"
#define  ASSOCIATOR_LOG    "chk_associator.txt"
static const char * OUTPUT_DIR = "/tmp/ass_check/";

static const char * log_dir_ = NULL;

static image_type *pan_in_ = NULL;

enum {
   STREAM_TRACKER_0,
   STREAM_TRACKER_1,
   STREAM_ASSOCIATOR,
   NUM_STREAMS
};
static image_type *pan_out_[3] = { NULL };
static uint32_t pan_state_[3] = { 0 };    // 1 if 'dirty'

#define PAN_DEGS_ABOVE_HORIZON   15.0

static double ppd_ = 1.0;

/**
preprocess t0, t1, ass
for t0,t1, list targets posted for each panorama time
for ass, list targets posted for each publish time

round 1, overlay targets on pan. separately, overlay records on most recent pan

log format
   %.3f           frame time (for tgt, matches pan file name)
      %d          number of targets level 0
         <tgt n>


**/

#define LABEL_LEN 16

#define NUM_OUTPUT_CHANS   3

// stores state for data being read from target log file
struct target_stream {
   FILE *fp;
   // time of next data block; <0 when stream exhausted
   double t;
   // number of targets in block
   uint32_t num_targets;
   // number of targets read
   uint32_t num_read;
   // 
   uint32_t output_chan;
   // name
   char label[LABEL_LEN];
};
typedef struct target_stream target_stream_type;

static target_stream_type tracker0_stream_;
static target_stream_type tracker1_stream_;
static target_stream_type ass_stream_;


struct target_desc {
   uint32_t target_id;
   uint32_t instance_num;
   double left;
   double right;
   double top;
   double bottom;
   double dx_dps;
   double dy_dps;
   union {
      uint8_t rgba[4];
      uint32_t rgba_all;
   };
};
typedef struct target_desc target_desc_type;

////////////////////////////////////////////////////////////////////////
// read log data

// reads target ID and instance num from '%d-%d '
static char * read_target_id(
      /* in out */       char * head,
      /*    out */       target_desc_type *target
      )
{
   char *buf = head;
   char c = buf[0];
   // strip leading whitespace
   while ((c == ' ') || (c == '\t')) {
      buf++;
      c = *buf;
   }
   int32_t val = 0;
   while (((c >= '0') && (c <= '9')) || 
         (c == ',') || (c == '-') || (c == ' ')) {
      if (c == '-') {
         target->target_id = (uint32_t) val;
         val = 0;
      } else if (c == ' ') {
         target->instance_num = (uint32_t) val;
         break;
      } else {
         val *= 10;
         val += c - '0';
      }
      buf++;
      c = *buf;
   }
   if (c == 0) {
      return NULL;
   } else {
      return buf;
   }
}


// pulls '%d,%d,%d ' from stream and returns pointer to following char
static char * read_colors(
      /* in out */       char * head,
      /*    out */       target_desc_type *target
      )
{
   target->rgba_all = 0;
   char *buf = head;
   char c = buf[0];
   // strip leading whitespace
   while ((c == ' ') || (c == '\t')) {
      buf++;
      c = *buf;
   }
   int32_t val = 0;
   int32_t field = 0;
   while (((c >= '0') && (c <= '9')) || (c == ',') || (c == ' ')) {
      if (c == ',') {
         target->rgba[field] = (uint8_t) val;
         field++;
         val = 0;
      } else if (c == ' ') {
         target->rgba[field] = (uint8_t) val;
         break;
      } else {
         val *= 10;
         val += c - '0';
      }
      buf++;
      c = *buf;
   }
//printf("colors %d,%d,%d -- '%s'\n", target->rgba[0], target->rgba[1], target->rgba[2],  buf);
   if (c == 0) {
      return NULL;
   } else {
      return buf;
   }
}


static target_desc_type * read_next_target(
      /* in out */       target_stream_type *stream,
      /*    out */       target_desc_type *target
      )
{
   if (stream->num_targets == stream->num_read) {
      return NULL;
   }
   char buf[256];
   if (get_next_line(stream->fp, buf, 256) == NULL) {
      goto depleted;
   }
   // parse target data
   // r,g,b num-instance L R T B dx dy [extra]
   //    %d,%d,%d %d-%d %f %f %f %f %f %f ...
   char *head = buf;
   // color target ID
   head = read_colors(head, target);
   head = read_target_id(head, target);
   errno = 0;
   // left, right
   target->left = strtod(head, &head);
   if (errno != 0) {
      goto error;
   }
   target->right = strtod(head, &head);
   if (errno != 0) {
      goto error;
   }
   // top, bottom
   target->top = strtod(head, &head);
   if (errno != 0) {
      goto error;
   }
   target->bottom = strtod(head, &head);
   if (errno != 0) {
      goto error;
   }
   // top, bottom
   target->dx_dps = strtod(head, &head);
   if (errno != 0) {
      goto error;
   }
   target->dy_dps = strtod(head, &head);
   if (errno != 0) {
      goto error;
   }
   /////////////////////////////////////////////
   return target;
error:
   fprintf(stderr, "Stream %s, error parsing '%s' in '%s': %s\n",
         stream->label, head, buf, strerror(errno));
   return NULL;
depleted:
   // stream depleted
//   fprintf(stderr, "Stream %s depleted\n", stream->label);
   fclose(stream->fp);
   stream->fp = NULL;
   stream->num_targets = 0;
   stream->num_read = 0;
   stream->t = -1.0;
   return NULL;
}


// load 
static void load_next_set_head(
      /* in out */       target_stream_type *stream
      )
{
   // require that all targets have been fully processed (in future advance
   //    to next timestamp, but not now)
   assert(stream->num_targets == stream->num_read);
   stream->num_targets = 0;
   stream->num_read = 0;
   char buf[256];
   if (get_next_line(stream->fp, buf, 256) == NULL) {
      goto depleted;
   }
   errno = 0;
   char *head = buf;
   stream->t = strtod(head, &head);
   if (errno != 0) {
      fprintf(stderr, "Unable to parse timestamp '%s' in %s: %s\n", buf, 
            stream->label, strerror(errno));
      goto depleted;
   }
   stream->num_targets = (uint32_t) strtol(head, &head, 10);
   if (errno != 0) {
      fprintf(stderr, "Unable to parse num targets '%s' in %s: %s\n", buf, 
            stream->label, strerror(errno));
      goto depleted;
   }
   return;
   /////////////////////////////////////////////
depleted:
   // stream depleted
//   fprintf(stderr, "Stream %s depleted\n", stream->label);
   fclose(stream->fp);
   stream->fp = NULL;
   stream->t = -1.0;
}


// open log file and load first entry
// returns -1 on error
static int32_t open_target_log(
      /* in     */ const char *name,
      /* in out */       target_stream_type *stream,
      /* in     */ const char *label,
      /* in     */ const uint32_t output_chan
      )
{
   int32_t rc = -1;
   memset(stream, 0, sizeof *stream);
   strncpy(stream->label, label, LABEL_LEN-1);
   stream->fp = fopen(name, "r");
   assert(output_chan < NUM_OUTPUT_CHANS);
   stream->output_chan = output_chan;
   if (stream->fp == NULL) {
      fprintf(stderr, "Unable to open '%s': %s\n", name, strerror(errno));
      goto end;
   }
   // get next timestamp and number of targets
   load_next_set_head(stream);
   rc = 0;
end:
   return rc;
}


//static void print_target(
//      /* in     */ const target_desc_type *target,
//      /* in     */ const char * label
//      )
//{
//   printf("%s %d-%d  %.1f-%.1f  %.1f-%.1f\n", label,
//         target->target_id, target->instance_num,
//         target->left, target->right, target->top, target->bottom);
//}


static void draw_box(
      /* in out */       image_type *pan_out,
      /* in     */ const target_desc_type *target
      )
{
   uint32_t left = (uint32_t) (target->left * ppd_ + 0.5);
   uint32_t right = (uint32_t) (target->right * ppd_ + 0.5);
   if (right < left) {
      // unwrap if target is split by pole
      right += pan_in_->size.width;
   }
   uint32_t top = (uint32_t) 
         ((PAN_DEGS_ABOVE_HORIZON - target->top) * ppd_ + 0.5);
   uint32_t bottom = (uint32_t) 
         ((PAN_DEGS_ABOVE_HORIZON - target->bottom) * ppd_ + 0.5);
//printf("LR %.1f %.1f  (%d %d) TB %.1f %.1f (%d %d)\n", target->left, target->right, left, right, target->top, target->bottom, top, bottom);
   uint8_t r = target->rgba[0];
   uint8_t g = target->rgba[1];
   uint8_t b = target->rgba[2];
   // draw top and bottom of box
   for (uint32_t i=0; i<2; i++) {
      uint32_t row_idx;
      if (i == 0) {
         row_idx = (uint32_t) (top * pan_out->size.width);
      } else {
         row_idx = (uint32_t) (bottom * pan_out->size.width);
      }
      for (uint32_t x=left; x<=right; x++) {
         uint32_t xi = x;
         if (xi >= pan_out->size.width) {
            xi = xi - pan_out->size.width;
         }
         uint32_t idx = (uint32_t) (row_idx + xi);
         pan_out->rgb[idx].r = r;
         pan_out->rgb[idx].g = g;
         pan_out->rgb[idx].b = b;
      }
   }
   // sides of box
   for (uint32_t y=top+1; y<=bottom-1; y++) {
      uint32_t idx = (uint32_t) (y * pan_out->size.width + left);
      pan_out->rgb[idx].r = r;
      pan_out->rgb[idx].g = g;
      pan_out->rgb[idx].b = b;
      uint32_t xi = right;
      if (xi >= pan_out->size.width) {
         xi = xi - pan_out->size.width;
      }
      idx = (uint32_t) (y * pan_out->size.width + xi);
      pan_out->rgb[idx].r = r;
      pan_out->rgb[idx].g = g;
      pan_out->rgb[idx].b = b;
   }
}


static void draw_target(
      /* in     */ const target_desc_type *target,
      /* in     */ const target_stream_type *stream
      )
{
   uint32_t chan = stream->output_chan;
   assert(chan >= 0);
   assert(chan <= 2);
   pan_state_[chan] = 1;
   draw_box(pan_out_[chan], target);
}


// read stream data
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// utils


// utils
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// setup

static int32_t open_log_files(void)
{
   int32_t rc = -1;
   char buf[256];
   // tracker logs
   sprintf(buf, "%s/%s", log_dir_, TRACKER_LOG0);
   if (open_target_log(buf, &tracker0_stream_, "tracker-0", 0) != 0) {
      goto end;
   }
   sprintf(buf, "%s/%s", log_dir_, TRACKER_LOG1);
   if (open_target_log(buf, &tracker1_stream_, "tracker-1", 1) != 0) {
      goto end;
   }
   // associator log
   sprintf(buf, "%s/%s", log_dir_, ASSOCIATOR_LOG);
   if (open_target_log(buf, &ass_stream_, "associator", 2) != 0) {
      goto end;
   }
   rc = 0;
end:
   return rc;
}


static void close_log_files(void)
{
   if (tracker0_stream_.fp != NULL) {
      fclose(tracker0_stream_.fp);
   }
   if (tracker1_stream_.fp != NULL) {
      fclose(tracker1_stream_.fp);
   }
   if (ass_stream_.fp != NULL) {
      fclose(ass_stream_.fp);
   }
}


void usage(char **argv)
{
   printf("Tool to visualize target and associator data.\n");
   printf("Writes output to /tmp/ass_check/\n");
   printf("\n");
   printf("Usage: %s <log directory>\n", argv[0]);
   printf("\n");
   exit(1);
}

// creates a directory under ACQUISITION_STORAGE_ROOT and returns the
//    full name of the new directory. the returned string is static
//    and will change if this procedure is called again
static int32_t create_output_folder(void)
{
   int rc = -1;
   // check for output folder
   struct stat st = {0};
   if (stat(OUTPUT_DIR, &st) == 0) {
      // output folder exists. clean it out
      printf("Output directory exists. Make sure it's empty\n");
   } else {
      if (mkdir(OUTPUT_DIR, 0777) != 0) {
         perror("mkdir error for logging directory");
         goto end;
      }
   }
   //////////////////////
   rc = 0;
end:
   return rc;
}

// setup
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// main logic


// returns pointer to stream with earliest timestamp
static target_stream_type * get_earliest_stream(void)
{
   target_stream_type *stream = &tracker0_stream_;
   double t = stream->t;
   if ((t < 0.0) || (tracker1_stream_.t < t)) {
      stream = &tracker1_stream_;
      t = stream->t;
   }
   if ((t < 0.0) || (ass_stream_.t < t)) {
      stream = &ass_stream_;
      t = stream->t;
   }
   if (t < 0.0) {
      return NULL;
   }
   return stream;
}

// use pan to write empty content in output image
static void reset_outputs(void)
{
   image_size_type size = pan_in_->size;
   for (uint32_t i=0; i<NUM_STREAMS; i++) {
      assert(pan_out_[i]->size.width = size.width);
      assert(pan_out_[i]->size.height = size.height);
      uint32_t n_pix = (uint32_t) (size.x * size.y);
      memcpy(pan_out_[i]->rgb, pan_in_->rgb, n_pix * sizeof *pan_in_->rgb);
      pan_state_[i] = 0;
   }
}


// load panorama file for time t into pan_in_
// if this pan doesn't exist, pan_in_ is not changed
static void load_panorama(
      /* in     */ const double t
      )
{
   char fname[256];
   sprintf(fname, "%s/%s%.3f_0.pnm", log_dir_, PAN_DIR, t);
//printf("Looking for %s\n", fname);
   // see if pan file exists
   struct stat st = { 0 };
   if (stat(fname, &st) == 0) {
printf("  Loading %s\n", fname);
      // it does. load it
      // free old pan if it exists
      if (pan_in_ != NULL) {
         free_image(pan_in_);
      }
      pan_in_ = create_image(fname);
      assert(pan_in_ != NULL);
      ppd_ = (double) pan_in_->size.width / 360.0;
   }
   /////////////////////////////
   // if this is the first pass, create output images  
   if (pan_out_[0] == NULL) {
      for (uint32_t i=0; i<NUM_STREAMS; i++) {
         pan_out_[i] = raw_create_image(pan_in_->size);
      }
      reset_outputs();
   }
}


//
//// create output image
//static void init_output_image(void)
//{
//   // output images are pans repeated 3 times vertically
//   image_size_type size = { .width = 3600, .height = 1500 };
//   for (uint32_t i=0; i<NUM_STREAMS; i++) {
//      pan_out_[i] = raw_create_image(size);
//      pan_state_[i] = 1;
//   }
//   reset_outputs();
//}
//

static void save_outputs(
      /* in     */ const double t
      )
{
   char fname[256];
   for (uint32_t i=0; i<NUM_STREAMS; i++) {
      if (pan_state_[i] != 0) {
         sprintf(fname, "%s%.3f-%d_tgts.pnm", OUTPUT_DIR, t, i);
printf("Saving %s\n", fname);
         write_pnm_file(fname, pan_out_[i]->size, pan_out_[i]->rgb);
      }
   }
}


static void process_targets(void)
{
   target_stream_type *stream = get_earliest_stream();
   target_desc_type desc;
   double last_t = 0.0;
   while (stream != NULL) {
      //////////////////////////////////////////
      // if time has changed, advance to next t
      if (stream->t != last_t) {
         printf("--------------Advance time to %.3f\n", stream->t);
         // load [new] pan file
         load_panorama(stream->t);
         if (last_t > 0.0) {
            // save old outputs if they exist
            save_outputs(last_t);
         }
         // create fresh output canvas to draw on
         reset_outputs();
         //
         last_t = stream->t;
      }
      //////////////////////////////////////////
      // pull data from stream until it's depleted, then load next set
      while(stream->num_read < stream->num_targets) {
         target_desc_type *retval = read_next_target(stream, &desc);
         if (retval != NULL) {
            // draw target to pan_out_
            draw_target(&desc, stream);
         }
         stream->num_read++;
      }
      load_next_set_head(stream);
      //////////////////////////////////////////
      // get next earliest data set
      stream = get_earliest_stream();
      if ((stream == NULL) || (stream->t < 0.0)) {
         break;
      }
   }
   // save output files
   save_outputs(last_t);
}


int main(int argc, char **argv)
{
   int rc = -1;
   if (argc == 1) {
      log_dir_ = OUTPUT_DIR;
   }
   if (argc != 2) {
      usage(argv);
   }
   /////////////////////////////////////////////
   log_dir_ = argv[1];
   if (open_log_files() != 0) {
      goto end;
   }
   if (create_output_folder() != 0) {
      goto end;
   }
   process_targets();
   rc = 0;
end:
   close_log_files();
   return rc;
}

