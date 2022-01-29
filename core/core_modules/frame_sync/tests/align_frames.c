#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>

#include "core_modules/frame_sync.h"

// need to import tracker.c for constants defined there. that requires
//    the following boilerplate to resolve link errors
////////////////////////////////////////////////////////////////////////
#define hard_exit(a,b)  { fprintf(stderr, "%s : %d", a, b); exit(1); }

////////////////////////////////////////////////
// create dummy functions to resolve link errors
void dp_wait(datap_desc_type *foo) { (void) foo; }
void * dp_get_object_at(const datap_desc_type *foo, const uint32_t bar) { (void) foo; (void) bar; return NULL; };
struct datap_desc *dp_create() { return NULL; }
void dp_execute(datap_desc_type *foo) { (void) foo; }
void dp_signal_data_available(datap_desc_type *foo) { (void) foo; }
void report_thread_id(datap_desc_type *foo, log_info_type *bar) { (void) bar; (void) foo; }
////////////////////////////////////////////////

#include "../frame_sync.c"

// for initialization, use tracker class code. need to set up datap and
//    thread shell
datap_desc_type *dp_ = NULL;
thread_desc_type *td_ = NULL;
frame_sync_type *frame_sync_ = NULL;
// these objects are created in one unit test and destroyed in another

static void create_frame_sync(void)
{
   if (dp_ == NULL) {
      dp_ = calloc(1, sizeof *dp_);
      td_ = calloc(1, sizeof *td_);
      frame_sync_ = calloc(1, sizeof *frame_sync_);
      dp_->td = td_;
      td_->dp = dp_;
      strcpy(td_->class_name, FRAME_SYNC_CLASS_NAME);
      strcpy(td_->obj_name, "Test frame sync");
      dp_->local = frame_sync_;
      /////
      frame_sync_->num_input_cams = 4;
      frame_sync_class_pre_run(dp_);
   }
}


static uint32_t verify_heap_size(uint32_t size)
{
   uint32_t errs = 0;
   uint32_t cnt = 0;
   frame_node_type *node = frame_sync_->frame_node_list_head;
   while (node) {
      cnt++;
      node = node->next;
   }
   if (cnt != size) {
      fprintf(stderr, "Frame heap is of unexpected size. Expected %d, "
            "got %d\n", size, cnt);
   }
   frame_sync_->last_sync_time = 0.0;
   return errs;
}


static uint32_t reset_sync(void)
{
   purge_old_frames(frame_sync_, 1.0e6);
   frame_sync_->last_sync_time = 0.0;
   return verify_heap_size(FRAME_NODE_HEAP_SIZE);
}


uint32_t test_full_sets(void);
uint32_t test_full_sets(void)
{
   uint32_t errs = 0;
   printf("testing full_sets\n");
   /////////////////////////////////////////////////////////////
   reset_sync();
   double frame_times[] = {
         10.010, 10.015, 10.005, 10.020,
         10.170, 10.165, 10.175, 10.185,
         10.360, 10.370, 10.360, 10.345,
         10.500, 10.500, 10.500, 10.499,
         10.675, 10.680, 10.680, 10.675,
         10.825, 10.820, 10.835, 10.840,
         -1.0
   };
   // expected publish times and the frame indices for published frames
   uint32_t publish_idx[] = {
         3, 7, 11, 15, 19, 23
   };
   double publish_times[] = {
         10.0125,
         10.175,
         10.3575,
         10.4998,
         10.6775,
         10.830,
         -1.0
   };
   uint32_t published_frames = 0;
   uint32_t i = 0;
   double t;
   double expected_publish_time = publish_times[0];
   while ((t = frame_times[i]) > 0.0) {
      frame_time_type ft = { .t = t, .frame = NULL };
      add_frame_to_list(frame_sync_, &ft, i%4);
      double publish_time;
      if (expected_publish_time < 0.0) {
         fprintf(stderr, "  No more expected sets but test isn't over. "
               "Next frame = %.3f\n", t);
         errs++;
         break;
      }
      do {
         publish_time = check_for_frame_set(frame_sync_, t);
         if (publish_time > 0.0) {
            // if publication indicated, make sure it's at the expected
            //    time and after the appropriate input frame
            if (fabs(publish_time - expected_publish_time) > 0.001) {
               fprintf(stderr, "  Next set at %.3f, expected at %.3f\n",
                     publish_time, expected_publish_time);
               errs++;
               break;
            }
            if (i != publish_idx[published_frames]) {
               fprintf(stderr, "  Frame published at index %d, expected at "
                     "%d\n", i, publish_idx[published_frames]);
               errs++;
               break;
            }
            purge_old_frames(frame_sync_, 
                  publish_time + CAMERA_FRAME_INTERVAL_SEC);
            published_frames++;
         }
      } while (publish_time > 0.0);
      expected_publish_time = publish_times[published_frames];
      i++;
   }
   if (expected_publish_time >= 0.0) {
      fprintf(stderr, "  Didn't publish all sets. Set %d at %.3f "
            "wasn't published\n", published_frames, expected_publish_time);
      errs++;
   }
   /////////////////////////////////////////////////////////////
   if (errs == 0) {
      printf("    passed\n");
   } else {
      printf("    %d+ errors\n", errs);
   }
   return errs;
}


// only partial frame sets
uint32_t test_partial_sets(void);
uint32_t test_partial_sets(void)
{
   uint32_t errs = 0;
   printf("testing full_sets\n");
   /////////////////////////////////////////////////////////////
   reset_sync();
   double frame_times[] = {
         10.010, 10.015, 10.005,
         10.170, 10.165, 10.175,
         10.360, 10.370, 10.360,
         10.500, 10.500, 10.500,
         10.675, 10.680, 10.680,
         10.825, 10.820, 10.835,
         11.0,
         -1.0
   };
   // expected publish times and the frame indices for published frames
   uint32_t publish_idx[] = {
         3, 6, 9, 12, 15, 18
   };
   double publish_times[] = {
         10.010,
         10.170,
         10.365,
         10.500,
         10.6783,
         10.8285,
         -1.0
   };
   uint32_t published_frames = 0;
   uint32_t i = 0;
   double t;
   double expected_publish_time = publish_times[0];
   while ((t = frame_times[i]) > 0.0) {
      frame_time_type ft = { .t = t, .frame = NULL };
      add_frame_to_list(frame_sync_, &ft, i%4);
      double publish_time;
      if (expected_publish_time < 0.0) {
         fprintf(stderr, "  No more expected sets but test isn't over. "
               "Next frame = %.3f\n", t);
         errs++;
         break;
      }
      do {
         publish_time = check_for_frame_set(frame_sync_, t);
         if (publish_time > 0.0) {
            // if publication indicated, make sure it's at the expected
            //    time and after the appropriate input frame
            if (fabs(publish_time - expected_publish_time) > 0.001) {
               fprintf(stderr, "  Next set at %.3f, expected at %.3f\n",
                     publish_time, expected_publish_time);
               errs++;
               goto end;
            }
            if (i != publish_idx[published_frames]) {
               fprintf(stderr, "  Frame published at index %d, expected at "
                     "%d\n", i, publish_idx[published_frames]);
               errs++;
               goto end;
            }
            purge_old_frames(frame_sync_, 
                  publish_time + HARDWARE_INTERFRAME_INTERVAL_SEC);
            published_frames++;
         }
      } while (publish_time > 0.0);
      expected_publish_time = publish_times[published_frames];
      i++;
   }
   if (expected_publish_time >= 0.0) {
      fprintf(stderr, "  Didn't publish all sets. Set %d at %.3f "
            "wasn't published\n", published_frames, expected_publish_time);
      errs++;
   }
end:
   /////////////////////////////////////////////////////////////
   if (errs == 0) {
      printf("    passed\n");
   } else {
      printf("    %d+ errors\n", errs);
   }
   return errs;
}


// frame sets missing intermittenly frames
uint32_t test_missing_frame(void);
uint32_t test_missing_frame(void)
{
   uint32_t errs = 0;
   printf("testing missing_frame\n");
   /////////////////////////////////////////////////////////////
   reset_sync();
   double frame_times[] = {
         10.010, 10.015, 10.005, 
         10.170, 10.165, 10.175, 10.185,
         10.360, 10.370, 10.360, 
         10.500, 10.500, 10.500, 
         10.675, 10.680, 10.680, 10.675,
         10.825, 10.820, 10.835, 10.840,
         11.500, 11.500, 11.500,    // partial set won't complete
         13.675, 13.680, 13.680, 13.675,
         -1.0
   };
   // expected publish times and the frame indices for published frames
   uint32_t publish_idx[] = {
         3, 6, 10, 13, 16, 20, 27
   };
   double publish_times[] = {
         10.010,
         10.175,
         10.365,
         10.5,
         10.6775,
         10.830,
         13.6775,
         -1.0
   };
   uint32_t published_frames = 0;
   uint32_t i = 0;
   double t;
   double expected_publish_time = publish_times[0];
   while ((t = frame_times[i]) > 0.0) {
      frame_time_type ft = { .t = t, .frame = NULL };
      add_frame_to_list(frame_sync_, &ft, i%4);
      double publish_time;
      if (expected_publish_time < 0.0) {
         fprintf(stderr, "  No more expected sets but test isn't over. "
               "Next frame = %.3f\n", t);
         errs++;
         break;
      }
      do {
         publish_time = check_for_frame_set(frame_sync_, t);
         if (publish_time > 0.0) {
            // if publication indicated, make sure it's at the expected
            //    time and after the appropriate input frame
            if (fabs(publish_time - expected_publish_time) > 0.001) {
               fprintf(stderr, "  Next set at %.3f, expected at %.3f\n",
                     publish_time, expected_publish_time);
               errs++;
               goto end;
            }
            if (i != publish_idx[published_frames]) {
               fprintf(stderr, "  Frame published at index %d, expected at "
                     "%d\n", i, publish_idx[published_frames]);
               errs++;
               goto end;
            }
            purge_old_frames(frame_sync_, 
                  publish_time + HARDWARE_INTERFRAME_INTERVAL_SEC);
            published_frames++;
         }
      } while (publish_time > 0.0);
      expected_publish_time = publish_times[published_frames];
      i++;
   }
   if (expected_publish_time >= 0.0) {
      fprintf(stderr, "  Didn't publish all sets. Set %d at %.3f "
            "wasn't published\n", published_frames, expected_publish_time);
      errs++;
   }
end:
   /////////////////////////////////////////////////////////////
   if (errs == 0) {
      printf("    passed\n");
   } else {
      printf("    %d+ errors\n", errs);
   }
   return errs;
}


// tests recovery from frames resynchronizing
uint32_t test_misalign(void);
uint32_t test_misalign(void)
{
   uint32_t errs = 0;
   printf("testing misalign\n");
   /////////////////////////////////////////////////////////////
   // frames slowly go 1/2 phase out of sync (+80ms)
   reset_sync();
   double frame_times[] = {
         10.010, 10.015, 10.005, 10.020,  // full set
         10.170, 10.165, 10.175, 10.265,  // #4 +80
         10.360, 10.370, 10.440, 10.425,  // #3-4 +80
         10.500, 10.580, 10.580, 10.579,  // #2-4 +80
         10.755, 10.760, 10.760, 10.755,  // all +80
         10.905, 10.900, 10.915, 11.000,  // #1-3 +80, #4 +160
         // last not published as no frame is future enough to trigger it
         -1.0
   };
   // expected publish times and the frame indices for published frames
   uint32_t publish_idx[] = {
         3, 8, 12, 16, 19
   };
   double publish_times[] = {
         10.0125,
         10.170,
         10.365,
         10.5797,
         10.7575,
         -1.0
   };
   uint32_t published_frames = 0;
   uint32_t i = 0;
   double t;
   double expected_publish_time = publish_times[0];
   while ((t = frame_times[i]) > 0.0) {
      frame_time_type ft = { .t = t, .frame = NULL };
      add_frame_to_list(frame_sync_, &ft, i%4);
      double publish_time;
      do {
         publish_time = check_for_frame_set(frame_sync_, t);
         if (publish_time > 0.0) {
            // if publication indicated, make sure it's at the expected
            //    time and after the appropriate input frame
            if (fabs(publish_time - expected_publish_time) > 0.001) {
               fprintf(stderr, "  Next set at %.3f, expected at %.3f\n",
                     publish_time, expected_publish_time);
               errs++;
               goto end;
            }
            if (i != publish_idx[published_frames]) {
               fprintf(stderr, "  Frame published at index %d, expected at "
                     "%d\n", i, publish_idx[published_frames]);
               errs++;
               goto end;
            }
            purge_old_frames(frame_sync_, 
                  publish_time + HARDWARE_INTERFRAME_INTERVAL_SEC);
            published_frames++;
         }
      } while (publish_time > 0.0);
      expected_publish_time = publish_times[published_frames];
      i++;
   }
   if (expected_publish_time >= 0.0) {
      fprintf(stderr, "  Didn't publish all sets. Set %d at %.3f "
            "wasn't published\n", published_frames, expected_publish_time);
      errs++;
   }
end:
   /////////////////////////////////////////////////////////////
   if (errs == 0) {
      printf("    passed\n");
   } else {
      printf("    %d+ errors\n", errs);
   }
   return errs;
}


uint32_t test_sort(void);
uint32_t test_sort(void)
{
   uint32_t errs = 0;
   printf("testing sort\n");
   /////////////////////////////////////////////////////////////
   reset_sync();
   double frame_times[] = {
         10.010, 10.100, 10.150, 10.020, 10.010,
         10.170, 10.001, 10.200, 10.300, 10.200,
         -1.0
   };
   double t;
   uint32_t i = 0;
   while ((t = frame_times[i++]) > 0.0) {
      frame_time_type ft = { .frame = NULL, .t = t };
      add_frame_to_list(frame_sync_, &ft, 0);
   }
   frame_node_type *node = frame_sync_->active_frame_list_head;
   while (node) {
      if (node->next) {
         if (node->next->t < node->t) {
            fprintf(stderr, "Out of order frames: %.3f before %.3f\n",
                  node->next->t, node->t);
            errs++;
         }
      }
      node = node->next;
   }
   /////////////////////////////////////////////////////////////
   if (errs == 0) {
      printf("    passed\n");
   } else {
      printf("    %d+ errors\n", errs);
   }
   return errs;
}


// tests recovery from frames resynchronizing
uint32_t test_alloc(void);
uint32_t test_alloc(void)
{
   uint32_t errs = 0;
   printf("testing alloc\n");
   /////////////////////////////////////////////////////////////
   reset_sync();
   errs += verify_heap_size(FRAME_NODE_HEAP_SIZE);
   for (uint32_t i=0; i<FRAME_NODE_HEAP_SIZE; i++) {
      for (uint32_t j=0; j<i; j++) {
         frame_time_type ft = { 
               .frame = NULL, 
               .t = (double) i + 0.001 * (double) j 
         };
         add_frame_to_list(frame_sync_, &ft, 0);
      }
      errs += verify_heap_size(FRAME_NODE_HEAP_SIZE - i);
      purge_old_frames(frame_sync_, 1.0e6);
      errs += verify_heap_size(FRAME_NODE_HEAP_SIZE);
   }
   /////////////////////////////////////////////////////////////
   if (errs == 0) {
      printf("    passed\n");
   } else {
      printf("    %d+ errors\n", errs);
   }
   return errs;
}


// tests construction of output frame set
uint32_t test_output(void);
uint32_t test_output(void)
{
   uint32_t errs = 0;
   printf("testing output\n");
   /////////////////////////////////////////////////////////////
   reset_sync();
   double frame_times[] = {
         10.010, 10.015, 10.005, 10.020,
         -1.0
   };
   uint32_t i = 0;
   double t;
   while ((t = frame_times[i]) > 0.0) {
      frame_time_type ft = { .t = t, .frame = (void*) frame_sync_ };
      add_frame_to_list(frame_sync_, &ft, i%4);
      //
      double publish_time = check_for_frame_set(frame_sync_, t);
      if (publish_time > 0.0) {
         frame_sync_output_type output;
         memset(&output, 0, sizeof output);
         build_frame_set(frame_sync_, &output, publish_time);
         //
         uint32_t num_frames = 0;
         for (uint32_t j=0; j<MAX_NUM_CAMERAS; j++) {
            if (output.frames[j] != NULL) {
               num_frames++;
            }
         }
         if (num_frames != 4) {
            fprintf(stderr, "Failed to publish full frame set. "
                  "Publishing %d of 4\n", num_frames);
            errs++;
         }
         goto end;
      }
      i++;
   }
   fprintf(stderr, "  Failed to find frame set to publish\n");
   errs++;
end:
   /////////////////////////////////////////////////////////////
   if (errs == 0) {
      printf("    passed\n");
   } else {
      printf("    %d errors\n", errs);
   }
   return errs;
}


int main(int argc, char** argv)
{
   (void) argc;
   uint32_t errs = 0;
   set_ppd(10.0);
   create_frame_sync();
   //////////////////
   errs += test_full_sets();
   errs += test_partial_sets();
   errs += test_missing_frame();
   errs += test_misalign();
   errs += test_alloc();
   errs += test_sort();
   errs += test_output();
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


