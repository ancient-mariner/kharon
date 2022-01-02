// sends Y and V channels at 1/4 resolution
/*
Copyright (c) 2018, Raspberry Pi (Trading) Ltd.
Copyright (c) 2014, DSP Group Ltd.
Copyright (c) 2014, James Hughes
Copyright (c) 2013, Broadcom Europe Ltd.

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/**
 * \file RaspiVidYUV.c
 * Command line program to capture a camera video stream and save file
 * as uncompressed YUV420 data
 * Also optionally display a preview/viewfinder of current camera input.
 *
 * Description
 *
  * 2 components are created; camera and preview.
 * Camera component has three ports, preview, video and stills.
 * Preview is connected using standard mmal connections, the video output
 * is written straight to the file in YUV 420 format via the requisite buffer
 * callback. Still port is not used
 *
 * We use the RaspiCamControl code to handle the specific camera settings.
 * We use the RaspiPreview code to handle the generic preview
*/

// We use some GNU extensions (basename)
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include "pin_types.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <memory.h>
#include <sysexits.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/time.h>

#include "bcm_host.h"
#include "interface/vcos/vcos.h"

#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_logging.h"
#include "interface/mmal/mmal_buffer.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_connection.h"

#include "RaspiCommonSettings.h"
#include "RaspiCamControl.h"
#include "RaspiPreview.h"
#include "RaspiCLI.h"
#include "RaspiHelpers.h"
#include "RaspiGPS.h"

#include "pinet.h"
#include "lin_alg.h"
#include "dev_info.h"
#include "mem.h"
#include "udp_sync_receiver.h"
#include "timekeeper.h"

#include <semaphore.h>

/////////////////////
// include down.c source inline
#include "down.c"


//////////////////////////////////////
// changes from stock RaspiStillYUV.c:
//
// prints timestamp from system clock (microsecond resolution)
//
//////////////////////////////////////


// Standard port setting for the camera component
#define MMAL_CAMERA_PREVIEW_PORT 0
#define MMAL_CAMERA_VIDEO_PORT 1
#define MMAL_CAMERA_CAPTURE_PORT 2

// Video format information
// 0 implies variable
// frames are captured at a much higher rate than used. there's no
//    way to synchronize the frames between cameras (the hardware
//    doesn't support external sync/trigger). to compensate, fetch
//    frames at a high rate and only keep the ones that are closest
//    to a synchronized pulse at the desired frame rate
// it's probably best to keep frame rate at an even multiple of system
//    frame rate
// 2x2 binned images are supposed to be streamable at up to 40fps
#define VIDEO_FRAME_RATE_NUM     (HARDWARE_FRAME_RATE)
#define VIDEO_FRAME_INTERVAL     (1.0f / ((float) VIDEO_FRAME_RATE_NUM))
#define VIDEO_FRAME_RATE_DEN 1

/// Video render needs at least 2 buffers.
#define VIDEO_OUTPUT_BUFFERS_NUM 3

/// Interval at which we check for an failure abort during capture
const int ABORT_INTERVAL = 100; // ms


/// Capture/Pause switch method
enum
{
   WAIT_METHOD_NONE,       /// Simply capture for time specified
   WAIT_METHOD_TIMED,      /// Cycle between capture and pause for times specified
   WAIT_METHOD_KEYPRESS,   /// Switch between capture and pause on keypress
   WAIT_METHOD_SIGNAL,     /// Switch between capture and pause on signal
   WAIT_METHOD_FOREVER     /// Run/record forever
};

// Forward
typedef struct RASPIVIDYUV_STATE_S RASPIVIDYUV_STATE;

/** Struct used to pass information in camera video port userdata to callback
 */
typedef struct
{
   RASPIVIDYUV_STATE *pstate;           /// pointer to our state in case required in callback
   int abort;                           /// Set to 1 in callback if an error occurs to attempt to abort the capture
} PORT_USERDATA;

/** Structure containing all state information for the current run
 */
struct RASPIVIDYUV_STATE_S
{
   RASPICOMMONSETTINGS_PARAMETERS common_settings;
   int timeout;                        /// Time taken before frame is grabbed and app then shuts down. Units are milliseconds
   int framerate;                      /// Requested frame rate (fps)
   int demoMode;                       /// Run app in demo mode
   int demoInterval;                   /// Interval between camera settings changes
   int waitMethod;                     /// Method for switching between pause and capture

   int onTime;                         /// In timed cycle mode, the amount of time the capture is on per cycle
   int offTime;                        /// In timed cycle mode, the amount of time the capture is off per cycle

   int onlyLuma;                       /// Only output the luma / Y plane of the YUV data
   int useRGB;                         /// Output RGB data rather than YUV

   RASPIPREVIEW_PARAMETERS preview_parameters;   /// Preview setup parameters
   RASPICAM_CAMERA_PARAMETERS camera_parameters; /// Camera setup parameters

   MMAL_COMPONENT_T *camera_component;    /// Pointer to the camera component
   MMAL_CONNECTION_T *preview_connection; /// Pointer to the connection from camera to preview

   MMAL_POOL_T *camera_pool;            /// Pointer to the pool of buffers used by camera video port

   PORT_USERDATA callback_data;         /// Used to move data to the camera callback

   int bCapturing;                      /// State of capture/pause
};

static XREF_T  initial_map[] =
{
   {"record",     0},
   {"pause",      1},
};

static int initial_map_size = sizeof(initial_map) / sizeof(initial_map[0]);

/// Command ID's and Structure defining our command line options
enum
{
   CommandTimeout,
   CommandDemoMode,
   CommandFramerate,
   CommandTimed,
   CommandSignal,
   CommandKeypress,
   CommandInitialState,
   CommandOnlyLuma,
   CommandUseRGB,
   CommandSavePTS,
   CommandNetListen
};

static COMMAND_LIST cmdline_commands[] =
{
   { CommandTimeout,       "-timeout",    "t",  "Time (in ms) to capture for. If not specified, set to 5s. Zero to disable", 1 },
   { CommandDemoMode,      "-demo",       "d",  "Run a demo mode (cycle through range of camera options, no capture)", 1},
   { CommandFramerate,     "-framerate",  "fps","Specify the frames per second to record", 1},
   { CommandTimed,         "-timed",      "td", "Cycle between capture and pause. -cycle on,off where on is record time and off is pause time in ms", 0},
   { CommandSignal,        "-signal",     "s",  "Cycle between capture and pause on Signal", 0},
   { CommandKeypress,      "-keypress",   "k",  "Cycle between capture and pause on ENTER", 0},
   { CommandInitialState,  "-initial",    "i",  "Initial state. Use 'record' or 'pause'. Default 'record'", 1},
   { CommandOnlyLuma,      "-luma",       "y",  "Only output the luma / Y of the YUV data'", 0},
   { CommandUseRGB,        "-rgb",        "rgb","Save as RGB data rather than YUV", 0},
};

static int cmdline_commands_size = sizeof(cmdline_commands) / sizeof(cmdline_commands[0]);


static struct
{
   char *description;
   int nextWaitMethod;
} wait_method_description[] =
{
      {"Simple capture",         WAIT_METHOD_NONE},
      {"Capture forever",        WAIT_METHOD_FOREVER},
      {"Cycle on time",          WAIT_METHOD_TIMED},
      {"Cycle on signal",        WAIT_METHOD_SIGNAL},
};

static int wait_method_description_size = sizeof(wait_method_description) / sizeof(wait_method_description[0]);


////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// frame communication thread
//
// this process takes frame data from the camera (pre-processed or not)
//    and transmits it to receiver. data is copied from camera buffer
//    and queued for transmit, releasing camera to record next frame.
//
// y channel is downsampled before broadcast
// v channel sent first (u is dropped)
//    

// thread and sync objects
static pthread_mutex_t  s_frame_mutex;
static pthread_cond_t  s_frame_cond;
static pthread_t s_comm_tid;

static pthread_t s_main_tid;

// timestamp of next frame set for transmission
static double s_frame_timestamp = 0.0;
static double s_frame_start = -1.0;
static double s_frame_start_copy;

// capture at high frame rate and keep only the frames that arrive
//    right after the heartbeat pulse. drop the rest
static int32_t s_frame_pending = 0;

// frame that is set for transmission
static uint8_t * s_y_frame_buffer = NULL;
static uint8_t * s_v_frame_buffer = NULL;
static uint8_t * s_blur_buffer = NULL;
// 1 if buffer available for copying frame data into, 0 if not
static int s_buffer_avail = 0;

static int s_shutdown = 0; // flag to indicate if comm thread should eit
static int s_connfd = -1;  // socket connection descriptor
static int s_comm_error = 0;  // local equiv of errno

MMAL_PORT_T *s_camera_video_port = NULL;

// frame received from device
// queue frame for broadcast. if previous frame not fully handled,
//    present frame is dropped
// NOTE: communication thread exits on network error, so by time
//    camera process learns of problem, communication thread has
//    exited and is ready to join
static void push_frame_buffer(uint8_t *buf, int32_t n_bytes)
{
   // n_bytes is size of acquired image
   if (n_bytes != YUV_BUFFER_SIZE) {
      fprintf(stderr, "Internal error - unexpected frame size\n");
      fprintf(stderr, "Expected %dx%d=%d -> %d bytes\n", 
            YUV_Y_COLS, YUV_Y_ROWS, YUV_Y_PIX, YUV_IMAGE_SIZE);
      fprintf(stderr, "Received %d bytes\n", n_bytes);
      hard_exit("camera_vy:push_frame_buffer", 1);
   }
   //-------------------------------------- lock
   s_frame_start_copy = s_frame_start;
   s_frame_timestamp = now();
   float dt = s_frame_timestamp - s_frame_start;
   // drop frames that are too far outside of expected range
   // this should only happen on the first frame
   if (dt < 2.0f * VIDEO_FRAME_INTERVAL) {
      pthread_mutex_lock(&s_frame_mutex);
      // 
      if (s_buffer_avail) {
         s_buffer_avail = 0;
         downsample(buf, s_blur_buffer, s_y_frame_buffer, 
               YUV_Y_COLS, YUV_Y_ROWS, CAM_COLS, CAM_ROWS);
         copy_image(&buf[YUV_V_OFFSET], s_v_frame_buffer, 
               YUV_V_COLS, YUV_V_ROWS, CAM_COLS, CAM_ROWS);
         // signal that buffer is ready to be sent
         pthread_cond_signal(&s_frame_cond);
      } else {
         printf("Frame dropped\n");
      }
      pthread_mutex_unlock(&s_frame_mutex);
   }
   //-------------------------------------- unlock
   s_frame_pending = 0;
}

// passes ownership of communication socket to comm thread
static void comm_set_socket(int connfd)
{
   s_connfd = connfd;
}


//static void remove_uv_bounds(
//      /* in     */ const uint8_t * restrict raw,
//      /*    out */       uint8_t * restrict output
//      )
//{
//   for (uint32_t r=0; r<VY_ROWS_NET_IMG; r++) {
//      memcpy(&output[r*VY_COLS_NET_IMG], &raw[r*VY_COLS_NET_TOT], VY_COLS_NET_IMG);
//   }
//}
//
//void downsample_2(
//      /* in     */ const uint8_t *restrict img,
//      /* in     */ const image_coordinate_type img_size,
//      /* in     */ const image_coordinate_type down_size,
//      /*    out */ uint8_t *restrict down);


// thread main
// on communication error, s_comm_error is set
//    0 is no error
//    1 means error occurred when sending header
//    2 means error occurred when sending body, part 1
//    3 means error occurred when sending body, part 2
static void * communication_main(void * not_used)
{
   pthread_mutex_init(&s_frame_mutex, NULL);
   pthread_cond_init(&s_frame_cond, NULL);
   s_y_frame_buffer = malloc(CAM_N_PIX);
   s_v_frame_buffer = malloc(CAM_N_PIX);
   s_blur_buffer = malloc(CAM_N_PIX);
   // 
   s_buffer_avail = 1;
   struct sensor_packet_header header;
   //
   while (s_shutdown == 0) {
      // wait for signal that frame is ready
      pthread_mutex_lock(&s_frame_mutex);
      while (s_buffer_avail == 1) {
         pthread_cond_wait(&s_frame_cond, &s_frame_mutex);
         if (s_shutdown)
            break;
      }
//printf("cleared condition wait\n");
      pthread_mutex_unlock(&s_frame_mutex);
      // check for shutdown flag being set outside of wait, as shutdown
      //    could have have been set by another thread during wait
      if (s_shutdown)
         break;
      // send v components of image, then y
      serialize_sensor_header2(VY_PACKET_TYPE, s_frame_start_copy,
            s_frame_timestamp, &header);
      header.custom_16[0] = htons(CAM_ROWS);
      header.custom_16[1] = htons(CAM_COLS);
//printf("sending %dx%d yv data at %.4f (%.4f)\n", VY_COLS_NET_TOT, VY_ROWS_NET_TOT, now(), s_frame_timestamp);
double t1 = now();
//printf(".. %ld bytes\n", sizeof(header));
      if (send_block(s_connfd, &header, sizeof(header)) < 0) {
         fprintf(stderr, "Network error sending header packet\n");
         s_comm_error = -1;
         s_shutdown = 1;
         break;
      }
      // send body, part 1 (ie, V channel)
      if (send_block(s_connfd, s_v_frame_buffer, CAM_N_PIX) < 0) {
         fprintf(stderr, "Error sending part 1 of VY image frame\n"); 
         s_comm_error = -2;
         s_shutdown = 1;
         break;
      }
      // send Y
      if (send_block(s_connfd, s_y_frame_buffer, CAM_N_PIX) < 0) {
         fprintf(stderr, "Error sending part 2 of VY image frame\n"); 
         s_comm_error = -3;
         s_shutdown = 1;
         break;
      }
      // signal frame broadcast is complete
      // this doesn't have to be protected by mutex as variable can't
      //    be altered by another process so long as it's zero, and 
      //    this code can't be executed if it's non-zero
      s_buffer_avail = 1;
double t2 = now();
printf("acq: %.3f, delivery: %.4f (dt=%.3f), proc: %.4f (dt=%.3f)\n", s_frame_start_copy, s_frame_timestamp, s_frame_timestamp-s_frame_start_copy, t2, t2-t1);
   };
printf("Comm exiting\n");
   // send signal to unblock acquisition thread, in case it's blocking
   if (pthread_kill(s_main_tid, SIGUSR1) != 0) {
      perror("pthread_kill error unblocking acq thread");
   }
   //
   return NULL;
}

// returns return code of pthread_create
static int launch_comm_thread()
{
   return pthread_create(&s_comm_tid, NULL, communication_main, NULL);
}

// signals comm thread to exit, waits for exit, and closes down
//    resources (including provided socket)
static void shutdown_comm()
{
printf("shutting down Comm\n");
   // signal time for comm thread to exit
   s_shutdown = 1;  // this is redundant but doesn't hurt
   // signal condition so comm thread is awoken
   pthread_mutex_lock(&s_frame_mutex);
   pthread_cond_signal(&s_frame_cond);
   pthread_mutex_unlock(&s_frame_mutex);
   // wait to rejoin w/ thread
   pthread_join(s_comm_tid, NULL);
   // release mutex and contition vars
   pthread_mutex_destroy(&s_frame_mutex);
   pthread_cond_destroy(&s_frame_cond);
   // shutdown socket
   if (s_connfd >= 0)
      close(s_connfd);
}

// frame communication thread
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

/**
 * Assign a default set of parameters to the state passed in
 *
 * @param state Pointer to state structure to assign defaults to
 */
static void default_status(RASPIVIDYUV_STATE *state)
{
   if (!state)
   {
      vcos_assert(0);
      return;
   }

   // Default everything to zero
   memset(state, 0, sizeof(RASPIVIDYUV_STATE));

   raspicommonsettings_set_defaults(&state->common_settings);

   // customize defaults for desired behavior
   state->timeout = 0;  // run forever
   state->common_settings.width = 1640; // default to 1/4 resolution for cam v2 (binned mode)
   state->common_settings.height = 1232;
   state->framerate = VIDEO_FRAME_RATE_NUM;
   state->demoMode = 0;
   state->demoInterval = 250; // ms
   state->waitMethod = WAIT_METHOD_SIGNAL;
   state->onTime = 5000;
   state->offTime = 5000;

   state->bCapturing = 0;

   state->common_settings.sensor_mode = 4; // 2x2 binning, full FOV
   state->onlyLuma = 0;

   // Setup preview window defaults
   raspipreview_set_defaults(&state->preview_parameters);

   // Set up the camera_parameters to default
   raspicamcontrol_set_defaults(&state->camera_parameters);
}


/**
 * Dump image state parameters to stderr.
 *
 * @param state Pointer to state structure to assign defaults to
 */
static void dump_status(RASPIVIDYUV_STATE *state)
{
   int i, size, ystride, yheight;

   if (!state)
   {
      vcos_assert(0);
      return;
   }

   raspicommonsettings_dump_parameters(&state->common_settings);

   fprintf(stderr, "framerate %d, time delay %d\n", state->framerate, state->timeout);

   // Calculate the individual image size
   // Y stride rounded to multiple of 32. U&V stride is Y stride/2 (ie multiple of 16).
   // Y height is padded to a 16. U/V height is Y height/2 (ie multiple of 8).

   // Y plane
   ystride = ((state->common_settings.width + 31) & ~31);
   yheight = ((state->common_settings.height + 15) & ~15);

   size = ystride * yheight;

   // U and V plane
   size += 2 * ystride/2 * yheight/2;

   fprintf(stderr, "Sub-image size %d bytes in total.\n  Y pitch %d, Y height %d, UV pitch %d, UV Height %d\n", size, ystride, yheight, ystride/2,yheight/2);

   fprintf(stderr, "Wait method : ");
   for (i=0; i<wait_method_description_size; i++)
   {
      if (state->waitMethod == wait_method_description[i].nextWaitMethod)
         fprintf(stderr, "%s", wait_method_description[i].description);
   }
   fprintf(stderr, "\nInitial state '%s'\n", raspicli_unmap_xref(state->bCapturing, initial_map, initial_map_size));
   fprintf(stderr, "\n");

   raspipreview_dump_parameters(&state->preview_parameters);
   raspicamcontrol_dump_parameters(&state->camera_parameters);
}

/**
 * Display usage information for the application to stdout
 *
 * @param app_name String to display as the application name
 */
static void application_help_message(char *app_name)
{
   fprintf(stdout, "Display camera output to display, and optionally saves an uncompressed YUV420 or RGB file \n\n");
   fprintf(stdout, "NOTE: High resolutions and/or frame rates may exceed the bandwidth of the system due\n");
   fprintf(stdout, "to the large amounts of data being moved to the SD card. This will result in undefined\n");
   fprintf(stdout, "results in the subsequent file.\n");
   fprintf(stdout, "The single raw file produced contains all the images. Each image in the files will be of size\n");
   fprintf(stdout, "width*height*1.5 for YUV or width*height*3 for RGB, unless width and/or height are not divisible by 16.");
   fprintf(stdout, "Use the image size displayed during the run (in verbose mode) for an accurate value\n");

   fprintf(stdout, "The Linux split command can be used to split up the file to individual frames\n");

   fprintf(stdout, "\nusage: %s [options]\n\n", app_name);

   fprintf(stdout, "Image parameter commands\n\n");

   raspicli_display_help(cmdline_commands, cmdline_commands_size);

   fprintf(stdout, "\n");

   return;
}

/**
 * Parse the incoming command line and put resulting parameters in to the state
 *
 * @param argc Number of arguments in command line
 * @param argv Array of pointers to strings from command line
 * @param state Pointer to state structure to assign any discovered parameters to
 * @return Non-0 if failed for some reason, 0 otherwise
 */
static int parse_cmdline(int argc, const char **argv, RASPIVIDYUV_STATE *state)
{
   // Parse the command line arguments.
   // We are looking for --<something> or -<abbreviation of something>

   int valid = 1;
   int i;

   for (i = 1; i < argc && valid; i++)
   {
      int command_id, num_parameters;

      if (!argv[i])
         continue;

      if (argv[i][0] != '-')
      {
         valid = 0;
         continue;
      }

      // Assume parameter is valid until proven otherwise
      valid = 1;

      command_id = raspicli_get_command_id(cmdline_commands, cmdline_commands_size, &argv[i][1], &num_parameters);

      // If we found a command but are missing a parameter, continue (and we will drop out of the loop)
      if (command_id != -1 && num_parameters > 0 && (i + 1 >= argc) )
         continue;

      //  We are now dealing with a command line option
      switch (command_id)
      {
      case CommandTimeout: // Time to run viewfinder/capture
      {
         if (sscanf(argv[i + 1], "%d", &state->timeout) == 1)
         {
            // Ensure that if previously selected a waitMethod we don't overwrite it
            if (state->timeout == 0 && state->waitMethod == WAIT_METHOD_NONE)
               state->waitMethod = WAIT_METHOD_FOREVER;

            i++;
         }
         else
            valid = 0;
         break;
      }

      case CommandDemoMode: // Run in demo mode - no capture
      {
         // Demo mode might have a timing parameter
         // so check if a) we have another parameter, b) its not the start of the next option
         if (i + 1 < argc  && argv[i+1][0] != '-')
         {
            if (sscanf(argv[i + 1], "%u", &state->demoInterval) == 1)
            {
               // TODO : What limits do we need for timeout?
               if (state->demoInterval == 0)
                  state->demoInterval = 250; // ms

               state->demoMode = 1;
               i++;
            }
            else
               valid = 0;
         }
         else
         {
            state->demoMode = 1;
         }

         break;
      }

      case CommandFramerate: // fps to record
      {
         if (sscanf(argv[i + 1], "%u", &state->framerate) == 1)
         {
            // TODO : What limits do we need for fps 1 - 30 - 120??
            i++;
         }
         else
            valid = 0;
         break;
      }

      case CommandTimed:
      {
         if (sscanf(argv[i + 1], "%u,%u", &state->onTime, &state->offTime) == 2)
         {
            i++;

            if (state->onTime < 1000)
               state->onTime = 1000;

            if (state->offTime < 1000)
               state->offTime = 1000;

            state->waitMethod = WAIT_METHOD_TIMED;

            if (state->timeout == -1)
               state->timeout = 0;
         }
         else
            valid = 0;
         break;
      }

      case CommandSignal:
         state->waitMethod = WAIT_METHOD_SIGNAL;
         // Reenable the signal
         signal(SIGUSR1, default_signal_handler);

         if (state->timeout == -1)
            state->timeout = 0;

         break;

      case CommandInitialState:
      {
         state->bCapturing = raspicli_map_xref(argv[i + 1], initial_map, initial_map_size);

         if( state->bCapturing == -1)
            state->bCapturing = 0;

         i++;
         break;
      }

      case CommandOnlyLuma:
         if (state->useRGB)
         {
            fprintf(stderr, "--luma and --rgb are mutually exclusive\n");
            valid = 0;
         }
         state->onlyLuma = 1;
         break;

      case CommandUseRGB: // display lots of data during run
         if (state->onlyLuma)
         {
            fprintf(stderr, "--luma and --rgb are mutually exclusive\n");
            valid = 0;
         }
         state->useRGB = 1;
         break;

      default:
      {
         // Try parsing for any image specific parameters
         // result indicates how many parameters were used up, 0,1,2
         // but we adjust by -1 as we have used one already
         const char *second_arg = (i + 1 < argc) ? argv[i + 1] : NULL;
         int parms_used = (raspicamcontrol_parse_cmdline(&state->camera_parameters, &argv[i][1], second_arg));

         // Still unused, try common settings
         if (!parms_used)
            parms_used = raspicommonsettings_parse_cmdline(&state->common_settings, &argv[i][1], second_arg, &application_help_message);

         // Still unused, try preview options
         if (!parms_used)
            parms_used = raspipreview_parse_cmdline(&state->preview_parameters, &argv[i][1], second_arg);


         // If no parms were used, this must be a bad parameters
         if (!parms_used)
            valid = 0;
         else
            i += parms_used - 1;

         break;
      }
      }
   }

   if (!valid)
   {
      fprintf(stderr, "Invalid command line option (%s)\n", argv[i-1]);
      return 1;
   }

   return 0;
}

/**
 *  buffer header callback function for camera
 *
 *  Callback will dump buffer data to internal buffer
 *
 * @param port Pointer to port from which callback originated
 * @param buffer mmal buffer header pointer
 */
static void camera_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
   MMAL_BUFFER_HEADER_T *new_buffer;

   // We pass our file handle and other stuff in via the userdata field.
   PORT_USERDATA *pData = (PORT_USERDATA *)port->userdata;
   RASPIVIDYUV_STATE *pstate = pData->pstate;
   if (s_shutdown != 0)
      return;

   if (MESSAGE_BOARD.exit)
      return;
//   if (!MESSAGE_BOARD.acquiring || MESSAGE_BOARD.paused) {
////printf("camera buffer callback during pause -- stopping capture\n");
//      pstate->bCapturing = 0;
//      mmal_port_parameter_set_boolean(s_camera_video_port, 
//            MMAL_PARAMETER_CAPTURE, pstate->bCapturing);
//   } else 
   if (s_frame_pending && pData) {
//printf("camera buffer callback\n");
      int n_bytes = buffer->length;
      if (buffer->length && pstate->onlyLuma)
         n_bytes = port->format->es->video.width*port->format->es->video.height;
      if (n_bytes)
      {
         mmal_buffer_header_mem_lock(buffer);
         push_frame_buffer(buffer->data, n_bytes);
         mmal_buffer_header_mem_unlock(buffer);
      }
   }
//   else {
//      vcos_log_error("Received a camera buffer callback with no state");
//   }

   // release buffer back to the pool
   mmal_buffer_header_release(buffer);

   // and send one back to the port (if still open)
   if (port->is_enabled)
   {
      MMAL_STATUS_T status;

      new_buffer = mmal_queue_get(pstate->camera_pool->queue);

      if (new_buffer)
         status = mmal_port_send_buffer(port, new_buffer);

      if (!new_buffer || status != MMAL_SUCCESS)
         vcos_log_error("Unable to return a buffer to the camera port");
   }
}


/**
 * Create the camera component, set up its ports
 *
 * @param state Pointer to state control struct
 *
 * @return MMAL_SUCCESS if all OK, something else otherwise
 *
 */
static MMAL_STATUS_T create_camera_component(RASPIVIDYUV_STATE *state)
{
   MMAL_COMPONENT_T *camera = 0;
   MMAL_ES_FORMAT_T *format;
   MMAL_PORT_T *preview_port = NULL, *video_port = NULL, *still_port = NULL;
   MMAL_STATUS_T status;
   MMAL_POOL_T *pool;

   /* Create the component */
   status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA, &camera);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Failed to create camera component");
      goto error;
   }

   MMAL_PARAMETER_INT32_T camera_num =
   {{MMAL_PARAMETER_CAMERA_NUM, sizeof(camera_num)}, state->common_settings.cameraNum};

   status = mmal_port_parameter_set(camera->control, &camera_num.hdr);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Could not select camera : error %d", status);
      goto error;
   }

   if (!camera->output_num)
   {
      status = MMAL_ENOSYS;
      vcos_log_error("Camera doesn't have output ports");
      goto error;
   }

   status = mmal_port_parameter_set_uint32(camera->control, MMAL_PARAMETER_CAMERA_CUSTOM_SENSOR_CONFIG, state->common_settings.sensor_mode);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Could not set sensor mode : error %d", status);
      goto error;
   }

   preview_port = camera->output[MMAL_CAMERA_PREVIEW_PORT];
   video_port = camera->output[MMAL_CAMERA_VIDEO_PORT];
   still_port = camera->output[MMAL_CAMERA_CAPTURE_PORT];

   // Enable the camera, and tell it its control callback function
   status = mmal_port_enable(camera->control, default_camera_control_callback);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Unable to enable control port : error %d", status);
      goto error;
   }

   //  set up the camera configuration
   {
      MMAL_PARAMETER_CAMERA_CONFIG_T cam_config =
      {
         { MMAL_PARAMETER_CAMERA_CONFIG, sizeof(cam_config) },
         .max_stills_w = state->common_settings.width,
         .max_stills_h = state->common_settings.height,
         .stills_yuv422 = 0,
         .one_shot_stills = 0,
         .max_preview_video_w = state->common_settings.width,
         .max_preview_video_h = state->common_settings.height,
         .num_preview_video_frames = 3,
         .stills_capture_circular_buffer_height = 0,
         .fast_preview_resume = 0,
         .use_stc_timestamp = MMAL_PARAM_TIMESTAMP_MODE_RESET_STC
      };
      mmal_port_parameter_set(camera->control, &cam_config.hdr);
   }

   // Now set up the port formats

   // Set the encode format on the Preview port
   // HW limitations mean we need the preview to be the same size as the required recorded output

   format = preview_port->format;

   if(state->camera_parameters.shutter_speed > 6000000)
   {
      MMAL_PARAMETER_FPS_RANGE_T fps_range = {{MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},
         { 50, 1000 }, {166, 1000}
      };
      mmal_port_parameter_set(preview_port, &fps_range.hdr);
   }
   else if(state->camera_parameters.shutter_speed > 1000000)
   {
      MMAL_PARAMETER_FPS_RANGE_T fps_range = {{MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},
         { 166, 1000 }, {999, 1000}
      };
      mmal_port_parameter_set(preview_port, &fps_range.hdr);
   }

   //enable dynamic framerate if necessary
   if (state->camera_parameters.shutter_speed)
   {
      if (state->framerate > 1000000./state->camera_parameters.shutter_speed)
      {
         state->framerate=0;
         if (state->common_settings.verbose)
            fprintf(stderr, "Enable dynamic frame rate to fulfil shutter speed requirement\n");
      }
   }

   format->encoding = MMAL_ENCODING_OPAQUE;
   format->es->video.width = VCOS_ALIGN_UP(state->common_settings.width, 32);
   format->es->video.height = VCOS_ALIGN_UP(state->common_settings.height, 16);
   format->es->video.crop.x = 0;
   format->es->video.crop.y = 0;
   format->es->video.crop.width = state->common_settings.width;
   format->es->video.crop.height = state->common_settings.height;
   format->es->video.frame_rate.num = PREVIEW_FRAME_RATE_NUM;
   format->es->video.frame_rate.den = PREVIEW_FRAME_RATE_DEN;

   status = mmal_port_format_commit(preview_port);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("camera viewfinder format couldn't be set");
      goto error;
   }

   // Set the encode format on the video  port

   format = video_port->format;

   if(state->camera_parameters.shutter_speed > 6000000)
   {
      MMAL_PARAMETER_FPS_RANGE_T fps_range = {{MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},
         { 50, 1000 }, {166, 1000}
      };
      mmal_port_parameter_set(video_port, &fps_range.hdr);
   }
   else if(state->camera_parameters.shutter_speed > 1000000)
   {
      MMAL_PARAMETER_FPS_RANGE_T fps_range = {{MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},
         { 167, 1000 }, {999, 1000}
      };
      mmal_port_parameter_set(video_port, &fps_range.hdr);
   }

   if (state->useRGB)
   {
      format->encoding = mmal_util_rgb_order_fixed(still_port) ? MMAL_ENCODING_RGB24 : MMAL_ENCODING_BGR24;
      format->encoding_variant = 0;  //Irrelevant when not in opaque mode
   }
   else
   {
      format->encoding = MMAL_ENCODING_I420;
      format->encoding_variant = MMAL_ENCODING_I420;
   }

   format->es->video.width = VCOS_ALIGN_UP(state->common_settings.width, 32);
   format->es->video.height = VCOS_ALIGN_UP(state->common_settings.height, 16);
   format->es->video.crop.x = 0;
   format->es->video.crop.y = 0;
   format->es->video.crop.width = state->common_settings.width;
   format->es->video.crop.height = state->common_settings.height;
   format->es->video.frame_rate.num = state->framerate;
   format->es->video.frame_rate.den = VIDEO_FRAME_RATE_DEN;

   status = mmal_port_format_commit(video_port);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("camera video format couldn't be set");
      goto error;
   }

   // Ensure there are enough buffers to avoid dropping frames
   if (video_port->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
      video_port->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;

   status = mmal_port_parameter_set_boolean(video_port, MMAL_PARAMETER_ZERO_COPY, MMAL_TRUE);
   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("Failed to select zero copy");
      goto error;
   }

   // Set the encode format on the still  port

   format = still_port->format;

   format->encoding = MMAL_ENCODING_OPAQUE;
   format->encoding_variant = MMAL_ENCODING_I420;

   format->es->video.width = VCOS_ALIGN_UP(state->common_settings.width, 32);
   format->es->video.height = VCOS_ALIGN_UP(state->common_settings.height, 16);
   format->es->video.crop.x = 0;
   format->es->video.crop.y = 0;
   format->es->video.crop.width = state->common_settings.width;
   format->es->video.crop.height = state->common_settings.height;
   format->es->video.frame_rate.num = 0;
   format->es->video.frame_rate.den = 1;

   status = mmal_port_format_commit(still_port);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("camera still format couldn't be set");
      goto error;
   }

   /* Ensure there are enough buffers to avoid dropping frames */
   if (still_port->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
      still_port->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;

   /* Enable component */
   status = mmal_component_enable(camera);

   if (status != MMAL_SUCCESS)
   {
      vcos_log_error("camera component couldn't be enabled");
      goto error;
   }

   raspicamcontrol_set_all_parameters(camera, &state->camera_parameters);

   /* Create pool of buffer headers for the output port to consume */
   pool = mmal_port_pool_create(video_port, video_port->buffer_num, video_port->buffer_size);

   if (!pool)
   {
      vcos_log_error("Failed to create buffer header pool for camera still port %s", still_port->name);
   }

   state->camera_pool = pool;
   state->camera_component = camera;

   if (state->common_settings.verbose)
      fprintf(stderr, "Camera component done\n");

   return status;

error:

   if (camera)
      mmal_component_destroy(camera);

   return status;
}

/**
 * Destroy the camera component
 *
 * @param state Pointer to state control struct
 *
 */
static void destroy_camera_component(RASPIVIDYUV_STATE *state)
{
   if (state->camera_component)
   {
      mmal_component_destroy(state->camera_component);
      state->camera_component = NULL;
   }
}

/**
 * Pause for specified time, but return early if detect an abort request
 *
 * @param state Pointer to state control struct
 * @param pause Time in ms to pause
 * @param callback Struct contain an abort flag tested for early termination
 *
 */
static int pause_and_test_abort(RASPIVIDYUV_STATE *state, int pause)
{
   int wait;

   if (!pause)
      return 0;

   // Going to check every ABORT_INTERVAL milliseconds
   for (wait = 0; wait < pause; wait+= ABORT_INTERVAL)
   {
      vcos_sleep(ABORT_INTERVAL);
      if (state->callback_data.abort)
         return 1;
   }

   return 0;
}


/**
 * Function to wait in various ways (depending on settings)
 *
 * @param state Pointer to the state data
 *
 * @return !0 if to continue, 0 if reached end of run
 */
static int wait_for_next_change(RASPIVIDYUV_STATE *state)
{
   int keep_running = 1;
   static int64_t complete_time = -1;

   // Have we actually exceeded our timeout?
   int64_t current_time =  get_microseconds64()/1000;

   if (complete_time == -1)
      complete_time =  current_time + state->timeout;

   // if we have run out of time, flag we need to exit
   if (current_time >= complete_time && state->timeout != 0)
      keep_running = 0;

   switch (state->waitMethod)
   {
   case WAIT_METHOD_NONE:
      (void)pause_and_test_abort(state, state->timeout);
      return 0;

   case WAIT_METHOD_FOREVER:
   {
      // We never return from this. Expect a ctrl-c to exit or abort.
      while (!state->callback_data.abort)
          // Have a sleep so we don't hog the CPU.
         vcos_sleep(ABORT_INTERVAL);

      return 0;
   }

   case WAIT_METHOD_TIMED:
   {
      int abort;

      if (state->bCapturing)
         abort = pause_and_test_abort(state, state->onTime);
      else
         abort = pause_and_test_abort(state, state->offTime);

      if (abort)
         return 0;
      else
         return keep_running;
   }

   case WAIT_METHOD_SIGNAL:
   {
      // Need to wait for a SIGUSR1 signal
      sigset_t waitset;
      int sig;
      int result = 0;

      sigemptyset( &waitset );
      sigaddset( &waitset, SIGUSR1 );

      // We are multi threaded because we use mmal, so need to use the pthread
      // variant of procmask to block SIGUSR1 so we can wait on it.
      pthread_sigmask( SIG_BLOCK, &waitset, NULL );

      if (state->common_settings.verbose)
      {
         fprintf(stderr, "Waiting for SIGUSR1 to %s\n", state->bCapturing ? "pause" : "capture");
      }

      result = sigwait( &waitset, &sig );

      if (state->common_settings.verbose && result != 0)
         fprintf(stderr, "Bad signal received - error %d\n", errno);

      return keep_running;
   }

   } // switch

   return keep_running;
}

/**
 * main
 */
int main(int argc, const char **argv)
{
   // Our main data storage vessel..
   RASPIVIDYUV_STATE state;
   int exit_code = EX_OK;

   MMAL_STATUS_T status = MMAL_SUCCESS;
   MMAL_PORT_T *camera_preview_port = NULL;
   MMAL_PORT_T *camera_video_port = NULL;
   MMAL_PORT_T *camera_still_port = NULL;
   MMAL_PORT_T *preview_input_port = NULL;

   bcm_host_init();

   ////////////////////////
   // modifications to main
   printf("%s, %s\n", argv[0], pinet_lib_version());
   s_main_tid = pthread_self();
   // create interface for synchronizing and sending data
   create_sync_receiver();
   launch_comm_thread();
   register_camera();
   // end modifications
   ////////////////////////


   // Register our application with the logging system
   vcos_log_register("RaspiVid", VCOS_LOG_CATEGORY);

   signal(SIGINT, default_signal_handler);

   // Disable USR1 for the moment - may be reenabled if go in to signal capture mode
   signal(SIGUSR1, SIG_IGN);

   set_app_name(argv[0]);

   default_status(&state);

   // Parse the command line and put options in to our status structure
   if (parse_cmdline(argc, argv, &state))
   {
      status = -1;
      exit(EX_USAGE);
   }

   if (state.common_settings.verbose)
   {
      print_app_details(stderr);
      dump_status(&state);
   }

   if (state.common_settings.gps)
      if (raspi_gps_setup(state.common_settings.verbose))
         state.common_settings.gps = false;

   // OK, we have a nice set of parameters. Now set up our components
   // We have two components. Camera, Preview

   if ((status = create_camera_component(&state)) != MMAL_SUCCESS)
   {
      vcos_log_error("%s: Failed to create camera component", __func__);
      exit_code = EX_SOFTWARE;
   }
   else if ((status = raspipreview_create(&state.preview_parameters)) != MMAL_SUCCESS)
   {
      vcos_log_error("%s: Failed to create preview component", __func__);
      destroy_camera_component(&state);
      exit_code = EX_SOFTWARE;
   }
   else
   {
      if (state.common_settings.verbose)
         fprintf(stderr, "Starting component connection stage\n");

      camera_preview_port = state.camera_component->output[MMAL_CAMERA_PREVIEW_PORT];
      camera_video_port   = state.camera_component->output[MMAL_CAMERA_VIDEO_PORT];
      s_camera_video_port = camera_video_port;
      camera_still_port   = state.camera_component->output[MMAL_CAMERA_CAPTURE_PORT];
      preview_input_port  = state.preview_parameters.preview_component->input[0];

      if (state.preview_parameters.wantPreview )
      {
         if (state.common_settings.verbose)
         {
            fprintf(stderr, "Connecting camera preview port to preview input port\n");
            fprintf(stderr, "Starting video preview\n");
         }

         // Connect camera to preview
         status = connect_ports(camera_preview_port, preview_input_port, &state.preview_connection);

         if (status != MMAL_SUCCESS)
            state.preview_connection = NULL;
      }
      else
      {
         status = MMAL_SUCCESS;
      }

      if (status == MMAL_SUCCESS)
      {
         // Set up our userdata - this is passed though to the callback where we need the information.
         state.callback_data.pstate = &state;

         printf("Establishing connection for image stream\n");
         struct network_id net_id;
         int connfd;
         if (resolve_sensor_endpoint("camera_endpoint", &net_id) != 0) {
            fprintf(stderr, "Unable to find network target\n");
            goto error;
         }
         printf("Network target is %s:%d\n", net_id.ip, net_id.port);
         connfd = connect_to_server(&net_id);
//printf("connected to server\n");
         if (connfd < 0) {
            vcos_log_error("%s: Error opening network connection for "
                  "camera\n", __func__);
            vcos_log_error("Tried to connect to %s::%d\n", 
                  net_id.ip, net_id.port);
            goto error;
         }
         // send connection type, to make sure that receiver is OK with
         //    this data source
         {
            int32_t magic = htonl(VY_STREAM_ID);
            int32_t response;
            if (send_block(connfd, &magic, sizeof(magic)) < 0) {
               fprintf(stderr, "Error sending magic connection number");
               goto error;
            }
            if (recv_block(connfd, &response, sizeof(response)) < 0) {
               fprintf(stderr, "Error receiving connection handshake\n");
               goto error;
            }
            response = htonl(response);
            if (response != HANDSHAKE_OK) {
               fprintf(stderr, "Failed communication handshake\n");
               fprintf(stderr, "Received 0x%08x\n", response);
               goto error;
            }
         }
         comm_set_socket(connfd);
         printf("Comm socket set\n");
         //state.callback_data.connfd = connfd;

         state.callback_data.abort = 0;

         camera_video_port->userdata = (struct MMAL_PORT_USERDATA_T *)&state.callback_data;

         if (state.common_settings.verbose)
            fprintf(stderr, "Enabling camera video port\n");

         // Enable the camera video port and tell it its callback function
         status = mmal_port_enable(camera_video_port, camera_buffer_callback);

         if (status != MMAL_SUCCESS)
         {
            vcos_log_error("Failed to setup camera output");
            goto error;
         }


         if (state.demoMode)
         {
            // Run for the user specific time..
            int num_iterations = state.timeout / state.demoInterval;
            int i;

            if (state.common_settings.verbose)
               fprintf(stderr, "Running in demo mode\n");

            for (i=0; state.timeout == 0 || i<num_iterations; i++)
            {
               raspicamcontrol_cycle_test(state.camera_component);
               vcos_sleep(state.demoInterval);
            }
         }
         else
         {
            int running = 1;

            // Send all the buffers to the camera video port
            {
               int num = mmal_queue_length(state.camera_pool->queue);
               int q;
               for (q=0; q<num; q++)
               {
                  MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(state.camera_pool->queue);

                  if (!buffer)
                     vcos_log_error("Unable to get a required buffer %d from pool queue", q);

                  if (mmal_port_send_buffer(camera_video_port, buffer)!= MMAL_SUCCESS)
                     vcos_log_error("Unable to send a buffer to camera video port (%d)", q);
               }
            }

            while (running)
            {
               if (MESSAGE_BOARD.exit) {
                  state.bCapturing = 0;
                  mmal_port_parameter_set_boolean(camera_video_port, 
                        MMAL_PARAMETER_CAPTURE, state.bCapturing);
                  break;
               } else if (MESSAGE_BOARD.acquiring || !MESSAGE_BOARD.paused) {
                  state.bCapturing = 1;
                  s_frame_pending = 1;
                  s_frame_start = now();
                  mmal_port_parameter_set_boolean(camera_video_port, 
                        MMAL_PARAMETER_CAPTURE, state.bCapturing);
               }

               if (state.common_settings.verbose)
               {
                  if (state.bCapturing)
                     fprintf(stderr, "Starting video capture\n");
                  else
                     fprintf(stderr, "Pausing video capture\n");
               }

               running = wait_for_next_change(&state);
               if ((s_shutdown != 0) || MESSAGE_BOARD.exit)
                  running = 0;
            }

            if (state.common_settings.verbose)
               fprintf(stderr, "Finished capture\n");
         }
      }
      else
      {
         mmal_status_to_int(status);
         vcos_log_error("%s: Failed to connect camera to preview", __func__);
      }

error:

      mmal_status_to_int(status);

      if (state.common_settings.verbose)
         fprintf(stderr, "Closing down\n");

      // Disable all our ports that are not handled by connections
      check_disable_port(camera_video_port);

      if (state.preview_parameters.wantPreview && state.preview_connection)
         mmal_connection_destroy(state.preview_connection);

      if (state.preview_parameters.preview_component)
         mmal_component_disable(state.preview_parameters.preview_component);

      if (state.camera_component)
         mmal_component_disable(state.camera_component);

      raspipreview_destroy(&state.preview_parameters);
      destroy_camera_component(&state);

      if (state.common_settings.gps)
         raspi_gps_shutdown(state.common_settings.verbose);

      if (state.common_settings.verbose)
         fprintf(stderr, "Close down completed, all components disconnected, disabled and destroyed\n\n");
   }

   if (status != MMAL_SUCCESS)
      raspicamcontrol_check_configuration(128);

   ////////////////////////
   // modifications to main
   shutdown_comm();
   shutdown_sync_receiver();
   ////////////////////////

   return exit_code;
}


