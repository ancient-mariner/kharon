#include "pin_types.h"
#include <stdio.h>
#include <assert.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <dirent.h>
#include "pinet.h"
#include "time_lib.h"
#include "sens_net.h"
#include "sens_db.h"
#include "dev_info.h"
#include "image.h"
#include "timekeeper.h"


////////////////////////////////////////////////////////////////////////
// static variables and housekeeping

char imu_log_[256];
char gps_log_[256];

char camera_log_[256];
char frame_dir_[256];

char device_name_[256];

int imu_sock_fd_ = -1;
int cam_sock_fd_ = -1;
int gps_sock_fd_ = -1;

FILE * imu_log_fp_ = NULL;
FILE * gps_log_fp_ = NULL;

// quite flag -- program exits when quit signal received
static volatile int quit_ = 0;

// try to exit gracefully (whether from signal or explicit exit)
// SIGUSR1, SIGINT, SIGPIPE
static void signal_soft_exit(int signum)
{
   (void) signum;
   quit_ = 1;
}

// offset between system clock and timestamps in data stream
static double t_delta_ = 0.0;

static int streaming_ = 0;

// SIGUSR2
static void signal_start_streaming(int signum)
{
   if (signum != SIGUSR2) {
      fprintf(stderr, "Internal error -- start_streaming received signal %d. "
            "Expected SIGUSR2 (%d)\n", signum, SIGUSR2);
      exit(1);
   }
   streaming_ = 1;
}


// static variables and housekeeping
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// initialization and setup



static void usage(const char *arg0)
{
   printf("Usage: %s -d <name> [-i <filename>]  [-c <filename>] "
         "[-g <filename>]\n", arg0);
   printf("\n");
   printf("where:\n");
   printf("    -d    name of device to emulate\n");
   printf("    -i    name of file with IMU data\n");
   printf("    -c    name of file listing image frame files\n");
   printf("          first line of file is directory. subsequent lines\n");
   printf("          are file names\n");
   printf("    -g    name of file with GPS data\n");
   printf("    -t    time to set clock\n");
   printf("    -h    help (prints this message)\n");
   printf("\n");
   printf("To trigger emulator, run 'em_start.sh'\n");
   printf("To stop emulator, run 'em_kill.sh'\n");
   printf("\n");
   exit(1);
}


static void parse_command_line(int argc, char *argv[])
{
   int opt;
   imu_log_[0] = 0;
   camera_log_[0] = 0;
   device_name_[0] = 0;
   while ((opt = getopt(argc, argv, "c:d:g:i:t:h")) != -1) {
      switch (opt) {
         case 'd':
            {
               if (device_name_[0]) {
                  usage(argv[0]);
               }
               const char * name = optarg;
               strcpy(device_name_, name);
            }
            break;
         case 'c':
            {
               if (camera_log_[0]) {
                  usage(argv[0]);
               }
               const char * name = optarg;
               strcpy(camera_log_, name);
            }
            break;
         case 'g':
            {
               if (gps_log_[0]) {
                  usage(argv[0]);
               }
               const char * name = optarg;
               strcpy(gps_log_, name);
            }
            break;
         case 'i':
            {
               if (imu_log_[0]) {
                  usage(argv[0]);
               }
               const char * name = optarg;
               strcpy(imu_log_, name);
            }
            break;
         case 't':
            {
               if (t_delta_ != 0.0) {
                  usage(argv[0]);
               }
               // calculate offset between system clock and streamed data
               const char * name = optarg;
               double stream_t0 = strtod(name, NULL);
               if (errno != 0) {
                  const char *err = strerror(errno);
                  printf("%s. Error parsing time '%s'\n", err, name);
                  usage(argv[0]);
               }
               struct timespec start_time;
               clock_gettime(CLOCK_MONOTONIC, &start_time);
               double base_time = timespec_to_double(&start_time);
               t_delta_ = base_time - stream_t0;
               printf("t-delta = %f\n", t_delta_);
            }
            break;
         case 'h':
            usage(argv[0]);
            break;
         //case '1':
         //case '?':
         //case ':':
         default:
            usage(argv[0]);
      }
   }
   // need at least one of camera, imu or gps logs
   if ((camera_log_[0] == 0) && (imu_log_[0] == 0) && (gps_log_[0] == 0)) {
      usage(argv[0]);
   }
   if (device_name_[0] == 0) {
      usage(argv[0]);
   }
   printf("image list: %s\n", camera_log_[0] ? camera_log_ : "-------");
   printf("imu data:   %s\n", imu_log_[0] ? imu_log_ : "-------");
   printf("gps data:   %s\n", gps_log_[0] ? gps_log_ : "-------");
   printf("device:     %s\n", device_name_[0] ? device_name_ : "-------");
   printf("----------\n");
}


static void initialize_consensus(
      /*    out */       consensus_sensor_type *cons
      )
{
   SET_VEC(&cons->acc, 0.0f);
   SET_VEC(&cons->mag, 0.0f);
   SET_VEC(&cons->gyr_axis, 0.0f);
   SET_VEC(&cons->latlon, 0.0f);
   cons->temp = 0.0f;
   cons->baro = 0.0f;
}

static void print_consensus(
      /* in     */ const consensus_sensor_type *cons
      )
{
   const vector_type *gyr = &cons->gyr_axis;
   const vector_type *acc = &cons->acc;
   const vector_type *mag = &cons->mag;
   printf("%6.2f,%6.2f,%6.2f,  ", 
         (double) gyr->v[0], (double) gyr->v[1], (double) gyr->v[2]);
   printf("%7.3f,%7.3f,%7.3f,  ", 
         (double) acc->v[0], (double) acc->v[1], (double) acc->v[2]);
   printf("%6.2f,%6.2f,%6.2f,  ", 
         (double) mag->v[0], (double) mag->v[1], (double) mag->v[2]);
   printf("%6.2f\n", (double) cons->temp);
}


////////////////////////////////////////////////////////////////////////

static int32_t base_initialization(
      /* in     */ const int32_t argc,
      /* in out */       char **argv
      )
{
   int32_t error_state = -1;
   parse_command_line(argc, argv);
//   if (read_config_info() != 0) {
//      goto done;
//   }
   //
   // try to gracefully exit after sigint or broken pipe
   signal(SIGINT, signal_soft_exit);
   signal(SIGPIPE, signal_soft_exit);
   signal(SIGUSR1, signal_soft_exit);
   signal(SIGUSR2, signal_start_streaming);
   struct network_id net_id;
   //
   if (imu_log_[0] != 0) {
      if ((resolve_sensor_endpoint_for_host("i2c_endpoint", device_name_,
                  &net_id) != 0) || (net_id.ip[0] == 0)) {
         fprintf(stderr, "Unable to load network target for i2c endpoint\n");
         goto done;
      }
      printf("Connecting to IMU endpoint %s:%d\n", net_id.ip, net_id.port);
      if ((imu_sock_fd_ = connect_to_server(&net_id)) < 0) {
         printf("Unable to connect to server\n");
         goto done;
      }
   }
   if (gps_log_[0] != 0) {
      if ((resolve_sensor_endpoint_for_host("gps_endpoint", device_name_,
                  &net_id) != 0) || (net_id.ip[0] == 0)) {
         fprintf(stderr, "Unable to load network target for gps endpoint\n");
         goto done;
      }
      printf("Connecting to GPS endpoint %s:%d\n", net_id.ip, net_id.port);
      if ((gps_sock_fd_ = connect_to_server(&net_id)) < 0) {
         printf("Unable to connect to server\n");
         goto done;
      }
   }
   if (camera_log_[0] != 0) {
      if ((resolve_sensor_endpoint_for_host("camera_endpoint", device_name_,
                  &net_id) != 0) || (net_id.ip[0] == 0)) {
         fprintf(stderr, "Unable to load network target for camera\n");
         goto done;
      }
      printf("Connecting to camera endpoint: %s:%d\n", net_id.ip, net_id.port);
      if ((cam_sock_fd_ = connect_to_server(&net_id)) < 0) {
         printf("Unable to connect to server\n");
         goto done;
      }
printf("Connected. Handshaking\n");
      // make sure receiver is OK with this data source
      uint32_t magic = htonl(VY_STREAM_ID);
      uint32_t response;
      if (send_block(cam_sock_fd_, &magic, sizeof(magic)) < 0) {
         fprintf(stderr, "Error sending magic connection number");
         goto done;
      }
      if (recv_block(cam_sock_fd_, &response, sizeof(response)) < 0) {
         fprintf(stderr, "Error receiving connection handshake\n");
         goto done;
      }
      response = htonl(response);
      if (response != HANDSHAKE_OK) {
         fprintf(stderr, "Failed communication handshake\n");
         fprintf(stderr, "Received 0x%08x\n", response);
         goto done;
      }
      printf("Connected to port: %d\n", net_id.port);
   }
   if ((imu_sock_fd_ < 0) && (cam_sock_fd_ < 0) && (gps_sock_fd_ < 0)) {
      printf("No IMU, GPS or camera endpoint connected. Bailing out\n");
      goto done;
   }
   error_state = 0;
done:
   return error_state;
}


static void app_shutdown(void)
{
   if (cam_sock_fd_ >= 0) {
      close(cam_sock_fd_);
   }
   if (imu_sock_fd_ >= 0) {
      close(imu_sock_fd_);
   }
   if (gps_sock_fd_ >= 0) {
      close(gps_sock_fd_);
   }
   if (imu_log_fp_ != NULL) {
      fclose(imu_log_fp_);
   }
   if (gps_log_fp_ != NULL) {
      fclose(gps_log_fp_);
   }
}

//
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// image data

image_type *frame_image_ = NULL;

// loads next IMU sample from IMU log file and returns timestamp of sample
static double load_next_image_frame(void)
{
   static FILE *fp = NULL;
   double t = -1.0;
   char *lineptr = NULL;
   size_t ptr_size = 0;
   int32_t line_num = 0;
   // open catalog file
   // first line of file is storage directory of image files -- read that
   // subsequent lines are image file names
   if ((fp == NULL) && (camera_log_[0] != 0)) {
      printf("Opening camera log %s\n", camera_log_);
      fp = fopen(camera_log_, "r");
      if (!fp) {
         printf("Failed to open input log file\n");
      }
      camera_log_[0] = 0;
      // first line of file is storage directory
      if (getline(&lineptr, &ptr_size, fp) <= 0) {
         goto err;   // error or end of file -- either way, we're done
      }
      if (trim_whitespace(lineptr) == NULL) {
         fprintf(stderr, "Failed to read input directory from image "
               "file list\n");
         goto err;
      }
      strcpy(frame_dir_, lineptr);
      printf("Reading image frames from '%s'\n", frame_dir_);
   }
   if (fp) {
      // read next line and load that file
      if (getline(&lineptr, &ptr_size, fp) <= 0) {
         printf("End of image stream (%s)\n", device_name_);
         goto err;   // error or end of file -- either way we're done
      } 
      char *str = trim_whitespace(lineptr);
      if (str == NULL) {
         fprintf(stderr, "Encountered empty line in input image file\n");
         goto err;
      }
printf("Loading image %s\n", str);
      // parse time from filename (time is filename)
      t = strtod(str, NULL);
      if (errno != 0) {
         fprintf(stderr, "Error parsing time from filename: %s\n", str);
         fprintf(stderr, "Expected <xxxx.xxx>.pnm\n");
         goto err;
      }
      char path[256];
      sprintf(path, "%s%s", frame_dir_, str);
      frame_image_ = create_image_pgm(path);
      if (!frame_image_) {
         fprintf(stderr, "Failed to open image file '%s'\n", path);
         goto err;
      }
   }
   goto done;
err:
   printf("Read %d lines from the IMU log\n", line_num);
   fclose(fp);
   fp = NULL;
   t = -1.0;
done:
   return t;
}


static int32_t send_image(double t)
{
   int32_t rc = -1;
   // sanity checks
   if (frame_image_ == NULL) {
      fprintf(stderr, "No image loaded -- cannot send\n");
      goto err;
   }
   uint32_t n_pix = (uint32_t) (frame_image_->size.x * frame_image_->size.y);
   if (CAM_N_PIX != n_pix/2) {
      fprintf(stderr, "Input image of unexpected size\n");
      fprintf(stderr, "Got %d,%d. Expected %d,%d\n", frame_image_->size.x, 
            frame_image_->size.y, CAM_COLS, CAM_ROWS);
      goto err;
   }
   // create header packet
   struct sensor_packet_header header;
   serialize_sensor_header2(VY_PACKET_TYPE, t, t, &header);
   header.custom_16[0] = (int16_t) htons(CAM_ROWS);
   header.custom_16[1] = (int16_t) htons(CAM_COLS);
   // send header
   if (send_block(cam_sock_fd_, &header, sizeof(header)) < 0) {
      fprintf(stderr, "Network error sending header packet\n");
      goto err;
   }
   // send V channel -- this should be first 1/2 of buffer
   if (send_block(cam_sock_fd_, frame_image_->gray, n_pix/2) < 0) {
      fprintf(stderr, "Error sending part 1 of VY image frame\n"); 
      goto err;
   }
   // send Y channel -- this should be 2nd half of buffer
   if (send_block(cam_sock_fd_, &frame_image_->gray[n_pix/2], n_pix/2) < 0) {
      fprintf(stderr, "Error sending part 2 of VY image frame\n"); 
      goto err;
   }
   rc = 0;
err:
   free_image(frame_image_);
   frame_image_ = NULL;
   return rc;
}

// image data
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// IMU data

// loads next IMU sample from IMU log file and returns timestamp of sample
static double load_next_imu_sample(
      /* in out */       consensus_sensor_type *consensus
      )
{
   static char *lineptr = NULL;
   static size_t ptr_size = 0;
   double t = -1.0;
   int32_t line_num = 0;
   //
   if ((imu_log_fp_ == NULL) && (imu_log_[0] != 0)) {
      printf("Opening imu log %s\n", imu_log_);
      imu_log_fp_ = fopen(imu_log_, "r");
      if (!imu_log_fp_) {
         printf("Failed to open input log file\n");
      }
      imu_log_[0] = 0;
   }
   if (imu_log_fp_) {
      // some logs have empty lines -- loop until actual data read
      while (t <= 0.0) {
         // read next line and parse it
         // close file when reading complete
         if (getline(&lineptr, &ptr_size, imu_log_fp_) <= 0) {
            goto err;   // error or end of file -- either way, we're done
         } else {
            char *str = lineptr;
            line_num++;
            t = strtod(str, &str);
            int err = 0;
            errno = 0;
            consensus->gyr_axis.v[0] = strtod(str, &str);
            err |= errno;
            consensus->gyr_axis.v[1] = strtod(str, &str);
            err |= errno;
            consensus->gyr_axis.v[2] = strtod(str, &str);
            err |= errno;
            consensus->acc.v[0] = strtod(str, &str);
            err |= errno;
            consensus->acc.v[1] = strtod(str, &str);
            err |= errno;
            consensus->acc.v[2] = strtod(str, &str);
            err |= errno;
            consensus->mag.v[0] = strtod(str, &str);
            err |= errno;
            consensus->mag.v[1] = strtod(str, &str);
            err |= errno;
            consensus->mag.v[2] = strtod(str, &str);
            err |= errno;
            consensus->temp = strtod(str, &str);
            err |= errno;
            if (err != 0) {
               printf("Parse error on line %d in IMU log\n", line_num);
               goto err;
            }
            // in spring 2019 the log format changed. it used to have
            //    <received timestamp>  <data timestamp>  ...
            // the received timestamp was dropped. check to see if we might
            //    be reading an old file though -- bad things can happen
            //    if so
            if (fabs(t - (double) consensus->gyr_axis.v[0]) < 1.0) {
               fprintf(stderr, "Suspected data error. Detected what "
                     "appear to be 2 timestamps in IMU log file. This "
                     "format is no longer supported. Please delete the "
                     "'received' timestamp from the file and rerun\n");
               goto err;
            }
         }
      }
   }
   goto done;
   print_consensus(consensus);   // unreachable (avoids compile warning)
err:
   printf("Read %d lines from the IMU log\n", line_num);
   fclose(imu_log_fp_);
   imu_log_fp_ = NULL;
   t = -1.0;
done:
   return t;
}

// IMU data
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// GPS data

// loads next GPS packet from log file and returns timestamp of sample
// content of packet (with timestamp stripped) is returned in 'data'
static double load_next_gps_sample(
      /* in out */       char *data
      )
{
   static char *lineptr = NULL;
   static size_t ptr_size = 0;
   double t = -1.0;
   int32_t line_num = 0;
   //
   if ((gps_log_fp_ == NULL) && (gps_log_[0] != 0)) {
      printf("Opening gps log %s\n", gps_log_);
      gps_log_fp_ = fopen(gps_log_, "r");
      if (!gps_log_fp_) {
         printf("Failed to open input log file\n");
      }
      gps_log_[0] = 0;
   }
   if (gps_log_fp_) {
      // some logs have empty lines -- loop until actual data read
      while (t <= 0.0) {
         // read next line and parse it
         // close file when reading complete
         if (getline(&lineptr, &ptr_size, gps_log_fp_) <= 0) {
            goto err;   // error or end of file -- either way, we're done
         } else {
            char *str = lineptr;
            line_num++;
            t = strtod(str, &str);
            // remove white space between timestamp and nmea sentence
            while (*str == ' ') {
               str++;
            }
            // copy sentenct to buffer
            strcpy(data, str);
         }
      }
   }
   goto done;
err:
   printf("Read %d lines from the GPS log\n", line_num);
   if (gps_log_fp_ != NULL) {
      fclose(gps_log_fp_);
   }
   gps_log_fp_ = NULL;
   t = -1.0;
done:
   return t;
}

// send GPS_BLOCK_SIZE worth of data -- timestamp followed by NMEA sentence
static int32_t send_gps_sentence(
      /* in     */ const char *sentence,
      /* in     */ const double t
      )
{
   int32_t rc = -1;
   char buf[GPS_BLOCK_SIZE];
   memset(buf, 0, sizeof(buf));
   // write timestamp
   int n = sprintf(buf, "%.3f", t);
   if ((n < 0) || (n > 40)) {
      // wtf? pigs must fly
      fprintf(stderr, "Unexpected value (%d) returned from sprintf for "
            "time %.3f\n", n, t);
      goto err;
   }
   // convert termination char from 0 to ' '
   buf[n++] = ' ';
   n += sprintf(&buf[n], "%s", sentence);
printf("Sending GPS data '%s'\n", buf);
   if (send_block(gps_sock_fd_, buf, sizeof(buf)) < 0) {
      fprintf(stderr, "Network error sending gps packet\n");
      goto err;
   }
   rc = 0;
err:
   return rc;
}


// GPS data
////////////////////////////////////////////////////////////////////////

// find earliest time that's greater than zero. return this time with
//    an indication of the source (imu, cam or gps)
// which=0 for imu, 1 for cam, 2 for gps
static double next_timestamp(
      /* in     */ const double t_imu,
      /* in     */ const double t_cam,
      /* in     */ const double t_gps,
      /*    out */       int32_t *which
      )
{
   // convert -1 to very high number so we can simplify search to finding
   //    earliest timestamp
   const double LONG_TIME = 1.0e100;
   double imu = t_imu < 0.0 ? LONG_TIME : t_imu;
   double cam = t_cam < 0.0 ? LONG_TIME : t_cam;
   double gps = t_gps < 0.0 ? LONG_TIME : t_gps;
   // assume imu is best -- see if anyone can beat it
   int32_t best = 0;
   double t = imu;
   if (cam < t) {
      // cam is earlier than imu -- reset to that
      best = 1;
      t = cam;
   }
   if (gps < t) {
      // gps is earlier than either cam or imu -- reset to that
      best = 2;
      t = gps;
   }
   *which = best;
   // if all times are set to very large value, no data is available
   //    so reset t to -1
   if (t == LONG_TIME) {
      t = -1.0;
   }
//printf("   %.3f   %.3f   %.3f  -> %.3f  %d\n", t_imu, t_cam, t_gps, t, *which);
   return t;
}


static int32_t acquisition_loop(void)
{
   int32_t error_state = -1;
   /////////////////////////////////////////////////////////////
   // data acquisition loop
   // structure where sensor data is collected for broadcast ('consensus
   //    data')
   consensus_sensor_type consensus;
   char gps_data[256];
   initialize_consensus(&consensus);
   //
   struct timespec start_time;
   clock_gettime(CLOCK_MONOTONIC, &start_time);
   double wake_time = system_now() + 0.1; // default to short nap
   //
//printf("in loop\n");
   double t_imu = load_next_imu_sample(&consensus);
   consensus.state.flags = 0;
   consensus.state.avail[IMU_ACC] = 1;
   consensus.state.avail[IMU_MAG] = 1;
   consensus.state.avail[IMU_GYR] = 1;
   double t_cam = load_next_image_frame();
   double t_gps = load_next_gps_sample(gps_data);
   int32_t which;
   double t_next = next_timestamp(t_imu, t_cam, t_gps, &which);
   //
   while ((t_next >= 0.0) && (quit_ == 0)) {
      // acquired data on different clock than system right now.
      // apply time delta to data timestamps before sending
      //////////////////////////////////////////////////////////////////
      // perform next action
      if (streaming_ != 0) {
         if (which == 0) {
//printf("send bcast timestamp\n");
            consensus.log_data[0] = 0;
//print_consensus(&consensus);
	         if (send_broadcast_timestamp(imu_sock_fd_, t_imu + t_delta_, &consensus) < 0) {
               fprintf(stderr, "Failed to send broadcast of concensus data\n");
               break;   // network failure. exit and let supervisor restart
            }
//printf("load next imu sample\n");
            t_imu = load_next_imu_sample(&consensus);
         } else if (which == 1) {
            // load and send images
            if (send_image(t_cam + t_delta_) != 0) {
               // send error occurred. bitch and stop (server may be in
               //    unstable state)
               fprintf(stderr, "Failure sending image. Assuming the worst "
                     "and bailing out\n");
               break;
            }
//printf("load next image\n");
            t_cam = load_next_image_frame();
         } else if (which == 2) {
            // send gps data
            if (send_gps_sentence(gps_data, t_gps + t_delta_) != 0) {
               // send error occurred. bitch and stop (server may be in
               //    unstable state)
               fprintf(stderr, "Failure sending GPS data. Assuming the worst "
                     "and bailing out\n");
               break;
            }
//printf("load next position\n");
            t_gps = load_next_gps_sample(gps_data);
         } else {
            // internal error
            fprintf(stderr, "Unknown source with earliest timestamp (%d)\n",
                  which);
            break;
         }
         t_next = next_timestamp(t_imu, t_cam, t_gps, &which);
         if (t_next < 0.0) {
            printf("No more data to send\n");
            break;
         }
         wake_time = t_next + t_delta_;
//printf("Waking at %f (%f, now=%f)\n", t_next, wake_time, system_now());
      } else {
         wake_time = system_now() + 0.1;
      }
      //////////////////////////////////////////////////////////////////
      // 
      struct timespec t;
      double_to_timespec(wake_time, &t);
      while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL) != 0) {
         if (quit_ != 0) {
            printf("Quit: %d\n", quit_);
            break;
         }
         if ((errno != 0) && (errno != EINTR)) {
            perror("Error in nanosleep");
            goto err;
         }
      }
   }
   error_state = 0;
err:
printf("done\n");
   return error_state;
}


// initialization and setup
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////


int main(int argc, char **argv)
{
   printf("Sensor simulation process launching\n");
   set_device_dir_path(PI_DEV_ROOT);
   // initialize components and establish link to server, if appropraite
   int32_t error_state = base_initialization(argc, argv);
   if (error_state != 0) {
      goto end;
   }
   // main loop
   error_state = acquisition_loop();
end:
   // system shutdown
   app_shutdown();
   // recast error state as int
   return (int) error_state;
}

