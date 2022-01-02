#include "s2.h"
#include <stdio.h>
#include <assert.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <sys/types.h>
#include <dirent.h>
#include "pinet.h"
#include "logger.h"
#include "udp_sync_receiver.h"
#include "time_lib.h"
#include "sens_db.h"
#include "sens_net.h"
#include "sens_lib.h"
#include "dev_info.h"
#include "lin_alg.h"
#include "build_version_sensor.h"

#include "timekeeper.h"

#include "hardware/lsm9ds0.h"
#include "hardware/lsm303.h"
#include "hardware/l3g.h"
#include "hardware/bmg160.h"
#include "hardware/hmc6343.h"
#include "hardware/lis3mdl.h"
#include "hardware/lis3dh.h"
#include "hardware/a3g4250d.h"
#include "hardware/mag3110.h"


// GYR sensors should sample at 12.5ms intervals
// ACC and MAG sensors should sample at 25ms intervals, or at a larger
//    multiple of 12.5. to ease traffic on the I2C bus, sensor reads
//    can be offset relative to each other. by convention, if an 
//    offset is used, run the ACC on multiples of 25ms intervals, and
//    MAG delayed 12.5ms from this
// GPS assumed to sample at 1sec intervals, or at other multiple of
//    12.5ms


const char *HARDWARE_LIST[] = {
         HARDWARE_NAME_LSM9DS0,
         HARDWARE_NAME_LSM303,
         HARDWARE_NAME_L3G,
         HARDWARE_NAME_BMG160,
         HARDWARE_NAME_HMC6343,
         HARDWARE_NAME_LIS3MDL,
         HARDWARE_NAME_LIS3DH,
         HARDWARE_NAME_A3G4250D,
         HARDWARE_NAME_MAG3110,
         NULL };

static log_info_type *log_ = NULL;

////////////////////////////////////////////////////////////////////////
// static variables and housekeeping

char *outfile_ = NULL;
FILE *outfp_ = NULL;

static sensor_runtime_type sensor_stack_[MAX_SENSORS];

// flag to indicate if broadcasting data on network (1) or not (0). default on
static uint32_t bcast_ = 1;

//// flag to indicate if log data should go to stdout/stderr or to log files
////    under /pinet/data/<date>/
//static uint32_t use_stdout_ = 0;

// quite flag -- program exits when quit signal received
static volatile int quit_ = 0;

// if time is passed in via command line, use that as master and override
//    time negotiated with server
static double master_time_ = -1.0;
static double process_clock_ = -1.0;

// buffer to store data that's destined for kernel log
static char log_data_[SENSOR_PACKET_LOG_DATA];

// try to exit gracefully (whether from signal or explicit exit)
void signal_soft_exit(int);

void signal_soft_exit(int unused)
{
   (void) unused;
   quit_ = 1;
}

static void initialize_static_memory(void)
{
   memset(sensor_stack_, 0, MAX_SENSORS * sizeof *sensor_stack_);
   for (uint32_t i=0; i<MAX_SENSORS; i++) {
      // mark all as inactive. fill array from beginning and overwrite
      //    inactive flag. if we encounter flag when iterating through
      //    list then there are no valid/active sensors left
      sensor_stack_[i].flags = SENSOR_FLAG_INACTIVE;
   }
}

static void sanity_check(void)
{
   // sensors set up to use little-endian for data transfer
   // make sure that this is the case
   union {
      uint8_t c[4];
      int32_t i;
   } endian;
   endian.i = 1;
   assert(endian.c[0] == 1);
   assert(endian.c[1] == 0);
   assert(endian.c[2] == 0);
   assert(endian.c[3] == 0);
}


// static variables and housekeeping
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// misc functions

static void report_timing_error(
      /* in     */ const struct timespec *now
      )
{
   // for dev and testing, consider this a fatal error
   // use dev/testing results to decide what to do in future.
   //    eg, is restarting sensor app appropriate (hard fail) or
   //    should situation only be logged and ignored
   log_err(log_, "TIMING ERROR -- waketime is in the past");
   log_err(log_, "NOW: sec=%ld  nsec=%ld", now->tv_sec, now->tv_nsec);
   snprintf(log_data_, SENSOR_PACKET_LOG_DATA, 
         "Waketime in the past: sec=%ld  nsec=%ld", now->tv_sec, now->tv_nsec);
   for (uint32_t i=0; i<MAX_SENSORS; i++) {
      const uint32_t flags = sensor_stack_[i].flags;
      if (flags & SENSOR_FLAG_INACTIVE)
         break;
      if (flags & SENSOR_FLAG_DISABLED)
         continue;
      log_err(log_, "%s:%s  sec=%ld  nsec=%ld", 
            sensor_stack_[i].device_addr,
            sensor_stack_[i].type_name,
            sensor_stack_[i].waketime.tv_sec,
            sensor_stack_[i].waketime.tv_nsec);
   }
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


// misc functions
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// initialization and setup

static void print_hardware_types(void)
{
   log_err(log_, "Valid hardware types:");
   const char **names = HARDWARE_LIST;
   for (uint32_t i=0; i<128; i++) {
      if (names[i] == NULL)
         break;
      log_err(log_, "    %s", names[i]);
   }
}


// read data from dev/nodes/sensors/i2c/
// use that to create sensors
static uint32_t read_config_info(void)
{
   uint32_t errs = 0;
   DIR *d = NULL;
   struct dirent *dir = NULL;
   char root[SENSOR_PATH_LEN];
   char path[SENSOR_PATH_LEN];
   char node[HOST_NAME_MAX];
   //
   if (gethostname(node, HOST_NAME_MAX) != 0) {
      perror("Error reading hostname");
      errs++;
      goto error;
   }
   sprintf(root, "%s%s/sensors/i2c/", PI_DEV_ROOT, node);
   uint32_t num_sensors = 0;
   // scan directory for list of sensors and create a new sensor record
   //    for each
   d = opendir(root);
   dir = NULL;
   if (!d) {
      log_err(log_, "No i2c sensors for '%s'. Check %s", node, root);
      errs++;
      goto error;
   }
   while ((dir = readdir(d)) != NULL) {
      if (dir->d_name[0] == '.')
         continue;
      // ignore entries with leading underscore (e.g., _readme.txt)
      if (dir->d_name[0] == '_') 
         continue;
      if (num_sensors >= MAX_SENSORS) {
         log_err(log_, "Too many sensors i2c specified (max=%d)",
               MAX_SENSORS);
         log_err(log_, "Check '%s'", path);
         log_err(log_, "If more sensors are required, alter MAX_SENSORS "
               "and recompile");
         errs++;
         goto error;
      }
      sensor_runtime_type *sensor = &sensor_stack_[num_sensors++];
      strncpy(sensor->name, dir->d_name, SENSOR_NAME_LEN);
      log_info(log_, "loading %s", dir->d_name);
      snprintf(sensor->config_root, SENSOR_PATH_LEN, "%s%s/", root, 
            dir->d_name);
      // get device address info
      FILE *fp;
      fp = open_config_file_ro2(PI_DEV_ROOT, NULL, "sensors/i2c/", dir->d_name, 
            "dev_addr");
      if (!fp) {
         log_err(log_, "Failed to open dev_addr for %s", dir->d_name);
         errs++;
         continue;
      }
      if (config_read_string(fp, sensor->device_addr, SENSOR_NAME_LEN) != 0) {
         log_err(log_, "Failed to read dev_addr in %s", path);
         errs++;
      }
      fclose(fp);
      // get name address info
      fp = open_config_file_ro2(PI_DEV_ROOT, NULL, "sensors/i2c/", 
            dir->d_name, "dev_name");
      if (!fp) {
         log_err(log_, "Failed to open dev_name for %s", dir->d_name);
         errs++;
         continue;
      }
      if (config_read_string(fp, sensor->type_name, SENSOR_NAME_LEN) != 0) {
         log_err(log_, "Failed to read dev_name in %s", path);
         errs++;
      }
      fclose(fp);
//      sprintf(path, "%s/dev_addr", sensor->config_root);
//      dev_read_config_line_fullpath(path, sensor->device_addr, SENSOR_NAME_LEN);
//      if (sensor->device_addr[0] == 0) {
//         log_err(log_, "Failed to read dev_addr in %s", path);
//         errs++;
//      }
//      // get name address info
//      sprintf(path, "%s/dev_name", sensor->config_root);
//      dev_read_config_line_fullpath(path, sensor->type_name, SENSOR_NAME_LEN);
//      if (sensor->type_name[0] == 0) {
//         log_err(log_, "Failed to read dev_name in %s", path);
//         errs++;
//      }
   }
error:
   log_info(log_, "Found %d devices for %s", num_sensors, node);
   if (errs > 0) {
      log_err(log_, "Encountered %d errors", errs);
   }
   if (d)
      closedir(d);
   return errs;
}

////////////////////////////////////////////////////////////////////////

static void usage(const char *arg0)
{
   printf("Usage: %s [-f <name>] -x\n", arg0);
   printf("\n");
   printf("where:\n");
   printf("    -f    name of file to write data to\n");
   printf("    -d    enable fast gyro drift detection/compensation\n");
   printf("    -l    activate logger on start (for debugging, this should be first flag)\n");
//   printf("    -s    print stderr/stdout to console (defaults to log)\n");
   printf("    -t    master clock in seconds; overrides network "
                    "negotiated time)\n");
   printf("    -x    inhibit network broadcast of data\n");
   printf("    -h    help (prints this message)\n");
   printf("\n");
   exit(1);
}


static void parse_command_line(int argc, char *argv[])
{
   // -d,-t must be paired to complete device definition
   //imu_device_type *dev = NULL;
   int opt;
   while ((opt = getopt(argc, argv, "df:t:xhl")) != -1) {
      switch (opt) {
         case 'd':
            enable_fast_gyro_drift_detection();
            break;
         case 'f':
            {
               if (outfile_)
                  usage(argv[0]);
               const char * name = optarg;
               outfile_ = malloc(strlen(name) + 1);
               strcpy(outfile_, name);
            }
            break;
         case 'h':
            usage(argv[0]);
            break;
         case 'l':
            log_ = get_logger("s2");
            break;
         case 't':
            {
               const char * name = optarg;
               master_time_ = strtod(name, NULL);
               if (errno != 0) {
                  log_err(log_, "Unable to parse time (use seconds)");
                  usage(argv[0]);
               }
            }
            break;
         case 'x':
            bcast_ = 0;
            break;
         //case '1':
         //case '?':
         //case ':':
         default:
            usage(argv[0]);
      }
   }
   // open out file, if it's been indicated
   if (outfile_) {
      if (strcmp(outfile_, "stdout") == 0) {
         outfp_ = stdout;
      } else {
         outfp_ = fopen(outfile_, "w");
         if (!outfp_) {
            log_err(log_, "Failed to open output file (%s)", strerror(errno));
            usage(argv[0]);
         }
      }
   }
}


static void initialize_sensor_descriptions(void)
{
   // assign hardware-specific functions to sensor description
   for (uint32_t i=0; i<MAX_SENSORS; i++) {
      sensor_runtime_type *sensor = &sensor_stack_[i];
      if(sensor->type_name[0] == 0)
         break;
      sensor->flags &= ~SENSOR_FLAG_INACTIVE;
      if (strcmp(sensor->type_name, HARDWARE_NAME_LSM9DS0) == 0) {
         sensor->update = lsm9ds0_update;
         sensor->self_test = NULL;
         sensor->setup = lsm9ds0_setup;
         sensor->shutdown = lsm9ds0_shutdown;
      } else if (strcmp(sensor->type_name, HARDWARE_NAME_LSM303) == 0) {
         sensor->update = lsm303_update;
         sensor->self_test = NULL;
         sensor->setup = lsm303_setup;
         sensor->shutdown = lsm303_shutdown;
      } else if (strcmp(sensor->type_name, HARDWARE_NAME_L3G) == 0) {
         sensor->update = l3g_update;
         sensor->self_test = NULL;
         sensor->setup = l3g_setup;
         sensor->shutdown = l3g_shutdown;
      } else if (strcmp(sensor->type_name, HARDWARE_NAME_BMG160) == 0) {
         sensor->update = bmg160_update;
         sensor->self_test = NULL;
         sensor->setup = bmg160_setup;
         sensor->shutdown = bmg160_shutdown;
      } else if (strcmp(sensor->type_name, HARDWARE_NAME_HMC6343) == 0) {
         sensor->update = hmc6343_update;
         sensor->self_test = NULL;
         sensor->setup = hmc6343_setup;
         sensor->shutdown = hmc6343_shutdown;
      } else if (strcmp(sensor->type_name, HARDWARE_NAME_LIS3MDL) == 0) {
         sensor->update = lis3mdl_update;
         sensor->self_test = lis3mdl_check_whoami;
         sensor->setup = lis3mdl_setup;
         sensor->shutdown = lis3mdl_shutdown;
      } else if (strcmp(sensor->type_name, HARDWARE_NAME_LIS3DH) == 0) {
         sensor->update = lis3dh_update;
         sensor->self_test = lis3dh_check_whoami;
         sensor->setup = lis3dh_setup;
         sensor->shutdown = lis3dh_shutdown;
      } else if (strcmp(sensor->type_name, HARDWARE_NAME_A3G4250D) == 0) {
         fprintf(stderr, "%s temporarily disabled pending code review "
               "and update\n", sensor->type_name);
         exit(1);
         sensor->update = a3g4250d_update;
         sensor->self_test = a3g4250d_self_test;
         sensor->setup = a3g4250d_setup;
         sensor->shutdown = a3g4250d_shutdown;
      } else if (strcmp(sensor->type_name, HARDWARE_NAME_MAG3110) == 0) {
         sensor->update = mag3110_update;
         sensor->self_test = mag3110_check_whoami;
         sensor->setup = mag3110_setup;
         sensor->shutdown = a3g4250d_shutdown;
      } else {
         log_err(log_, "Unknown hardware type for sensor %d", i);
         log_err(log_, "    device: %s", sensor_stack_[i].device_addr);
         log_err(log_, "    type: %s", sensor_stack_[i].type_name);
         print_hardware_types();
         exit(1);
      }
   }
}

// looks for dev/<device>/sensors/<dev_name>/axis_alignment
// if that file is present, it's loaded into the alignment matrix
static void load_axis_alignment(
      /* in     */ const sensor_runtime_type * sensor,
      /* in out */       matrix_type *alignment
      )
{
   // if alignment not available, default to identity matrix
   identity_matrix(alignment);
   //
   char buf[STR_LEN];
   sprintf(buf, "%saxis_alignment", sensor->config_root);
   FILE *fp = fopen(buf, "r");
   if (fp) {
      if (config_read_matrix(fp, alignment) != 0) {
         log_err(log_, "Unable to parse axis alignment matrix %s", buf);
         hard_exit(__func__, __LINE__);
      }
      fclose(fp);
   } else {
      log_info(log_, "Axis alignment matrix '%s' does not exist", buf);
   }
}

static int32_t initialize_sensors(void)
{
   // pass 1
   for (uint32_t i=0; i<MAX_SENSORS; i++) {
      sensor_runtime_type *sensor = &sensor_stack_[i];
      if (sensor->flags & SENSOR_FLAG_INACTIVE)
         break;
      // perform initialization
      if (sensor->setup(sensor) != 0)
         goto err;
      // flags are now set -- make a copy
      const uint32_t flags = sensor->flags;
      if (flags & SENSOR_FLAG_DISABLED)
         continue;
      // load and init axis_alignment matrix if alignment available
      if (flags & SENSOR_FLAG_GYRO) {
         load_axis_alignment(sensor, &sensor->gyro.axis_alignment);
      }
      if (flags & SENSOR_FLAG_ACC) {
         load_axis_alignment(sensor, &sensor->accel.axis_alignment);
      }
      if (flags & SENSOR_FLAG_MAG) {
         load_axis_alignment(sensor, &sensor->mag.axis_alignment);
      }
      // base initialization complete
      // sensors initialize what they manage. set values for other
      //    modalities to null
      if (!(flags & SENSOR_FLAG_GYRO))
         init_sensor_gyro_null(&sensor->gyro);
      if (!(flags & SENSOR_FLAG_ACC))
         init_sensor_accel_null(&sensor->accel);
      if (!(flags & SENSOR_FLAG_MAG))
         init_sensor_mag_null(&sensor->mag);
      if (!(flags & SENSOR_FLAG_TEMP))
         init_sensor_temp_null(&sensor->temp);
      if (!(flags & SENSOR_FLAG_BARO))
         init_sensor_baro_null(&sensor->baro);
      if (!(flags & SENSOR_FLAG_GPS))
         init_sensor_gps_null(&sensor->gps);
   }
   return 0;
err:
   return -1;
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
   //
   cons->state.flags = 0;
   memset(cons->log_data, 0, SENSOR_PACKET_LOG_DATA);
}

////////////////////////////////////////////////////////////////////////

static int32_t base_initialization(
      /*    out */       int32_t *sock_fd
      )
{
   int32_t error_state = -1;
   initialize_static_memory();
   if (read_config_info() != 0) {
      goto done;
   }
   //
   initialize_sensor_descriptions();
   if (initialize_sensors() != 0) {
      goto done;
   }
   // try to gracefully exit after sigint or broken pipe
   signal(SIGINT, signal_soft_exit);
   signal(SIGPIPE, signal_soft_exit);
   if (bcast_) {
      struct network_id net_id;
      if ((resolve_endpoint_for_i2c(&net_id) != 0) || (net_id.ip[0] == 0)) {
         log_err(log_, "Unable to load network target for this host");
         goto done;
      }
      if ((*sock_fd = connect_to_server(&net_id)) < 0) {
         goto done;
      }
      // if master time was specified, override time negotiated w/ server
      if (master_time_ > 0.0) {
         // update master time with runtime delay to get to this point
         double delay = system_now() - process_clock_;
         master_time_ += delay;
         // update clock and log change
         double dt = now() - master_time_;
         snprintf(log_data_, SENSOR_PACKET_LOG_DATA, 
               "Clock override -- dT=%.4f", dt);
         log_data_[SENSOR_PACKET_LOG_DATA-1] = 0;
         timekeeper_set_time_f(master_time_);
      }
   }
   // negotiate connection with server, checking for protocol
   //    compatibility. time also synchrpnized here
   if (bcast_)
      create_sync_receiver(); // receiver for synchronization time packets
   error_state = 0;
done:
   return error_state;
}

static void app_shutdown(
      /*    out */       int32_t sock_fd
      )
{
   // send signal to sensor indicating shutdown, to let it exit gracefully
   for (uint32_t i=0; i<MAX_SENSORS; i++) {
      sensor_runtime_type *sensor = &sensor_stack_[i];
      const uint32_t flags = sensor->flags;
      if (flags & SENSOR_FLAG_INACTIVE)
         break;
      if (flags & SENSOR_FLAG_DISABLED)
         continue;
      sensor->shutdown(sensor);
   }
   // shutdown output streams
   if (outfp_) {
      fflush(outfp_);
      if (outfp_ != stdout)
         fclose(outfp_);
      outfp_ = NULL;
   }
   //
   if (sock_fd >= 0) {
      close(sock_fd);
   }
   if (bcast_)
      shutdown_sync_receiver();
   close_logs();
}

static int32_t acquisition_loop(
      /*    out */       int32_t sock_fd
      )
{
   int32_t error_state = 0;
   /////////////////////////////////////////////////////////////
   // data acquisition loop
   struct timespec present;   // present time, i.e., 'now'
   struct timespec waketime;  // next scheduled time to wake up
   struct timespec start_time;   // app start time
   // sensors stored their initial delay in waketime, as at time of
   //    initialization none could know what time the main loop would
   //    be starting. we have that time now. reset waketimes to be
   //    relative to start time
   clock_gettime(CLOCK_MONOTONIC, &start_time);
   for (uint32_t i=0; i<MAX_SENSORS; i++) {
      sensor_runtime_type *sensor = &sensor_stack_[i];
      const uint32_t flags = sensor->flags;
      if (flags & SENSOR_FLAG_INACTIVE)
         break;
      if (flags & SENSOR_FLAG_DISABLED)
         continue;
      timeadd(&sensor->waketime, start_time.tv_sec, start_time.tv_nsec);
//printf("Process %d has waketime of %f\n", i, timespec_to_double(&sensor->waketime));
   }
   // structure where sensor data is collected for broadcast ('consensus
   //    data')
   consensus_sensor_type consensus;
   initialize_consensus(&consensus);
   // number of initial samples to discard. some sensors may start in
   //    an inaccurate state (e.g., in stream bypass mode, a very
   //    old value may be stored in the output queue, a value that
   //    should be discarded)
   int32_t discard_next = 10;
   while(quit_ == 0) {
      //////////////////////////////////////////////////////////
      // sleep until it's time to wake the next thread
      // find sensor with next earliest waketime
      clock_gettime(CLOCK_MONOTONIC, &present);
      timecpy(&waketime, &sensor_stack_[0].waketime);
      for (uint32_t i=1; i<MAX_SENSORS; i++) {
         const uint32_t flags = sensor_stack_[i].flags;
         if (flags & SENSOR_FLAG_INACTIVE)
            break;
         if (flags & SENSOR_FLAG_DISABLED)
            continue;
         if (timecmp(&waketime, &sensor_stack_[i].waketime) > 0) {
            timecpy(&waketime, &sensor_stack_[i].waketime);
         }
      }
//print_time(&sensor_stack_[0].waketime, "sensor time");
      // if waketime is in the past, report the error but keep going
      if (timecmp(&present, &waketime) > 0) {
         report_timing_error(&present);
      }
      // go to sleep
      if ((error_state = sleep_until(&waketime, &quit_)) != 0) {
         log_info(log_, "sleep_until returned non-zero result: %s",
               strerror(error_state));
         break;
      }
      //////////////////////////////////////////////////////////
      // update each object whose waketime is at or before now
      clock_gettime(CLOCK_MONOTONIC, &present);
//print_time(&present, "present 2");
//      uint32_t acc_flag = 0;
//      uint32_t mag_flag = 0;
//      uint32_t gyr_flag = 0;
      uint32_t flags = 0;
      for (uint32_t i=0; i<MAX_SENSORS; i++) {
         sensor_runtime_type *sensor = &sensor_stack_[i];
         if (sensor->flags & SENSOR_FLAG_INACTIVE)
            break;
         if (sensor->flags & SENSOR_FLAG_DISABLED)
            continue;
         if (timecmp(&present, &sensor->waketime) >= 0) {
            int32_t data_available = 0;
            // first time through, do a check on the sensor. if it
            //    passes, disable future checks. if it fails, deactivate
            //    the sensor
            if (sensor->self_test) {
               if (sensor->self_test(sensor) != 0) {
//fprintf(stderr, "WHO_AM_I failed for %s. Deactivating.\n", sensor->name);
                  log_info(log_, "WHO_AM_I failed for %s. Deactivating.",
                        sensor->name);
                  sensor->flags |= SENSOR_FLAG_DISABLED;
                  continue;
               }  
               sensor->self_test = NULL;
            }
            sensor->update(sensor, &data_available);
            if (data_available) {
               // approx store time of acquisition
               clock_gettime(CLOCK_MONOTONIC, &sensor->last_update);
               flags |= sensor->flags;
//               if (flags & SENSOR_FLAG_ACC)
//                  acc_flag = 1;
//               if (flags & SENSOR_FLAG_MAG)
//                  mag_flag = 1;
//               if (flags & SENSOR_FLAG_GYRO)
//                  gyr_flag = 1;
            }
         }
      }
      if (flags & SENSOR_FLAGS_GYRO_ACC_MAG) {
//      if (acc_flag | gyr_flag | mag_flag) {
         // update consensus, including [hidden] log data
         if (log_data_[0] != 0) {
            strncpy(consensus.log_data, log_data_, SENSOR_PACKET_LOG_DATA);
            log_data_[0] = 0;
         }
         //////////////////////////////////////////////////////////
         // combine signals and merge channels
         // on updated to one acc/mag unit, update consensus attitude
         if (flags & SENSOR_FLAG_ACC) {
            update_acc(sensor_stack_, &consensus);
         }
         if (flags & SENSOR_FLAG_MAG) {
            update_mag(sensor_stack_, &consensus);
         }
         // on update to gyro, update consensus orientation then add 
         //    bias to signal from consensus attitude
         if (flags & SENSOR_FLAG_GYRO) { 
            update_gyr(sensor_stack_, &consensus);
         }
         //
         if (flags & SENSOR_FLAG_TEMP) {
            update_temp(sensor_stack_, &consensus);
         }
         //
         if (outfp_)
            print_consensus(&consensus);
         //////////////////////////////////////////////////////////
         // broadcast packet to core with sensor information
         if (bcast_) {
            if (discard_next > 0) {
               discard_next--;
            } else {
               consensus.log_data[0] = 0;
               if (send_broadcast(sock_fd, &consensus) < 0) {
                  log_err(log_, "Failed to send broadcast of concensus data");
                  break;   // network failure. exit and let supervisor restart
               }
               // don't start logging until we successfully broadcast data. 
               //    otherwise, if kernel not running but supervisor is, 
               //    new logging directory will be created every 5 seconds 
               //    (or whatever super's check interval is)
               if (!log_) {
                  log_ = get_logger("s2");
                  log_info(log_, "Sensor acquisition process (%s)", 
                        SENSOR_BUILD_VERSION);
               }
            }
         }
         memset(&consensus.log_data, 0, SENSOR_PACKET_LOG_DATA);
      }
   }
   return error_state;
}


// initialization and setup
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

// TODO FIXME synchronizing time on startup may be wrong in event
//    of heavy network traffic. consider having supervisor keep track
//    of time and pass this in on startup (this would have to override
//    time sent when establishing network protocol compatibility)

int main(int argc, char **argv)
{
   // logger started once network link is established, unless explicitly requested on command line
   parse_command_line(argc, argv);
   // note time that process starts
   process_clock_ = system_now();
   //
   int32_t sock_fd = -1;
   int32_t error_state = 0;
   sanity_check();
   // initialize components and establish link to server, if appropraite
   error_state = base_initialization(&sock_fd);
   if (error_state != 0)
      goto end;
   // main loop
   error_state = acquisition_loop(sock_fd);
   // system shutdown
   app_shutdown(sock_fd);
   // recast error state as int
end:
   return (int) error_state;
}

