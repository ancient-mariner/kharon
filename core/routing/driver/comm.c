#include <stdint.h>

// this file should be able to be run stand-alone for testing so 
//    include header content in it

// computer data sent to autopilot
union heading_data {
   // valid values on 0-359
   struct {
      uint16_t heading;
      uint16_t course;  // if value >= 360, rudder should be centered
      float dps;     // approx turn rate
   };
   uint64_t all;
};
typedef union heading_data heading_data_type;


// autopilot data returned to computer
union tiller_data {
   struct {
      // valid values on 0,1024. error codes above that
      int16_t tiller_position;
      // valid values on 0,359. this should match value sent to autopilot
      int16_t course;
      int16_t heading;
      // heading dps * 100
      int16_t dps;
   };
   uint64_t all;
};
typedef union tiller_data tiller_data_type;

// 8-byte serial packet
struct serial_packet_8 {
   union {
      uint8_t data[8];
      uint64_t data_all;
   };
   union {
      struct {
         uint16_t num_samples;   // number of received bytes (0-8)
         uint16_t num_expected;  // number expected bytes (1-8)
         uint8_t complete;   // 1 if packet complete, 0 otherwise
      };
      uint64_t control_all;
   };
};
typedef struct serial_packet_8 serial_packet_8_type;

#if defined(SIMULATE_DRIVER)
struct datap_desc {
   int32_t run_state;
};
typedef struct datap_desc datap_desc_type;

#define DP_STATE_DONE      1

#endif // SIMULATE_DRIVER
#if defined(SIMULATE_DRIVER)
#endif // SIMULATE_DRIVER

// sleep X milliseconds between looking for new data to send/recv
#define NAP_DURATION_USEC    20000

// data sent in 7-bit chunks. 8th bit is used for control vals
#define SERIAL_PACKET_START      0x81
#define SERIAL_PACKET_END        0x82
// debug data is preceded by text content, until a newline is encountered
#define SERIAL_DEBUG_MASK        0x90

#define TTY_DEV_NAME     "/dev/ttyACM0"


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <pthread.h>
#include <math.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/time.h>

// autopilot interface
// establishes serial communication with arduino. send course and
//    heading data and receives tiller position information

// number of bytes of debug information expected on serial stream
static int32_t debug_stream_ = 0;

#define DEBUG_DATA_LEN     256
static char debug_data_[DEBUG_DATA_LEN];
static uint32_t debug_len_ = 0;

static int serial_fd_ = -1;


////////////////////////////////////////////////
// interface with driver

// shared memory. tiller is written here and read by driver. heading
//    is written by driver and read here
// access is not wrapped in mutex for any shared data as it's r/w actions
//    are atomic, avoiding corruption, and if new|old data is read or
//    some data is dropped that's not a big problem, as new data will
//    be arriving again soon

static tiller_data_type tiller_data_ = { .all = 0 };
static heading_data_type heading_data_ = { .all = 0 };
#if !defined(SIMULATE_DRIVER)
static tiller_data_type tiller_data_prev_ = { .all = 0 };
#endif   // SIMULATE_DRIVER

// these flags are set to 1 when data is available and cleared when data read
uint32_t tiller_data_available_ = 0;
uint32_t heading_data_available_ = 0;


////////////////////////////////////////////////////////////////////////
// serial setup

static int set_interface_attributes(
      /* in     */ const int32_t fd, 
      /* in     */ const speed_t speed, 
      /* in     */ const uint32_t parity
      )
{
   struct termios tty;
   if (tcgetattr (fd, &tty) != 0)
   {
      fprintf(stderr, "Error %d from tcgetattr\n", errno);
      return -1;
   }
   cfsetospeed (&tty, speed);
   cfsetispeed (&tty, speed);
   //
   tty.c_cflag = (tty.c_cflag & ~((uint32_t) CSIZE)) | CS8;     // 8-bit chars
   // disable IGNBRK for mismatched speed tests; otherwise receive break
   // as \000 chars
   tty.c_iflag &= ~((uint32_t) IGNBRK);         // disable break processing
   tty.c_lflag = 0;                // no signaling chars, no echo,
   // no canonical processing
   tty.c_oflag = 0;                // no remapping, no delays
   tty.c_cc[VMIN]  = 0;            // read doesn't block
   tty.c_cc[VTIME] = 1;            // 0.1 seconds read timeout
   //
   tty.c_iflag &= ~((uint32_t) (IXON | IXOFF | IXANY)); // shut off xon/xoff ctrl
   tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
   // enable reading
   tty.c_cflag &= ~((uint32_t) (PARENB | PARODD));      // shut off parity
   tty.c_cflag |= parity;
   tty.c_cflag &= ~((uint32_t) CSTOPB);
   tty.c_cflag &= ~CRTSCTS;
   //
   if (tcsetattr (fd, TCSANOW, &tty) != 0)
   {
      fprintf(stderr, "Error %d from tcsetattr\n", errno);
      return -1;
   }
   return 0;
}


static void open_serial_device(void)
{
   static int32_t fail_cnt = 0;
   if (serial_fd_ < 0) {
      serial_fd_= open(TTY_DEV_NAME, O_RDWR | O_NOCTTY | O_SYNC);
      if (serial_fd_ < 0)
      {
         // post failure-to-open message, but only on first failure
         //    and intermittent subsequent ones (not every attempt)
         if ((fail_cnt++ & 7) == 0) {
            fprintf(stderr, "Error opening '%s': %s\n", 
                  TTY_DEV_NAME, strerror (errno));
         }
      } else {
         fprintf(stderr, "Opened '%s'\n", TTY_DEV_NAME);
         // set to 9600bps, no parity (8n1), blocking
         // or 57k, or 19k
         set_interface_attributes(serial_fd_, B19200, 0);
         //set_interface_attributes(serial_fd_, B57600, 0);
         //set_interface_attributes(serial_fd_, B9600, 0);
         fail_cnt = 0;
      }
   }
}

static void shutdown_connections(void)
{
   if (serial_fd_ >= 0) {
      if (close(serial_fd_) != 0) {
         fprintf(stderr, "Error shutting down serial fd (%s)\n", 
               strerror(errno));
      }
      serial_fd_ = -1;
   }
}


// serial setup
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// serial interface


static void post_heading(
      /* in out */       serial_packet_8_type *out_data
      )
{
   // copy packet data to output buffer
   uint8_t buf[16];
   uint32_t ctr = 0;
   buf[ctr++] = SERIAL_PACKET_START;
   for (uint32_t i=0; i<out_data->num_samples; i++) {
      buf[ctr++] = out_data->data[i];
   }
   buf[ctr++] = SERIAL_PACKET_END;
   // a proper solution is to make sure entire packet is sent. keep
   //    logic easy for now and assume it's gone. packet size is so
   //    small that this shouldn't be a problem and even if only part
   //    of buffer is sent, a new packet will be following shortly
   //    to resend
   ssize_t n;
   if ((n = write(serial_fd_, buf, ctr)) <= 0) {
      // error write
      fprintf(stderr, "Error writing to serial port. %s\n", strerror(errno));
      // try to close port and mark it as out-of-action
      close(serial_fd_);
      serial_fd_ = -1;
   } else if (n != ctr) {
      fprintf(stderr, "Partial packet written to serial port: "
            "%ld of %d bytes\n", n, ctr);
   }
}


// read any received data from serial connection. if full packet has
//    been received, flag that. divert debug info to stdout
static void check_autopilot_response(
      /* in out */       serial_packet_8_type *packet_data
      )
{
   // while available serial data
   uint8_t buf[8];
   ssize_t n;
   assert(packet_data->num_expected > 0);
   while ((n = read(serial_fd_, buf, 1)) == 1) {
      uint8_t val = buf[0];
      // check for debug data
      if (debug_stream_ != 0) {
         printf("%c", val);
         if (val == 10) {
            debug_stream_ = 0;
#if !defined(SIMULATE_DRIVER)
            log_info(driver_->log, "OTTO '%s'", debug_data_);
#endif // SIMULATE_DRIVER
            printf("OTTO '%s'\n", debug_data_);
            debug_len_ = 0;
         } else {
            if (debug_len_ < (DEBUG_DATA_LEN - 1)) {
               debug_data_[debug_len_++] = (char) val;
            }
         }
         continue;
      } else if ((val & SERIAL_DEBUG_MASK) == SERIAL_DEBUG_MASK) {
         debug_stream_ = 1;
         printf("DEBUG ");
         continue;
      }
      // check for packet start
      if (val == SERIAL_PACKET_START) {
         // if packet start encountered, reset packet
         packet_data->num_samples = 0;
         continue;
      }
      // check for packet end
      if (val == SERIAL_PACKET_END) {
         if (packet_data->num_samples == packet_data->num_expected) {
            // packet received. flag that it's complete and break out
            packet_data->complete = 1;
            break;
         } else {
            // incomplete packet. drop data and start over
fprintf(stderr, "Incomplete packet. Starting over\n");
            packet_data->num_samples = 0;
            continue;
         }
      }
      // check for packet overflow
      if (packet_data->num_samples >= packet_data->num_expected) {
         // data corrupted -- no packet end detected. drop data
         //    and start over
fprintf(stderr, "Packet overflow. Starting over\n");
         packet_data->num_samples = 0;
         continue;
      }
      // pull data
      packet_data->data[packet_data->num_samples] = val;
      packet_data->num_samples++;
   }
}

// serial interface
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// conversion

// unpack tiller data from serial transfer
static void convert_packet8_to_tiller(
      /* in out */       serial_packet_8_type *serial_data
      )
{
   // unpack tiller data in a local variable. copy contents in bulk to
   //    shared memory
   tiller_data_type tiller;
   // make sure high order bit not set on any data byte
   assert((serial_data->data_all & 0x8080808080808080l) == 0);
   // tiller data
   // tiller on 0,1023
   tiller.tiller_position = (int16_t) (
         (serial_data->data[0] << 7) | (serial_data->data[1] & 0x7f));
   // check that it's w/in expected range
   if (tiller.tiller_position >= 2048) {
      // error in tiller. either without power or it has reverse
      //    polarity
      fprintf(stderr, "Tiller not responding. Check power or "
            "resistor polarity (code %d)\n", tiller.tiller_position);
   } else {
      assert(tiller.tiller_position <= 1024);
   }
   // course data
   tiller.course = (int16_t) 
         ((serial_data->data[2] << 7) | (serial_data->data[3] & 0x7f));
   // heading data
   tiller.heading = (int16_t) 
         ((serial_data->data[4] << 7) | (serial_data->data[5] & 0x7f));
   // turn rate data
   tiller.dps = (int16_t) 
         ((serial_data->data[6] << 7) | (serial_data->data[7] & 0x7f));
   // carry up sign bits
   if (tiller.dps & 0x2000) {
      tiller.dps = (int16_t) (tiller.dps | 0xc000);
   } else {
   }
   // copy to shared memory
   tiller_data_.all = tiller.all;
   tiller_data_available_ = 1;
static uint32_t ctr = 0;
if ((ctr++ & 3) == 0) {
   printf("TILLER   pos %d  course %d  heading %d  turn %d/100  0x%04x\n", tiller_data_.tiller_position, tiller_data_.course, tiller_data_.heading, tiller_data_.dps, tiller_data_.dps);
}
}


// prepare heading data for serial transfer
static void convert_heading_to_packet8(
      /* in out */       serial_packet_8_type *serial_data
      )
{
   // make copy of shared memory to send
   heading_data_type heading = heading_data_;
   //
   serial_data->data_all = 0;
   // take low-order 14 bits from int
   serial_data->data[0] = (heading.heading >> 7) & 0x7f;
   serial_data->data[1] = (heading.heading     ) & 0x7f;
   serial_data->data[2] = (heading.course >> 7) & 0x7f;
   serial_data->data[3] = (heading.course     ) & 0x7f;
   int32_t idps = (int32_t) roundf(heading.dps * 100.0f);
   serial_data->data[4] = (idps >> 7) & 0x7f;
   serial_data->data[5] = (idps     ) & 0x7f;
   serial_data->num_samples = 6;
   serial_data->complete = 1;
}

// conversion
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// main

static void comm_handler_sigusr1(int x)
{
   (void) x;
   // triggering this handler should wake the thread, which is what
   //    we wanted
   // swallow the signal itself
}

static void * comm_thread_main(
      /* in     */       void * arg
      )
{
   // setup signal handler. signals used to wake sleeping thread
   struct sigaction sa;
   memset(&sa, 0, sizeof sa);
   sa.sa_handler = comm_handler_sigusr1;
   sigaction(SIGUSR1, &sa, NULL);
   memset(debug_data_, 0, DEBUG_DATA_LEN);
   // 
   // packet to autopilot
   serial_packet_8_type out_data;
   out_data.data_all = 0;
   out_data.control_all = 0;
   out_data.num_expected = 6;
   // response from autopilot
   serial_packet_8_type in_data;
   in_data.data_all = 0;
   in_data.control_all = 0;
   in_data.num_expected = 8;
   //
   datap_desc_type *self = (datap_desc_type *) arg;
static uint32_t ctr = 0;
   while ((self->run_state & DP_STATE_DONE) == 0) {
      // open serial connection if it's not yet available
      open_serial_device();
if ((ctr++ & 7) == 0) {
   printf("ROUTE  course %d  heading %d  turn %.2f\n", heading_data_.course, heading_data_.heading, (double) heading_data_.dps);
}
      if (serial_fd_ < 0) {
         sleep(1);
         continue;
      }
      // post updated data if it's available
      if (heading_data_available_ != 0) {
         // heading data's changed. post data to autopilot
         convert_heading_to_packet8(&out_data);
         heading_data_available_ = 0;
         post_heading(&out_data);
         //
#if !defined(SIMULATE_DRIVER)
         log_info(driver_->log, "crs %d  head %d",
               heading_data_.course, heading_data_.heading);
#endif // SIMULATE_DRIVER
      }
      if (serial_fd_ < 0) {
         continue;
      }
      // check for autopilot response. handle it if packet complete
      check_autopilot_response(&in_data);
      if (in_data.complete == 1) {
         // convert packet into tiller data
         convert_packet8_to_tiller(&in_data);
         if (tiller_data_available_) {
            // clear state data to prepare to receive new content
            tiller_data_available_ = 0;
            in_data.complete = 0;
            in_data.num_samples = 0;
#if !defined(SIMULATE_DRIVER)
            // update record of packet received and ack of course
            //    change
            double t = now();
            driver_->last_otto_reply_sec = t;
            if (tiller_data_.course != tiller_data_prev_.course) {
               log_info(driver_->log, "Autopilot acknowledge course "
                     "change to %d", tiller_data_.course);
               driver_->route.course_changed_sec = t;
            }
            //
            log_info(driver_->log, "Tiller at %d  crs %d  head %d  dps %d", 
                  tiller_data_.tiller_position, tiller_data_.course, 
                  tiller_data_.heading, tiller_data_.dps);
            tiller_data_prev_ = tiller_data_;
#endif // SIMULATE_DRIVER
         }
      }
      // take a nap to wait for more data
      usleep(NAP_DURATION_USEC);
   }
   shutdown_connections();
   return NULL;
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
#if defined(SIMULATE_DRIVER)

static datap_desc_type dp_;

static void print_time_str(const char *label)
{
   struct timeval tv;
   gettimeofday(&tv, NULL);
   double t = (double) tv.tv_sec + (double) tv.tv_usec * 1.0e-6;
   printf("%.3f  %s\n", t, label);
}

static void run_simulation(void)
{
   for (uint32_t i=0; i<=20; i++) {
      if (heading_data_available_ == 0) {
//printf("Sending A:%d\n", i);
         heading_data_.course = 90;
         heading_data_.heading = (uint16_t) (70 + 2*i);
         heading_data_.dps = 1.00f;
         heading_data_available_ = 1;
      } else {
         print_time_str("MAIN Heading data not processed");
      }
      sleep(1);
   }
   for (uint32_t i=0; i<=20; i++) {
      if (heading_data_available_ == 0) {
//printf("Sending A:%d\n", i);
         heading_data_.course = 90;
         heading_data_.heading = (uint16_t) (70 + 2*i);
         heading_data_.dps = 3.00f;
         heading_data_available_ = 1;
      } else {
         print_time_str("MAIN Heading data not processed");
      }
      sleep(1);
   }
//   for (uint32_t i=0; i<=15; i++) {
//      if (heading_data_available_ == 0) {
////printf("Sending B:%d\n", i);
//         heading_data_.course = 75;
//         heading_data_.heading = (uint16_t) (90 - i);
//         heading_data_.dps = -2.34f;
//         heading_data_available_ = 1;
//      } else {
//         print_time_str("MAIN Heading data not processed");
//         //print("%d Heading data not processed\n", i);
//      }
//      sleep(1);
//   }
   dp_.run_state |= DP_STATE_DONE;
}


int main(int argc, char** argv)
{
   (void) argc;
   (void) argv;
   pthread_t tid;
   dp_.run_state = 0;
   pthread_create(&tid, NULL, comm_thread_main, &dp_);
   run_simulation();
   pthread_join(tid, NULL);
   return 0;
}

#endif // SIMULATE_DRIVER

