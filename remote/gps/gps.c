#include <errno.h>
#include <unistd.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <signal.h>
#include <sys/socket.h>
#include "logger.h"
#include "timekeeper.h"
#include "dev_info.h"
#include "build_version_gps.h"


// read GPS data over serial line and forward to gps_receiver

// TODO when parsing data, advance to "$" and read until *nn. verify 
//    checksum and do basic error checking. if it's OK, send to receiver.
// basic error checking. all chars in [a-Z0-9,.-+]. if encounter '$' then
//    abort check on first and restart with new '$'

#define DEFAULT_TTY_DEV_NAME     "/dev/ttyUSB0"

#define GPS_BLOCK_SIZE  256

static int quit_ = 0;

static int serial_fd_ = -1;
static int sock_fd_ = -1;

static const char * tty_dev_name_ = NULL;
static const char * endpoint_name_ = NULL;

static log_info_type *log_ = NULL;

////////////////////////////////////////////////////////////////////////
// serial interface

// initial serial communication prototype from 
// https://stackoverflow.com/questions/6947413/how-to-open-read-and-write-from-serial-port-in-c/6947758#6947758

static int set_interface_attributes(
      /* in     */ const int32_t fd, 
      /* in     */ const speed_t speed, 
      /* in     */ const uint32_t parity
      )
{
   struct termios tty;
   if (tcgetattr (fd, &tty) != 0)
   {
      fprintf(stderr, "error %d from tcgetattr", errno);
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
   tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout
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
      fprintf(stderr, "error %d from tcsetattr", errno);
      return -1;
   }
   return 0;
}


static void set_blocking(
      /* in     */ const int fd, 
      /* in     */ const int should_block
      )
{
   struct termios tty;
   memset (&tty, 0, sizeof tty);
   if (tcgetattr (fd, &tty) != 0)
   {
      printf("error %d from tggetattr", errno);
      return;
   }
   tty.c_cc[VMIN]  = should_block ? 1 : 0;
   tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout
   if (tcsetattr (fd, TCSANOW, &tty) != 0) {
      fprintf(stderr, "error %d setting term attributes", errno);
   }
}


static void open_serial_device(void)
{
   if (serial_fd_ < 0) {
      serial_fd_= open (tty_dev_name_, O_RDWR | O_NOCTTY | O_SYNC);
      if (serial_fd_ < 0)
      {
         fprintf(stderr, "error opening '%s': %s\n", tty_dev_name_, 
               strerror (errno));
      } else {
         // set to 4800bps, no parity (8n1), blocking
         set_interface_attributes(serial_fd_, B4800, 0);
         set_blocking (serial_fd_, 1);
      }
   }
}

// serial interface
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// nmea

// nmea's "checksum" -- very weak, but that's what we have
static uint8_t nmea_checksum(
      /* in     */ const char *sentence,
      /* in     */ const uint32_t len
      )
{
   uint8_t cksum = 0;
   for (size_t i=0; i<len; i++) {
      cksum ^= (uint8_t) sentence[i];
   }
   return cksum;
}


static void handle_sentence(
      /* in     */ const char *sentence, 
      /* in     */ const uint32_t sentence_len, 
      /* in     */ const uint8_t reported_checksum
      )
{
   // validate checksum
   uint8_t measured_checksum = nmea_checksum(sentence, sentence_len);
   if (measured_checksum == reported_checksum) {
      printf("Received: '%s'\n", sentence);
      // form output packet
      char buf[GPS_BLOCK_SIZE];
      sprintf(buf, "%.3f %s", now(), sentence);
      send_block(sock_fd_, buf, sizeof(buf));
   } else {
      // ignore sentence as corrupt
      //
      printf("CORRUPT: '%s'\n", sentence);
   }
}


// nmea
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// content parsing

static uint8_t parse_table_[128];
#define NMEA_CHAR_CONTENT     0x01
#define NMEA_CHAR_START       0x02
#define NMEA_CHAR_END         0x04
#define NMEA_CHAR_HEX         0x08
#define NMEA_CHAR_NUMBER      0x10
#define NMEA_CHAR_INVALID     0x80


// TODO write parse algorithm
static void prepare_parser(void)
{
   // mark all chars (lower ascii) as invalid to start. this flag will be
   //    removed on chars deemed to be useful
   memset(parse_table_, NMEA_CHAR_INVALID, sizeof(parse_table_));
   // set flags for being viable content
   // [a-Z0-9] and "$*,.-+" are considered viable
   // numbers are content, numeric and hex
   for (uint32_t i=0; i<10; i++) {
      parse_table_['0' + i] = 
            NMEA_CHAR_HEX | NMEA_CHAR_NUMBER | NMEA_CHAR_CONTENT;
   }
   // alpha are content, <='f' are hex
   for (uint32_t i=0; i<26; i++) {
      parse_table_[(uint8_t) (i + 'A')] = NMEA_CHAR_CONTENT;
      parse_table_[(uint8_t) (i + 'a')] = NMEA_CHAR_CONTENT;
      if (i < 6) {
         parse_table_[(uint8_t) (i + 'A')] |= NMEA_CHAR_HEX;
         parse_table_[(uint8_t) (i + 'a')] |= NMEA_CHAR_HEX;
      }
   }
   // remaining characters
   parse_table_['$'] = NMEA_CHAR_CONTENT | NMEA_CHAR_START;
   parse_table_['*'] = NMEA_CHAR_CONTENT | NMEA_CHAR_END;
   parse_table_['.'] = NMEA_CHAR_CONTENT;
   parse_table_[','] = NMEA_CHAR_CONTENT;
   parse_table_['+'] = NMEA_CHAR_CONTENT;
   parse_table_['-'] = NMEA_CHAR_CONTENT;
}


static uint8_t dehex(
      /* in     */ const char c
      )
{
   uint8_t val = 0;
   if (c <= '9') {
      val = (uint8_t) (c - '0');
   } else if (c <= 'F') {
      val = (uint8_t) (10 + c - 'A');
   } else if (c <= 'f') {
      val = (uint8_t) (10 + c - 'f');
   } else {
      fprintf(stderr, "Internal error -- '%c' not recognized as hex\n", c);
   }
   return val;
}


enum parse_states { SEARCHING, READING, CHECKSUM_0, CHECKSUM_1 };

static void parse_error(
      /* in     */ const uint8_t *block,
      /* in     */ const char *sentence
      )
{
   printf("Failed while parsing '%s'\n", sentence);
   // print block content
   for (uint32_t i=0; i<GPS_BLOCK_SIZE; i++) {
      printf(" %02x", block[i]);
      if ((i & 0x001f) == 0x001f) {
         printf("\n");
      } else if ((i & 0x0007) == 0x0007) {
         printf(" ");
      }
   }
   printf("\n");
}


static void parse_buffer(
      /* in     */ const uint8_t *buf,
      /* in     */ const uint32_t n
      )
{
   char sentence[GPS_BLOCK_SIZE];
   uint32_t sentence_len;
   uint8_t checksum;
   enum parse_states state = SEARCHING;
   for (uint32_t i=0; i<n; i++) {
      // parse stream. process sentence when one is identified and extracted
      char c = (char) buf[i];
      uint8_t code = parse_table_[c & 0x7f];
//printf("%c -> 0x%02x\n", c, code);
      switch (state) {
         case SEARCHING:
            if (code & NMEA_CHAR_START) {
               state = READING;
            }
            break;
         case READING:
            if (code & NMEA_CHAR_END) {
               sentence[sentence_len] = 0;
               state = CHECKSUM_0;
            } else if ((code & NMEA_CHAR_CONTENT) == 0) {
printf("Fail sentence\n");
               parse_error(buf, sentence);
               sentence_len = 0;
               state = SEARCHING;
            } else {
               sentence[sentence_len++] = c;
            }
            break;
         case CHECKSUM_0:
            if (code & NMEA_CHAR_HEX) {
               checksum = dehex(c);
               state = CHECKSUM_1;
            } else {
printf("Fail cksum0\n");
               parse_error(buf, sentence);
               sentence_len = 0;
               state = SEARCHING;
            }
            break;
         case CHECKSUM_1:
            if (code & NMEA_CHAR_HEX) {
               checksum = (uint8_t) (checksum * 16 + dehex(c));
               handle_sentence(sentence, sentence_len, checksum);
            } else {
printf("Fail cksum1\n");
               parse_error(buf, sentence);
            }
            sentence_len = 0;
            state = SEARCHING;
            break;
         default:
            fprintf(stderr, "Internal error -- broke on '%s' + '%c'\n",
                  sentence, c);
            sentence_len = 0;
            state = SEARCHING;
      };
   }
}

// content parsing
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// main 

static void shutdown_connections(void)
{
   if (serial_fd_ >= 0) {
      if (close(serial_fd_) != 0) {
         log_err(log_, "Error shutting down serial fd (%s)", strerror(errno));
      }
      serial_fd_ = -1;
   }
   if (sock_fd_ >= 0) {
      if (shutdown(sock_fd_, SHUT_RDWR) != 0) {
         log_err(log_, "Socket shutdown() error (%s)", strerror(errno));
      }
      sock_fd_ = -1;
   }
   /////////////////////////////////////////////
}


static void handler_sigusr1(int x)
{
   // time to exit
   quit_ = 1;
   printf("Received termination signal %d\n", x);
}


int main(int argc, const char **argv)
{
   printf("version %s\n", GPS_BUILD_VERSION);
   int rc = 1;
   if ((argc == 1) || (argc > 3)) {
      fprintf(stderr, "Usage: %s <endpoint-name> [tty-device]\n", argv[0]);
      fprintf(stderr, "Endpoint name refers to entry in "
            "/dev/<device>/endpoints/ (or 'test' for no networking)\n");
      fprintf(stderr, "Likely tty-device is '%s' (default)\n", 
            DEFAULT_TTY_DEV_NAME);
      fprintf(stderr, "Send SIGUSR1 to exit gracefully\n");
      goto end;
   } else if (argc == 3) {
      tty_dev_name_ = argv[2];
   } else {
      tty_dev_name_ = DEFAULT_TTY_DEV_NAME;
   }
   endpoint_name_ = argv[1];
   uint8_t buf[GPS_BLOCK_SIZE];
   /////////////////////////////////////////////
   // set signal interface
   struct sigaction sa;
   sa.sa_handler = handler_sigusr1;
   sigaction(SIGUSR1, &sa, NULL);
   /////////////////////////////////////////////
   // get server info
   network_id_type net_id = { .ip="", .port=0 };
   if (resolve_sensor_endpoint(endpoint_name_, &net_id) != 0) {
      fprintf(stderr, "Unable to find network target '%s'\n", endpoint_name_);
      if (strcmp(endpoint_name_, "test") == 0) {
         fprintf(stderr, "Running with no network (test mode)\n");
      } else {
         fprintf(stderr, "Bailing out (run with endpoint='test' to override\n");
         quit_ = 1;
      }
   } else {
      printf("Connecting to %s:%d\n", net_id.ip, net_id.port);
   }
   /////////////////////////////////////////////
   prepare_parser();
   /////////////////////////////////////////////////////////////////////
   // loop until quit signal (this is triggered via SIGUSR1)
   while (quit_ == 0) {
      if ((sock_fd_ < 0) && (net_id.ip[0] != 0)) {
         // try to establish connection
         // ignore failure and continue so can read from serial line
         sock_fd_ = connect_to_server(&net_id);
         if (sock_fd_ >= 0) {
            log_info(log_, "Connected to %s:%d", net_id.ip, net_id.port);
printf("Connected to %s:%d\n", net_id.ip, net_id.port);
         }
      }
      if (quit_) {
         break;
      }
      //////////////////////////////////////////
      // read serial data
      if (serial_fd_ < 0) {
         // make sure serial device is open
         open_serial_device();
      }
      if (serial_fd_ >= 0) {
         do {
            // read at least one block from serial port. if server
            //    is available then keep going. otherwise break out
            //    and try to connect again
            ssize_t n = read(serial_fd_, buf, sizeof(buf));
            if (n > 2) {
               parse_buffer(buf, (uint32_t) n);
            }
            usleep(250000);
         } while ((sock_fd_ >= 0) && (!quit_));
      } else {
         sleep(2);
      }
   }
   shutdown_connections();
   rc = 0;
end:
   return rc;
}

