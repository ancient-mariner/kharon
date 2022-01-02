#if !defined(PIN_TYPES_H)
#define PIN_TYPES_H
#if !defined(_GNU_SOURCE)
#define _GNU_SOURCE
#endif   // _GNU_SOURCE
#include <stdint.h>
#include <math.h>


#define STR_LEN   (256u)

#define HOST_LEN  (32u)  // max supported hostname length

////////////////////////////////////////////////////////////////////////

//// until pix/deg is multiple of 4, use only 2 pyramid levels
#define NUM_PYRAMID_LEVELS    2

// maximum supported number of input cameras
#define MAX_NUM_CAMERAS    8

////////////////////////////////////////////////////////////////////////
// runtime "constants"
// normaly CAPS denotes a constant. several of are technically not, 
//    but practically they are, as they are set on initialization 
//    and don't change. it's a crime against style, but a victimless one
// set via runtime's init_globals; updated via set_ppd()
extern double PIX_PER_DEG[NUM_PYRAMID_LEVELS];
extern double DEG_PER_PIX[NUM_PYRAMID_LEVELS];

extern uint32_t BAM32_PER_PIX[NUM_PYRAMID_LEVELS];


// coords in pixels
// NOTE: if world width or height goes above 65K, image size and other
//    structs need to be updated to bigger storage classes
// set via runtime's init_globals; updated via set_ppd()
extern uint32_t WORLD_HEIGHT_PIX[NUM_PYRAMID_LEVELS];
extern uint32_t WORLD_WIDTH_PIX[NUM_PYRAMID_LEVELS];

// world view
// world is 360 degrees horizontally
#define WORLD_WIDTH_DEGS   360.0
// vertical is configuable
// set via runtime's init_globals; updated via set_view_[above|below]_horizon()
extern double WORLD_HEIGHT_ABOVE_HORIZ_DEGS;
extern double WORLD_HEIGHT_BELOW_HORIZ_DEGS;
extern double WORLD_HEIGHT_DEGS;


// a flag that's set once code reads from these global variables
// if the variables are subsequently changed, this provides a way to
//    identify that and error-out (not a fool-proof way, but better than
//    nothing...)
extern int globals_accessed_;

// sets pixels-per-degree and global variables that depend on it
void set_ppd(
      /* in     */ const double above_horizon
      );

// sets WORLD_HEIGHT*
void set_world_height(
      /* in     */ const double above_horizon, 
      /* in     */ const double below_horizon
      );


////////////////////////////////////////////////////////////////////////
//// networking

#if !defined(HOST_NAME_MAX)
#define HOST_NAME_MAX   64
#endif

#define MAX_IP_LEN   32

// timestamps are sent as text over the network
// this is the size of the buffer that they're written to
// timestamps are in seconds and are generally to 4 significant digits: %.4f
// one year of seconds is approx: 
//     +31500000.0000
//     12345678901234
// -> 15 digits (14 + null)
// 100 years of seconds requires 17 digits. 20 is safe
#define TIMESTAMP_STR_LEN    20

// package describing network target
struct network_id {
   char  ip[MAX_IP_LEN];
   uint32_t   port;
};
typedef struct network_id network_id_type;

// header that is to precede sensor data sent over the network
// dt is the amount of time, in seconds, since the previous packet
//    was sent. negative value indicates no previous packet
// custom fields are for use by sensor streams
// all ints are stored in network byte order, and floats are 
//    stored as text strings
#define SENSOR_PACKET_LOG_DATA   64
struct sensor_packet_header {
   uint32_t sensor_type;
   union {
      int16_t custom_16[4];
      int32_t custom_32[2];
   };
   // 
   char timestamp[TIMESTAMP_STR_LEN];
   // optional 2nd time. for camera, this is when frame was delivered to
   //    the application, with regular timestamp being approximate
   //    time of acquisition start
   char timestamp_2[TIMESTAMP_STR_LEN];   
   // consider moving sync service to pi node, as part of pi_super,
   //    with hub being receiver. possibly split frame sync from time sync
   char log_data[SENSOR_PACKET_LOG_DATA];
};
typedef struct sensor_packet_header sensor_packet_header_type;

#define IMU_PACKET_TYPE   (0x11235001)

//#define YUV_PACKET_TYPE   (0x11235002)
//// custom_s[0] is img height (rows), in pixels
//// custom_s[1] is img width (cols), in pixels
//
//// uvy packet is derived from yuv. yuv has one full-resolution Y channel
////    and two half-resolution uv channels. uvy has equal resolution y,u,v
////    channels (implementation: y is downsampled while uv are sent)
//#define UVY_PACKET_TYPE   (0x11235003)
//// custom_s[0] is img height (rows), in pixels
//// custom_s[1] is img width (cols), in pixels

// vy packet is derived from yuv. yuv has one full-resolution Y channel
//    and two half-resolution uv channels. vy has equal resolution y and v
//    channels (implementation: y is downsampled while v is sent; u is 
//    dropped)
#define VY_PACKET_TYPE   (0x11235004)
// custom_s[0] is img height (rows), in pixels
// custom_s[1] is img width (cols), in pixels

#define GPS_PACKET_TYPE   (0x11235005)


// NOTE: style inconsistency -- sensor packet header represents serialized
//    version of packet while imu sensor packet represents unserialized
//    packet, which must be manually converted to/from serialized version
// TODO fix this some day

/////////////////////////////
// inter-node synchronization
//
// packet to both synchronize all nodes on the network and also
//    send commands (e.g., request frame capture from camera)
struct udp_sync_packet {
   uint16_t  packet_type;
   char  timestamp[TIMESTAMP_STR_LEN]; // time stored as %.4f
};

// update system time based on contents of this packet
#define UDP_SYNC_PACKET_TIME              0x0001
// trigger frame capture from cameras
#define UDP_SYNC_PACKET_FRAME_CAPTURE     0x0002
// halt network data streams (local acquisition still OK)
// used internal by udp_sync to stop network traffic before sync
#define UDP_SYNC_PACKET_PAUSE             0x0004
// resume network data streams
// used internal by udp_sync when stopping network traffic before sync
#define UDP_SYNC_PACKET_CONTINUE          0x0008
// start data acquisition streams (packet initiated by external process)
#define UDP_SYNC_PACKET_START_ACQ         0x0010
// stop data acquisition streams (packet initiated by external process)
#define UDP_SYNC_PACKET_STOP_ACQ          0x0020
// tell listening processes to shutdown
#define UDP_SYNC_PACKET_EXIT              0x8000
// NOTE: receipt of any UDP packet cannot be assumed to occur


// communication protocol: v2.3
// udp packet sends periodic synchronization signal (eg, every 2 minutes)
// sync signal timed to occur when network activity is paused
// problem with v2.0 protocol (ie, each frame sync includes sync time)
//    resulted in bad syncs, presumably due network congestion (though
//    that congestion shouldn't have occurred in the given configuration)
// IMU streams constantly, unless paused
// camera switched to using constant streaming of images at near
//    the maximum rate. the camera app (this) keeps track of when the
//    next desired frame is to be and then takes the first frame that
//    arrives after this time (hardware sync not possible w/ rpi cams)
//
// communication protocol must take into account delay in frame capture
// TODO revisit finding out approx capture/exposure time
// sending command to (pi) camera to capture image takes approx. 3x 
//    exposure time (ie, 20ms exposure -> 60ms delay before frame ready)
// udp design is to control/synchronize frames between cameras. this only
//    reliable if signal gets through to all cams at same time. if one
//    network link (eg, cam3) is saturated w/ frame upload while another
//    (eg, cam4) is open then request delivery to one cam will be 
//    delayed relative to other. also, must be confident that receipt of
//    message will induce capture -- if camera process is busy uploading
//    image when message comes in then it may not be ready to start
//    capture yet, so frame can be lost
// bandwith % approximations below are based on ideal conditions, and do
//    not take into account delays for frame capture
// design idea: make 2 new threads in camera process, 1st for accepting
//    udp commands and 2nd sending frame to receiver. cam copies buffer
//    to 2nd thread when image ready (eg, memcpy). 2nd waits for next
//    capture request to arrive before sending frame. 1st can use
//    two pthread condition signals to sync main and 2nd thread.
//    frame delivery takes for 1 full acq cycle plus transfer time 
//    (eg, 350ms)
// problem: network will be saturated when IMU data for presently
//    acquired frame is happening, so arrival of positional packets
//    will be delayed. this can be compensated-for if udp packet sends
//    master time and each IMU packet uses this for clock sync, and sends
//    time of imu acquisition. then exact arrival time of packets not 
//    important
// network traffic will go in bursts, sync'd to bcast of udp packet.
//    network saturation follows bcast until all frame data delivered.
//    imu packets delivered during this time. after network quiets down,
//    imu packets continue, but on nearly quiet network. all data packets
//    send timestamp, sync'd by latest udp packet arrival time, which should
//    be ~constant (for practical intents/purposes) at each device.

// udp sender is default process on server. possible exception to default
//    is if multiple senders are possible. so no default, but make it
//    mandatory.
// udp receiver is independant thread in each sensor process
// some receivers only update time
// others trigger actions
// all receivers get the same UDP packets
//
// udp_sync_receiver

////////////////////////////////////////////////////////////////////////
// VY 
// derived from YUV. camera process (on RPi) takes YUV signal, strips out U,
//    and then downsamples Y to match V size. YUV buffer is removed and
//    only raw image is sent.
// camera assumed to be using 2x2 pixel binning. alternatives are possible.
// downsampling a full-frame image (3280x2464) takes approx 32ms
// downsampling a half-resolution image (1640x1232) takex approx 8ms
// stripping border from YUV image takes <1ms
//
// frames arrive approx every 28ms. if stream processing hasn't completed
//    by time next frame arrives, that following frame is dropped

// Acquisition size: 2048x1536
// VY size: 1024x768 = 786432 x 2 = 1572864
// @6Hz -> 9.4MB = 75.5Mb   (76%)
// @5Hz -> 7.9MB = 62.9Mb   (63%)

// Acquisition size: 1640x1232 (1/2 resolution)
// VY size: 820x616 (1/4 resolution) = 505120 x 2 = 1010240
// @10Hz -> 10.1MB = 80.8Mb   (81%)
// @5Hz -> 5.1MB = 40.4Mb   (40%)

#define IMAGE_ACQUISITION_COLS   1640u
#define IMAGE_ACQUISITION_ROWS   1232u

// published image size
#define CAM_COLS      (IMAGE_ACQUISITION_COLS / 2)
#define CAM_ROWS      (IMAGE_ACQUISITION_ROWS / 2)
#define CAM_N_PIX     (CAM_COLS * CAM_ROWS)

// internal size (camera)
#define YUV_Y_COLS   1664u
#define YUV_Y_ROWS   1232u
#define YUV_V_COLS   (YUV_Y_COLS / 2)
#define YUV_V_ROWS   (YUV_Y_COLS / 2)

#define YUV_Y_PIX    (YUV_Y_COLS * YUV_Y_ROWS)
#define YUV_U_PIX    (YUV_Y_PIX / 4)
#define YUV_V_PIX    (YUV_Y_PIX / 4)
#define YUV_IMAGE_SIZE  (YUV_Y_PIX + YUV_U_PIX + YUV_V_PIX)

#define YUV_Y_OFFSET    0
#define YUV_U_OFFSET    YUV_Y_PIX
#define YUV_V_OFFSET    (YUV_Y_PIX + YUV_U_PIX)

#define YUV_BUFFER_SIZE  (YUV_Y_PIX + YUV_U_PIX + YUV_V_PIX)

// TODO evaluate stream processing
// camera feed must remove any YUV boundary from image and only
//    deliver image data
// TODO implement blurring where appropriate (e.g., before edge detection). delivered image may have artifacts, esp. on Y channel, due downsampling method (average of 2x2 bin)

#define  CAMERA_FPS         (6)
#define  CAMERA_FRAME_INTERVAL         (1.0 / ((double) CAMERA_FPS))
#define  UDP_SYNC_INTERVAL             CAMERA_FRAME_INTERVAL

// approximate field of view of camera sensor
#define VY_FOV_HORIZ_DEG   62.2
#define VY_FOV_VERT_DEG   48.8

#define VY_FOV_DIAG     73.8

// camera hardware presently doesn't support external frame triggers, so
//    cameras are set to stream at high frame rates and only frames
//    arriving near a desired time (effective frame rate/interval) are
//    kept
// approximate rate of frame capture at hardware level, so all
//    kept frames should be within a window of ~this size
#define HARDWARE_FRAME_RATE      36
#define HARDWARE_INTERFRAME_INTERVAL   (1.0 / HARDWARE_FRAME_RATE)

////////////////////////////////////////////////////////////////////////
// shared 

//#define  CAMERA_FRAME_INTERVAL         (1.0 / 6.0)

////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////
// common data types

// define specific types for array data, so compiler can help spot
//    stupid errors

struct vector_type {
   double v[3];
};
typedef struct vector_type vector_type;

struct matrix_type {
   double m[9];
//   struct {
//      vector_type row1;
//      vector_type row2;
//      vector_type row3;
//   };
};
typedef struct matrix_type matrix_type;

// "IMU" is used here generically to mean navigation or environment
//    sensor

enum { IMU_ACC=0, 
         IMU_MAG, 
         IMU_GYR, 
         IMU_GPS, 
         IMU_BARO, 
         IMU_TEMP, 
         NUM_IMU_CHANNELS };

// state stores availability of each data channel
// 1 indicates channel data is available, 0 for not
union imu_modality_state {
   uint8_t avail[NUM_IMU_CHANNELS];
   uint64_t flags;
};
typedef union imu_modality_state imu_modality_state_type;


// IMU sensor data 
struct imu_sensor_packet {
   struct vector_type gyr;
   struct vector_type acc;
   struct vector_type mag;
   struct vector_type gps; // x,y,z : lon,lat,ht
   double temp;
   double baro;
   // keep track of which have valid data
   imu_modality_state_type state;
   double timestamp;
};
typedef struct imu_sensor_packet imu_sensor_packet_type;

struct second {
   double seconds;
//   double t;
};
typedef struct second second_type;

struct dt_second {
   double dt_sec;
};
typedef struct dt_second dt_second_type;

////////////////////////////////////////////////////////////////////////
// binary angular measurement
// NOTE 
//    all operations should be performed on .angleXX
//    .sangleXX should be read-only
// C specification has wrapping of signed ints as undefined (even though it
//    usually works in an intuitive way)

union bam8 {
   uint8_t angle8;
   int8_t sangle8;
};
typedef union bam8 bam8_type;

// direct degree->BAM conversions are not safe due C specification 
//    ambiguities in converting from float to integer when out of rage. they
//    usually work as expected but in edge cases they can fail. constants
//    should not be used in code w/o care. underscore appended to discourage
//    use

#define DEG_TO_BAM8_  (256.0 / 360.0)
// BAM to degree is safe as all operations here should be defined
#define BAM8_TO_DEG   (360.0 / 256.0)
//#define BAM8_TO_DEG_FLT   ((float) BAM8_TO_DEG)
//#define DEG_TO_BAM8_FLT   ((float) DEG_TO_BAM8)

// bam16 (formerly scoresec)
// one 'scoresec' represents approx 20 (ie, one score) arc-seconds
// there are 65536 scoresec for 64800 20-second arcs in a circle, yielding
//    an error of ~1.12%. one scoresec is approx 19.78 arc-seconds
union bam16 {
   uint16_t angle16;
   int16_t sangle16;
};
typedef union bam16 bam16_type;

// scoresec. a scoresec is approx one-score (ie, 20) arc-seconds
// one bam16 is approx 20 arc-seconds
#define BAM16_TO_DEG   (360.0 / 65536.0)
#define DEG_TO_BAM16_  (65536.0 / 360.0)

//#define BAM16_TO_DEG_FLT   ((float) BAM16_TO_DEG)
//#define DEG_TO_BAM16_FLT   ((float) DEG_TO_BAM16)


// discrete angle -- one circumfrence is 2^32 dangles. one dangle
//    is approx 0.0003 arc seconds
// this has same concept as scoresec, just w/ much higher resolution
union bam32 {
   uint32_t angle32;
   int32_t sangle32;
};
typedef union bam32 bam32_type;


union bam64 {
   uint64_t angle64;
   int64_t sangle64;
};
typedef union bam64 bam64_type;


#define DEG_TO_BAM32_     (65536.0 * 65536.0 / 360.0)
#define BAM32_TO_DEG      (360.0 / (65536.0 * 65536.0))

//#define BAM32_TO_DEG_FLT      ((float) BAM32_TO_DEG)
//#define DEG_TO_BAM32_FLT      ((float) DEG_TO_BAM32)

// when representing world space, x=0 is 0 longitude. x increases moving
//    eastward. y=0 is north pole, 2^31 is south pole, thus values are 
//    increasing going right and down, like in an image. y values above 
//    2^31 are undefined. 
union bam32_coordinate {
   struct {
      bam32_type x;
      bam32_type y;
   };
   uint64_t all;
};
typedef union bam32_coordinate bam32_coordinate_type;

////////////////////////////////////////////////
// degree to BAM conversion
// wrapping ints is technically an undefined behavior that may break
//    at high optimization levels
// casting negative float to uint is technically undefined and may break
//    at high optimization levels
// (compilers usually behave intuitively, but that's not guaranteed and
//    using a constant like '(uin32_t) (-3.1)' will break at -O2)
//
// provide macros for performing BAM operations, to try and stay w/in the
//    language specs

// to convert float to bam32:
//    bam32_type heading;
//    double heading_deg = get_heading();
//    heading.sangle32 = (int32_t) round(heading_deg * DEG_TO_BAM32)

// degs to bamXX
// degs can be on -180 to 360 (possibly beyond) which will overflow sangle32
// convert to int64 before recasting as uint32
#define CVT_DEG_TO_BAM32(deg, bam)    do                       \
   {                                                           \
      bam64_type tmp;                                          \
      tmp.sangle64 = (int64_t) round((deg) * DEG_TO_BAM32_);   \
      bam.angle32 = (uint32_t) tmp.angle64;                    \
   } while (0)

#define CVT_DEG_TO_BAM16(deg, bam)    do                       \
   {                                                           \
      bam32_type tmp;                                          \
      tmp.sangle32 = (int32_t) round((deg) * DEG_TO_BAM16_);   \
      bam.angle16 = (uint16_t) tmp.angle32;                    \
   } while (0)

#define CVT_DEG_TO_BAM8(deg, bam)    do                        \
   {                                                           \
      bam32_type tmp;                                          \
      tmp.sangle32 = (int32_t) round((deg) * DEG_TO_BAM8_);    \
      bam.angle8 = (uint8_t) tmp.angle32;                      \
   } while (0)

// bamXX to degs. it's safe to perform operation directly so this is
//    more of a shorthand to be used when printing
#define CVT_BAM32(bam)        ((double) bam.angle32 * BAM32_TO_DEG)
#define CVT_BAM32_S(bam)      ((double) bam.sangle32 * BAM32_TO_DEG)


// binary angular measurement
////////////////////////////////////////////////////////////////////////

struct true_heading {
   bam32_type tru;   // 'true' can be a keyword
//   uint32_t true_angle32;
//   int32_t true_sangle32;
};
typedef struct true_heading true_heading_type;

union mag_heading {
   bam32_type mag;
//   uint32_t mag_angle32;
//   int32_t mag_sangle32;
};
typedef union mag_heading mag_heading_type;

union degree {
   double degrees;
   double deg;  // shorthand
};
typedef union degree degree_type;

union declination {
   double degrees;
   double deg;  // shorthand
};
typedef union declination declination_type;

// angle of altitude (up is positive, down is negative)
// horizon is 0
struct altitude_angle {
   bam32_type alt;
//   int32_t alt_sangle32;
//   double altitude_degrees;
//   double alt_deg;    // shorthand
};
//typedef union altitude altitude_type;
typedef struct altitude_angle altitude_angle_type;

union altitude_degree {
   double altitude_degrees;
   double altitude_deg;
   double alt_deg;    // shorthand
};
typedef union altitude_degree altitude_degree_type;

struct altitude_time {
   double t;
   union {
      double altitude_degrees;
      double alt_deg;
   };
};
typedef struct altitude_time altitude_time_type;


// bam32 in azimuth (left is negative, right positive)
struct azimuth_angle {
   bam32_type az;
//   uint32_t az_angle32;
//   int32_t az_sangle32;
};
typedef struct azimuth_angle azimuth_angle_type;

union azimuth_degree {
   double azimuth_degrees;
   double azimuth_deg;
   double az_deg;
};
typedef union azimuth_degree azimuth_degree_type;

struct azimuth_time {
   double t;
   union {
      double azimuth_degrees;
      double az_deg;
      //double azimuth_deg;
   };
};
typedef struct azimuth_time azimuth_time_type;


struct pixels_per_second {
   double pixels;
};
typedef struct pixels_per_second pixels_per_second_type;

struct degree_per_second {
   double dps;
};
typedef struct degree_per_second degree_per_second_type;

struct degree_per_minute {
   double dpm;
};
typedef struct degree_per_minute degree_per_minute_type;

struct motion_2d_dps {
   double x_dps;
   double y_dps;
};
typedef struct motion_2d_dps motion_2d_dps_type;

////////////////////////////////////////////////////////////////////////
// 


//
////////////////////////////////////////////////////////////////////////

///////////////////

struct radian {
   double radians;
};
typedef struct radian radian_type;

#define MPS_TO_KNOTS       1.943844
#define MPS_TO_KNOTS_FLT   1.943884f

#define KNOTS_TO_MPS       (1.0 / MPS_TO_KNOTS)
#define KNOTS_TO_MPSR_FLT  (1.0f / MPS_TO_KNOTS_FLT)

struct knot {
   double knots;
};
typedef struct knot knot_type;

struct meter {
   double meters;
};
typedef struct meter meter_type;

union meters_per_second {
   double mps;
   double meters_per_second;
};
typedef union meters_per_second meters_per_second_type;
// common mis-spelling
typedef union meters_per_second meter_per_second_type;

// wrapper for image dimensions to reduce problems with x/y/row/col
//    descriptions or access
union image_coordinate {
   struct { uint16_t y, x; };
   struct { uint16_t row, col; };
   uint32_t all;
};
typedef union image_coordinate image_coordinate_type;

// wrapper for relative dimensions 
union signed_coordinate {
   struct { int16_t y, x; };
   uint32_t all;
};
typedef union signed_coordinate signed_coordinate_type;


// coordinate type is too often used to denote size
// TODO migrate size references to use this struct and keep coordinates
//    for coordinates
// wrapper for image dimensions to reduce problems with x/y/row/col
//    descriptions or access
union image_size_type {
   struct { uint16_t y, x; };
   struct { uint16_t rows, cols; };
   struct { uint16_t height, width; };
   uint32_t all;
};
typedef union image_size_type image_size_type;


// image coordinate with uint32 storage
union image_coordinate32 {
   struct { uint32_t y, x; };
   struct { uint32_t row, col; };
   struct { uint32_t rows, cols; };
   uint64_t all;
};
typedef union image_coordinate32 image_coordinate_type32;
#define NUM_PIX(sz)   ( sz.x * sz.y )

// storage for signed offset between two points
// also stores time delta corresponding to this offset, for velocity
//    calculation
struct pixel_offset {
   union {
      struct { int16_t dy, dx; };
      struct { int16_t rows, cols; };
      uint32_t dxdy_all;
   };
   double dt;
};
typedef struct pixel_offset pixel_offset_type;
#define NULL_PIXEL_OFFSET    0x80008000;

// value to indicate no or invalid data are dx or dy = 0x80 (ie, -128)
union int16_2d_offset {
   struct { int16_t dy, dx; };
   struct { int16_t rows, cols; };
   uint32_t all;
};
typedef union int16_2d_offset int16_2d_offset_type;
#define NULL_INT16_2D_OFFSET    0x80008000;

// value to indicate no or invalid data are dx or dy = 0x80 (ie, -128)
union small_2d_offset {
   struct { int8_t dy, dx; };
   struct { int8_t rows, cols; };
   uint16_t all;   // init to 0x8080 to indicate invalid, or 0 to clear
};
typedef union small_2d_offset small_2d_offset_type;
typedef small_2d_offset_type int8_2d_offset_type;
#define NULL_SMALL_2D_OFFSET    0x8080;

struct degree_offset {
   double y_deg;
   double x_deg;
};
typedef struct degree_offset degree_offset_type;

struct meter_2d_motion {
   double x_mps;
   double y_mps;
};
typedef struct meter_2d_motion meter_2d_motion_type;

////////////////////////////////////////////////////////////////////////
// world and ocean coordinate types use the same names and units,
//    but these are stored in different structures to keep the code
//    more clear
// world:
// stores latitude [-90,90] and longitude [0,360) of a visual scene
//    projected onto a sphere. north latitude is up (+Y axis) and
//    0 longitude is world compass north (+Z axis), east lon=90, 
//    west lon=270 (+X axis)
////    NOTE: for some calculations, longitude adjusted to (-180,180]
////    so cannot be assumed to be positive  TODO deprecate negative degrees
// ocean varies from world only in definition 0 longitude, which is
//    the direction that the bow is pointing, when collapsed to the
//    horizontal plane (this does create a singularity when the bow
//    is pointed vertically)
union world_coordinate {
   struct { double latitude, longitude; };
   struct { double lat, lon; };
   struct { double y_deg, x_deg; };
};
typedef union world_coordinate world_coordinate_type;


// physical postion on the planet surface
union surface_coordinate {
   struct { double latitude, longitude; };
   struct { double lat, lon; };
   struct { double y_deg, x_deg; };
};
typedef union surface_coordinate surface_coordinate_type;


//// world position in degrees of lat and lon, with different terms for
////    accessing data, depending on context
//// lat,lon coords have origin at equator and greenwich longitude
//union world_position {
//   struct {
//      double lat;
//      double lon;
//   };
//   struct {
//      double latitude;
//      double longitude;
//   };
//};
//typedef union world_position world_position_type;


// world position in alaska-north coords (ie, dateline-north pole is origin)
union akn_position {
   struct {
      double akn_y;
      double akn_x;
   };
   struct {
      double akn_lat;
      double akn_lon;
   };
   struct {
      double akn_latitude;
      double akn_longitude;
   };
};
typedef union akn_position akn_position_type;


// physical postion on the planet surface
union degree_offset_2d {
   struct { double latitude, longitude; };
   struct { double lat, lon; };
   struct { double y_deg, x_deg; };
};
typedef union degree_offset_2d degree_offset_2d_type;


// physical postion on the planet surface
struct xy_distance {
   meter_type x, y;
};
typedef struct xy_distance xy_distance_type;


// coordinate w/ embedded time
struct world_time_coordinate {
   double t;
   union {
      struct { double latitude, longitude; };
      struct { double lat, lon; };
      struct { double y_deg, x_deg; };
   };
};
typedef struct world_time_coordinate world_time_coordinate_type;


// MISNOMER ALERT -- this is more accurately described as cylindrical
//    coordinate
//
// alternative to world_coordinate, for use when dealing with a sphere
//    (eg, for image projection) that doesn't require fine resolution of
//    world lat/lon
// DEPRECATED due wrap-around bug(s) -- use sphere_coordinate32
//union sphere_coordinate {
//   struct { float latitude, longitude; };
//   struct { float lat, lon; };
//   struct { float y_deg, x_deg; };
//   uint64_t all;
//};
//typedef union sphere_coordinate sphere_coordinate_type;

union sphere_coordinate32 {
   struct { bam32_type latitude, longitude; };
   struct { bam32_type lat, lon; };
   struct { bam32_type y_pos, x_pos; };
//   struct { float y_deg, x_deg; };
   uint64_t all;
};
typedef union sphere_coordinate32 sphere_coordinate32_type;



// world coordinate of point of gaze
// use different types for same data to let compiler enforce that the
//    intended one is used
struct gaze_coordinate {
   double x_deg;
   double y_deg;
};
typedef struct gaze_coordinate gaze_coordinate_type;


////////////////////////////////////////////////////////////////////////

struct ground_track {
   meters_per_second_type speed;
   degree_type course;
};
typedef struct ground_track ground_track_type;

struct bearing {
   double r; 
   double theta;
};
typedef struct bearing bearing_type;

struct bearing_time {
   union {
      struct { double r; double theta; };
      struct { double range; double degrees; };
   };
   double t;
};
typedef struct bearing_time bearing_time_type;


#endif   // PIN_TYPES_H
