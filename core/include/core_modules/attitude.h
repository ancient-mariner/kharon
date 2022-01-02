#if !defined(ATTITUDE_H)
#define ATTITUDE_H
#if !defined(_GNU_SOURCE)
#define _GNU_SOURCE
#endif   // _GNU_SOURCE
#include "pinet.h"
#include "time_lib.h"
#include "logger.h"

#include "core_modules/support/imu_streams.h"

/**
examines published IMU data from all producers that are subscribed to

publication is delayed to provide all IMU producers time to publish
   their own data. data is merged, as appropriate, filtered and then
   published

as of Dec 2019 (post-303), 'north' has been redefined. it used to be
   orthogonal to up, in the direction of north. it is now a float value
   that is (-heading), and not actively stored. the former north value 
   can be extracted from x component of the ship2world axis.

heading output now split between magnetic and true. other modules
   should use true heading to avoid possible conflicts w/ mapping system,
   which is tied to true
**/

#define INSERT_PHANTOM_IMAGE  0

// TODO extract config-loading code into independent function so
//    config data can be hot-reloaded. WHY??


// 5 seconds of data should be sufficient. at 100Hz, 512 bins should do
// right now data is only read in semi-real-time, to adjust images. 
//    if more history is required, increase this freely -- one second
//    of data takes very little storage: 100 * 12(floats) * 4(bytes) = ~4k/sec
#define ATTITUDE_QUEUE_LEN   512

#define ATTITUDE_CLASS_NAME  "Attitude"

#define ATTITUDE_LOG_LEVEL    LOG_LEVEL_DEFAULT

// amount of time to wait for data to arrive before generating consensus
//    view
#define DELAY_WINDOW_MS    70
#define DELAY_WINDOW_US    (DELAY_WINDOW_MS * 1000)

// sample duration defined in support/imu_streams.h

// bootstrap interval. when starting up, or after gyro signal has
//    been lost, the time constant of the complementary filter is
//    shortened to integrate more data when estimating attitude,
//    instead of using whatever signal that happens to be present 
//    on startup (the idea is to average out noise, but in rough
//    seas the interval would need to be considerably larger than
//    in normal cases)
#define BOOTSTRAP_INTERVAL_SEC   1.0

// max time since last receipt of MAG or ACC data where sensor is still
//    assumed to be functioning. if more than TIMEOUT_USEC elapse since
//    a sample was received, the channel is assumed to be offline
#define ACC_MAG_TIMEOUT_USEC  300000

/**
Previous iteration of attitude used a reset system where attitude info
would be unavailable if a fault was detected in the data stream. While
useful in principle, too many errors were observed that interfered with
image stream. New approach is to publish whenever gyro info is available,
and use whatever acc and mag is available to help correct that signal.
If all acc or mag sensors go offline, their previous values will be recycled
until they are back, and the data stream will not be interrupted.

Using obsolete acc/mag data is considered to be better than using nothing
as gyro will still drift, and static acc/mag data will keep it from spinning,
even if stabilized gyro position is incorrect (ie, not upright, and north
in the wrong place). When signal returns, it is allowed to slowly correct
gyro and avoid rapid attitude jumps. 
An auxiliary system will be required to help correct for such a scenario,
e.g., horizon tracking, motion of previously static objects. This is
a future task.
**/

// time for measured ACC and MAG to bring erroneous 'stable' values
//    ~63% of the way to their correct value, assuming non-biased gyro
// TODO analyze how stable values are affected by this and biased gyro
// use different constants for ACC and MAG. ACC is much noisier, esp. 
//    in rough seas, where disruption can occur over much larger time
//    scales. MAG sensor is noisy on short time scales but should be
//    relativley stable in the absence of someone (something) actively 
//    disrupting it. a long time constsant for MAG can perform poorly
//    in the case of a long, slow turn, as this induces an effective
//    gyro drift of potentially large magnitude. a short MAG TAU can
//    help reduce this (as can disabling high-pass filter / gyro drift
//    compensation on device)
// TODO evaluate constants -- we're not in rough seas most of the time.
//    perhaps have dynamic constant in fugure
#define FILTER_TIME_CONSTANT_ACC_SEC    30.0
#define FILTER_TIME_CONSTANT_MAG_SEC    30.0
#define COMPLEMENTARY_TAU_ACC    ((double) SAMPLE_DUR_SEC / FILTER_TIME_CONSTANT_ACC_SEC)
#define COMPLEMENTARY_TAU_MAG    ((double) SAMPLE_DUR_SEC / FILTER_TIME_CONSTANT_MAG_SEC)
// drift correction uses same time constant as used in complementary
//    filter. for tau=20 sec it takes ~3 minutes for drift correction
//    to be mostly engaged (error in output down to ~0.15%)

// non-biased gyro assumption is not sufficient. it can be non-biased
//    through drift correction, but sometimes drift correction fails.
//    predicted error for each measurement is now collected, based on
//    what complementary filter indicates attitude should be and what
//    acc/mag sensors indicate. error is averaged over time and is 
//    applied to complementary output to get estimated instantaneous attitude
// TODO implement kalman filter
#define FILTER_EST_ERROR_TAU_SEC    20.0

// when performing gyro-based alignment estimates, don't attempt
//    estimate if motion is too limited as otherwise noise will dominate
// alignment estimate will be made if all axes summed is greater than this
//    threshold (in dps)
// TODO add more comments -- number is strangely accurate w/o justification
#define MIN_ALIGNMENT_DPS     2.1

// when making alignment estimate, a running average is used in order
//    to reduce influence of noise. this is the time constant of that
//    running average. note that this should be a power of 2 (this relates
//    to bootstrapping the running average)
#define TARGET_ALIGNMENT_TAU     (1.0 / 256.0)
// how many samples are skipped between reporting alignment in log
#define ALIGNMENT_REPORTING_INTERVAL   64
// flag to indicate if auto-alignment should be performed (0 for no)
#define PERFORM_AUTO_ALIGNMENT      0
// TODO document and test auto-alignment

// pulls data from IMU receivers
// takes data from receivers based on what data's available and what priority
//    each receiver has
// runs complementary filter on [combined] data
// publishes rotation matrix ship-2-world xform matrix and other forms of
//    attitude data, including filtere ACC and MAG values
//

struct attitude_output {
   // rotation matrix for ship's sensor values to world space
   matrix_type ship2world;
   // filtered accelerometer and magnetometer values
   vector_type acc; 
   vector_type mag; 
   // magnitude of input acc and mag vectors (note: subject to being noisy)
   double acc_len;
   double mag_len;
   // most recent gyro reading
   vector_type gyr; 
   // derived values
   // compass heading(s)
   degree_type true_heading;
   // use awkward name for mag heading to reduce chance of accidental use
   degree_type mag_heading_reference;  
   degree_type pitch;   // degrees above/below horizon
   degree_type roll;    // left/right tilt
   //
   degree_per_second_type turn_rate;   // of heading
   // GPS data -- not exactly attitude related, but this is the
   //    best place for it w/o creating a new processor
   // TODO move this to position processor, possibly including
   //    other ship data (eg, wind direction and speed)
   world_coordinate_type gps;
   degree_type gps_heading;
   meter_per_second_type gps_speed;
   double last_gps_data;   // timestamp of most recent GPS data
};

typedef struct attitude_output attitude_output_type;

struct attitude_class {
   // logfile is for raw ratrix output; log is for regular logging
   FILE *logfile;
   log_info_type *log;
   // timestamp of next publication time
   microsecond_timestamp_type next_publish_time;
   // acc and mag stream data is used in complementary filters. the
   //    time constant for the filter is long compared to the sampling
   //    interval, so only the most recent acc and mag values are used
   //    in the filters. any artifacts introduced by this simplification
   //    should average out
   // all gyr streams are resampled at 10ms time boundaries and combined
   simple_vector_stream_type     *acc_stream[MAX_ATTACHED_PRODUCERS];
   simple_vector_stream_type     *mag_stream[MAX_ATTACHED_PRODUCERS];
   resampled_vector_stream_type  *gyr_stream[MAX_ATTACHED_PRODUCERS];
   // keep track of how many P1 sources there are for each modality
   uint32_t num_p1_gyr;
   uint32_t num_p1_acc;
   uint32_t num_p1_mag;
   // present state. acc,mag are filtered values and unit vectors
   vector_type comp_acc;
   vector_type comp_mag;
   vector_type corrected_acc;
   vector_type corrected_mag;
   vector_type gyr;
   double acc_len;
   double mag_len;
   // to estimate rate of turn, keep a record of previous heading
   //    and when it was measured
   // seperate varialbles for calculating this stinks of a hack.
   // TODO use already-available data to calculate this
   degree_type mag_heading;
   double heading_sec; // when heading was measured
   degree_per_second_type turn_rate;
   /////////////////////////////////////////////////////////////
   // error estimation
   // estimated error on each axis. this is an average of difference
   //    between acc/mag and complementary filtered acc/mag
   vector_type est_error_acc;
   vector_type est_error_mag;
   /////////////////////////////////////////////////////////////
   // AUTO DEVICE ALIGNMENT based on agreement between gyros
   // TODO explain where this is used (or if it's fully implemented)
   // measure alignment between gyros through their axis of rotations
   // this is starting as a way to get a better alignment than using
   //    magnetometers on camera install, as that's been problematic
   //    (eg, metal docks). moving forward, this can be a way to
   //    automatically align devices using vessel motion, and can
   //    then compensate if a device is moved (given enough time)
   // lowest index P1 gyro is used as reference. all others are measured
   //    against it (this should be the first P1 gyro defined in the config)
   // NOTE measured alignment is the error of the dev2ship matrices
   //    for the non P1A gyros
   uint32_t master_gyro_idx;
   // running average of rotation axis and angle between sensor and ship
   vector_type rotation_axis[MAX_ATTACHED_PRODUCERS];
   // actual angle is determined from rotation axis, which is average
   //    cross product of vectors from 2 sensors. however, because
   //    the cross product only indicates how orthogonal two vectors
   //    are, vectors that are nearly opposite will have same cross as
   //    vectors that are nearly the same. use rotation_theta (based on
   //    dot product) to get an approximate magnitude of the rotation
   //    (use that to determine if cross product is for >90 or <90)
   degree_type rotation_theta[MAX_ATTACHED_PRODUCERS];
   uint32_t rotation_samples[MAX_ATTACHED_PRODUCERS];
   double active_tau; // starts at 1.0 and reduces until at target
   /////////////////////////////////////////////////////////////
   // bootstrap timer. affects time constant complementary filter when
   //    starting up, or when recovering after lost signal
   second_type init_timer;
};
typedef struct attitude_class attitude_class_type;


// thread entry point
void * attitude_class_init(void *);

// struct to pass config data to thread
struct attitude_setup {
   uint32_t logging;
};
typedef struct attitude_setup attitude_setup_type;


////////////////////////////////////////////////////////////////////////

//// GOOD means query was fullfilled
//// PENDING means data not yet available to answer query
//// MISSING means that requested data is missing
enum attitude_query_state { NA, FOUND, PENDING, MISSING };

// fetch attitude at specified time
// index of found sample is stored in idx
void get_attitude(
      /* in     */ const datap_desc_type *dp, 
      /* in out */       log_info_type *log,
      /* in     */ const double t, 
      /*    out */       enum attitude_query_state *status,
      /*    out */       attitude_output_type *mat,
      /*    out */       uint64_t *idx
      );

// start searching at idx for sample occurring at t
// index of found sample is stored in idx
void get_attitude_since(
      /* in     */ const datap_desc_type *dp, 
      /* in     */ const double t, 
      /*    out */       enum attitude_query_state *status,
      /*    out */       attitude_output_type *mat,
      /* in out */       uint32_t *prev_idx
      );

// sets magnetic declination (called by mapping system on map refresh)
void set_declination(
      /* in     */ const declination_type decl
      );

#endif   // ATTITUDE_H

