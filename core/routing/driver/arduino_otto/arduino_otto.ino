#include <math.h>
#if !defined(TEST_HARNESS)
#include <elapsedMillis.h>
#endif   // TEST_HARNESS

#if !defined(MIN)
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#endif   // MIN

elapsedMillis nap_timer_;

// control theory
// modified PID controller is used. P & D are combined, with P being 
//    being damped when D is high and actuator is nearer center
// DPS threshold is based on how many seconds it will take to reach
//    target heading based on present turn rate. maximum tiller deflection
//    (a gating of proportional response) is limited based on multiple
//    of turn rate over threshold
// limit is only applied to tiller when movement is toward set point.
//    when moving away from set point, DPS limit does not apply
// I is implemented as a running average of tiller position (calculated
//    only after turns are complete) and P is applied relative to this
//    running average position

#define PIN_EXTEND       9
#define PIN_RETRACT     10
#define PIN_POS_ADC     A0

#define RAW_EXTEND      1
#define STOP            0
#define RAW_RETRACT     -1
// these aren't techncally constants as they can be overridden during
//    setup if actuator extend/retract is inverted compared to what's
//    expected
// TODO add logic to handle inverting these
static int32_t EXTEND       = RAW_EXTEND;
static int32_t RETRACT      = RAW_RETRACT;

#define STROKE_LENGTH_INCH 11.5f
#define EXTENSION_INCH (1.0f / STROKE_LENGTH_INCH)

#define DEGS_FOR_MAX_DEFLECTION     45.0f

// how often tiller data is updated
#define UPDATE_INTERVAL_MSEC     100

// note that it takes tiller several seconds, under ideal conditions, 
//    to move from full deflection to centered. max deflection should
//    be large enough so that tiller has time to come to non-extreme
//    position when nearing set point
// turn limit is how long it takes to reach desired heading at present
//    rate of turn. rudder deflection is reduced when vessel predicted
//    to reach desired heading in less than this interval
#define DPS_TURN_LIMIT_SECS      5.0f
// don't limit turn rates lower than pass limit
#define DPS_PASS_LIMIT           0.5f

// max deflection of actuator from integral portion of PID
// value is applied +/- to 0.5
#define INTEGRATION_RANGE_LIMIT     0.20f
#define MIN_ACTIVE_CENTER     (0.5f - INTEGRATION_RANGE_LIMIT)
#define MAX_ACTIVE_CENTER     (0.5f + INTEGRATION_RANGE_LIMIT)

// integration is disabled after a course change is requested
// hard blackout is set when new course selected. it disables integration
//    (error accumulation and correction) for X milliseconds per degree
//    change
#define INT_BLACKOUT_MS_PER_DEG     100
// soft blackout is removed when turn rate falls below threshold, 
//    indicating near target course
#define INT_TURN_RATE_RESUME_THRESH_DPS   1.0f

// implemented as keeping running average of tiller position. this is
//    considered 'center' that proportional adjustment is applied to
#define INTEGRATION_TAU_SEC      8.0f

#define MAX_SPEED    255
#define MIN_SPEED    64

////////////////////////
#define SERIAL_PACKET_START   0x81
#define SERIAL_PACKET_END     0x82
#define SERIAL_DEBUG_MASK     0x90

// expected heading updates received by driver, per second (1/interval)
// slightly overestimate updates per second so that movement command from
//    previous update doesn't fully complete by time next update arrives.
//    this is to keep tiller motion less jerky
#define DRIVER_UPDATE_PER_SEC           2.2f
#define MAX_EXTENSION_INCH_PER_SECOND       2.0f
#define MAX_EXTENSION_INCH_PER_UPDATE       \
      (MAX_EXTENSION_INCH_PER_SECOND / DRIVER_UPDATE_PER_SEC)

#define MAX_EXTENSION_PER_UPDATE             \
      (MAX_EXTENSION_INCH_PER_UPDATE / STROKE_LENGTH_INCH)

// constants
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// control variables
// NOTE value set in init_globals() below

// use abstract representation of tiller position so we don't hard-code
//    assumptions about what values we get for extension and retraction
// 0.0 represents fully retracted
// 1.0 represents fully extended
float actuator_position_; // = 0.0f;
float active_center_; // = 0.5f;

// desired tiller position for a given point in time
float target_position_; // = 0.5f;

// measured position range
// extension range should be on 0,1023. move bounds in a bit
// with blue on ground and white on 3.3v, retracted value is max
int32_t mx_position_extended_; //  =  110;
int32_t mx_position_retracted_; // = 1005;


// error integration (I in PID)
// integration accumulator and correction active only after a course
//    change is initiated and DPS has falled back near zero, indicating
//    that turn is near complete
int32_t integration_active_; //   = 0;
// timer to control hard blackout of integration coorection
elapsedMillis integration_timer_; // = 0;


////////////////////////
int32_t heading_; // = 0;
// valid course on 0,359. init to invalid as this sill center tiller
int32_t course_; // = 512;  
int32_t new_data_received_; // = 0;

int32_t debug_flag_; // = 0;

// turn rate is sent by driver. it's in unit of hundredths of 
//    degrees per second
float turn_rate_dps_; // = 0.0f;

////////////////////////
uint8_t serial_data_[8];
int32_t serial_num_samples_; // = 0;
int32_t serial_expected_samples_; // = 6;

// set if hardware problem detected (eg, power or calibration)
int32_t err_; // = 0;

int32_t debug_; // = 0;



////////////////////////////////////////////////////////////////////////
// driving logic

void extend_arm(int speed)
{
   analogWrite(PIN_EXTEND, speed);
   analogWrite(PIN_RETRACT, 0);
}

void retract_arm(int speed)
{
   analogWrite(PIN_EXTEND, 0);
   analogWrite(PIN_RETRACT, speed);
}

void stop_arm()
{
   analogWrite(PIN_EXTEND, 0);
   analogWrite(PIN_RETRACT, 0);
}

void read_actuator_position(void)
{
   int32_t raw_pos = analogRead(PIN_POS_ADC);
   // right now, resistance value is high and extend value is low. hard
   //    code that for now
   // position is 1.0 when extended and 0 when retracted
   //    pos = (1000 - raw) / (1000 - 100)
   //    0   = (1000 - 1000) / 900  // raw = 1000 when retracted
   //    1   = (1000 -  100) / 900  // raw = 100 when retracted
   actuator_position_ = (float) (mx_position_retracted_ - raw_pos) /
         (float) (mx_position_retracted_ - mx_position_extended_);
}


// compare target_position_ to max extension and retracted positions, and
//    if target is beyond either, bring it back to bounds
void apply_piston_bounds(void)
{
   if (target_position_ > 1.0f) {
      target_position_ = 1.0f;
   } else if (target_position_ < 0.0f) {
      target_position_ = 0.0f;
   }
}


// reduce tiller target position as function of rate of change of
//    error and how close to neutral target position is
// accuracy of tiller placement (actuator position) is assumed to be
//    correct. if reaching desired heading before TURN_LIMIT_SECS at
//    present rate of turn, tiller is moved toward center
void limit_piston_deflection_by_dps(
      /* in     */ const float course_delta_deg
      )
{
   if ((course_delta_deg * turn_rate_dps_) > 0.0f) {
      // positive value means neither is zero and both course error and
      //    present motion are in same direction (ie, same sign)
      // check to see if we need to limit tiller deflection
      // estimate how long it'll take to reach course based on present
      //    turn rate. if reach course too soon, limit tiller deflection
      // only limit if turn rate greater than pass rate (dps)
      if (fabs(turn_rate_dps_) > DPS_PASS_LIMIT) {
         float when = fabs((float) course_delta_deg / turn_rate_dps_);
         if (when > DPS_TURN_LIMIT_SECS) {
            // turn slow enough that we're forecast to reach target heading
            //    outside of turn time limit
            // no modification required
         } else if (when <= DPS_TURN_LIMIT_SECS / 2.0f) {
            // turn is too sharp -- we're reaching target heading in less
            //    than half of the time limit. bring tiller to neutral
            // if turn rate is in opposite direction of active center,
            //    move to active center as the active center deflection
            //    can't be causing the turn (eg, if active center is
            //    pushing boat to starboard but DPS limit is to port)
            // if turn is in direction of active center, bring tiller
            //    back to actual center
            if (turn_rate_dps_ > 0.0f) {
               // positive DPS means turning to starboard. if tiller is
               //    to starboard (active center < 0.5) then base limit
               //    on active position, otherwise go with center
               target_position_ = MIN(active_center_, 0.5f);
            } else {
               // negative DPS means turning to port. if tiller is to
               //    port (active center > 0.5) then base limit on
               //    active position, otherwise go with center
               target_position_ = MAX(active_center_, 0.5f);
            }
         } else { // DPS_TURN_LIMIT > when > DPS_TURN_LIMIT_SECS / 2.0f
            // turn is too fast -- we're reaching target heading between
            //    half of time limit and full limit (eg, between 2.5 and 5
            //    seconds). reduce deflection from 1 at time limit to
            //    0.5 (neutral) at twice limit
            // reduce deflection to zero at 2x dps limit. use same logic for
            //    using absolute or active center as in above case
            float center;
            if (turn_rate_dps_ > 0.0f) {
               // positive DPS means turning to starboard. if tiller is
               //    to starboard (active center < 0.5) then base limit
               //    on active position, otherwise go with center
               center = MIN(active_center_, 0.5f);
            } else {
               // negative DPS means turning to port. if tiller is to
               //    port (active center > 0.5) then base limit on
               //    active position, otherwise go with center
               center = MAX(active_center_, 0.5f);
            }
            float scale = 1.0f -
                  2.0f * (DPS_TURN_LIMIT_SECS - when) / DPS_TURN_LIMIT_SECS;
            float delta = (float) (target_position_ - center);
            target_position_ = center + delta * scale;
         }
      }
   }
   // this should be redundant, but make sure 
   if ((target_position_ < 0.0f) || (target_position_ > 1.0f)) {
      Serial.write(SERIAL_DEBUG_MASK);
      Serial.print("ERROR -- DPS limit for actuator put it out of range. ");
      Serial.print(target_position_);
      Serial.print(" ");
      Serial.print(course_delta_deg);
      Serial.print(" ");
      Serial.print(turn_rate_dps_);
      Serial.print(" ");
      Serial.print(active_center_);
      Serial.print("\n");
   }
   apply_piston_bounds();
}


// recalculate actuator position and issue command to move toward it
void update_actuator()
{
   /////////////////////////////////////////////
   // determine appropriate tiller position for course and set point
   // for now, use simple model of X" of extension per Y degrees delta,
   //    then limit extension to 5-6"
   if (course_ > 359) {
      // invalid course -- this is a signal to center tiller. and if
      //    it's an error and not signal, we should center tiller
      target_position_ = 0.5f;
   } else {
      //////////////////////////////////////////
      // I in PID
      if (integration_active_ == 0) {
         // see if it's time to turn integration back on again
         if ((integration_timer_ >= 0) && 
               (turn_rate_dps_ < INT_TURN_RATE_RESUME_THRESH_DPS)) {
            integration_active_ = 1;
            active_center_ = 0.5f;  // start at middle position
         }
      }
      if (integration_active_ == 1) {
         // integration logic
         // adjust what's considered tiller's neutral position to be
         //    running average of recent position. this represents the
         //    integrated control component
         const float k = 
               (float) UPDATE_INTERVAL_MSEC * 0.001f / INTEGRATION_TAU_SEC;
         active_center_ = k * (float) target_position_ + 
               (1.0f - k) * active_center_;
         // limit active center to region near actual center, to prevent
         //    runaway condition
         if (active_center_ < MIN_ACTIVE_CENTER) {
            active_center_ = MIN_ACTIVE_CENTER;
         } else if (active_center_ > MAX_ACTIVE_CENTER) {
            active_center_ = MAX_ACTIVE_CENTER;
         }
      } 
      //////////////////////////////////////////
      // P and D in PID
      float course_delta_deg = (float) (course_ - heading_);
      if (course_delta_deg > 180) {
         course_delta_deg -= 360;
      } else if (course_delta_deg <= -180) {
         course_delta_deg += 360;
      }
      // hard-code starboard side actuator placement
      // positive delta means need to turn right, which means 
      //    actuator extension
      target_position_ = active_center_ +
            0.5f * course_delta_deg / DEGS_FOR_MAX_DEFLECTION;
      apply_piston_bounds();
      limit_piston_deflection_by_dps(course_delta_deg);  // applies D to P
   }
   /////////////////////////////////////////////
   // movement can push actuator beyond desired set point. if delta is
   //    small, consider what we have as good enough
   // only move if position is > 1/4 inch out
   read_actuator_position();
   float pos_delta = fabs(actuator_position_ - target_position_);
   if (pos_delta > 0.25f * EXTENSION_INCH) {
      // calculate movement speed. speed should be such that next heading
      //    update is received shortly before movement is complete, in
      //    order to avoid jerky actuator responses 
      //    (eg, start/stop/pause/start/stop)
      int32_t speed = MAX_SPEED;

      // calc number of updates required to achieve desired motion
      float num_updates = pos_delta / MAX_EXTENSION_PER_UPDATE;
      if (num_updates < 1.0f) {
         // motion will likely be completed this update. slow down movement
         //    to stretch it out until next update
         speed = (int32_t) (255.0f * num_updates);
         if (speed < MIN_SPEED) {
            speed = MIN_SPEED;
         }
      }
      // move actuator
      if (actuator_position_ > target_position_) {
         extend_arm(speed);
      } else if (actuator_position_ < target_position_) {
         retract_arm(speed);
      } else {
         stop_arm();
      }
   } else {
      stop_arm();
   }
}

// driving logic
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// motion and calibration



void drive_actuator(int direction, int speed){
   if (direction == EXTEND) {
      extend_arm(speed);
   } else if (direction == STOP) {
      stop_arm();
   } else if (direction == RETRACT) {
      retract_arm(speed);
   } else {
      Serial.write(SERIAL_DEBUG_MASK);
      Serial.print("ERROR -- unknown driver actuator command ");
      Serial.print(direction);
      Serial.print("\n");
   }
}

int move_to_limit(int direction){
   int prev_reading=0;
   int curr_reading=0;
   elapsedMillis time_elapsed = 0;
   do {
      prev_reading = curr_reading;
      drive_actuator(direction, 255);
      time_elapsed = 0;
      while (time_elapsed < 200) { 
         delay(1);
      }  //keep moving until analog reading remains the same for 200ms
      curr_reading = analogRead(PIN_POS_ADC);
   } while (prev_reading != curr_reading);
   return curr_reading;
}


// motion and calibration
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// serial interface

void parse_heading_data()
{
   heading_ = (serial_data_[0] << 7) | serial_data_[1];
   uint16_t new_course = (uint16_t) ((serial_data_[2] << 7) | serial_data_[3]);
   int16_t turn_rate = (int16_t) ((serial_data_[4] << 7) | serial_data_[5]);
   // turn rate is signed. extend sign of bit 13 to bits 14 and 15
   if (turn_rate & 0x2000) {
      turn_rate = (int16_t) (turn_rate | 0xc000);
   }
   // turn rate is also integer version of floating point. each int value is
   //    1/100th of DPS
   turn_rate_dps_ = (float) turn_rate * 0.01f;
   //
   if (new_course != course_) {
      course_ = new_course;
      // disable error integration during turn. set hard-blackout period
      //    to be based on distance to turn (soft blackout is based on
      //    DPS turn rate)
      int32_t delta = new_course - heading_;
      if (delta > 180) {
         delta -= 360;
      } else if (delta < -180) {
         delta += 360;
      }
      integration_timer_ = -delta * INT_BLACKOUT_MS_PER_DEG;
      integration_active_ = 0;
   }
}

void check_heading_data()
{
   while (Serial.available() > 0) {
      uint8_t val = Serial.read();
      if (val == SERIAL_PACKET_START) {
         // start signal -- flush stream and start listening
         serial_num_samples_ = 0;
      } else if (val == SERIAL_PACKET_END) {
         if (serial_num_samples_ == serial_expected_samples_) {
            // packet complete. extract contents
            new_data_received_ = 1;
            debug_flag_ = 1;
            parse_heading_data();
         } // else, incomplete packet. drop what we have
         // prepare for start of next packet
         serial_num_samples_ = 0;
      } else if (serial_num_samples_ >= serial_expected_samples_) {
         // too much data -- must be corrupted. drop data and start
         //    over
         serial_num_samples_ = 0;
      } else {
         // data has arrived. cache it
         serial_data_[serial_num_samples_] = val;
         serial_num_samples_++;
      }
   }
}


void send_tiller_data()
{
   int32_t position = (int32_t) (1024.0f * actuator_position_);
   if (debug_ != 0) {
      position = (int32_t) (1024.0f * target_position_);
   }
   if (err_ == 1) {
      // power or calibration problem
      position = 2048;
   } else if (err_ != 0) {
      // unknown error
      position = 4096;
   }
   Serial.write(SERIAL_PACKET_START);
   Serial.write((position >> 7) & 0x7f);
   Serial.write((position     ) & 0x7f);
   // send reflect received data so driver can verify what's been received
   Serial.write((course_ >> 7) & 0x7f);
   Serial.write((course_     ) & 0x7f);
   Serial.write((heading_ >> 7) & 0x7f);
   Serial.write((heading_     ) & 0x7f);
   int32_t dps = (int32_t) roundf(turn_rate_dps_ * 100.0f);
   Serial.write((dps >> 7) & 0x7f);
   Serial.write((dps     ) & 0x7f);
   Serial.write(SERIAL_PACKET_END);
}

// serial interface
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// init

// set all static/global variables here. this allows them to be reset
//    by test code
void init_globals()
{
   actuator_position_ = 0.0f;
   active_center_ = 0.5f;
   target_position_ = 0.5f;
   //
   mx_position_extended_  =  110;
   mx_position_retracted_ = 1005;
   //
   integration_active_   = 0;
   integration_timer_ = 0;
   ////////////////////////
   heading_ = 0;
   // valid course on 0,359. init to invalid as this sill center tiller
   course_ = 512;  
   new_data_received_ = 0;
   debug_flag_ = 0;
   // turn rate is sent by driver. it's in unit of hundredths of 
   //    degrees per second
   turn_rate_dps_ = 0.0f;
   ////////////////////////
   serial_num_samples_ = 0;
   serial_expected_samples_ = 6;
   // set if hardware problem detected (eg, power or calibration)
   err_ = 0;
   debug_ = 0;
}


void init_bounds(int measure)
{
   if (measure != 0) {
      mx_position_extended_ = move_to_limit(EXTEND);
      mx_position_retracted_ = move_to_limit(RETRACT);
      Serial.write(SERIAL_DEBUG_MASK);
      Serial.print("Actuator retracted: ");
      Serial.print(mx_position_retracted_);
      Serial.print(" extended: ");
      Serial.print(mx_position_extended_);
      Serial.print("\n");
   }
   int32_t range = mx_position_retracted_ - mx_position_extended_;
   if (range <= 0) {
      // TODO if this is ever executed, add logic to handle both cases of
      //    retract being > or < extended
      Serial.write(SERIAL_DEBUG_MASK);
      Serial.print("ERROR -- unexpected min/max positions. retracted: ");
      Serial.print(mx_position_retracted_);
      Serial.print(" extended: ");
      Serial.print(mx_position_extended_);
      Serial.print("\n");
      err_ = 1;
   }
}

// init
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// main

void setup() 
{
   pinMode(PIN_EXTEND, OUTPUT);
   pinMode(PIN_RETRACT, OUTPUT);
   pinMode(PIN_POS_ADC, INPUT);
   //Serial.begin(9600);
   Serial.begin(19200);
   //Serial.begin(38400);
   //Serial.begin(57600);
   init_globals();
   init_bounds(0);
}


void loop() 
{
   // read updated heading and course data if it's arrived
   check_heading_data();
   if (err_ == 0) {
      // update tiller
      update_actuator();
   }
   // if data was received, send report on tiller position
   // if data sent only in response, sender can detect if stream broken
   if (new_data_received_ == 1) {
      send_tiller_data();
      new_data_received_ = 0;
   }
   // take a break. hold present state during break
   nap_timer_ = 0;
   while(nap_timer_ < UPDATE_INTERVAL_MSEC) { 
      delay(1);
   }
}

