
// flags for sounds to be played
// to request a sound, set the flag for a given sound to non-zero
union sound_request {
   struct {
      uint8_t    beep_left;
      uint8_t    beep_right;
      uint8_t    klaxon_left;
      uint8_t    klaxon_right;
      uint8_t    klaxon_alarm;
      uint8_t    klaxon_fullstop;
      uint8_t    ping_soft;
      uint8_t    klaxon_autopilot_error;
   };
   uint64_t all;
   int64_t signed_all;
};
typedef union sound_request sound_request_type;

static sound_request_type requests_;

////////////////////////////////////////////////////////////////////////

union stereo_sample {
   struct {
      int16_t left;
      int16_t right;
   };
   uint32_t all;
};
typedef union stereo_sample stereo_sample_type;

union stereo_buffer {
   uint8_t *bytes;
   stereo_sample_type *samples;
};
typedef union stereo_buffer stereo_buffer_type;

// if sampling rate changes here, modify in generate/replay and rebuild
//    appropriate sound files
#define BEEPER_SAMPLING_RATE   44100

// keep rate as variable as BEEPER_SAMPLING_RATE is only a request -- the 
//    actual rate may be different
static uint32_t rate_ = BEEPER_SAMPLING_RATE;

static uint32_t audio_buffer_num_bytes_ = 0;
static uint32_t audio_buffer_num_samples_ = 0;
static stereo_buffer_type audio_buffer_;

static snd_pcm_t *pcm_handle_ = NULL;

////////////////////////////////////////////////
#define FRAMES_PER_PERIOD  8192

// number of periods
const uint32_t periods_ = 4;
// number of frames (samples) in a period
const snd_pcm_uframes_t frames_per_period_ = FRAMES_PER_PERIOD; 

const uint32_t PERIOD_BYTES = FRAMES_PER_PERIOD * sizeof(stereo_sample_type);

////////////////////////////////////////////////
// sound buffers

//#define KLAXON_MAX   700.0f
//#define KLAXON_ALARM_MAX   (1.5f * KLAXON_MAX)
//#define BEEP_MAX     500.0f

static stereo_buffer_type klaxon_left_;
static stereo_buffer_type klaxon_right_;
static stereo_buffer_type klaxon_alarm_;
static stereo_buffer_type klaxon_fullstop_;
static stereo_buffer_type klaxon_autopilot_error_;

static stereo_buffer_type beep_left_;
static stereo_buffer_type beep_right_;

static stereo_buffer_type ping_soft_;

// statics, constants and globals
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// sound definitions

static void init_ping_soft(void)
{
   // use 1khz signal (based on sampling_rate = 44K), fading
   for (uint32_t i=0; i<BEEPER_SAMPLING_RATE-1; i++) {
      double di = (double) (BEEPER_SAMPLING_RATE - i);
      double sig = (double) PING_MAX * sin(di * 2.0 * M_PI / 44.0);
      double mag = sig * di / (double) BEEPER_SAMPLING_RATE;
      ping_soft_.samples[i].left = (int16_t) mag;
      ping_soft_.samples[i].right = (int16_t) mag;
   }
}

static void init_beep(
      /*    out */       stereo_buffer_type *buffer,
      /* in     */ const uint32_t *transitions,
      /* in     */ const uint32_t n_transitions,
      /* in     */ const double left_mag,
      /* in     */ const double right_mag
      )
{
   assert((n_transitions & 1) == 0);     // make sure it's even
   // use 440 hz sound (50 high, 50 low)
   int16_t left = (int16_t) left_mag;
   int16_t right = (int16_t) right_mag;
   for (uint32_t i=0; i<n_transitions-1; i+=2) {
      // start sound. interval[i] to interval[i+1]
      uint32_t pos = transitions[i];
      while (pos < transitions[i+1]) {
         // go high
         for (uint32_t j=0; j<50; j++) {
            buffer->samples[j + pos].left = left;
            buffer->samples[j + pos].right = right;
         }
         pos += 50;
         // go low
         for (uint32_t j=0; j<50; j++) {
            buffer->samples[j + pos].left = (int16_t) -left;
            buffer->samples[j + pos].right = (int16_t) -right;
         }
         pos += 50;
      }
   }
}


// one long beep, 3 short
#define NUM_BEEP_LEFT_TRANSITIONS    8
static void init_beep_left(void)
{
   uint32_t transition[NUM_BEEP_LEFT_TRANSITIONS] = {
         // start sample                  stop sample
         0,                               3 * BEEPER_SAMPLING_RATE / 8,
         4 * BEEPER_SAMPLING_RATE / 8,    9 * BEEPER_SAMPLING_RATE / 16,
         5 * BEEPER_SAMPLING_RATE / 8,   11 * BEEPER_SAMPLING_RATE / 16,
         6 * BEEPER_SAMPLING_RATE / 8,   13 * BEEPER_SAMPLING_RATE / 16
   };
   init_beep(&beep_left_, transition, NUM_BEEP_LEFT_TRANSITIONS,
         BEEP_MAX, BEEP_MAX * 0.5);
}

// two long beep
#define NUM_BEEP_RIGHT_TRANSITIONS    4
static void init_beep_right(void)
{
   uint32_t transition[NUM_BEEP_RIGHT_TRANSITIONS] = {
         // start sample                  stop sample
         0,                               3 * BEEPER_SAMPLING_RATE / 8,
         4 * BEEPER_SAMPLING_RATE / 8,    7 * BEEPER_SAMPLING_RATE / 8
   };
   init_beep(&beep_right_, transition, NUM_BEEP_RIGHT_TRANSITIONS,
         BEEP_MAX * 0.5, BEEP_MAX);
}


static void init_klaxon_twotone(
      /*    out */       stereo_buffer_type *buffer,
      /* in     */ const uint32_t *transitions,
      /* in     */ const uint32_t n_transitions,
      /* in     */ const double left_mag,
      /* in     */ const double right_mag
      )
{
   assert(n_transitions & 1);     // make sure it's odd
   // 
   int16_t left_base = (int16_t) left_mag;
   int16_t right_base = (int16_t) right_mag;
   uint32_t f[8] = { 0 };
   uint32_t half_cycle[8] = {
         13, 23, 33, 49,      // tone A
         11, 25, 41, 59       // tone b
   };
   for (uint32_t n=0; n<n_transitions-1; n+=2) {
      // interval[n] to interval[n+1] is tone A (0-3), [+1] to [+2] is B (1-4)
      uint32_t pos = transitions[n];
      while (pos < transitions[n+2]) {
         short left = 0;
         short right = 0;
         //
         if (pos < transitions[n+1]) {
            for (uint32_t i=0; i<4; i++) {
               if (f[i] < half_cycle[i]) {
                  left = (int16_t) (left + left_base / 2);
                  right = (int16_t) (right + right_base / 2);
               } else {
                  left = (int16_t) -(left + left_base / 2);
                  right = (int16_t) -(right + right_base / 2);
               }
               if (++f[i] > 2*half_cycle[i]) {
                  f[i] = 0;
               }
            }
         } else {
            for (uint32_t i=4; i<8; i++) {
               if (f[i] < half_cycle[i]) {
                  left = (int16_t) (left + left_base / 2);
                  right = (int16_t) (right + right_base / 2);
               } else {
                  left = (int16_t) -(left + left_base / 2);
                  right = (int16_t) -(right + right_base / 2);
               }
               if (++f[i] > 2*half_cycle[i]) {
                  f[i] = 0;
               }
            }
         }
         ////////////
         buffer->samples[pos].left = left;
         buffer->samples[pos].right = right;
         pos++;
      }
   }
}

static void init_klaxon(
      /*    out */       stereo_buffer_type *buffer,
      /* in     */ const uint32_t *transitions,
      /* in     */ const uint32_t n_transitions,
      /* in     */ const double left_mag,
      /* in     */ const double right_mag
      )
{
   assert((n_transitions & 1) == 0);     // make sure it's even
   // 
   int16_t left_base = (int16_t) left_mag;
   int16_t right_base = (int16_t) right_mag;
   uint32_t f[4] = { 0 };
   uint32_t half_cycle[4] = {
         17, 23, 31, 59
   };
   for (uint32_t n=0; n<n_transitions; n+=2) {
      // start sound. interval[n] to interval[n+1]
      uint32_t pos = transitions[n];
      while (pos < transitions[n+1]) {
         short left = 0;
         short right = 0;
         //
         if (pos < transitions[n+1]) {
            for (uint32_t i=0; i<4; i++) {
               if (f[i] < half_cycle[i]) {
                  left = (int16_t) (left + left_base / 2);
                  right = (int16_t) (right + right_base / 2);
               } else {
                  left = (int16_t) -(left + left_base / 2);
                  right = (int16_t) -(right + right_base / 2);
               }
               if (++f[i] > 2*half_cycle[i]) {
                  f[i] = 0;
               }
            }
         }
         ////////////
         buffer->samples[pos].left = left;
         buffer->samples[pos].right = right;
         pos++;
      }
   }
}

static void init_klaxon_falling(
      /*    out */       stereo_buffer_type *buffer,
      /* in     */ const double left_mag,
      /* in     */ const double right_mag
      )
{
   uint32_t f[4] = { 0 };
   uint32_t half_cycle[4] = {
         17, 23, 31, 59
   };
   // use 1khz signal (based on sampling_rate = 44K), fading
   for (uint32_t n=0; n<BEEPER_SAMPLING_RATE-1; n++) {
      double di = (double) (BEEPER_SAMPLING_RATE - n);
      double lmag = left_mag * di / (double) BEEPER_SAMPLING_RATE;
      double rmag = right_mag * di / (double) BEEPER_SAMPLING_RATE;
      //
      short left = 0;
      short right = 0;
      for (uint32_t i=0; i<4; i++) {
         if (f[i] < half_cycle[i]) {
            left = (int16_t) (left + lmag / 2);
            right = (int16_t) (right + rmag / 2);
         } else {
            left = (int16_t) -(left + lmag / 2);
            right = (int16_t) -(right + rmag / 2);
         }
         if (++f[i] > 2*half_cycle[i]) {
            f[i] = 0;
         }
      }
      buffer->samples[n].left = left;
      buffer->samples[n].right = right;
   }
}


// one long klaxon, 3 short
#define NUM_KLAXON_LEFT_TRANSITIONS    8
static void init_klaxon_left(void)
{
   uint32_t transition[NUM_KLAXON_LEFT_TRANSITIONS] = {
         // start sample                  stop sample
         0,                               3 * BEEPER_SAMPLING_RATE / 8,
         4 * BEEPER_SAMPLING_RATE / 8,    9 * BEEPER_SAMPLING_RATE / 16,
         5 * BEEPER_SAMPLING_RATE / 8,   11 * BEEPER_SAMPLING_RATE / 16,
         6 * BEEPER_SAMPLING_RATE / 8,   13 * BEEPER_SAMPLING_RATE / 16
   };
   init_klaxon(&klaxon_left_, transition, NUM_KLAXON_LEFT_TRANSITIONS,
         KLAXON_MAX, KLAXON_MAX * 0.5);
}


// two long klaxon
#define NUM_KLAXON_RIGHT_TRANSITIONS    4
static void init_klaxon_right(void)
{
   uint32_t transition[NUM_KLAXON_RIGHT_TRANSITIONS] = {
         // start sample                  stop sample
         0,                               3 * BEEPER_SAMPLING_RATE / 8,
         4 * BEEPER_SAMPLING_RATE / 8,    7 * BEEPER_SAMPLING_RATE / 8
   };
   init_klaxon(&klaxon_right_, transition, NUM_KLAXON_RIGHT_TRANSITIONS,
         KLAXON_MAX * 0.5, KLAXON_MAX);
}


// one long klaxon
#define NUM_KLAXON_ALARM_TRANSITIONS    2
static void init_klaxon_alarm(void)
{
   uint32_t transition[NUM_KLAXON_RIGHT_TRANSITIONS] = {
         // start sample                  stop sample
         0,                               7 * BEEPER_SAMPLING_RATE / 8
   };
   init_klaxon(&klaxon_alarm_, transition, NUM_KLAXON_ALARM_TRANSITIONS,
         KLAXON_ALARM_MAX, KLAXON_ALARM_MAX);
}

// one long klaxon
#define NUM_KLAXON_FULLSTOP_TRANSITIONS    5
static void init_klaxon_fullstop(void)
{
   uint32_t transition[NUM_KLAXON_FULLSTOP_TRANSITIONS] = {
         // start sample                  change
         0,                               1 * BEEPER_SAMPLING_RATE / 4,
         2 * BEEPER_SAMPLING_RATE / 4,    3 * BEEPER_SAMPLING_RATE / 4,
         4 * BEEPER_SAMPLING_RATE / 4
   };
   init_klaxon_twotone(&klaxon_fullstop_, transition, 
         NUM_KLAXON_FULLSTOP_TRANSITIONS, KLAXON_ALARM_MAX, KLAXON_ALARM_MAX);
}

// one long falling klaxon
static void init_klaxon_autopilot_error(void)
{
   init_klaxon_falling(&klaxon_autopilot_error_, 
         KLAXON_ALARM_MAX, KLAXON_ALARM_MAX);
}


// sound definitions
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// initialization code


// initialize stereo sound buffer. buffer contents assumed to be
//    SAMPLING_RATE samples long, with a FRAMES_PER_PERIOD buffer
static void initialize_individual_buffer(
      /*    out */       stereo_buffer_type *buf
      )
{
   uint32_t buf_size = (uint32_t) (BEEPER_SAMPLING_RATE + FRAMES_PER_PERIOD);
   buf->samples = calloc(buf_size, sizeof *buf->samples);
}


static void initialize_sound_buffers(void)
{
   initialize_individual_buffer(&klaxon_left_);
   init_klaxon_left();
   initialize_individual_buffer(&klaxon_right_);
   init_klaxon_right();
   initialize_individual_buffer(&klaxon_alarm_);
   init_klaxon_alarm();
   initialize_individual_buffer(&klaxon_fullstop_);
   init_klaxon_fullstop();
   initialize_individual_buffer(&klaxon_autopilot_error_);
   init_klaxon_autopilot_error();
   //
   initialize_individual_buffer(&beep_left_);
   init_beep_left();
   initialize_individual_buffer(&beep_right_);
   init_beep_right();
   initialize_individual_buffer(&ping_soft_);
   init_ping_soft();
}


static int32_t establish_sound_interface(void)
{
   int32_t rc = 0;
   // sanity check -- sample size is assumed to be 4 bytes and we need
   //    to know if otherwise
   assert(sizeof(stereo_sample_type) == 4);
   // put hardware params on the stack -- it doesn't need to persist
   //    outside this function
   snd_pcm_hw_params_t *hwparams;
   snd_pcm_hw_params_alloca(&hwparams);
   // name of the PCM device
   // names like plughw:0,0 (referenced in many tutorials) seem 
   //    obsolete or rarely used, as using those requires additional
   //    configuration. 'default' seems to be a solid choice
   const char *pcm_name = "default";
   /////////////////////////////////////////////////////////////////////
   // configure hardware
   // establish PCM connection to sound card. the final '0' param
   //    sets writes to be blocking. other options include non-blocking
   //    and async
   if ((rc = snd_pcm_open(&pcm_handle_, pcm_name, 
            SND_PCM_STREAM_PLAYBACK, 0)) < 0) {
      fprintf(stderr, "Error opening device '%s': %s\n", pcm_name, 
            snd_strerror(rc));
      goto end;
   }
   //////////////////////////////
   // fill params with a full configuration space for a PCM
   if ((rc = snd_pcm_hw_params_any(pcm_handle_, hwparams)) < 0) {
      fprintf(stderr, "Error configuring device: %s\n", snd_strerror(rc));
      goto end;
   }
   //////////////////////////////
   // set access type
   if ((rc =snd_pcm_hw_params_set_access(pcm_handle_, hwparams, 
         SND_PCM_ACCESS_RW_INTERLEAVED)) < 0) {
      fprintf(stderr, "Error setting access: %s\n", snd_strerror(rc));
      goto end;
   }
   //////////////////////////////
   // set sample format
   if ((rc = snd_pcm_hw_params_set_format(pcm_handle_, hwparams, 
         SND_PCM_FORMAT_S16_LE)) < 0) {
      fprintf(stderr, "Error setting format: %s\n", snd_strerror(rc));
      goto end;
   }
   //////////////////////////////
   // set sampling rate
   // if the requested rate is not available, the nearest possible 
   //    rate will be selected
   if ((rc = snd_pcm_hw_params_set_rate_near(pcm_handle_, hwparams, 
         &rate_, 0)) < 0) {
      fprintf(stderr, "Error setting rate: %s\n", snd_strerror(rc));
      goto end;
   }
   if (rate_ != BEEPER_SAMPLING_RATE) {
      fprintf(stderr, "The rate %d Hz is not supported by the hardware. "
      "Using %d Hz instead.\n", BEEPER_SAMPLING_RATE, rate_);
   }
   // create 3-second buffer and feed from that
   // *4 because because 2x2 bytes per sample
   audio_buffer_num_samples_ = 3 * rate_;
   audio_buffer_num_bytes_ = audio_buffer_num_samples_ * 4;
   audio_buffer_.bytes = calloc(audio_buffer_num_bytes_, 1);
   //////////////////////////////
   // set number of channels
   if ((rc = snd_pcm_hw_params_set_channels(pcm_handle_, hwparams, 2)) < 0) {
      fprintf(stderr, "Error setting number of channels: %d.\n", rc);
      goto end;
   }
   //////////////////////////////
   // set number of periods
   if ((rc = snd_pcm_hw_params_set_periods(pcm_handle_, hwparams, 
         periods_, 0)) < 0) {
      fprintf(stderr, "Error setting periods: %s.\n", snd_strerror(rc));
      goto end;
   }
   //////////////////////////////
   // set buffer size
   // buffer size is in frames. in theory, this should equal 
   //    periods * period_size, which works, and with periods=4, four
   //    periods are loaded into internal buffers quickly, whereafter
   //    the refill rate equals the frame time. this seems like expected
   //    behavior
   // however, it's possible to set the buffer to be smaller, effectively
   //    reducing the period size (which fails if it's set to 2)
   // experiments show that this can be reduced by 4 and it still runs
   //    well, with one period being buffered. 
   // 'noisy' should use a larger buffer to account for possible
   //    processing delays
   const uint32_t num_samples = (uint32_t) frames_per_period_ * periods_ / 2;
   if ((rc = snd_pcm_hw_params_set_buffer_size(pcm_handle_, hwparams, 
         num_samples)) < 0) {
      fprintf(stderr, "Error setting buffersize: %s\n", snd_strerror(rc));
      goto end;
   }
   //////////////////////////////
   // apply hardware settings to PCM device and prepare device
   if ((rc = snd_pcm_hw_params(pcm_handle_, hwparams)) < 0) {
      fprintf(stderr, "Error setting HW params: %s\n", snd_strerror(rc));
      goto end;
   }
end:
   return rc;
}

// initialization
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// play sound

// blocking call that plays sends the contents of the supplied buffer
static void push_sound_data(
      /* in     */ const stereo_buffer_type *sound
      )
{
   int32_t rc;
   // sound buffer must be at least BEEPER_SAMPLING_RATE + FRAMES_PER_PERIOD
   //    samples long
   uint32_t idx = 0;
   while (idx < BEEPER_SAMPLING_RATE) {
//printf("Writing data from %d  (%d,%d)\n", idx, sound->samples[idx].left, sound->samples[idx].right);
      if ((rc = (int32_t) snd_pcm_writei(pcm_handle_, &sound->bytes[4*idx], 
               FRAMES_PER_PERIOD)) < 0) {
         fprintf(stderr, "Error: %s\n", snd_strerror(rc));
         snd_pcm_reset(pcm_handle_);
      }
      idx += FRAMES_PER_PERIOD;
   }
//printf("Done\n");
}

// play sound
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// shutdown

static void shutdown_sound_interface(void)
{
   if (pcm_handle_ != NULL) {
      snd_pcm_drop(pcm_handle_);
      pcm_handle_ = NULL;
   }
}

// shutdown
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// thread main

uint32_t beeper_errno_ = 0;

// sound card appears to be tempermental and desiring attention. if we
//    stop feeding it then the pipe breaks. to play sounds, a thread is
//    launched which feeds empty buffers to the soundcard. if a sound is
//    requested, that sound is slipped into the feed, to be followed again
//    by empty buffers


static void * thread_main(
      /* in     */ void *void_dp
      )
{
   // input param is pointer to parent's datap struct. this is useful for
   //    knowing when it's time to quit (and possibly more)
   const datap_desc_type *dp = (const datap_desc_type *) void_dp;
   /////////////////////////////////////////////
   // initialization
   requests_.all = 0;
   initialize_sound_buffers();
   if (establish_sound_interface() != 0) {
      fprintf(stderr, "Failed to establish sound interface\n");
      beeper_errno_ = 1;
   }
   /////////////////////////////////////////////
   // main loop
   int32_t rc;
   uint8_t empty_buffer[4 * FRAMES_PER_PERIOD];
   memset(empty_buffer, 0, sizeof empty_buffer);
   while ((dp->run_state & DP_STATE_DONE) == 0) {
      if (requests_.all == 0) {
         // push empty buffer
         if ((rc = (int32_t) snd_pcm_writei(pcm_handle_, empty_buffer,
                  FRAMES_PER_PERIOD)) < 0) {
            fprintf(stderr, "Error: %s\n", snd_strerror(rc));
            snd_pcm_reset(pcm_handle_);
         }
      } else {
         if (requests_.klaxon_left) {
            requests_.klaxon_left = 0;
            push_sound_data(&klaxon_left_);
         }
         if (requests_.klaxon_right) {
            requests_.klaxon_right = 0;
            push_sound_data(&klaxon_right_);
         }
         if (requests_.beep_left) {
            requests_.beep_left = 0;
            push_sound_data(&beep_left_);
         }
         if (requests_.beep_right) {
            requests_.beep_right = 0;
            push_sound_data(&beep_right_);
         }
         if (requests_.klaxon_alarm) {
            requests_.klaxon_alarm = 0;
            push_sound_data(&klaxon_alarm_);
         }
         if (requests_.klaxon_fullstop) {
            requests_.klaxon_fullstop = 0;
            push_sound_data(&klaxon_fullstop_);
         }
         if (requests_.ping_soft) {
            requests_.ping_soft = 0;
            push_sound_data(&ping_soft_);
         }
         if (requests_.klaxon_autopilot_error) {
            requests_.klaxon_autopilot_error = 0;
            push_sound_data(&klaxon_autopilot_error_);
         }
      }
   }
printf("Exiting sound thread\n");
   /////////////////////////////////////////////
   // shutdown
   shutdown_sound_interface();
   return NULL;
}


