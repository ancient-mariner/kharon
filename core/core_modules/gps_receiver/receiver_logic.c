
// see
// https://www.rfwireless-world.com/Terminology/GPS-sentences-or-NMEA-sentences.html

static char * nmea_device_[8] = {
   "GA",     // galileo
   "GB",     // beidou 
   "GP",     // gps
   "GI",     // navIC
   "GL",     // glonass
   "GN",     // based on multiple systems
   "GQ",     // qzss
   NULL
};

// TODO add support for GGL and VTG
static char * nmea_type_[8] = {
   "GGA,",     // fix
   "RMC,",     // fix
   NULL
};

static char * months_[13] = {
   "Jan", "Feb", "Mar", "Apr", "May", "Jun",
   "Jul", "Aug", "Sep", "Oct", "Nov", "Dec", "Unknown"
};

enum nmea_message { NMEA_GGA, NMEA_RMC, NUM_NMEA_MESSAGE_TYPES };
typedef enum nmea_message nmea_message_type;

////////////////////////////////////////////////////////////////////////
// network

static void start_network(
      /* in out */       datap_desc_type *dp
      )
{
   gps_receiver_class_type *gps = (gps_receiver_class_type*) dp->local;
   while ((dp->run_state & DP_STATE_DONE) == 0) {
printf("Waiting for connection\n");
      if ((gps->connfd = 
            wait_for_connection(gps->sockfd, dp->td->obj_name)) >= 0) {
printf("Have connection\n");
         break; // have connection
      }
      if ((dp->run_state & DP_STATE_DONE) != 0) {
         break; // quit flag set
      }
      log_err(gps->log, "Failed to initialize server. Waiting to try again");
      sleep(5);
   }
}

// network
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// nmea

void print_nmea_message(
      /* in     */ const gps_receiver_output_type *out
      )
{
   printf("NMEA message '%s'\n", out->message_type);
   uint32_t latlon_mask = GPS_REC_AVAILABLE_LATLON_MASK;
   if ((out->available & latlon_mask) == latlon_mask) {
      printf("    xy    %9.6f   %9.6f\n", out->pos.x_deg, out->pos.y_deg);
   }
   if (out->available & GPS_REC_AVAILABLE_TIME) {
      printf("    when  ");
      // zulu time is hhmmss.sss
      uint32_t hour = (uint32_t) (out->zulu_time / 10000.0);
      uint32_t min = (uint32_t) 
            (out->zulu_time - (double) (hour * 10000)) / 100;
      double sec = out->zulu_time - (double) (hour * 10000 + min * 100);
      printf("%02d:%02d:%02.3f", hour, min, sec);
      if (out->available & GPS_REC_AVAILABLE_DATE) {
         // zulu_date is ddmmyy
         uint32_t year = (uint32_t) (2000 + out->zulu_date % 100);
         uint32_t mon = (uint32_t) ((out->zulu_date / 100) % 100) - 1;
         uint32_t day = (uint32_t) (out->zulu_date / 10000);
         if (mon >= 12) {
            mon = 12;
         }
         printf("  %2d%s%4d", day, months_[mon], year);
      }
      printf("\n");
   }
   if (out->available & GPS_REC_AVAILABLE_TRACK) {
      printf("    track %03d deg\n", (uint32_t) out->heading.degrees);
   }
   if (out->available & GPS_REC_AVAILABLE_SPEED) {
      printf("    speed %.1f kts\n", out->speed.mps * KNOTS_TO_MPS);
   }
}


// returns nmea message type number. if type not recognized then returns
//    NUM_NMEA_MESSAGE_TYPES
static nmea_message_type ident_nmea_message(
      /* in     */ const char *sentence
      )
{
   // see if it's from a recognized device
   uint32_t ctr = 0;
   nmea_message_type type = NUM_NMEA_MESSAGE_TYPES;
   char *device = NULL;
   while ((device = nmea_device_[ctr++]) != NULL) {
      if (strncmp(sentence, device, 2) == 0) {
         break;
      }
   }
   if (device != NULL) {
      if (strncmp(&sentence[2], nmea_type_[NMEA_GGA], 4) == 0) {
         type = NMEA_GGA;
      } else if (strncmp(&sentence[2], nmea_type_[NMEA_RMC], 4) == 0) {
         type = NMEA_RMC;
      }
   }
   return type;
}

/*
GGA, from https://www.rfwireless-world.com/Terminology/GPS-sentences-or-NMEA-sentences.html
example;
   $GPGGA,161229.487,3723.2475,N,12158.3416,W,1,07,1.0,9.0,M,,,,0000*18
UTC time             161229.487  hhmmss.sss
Latitude             3723.2475 (37 degrees, 23.2475 minutes)   ddmm.mmmm
N/S Indicator        N  N = North, S = South
Longitude            12158.3416 (121 degrees, 58.3416 minutes)    dddmm.mmmm
E/W indicator        W  E = East or W = West
Position Fix Indicator  1  GPS Sentences Position Fix Indicator
Satellites used      07    Range is 0 to 12
HDOP                 1.0   Horizontal Dilution of Precision
MSL Altitude         9.0   Meters
Units                M  Meters
Geoid Separation     Meters
Units                M  Meters
Age of diff. corr.      Second
Diff. ref. station ID   0000  
*/
static void parse_gga(
      /* in out */       gps_receiver_class_type *gps,
      /* in     */ const char *sentence,
      /* in out */       gps_receiver_output_type *out
      )
{
   int32_t idx = 6;
   read_utc_time(sentence, &idx, gps, out);
   read_latitude(sentence, &idx, gps, out);
   read_longitude(sentence, &idx, gps, out);
}


/*
RMC, from https://www.rfwireless-world.com/Terminology/GPS-sentences-or-NMEA-sentences.html
example:
   $GPRMC,161229.487,A,3723.2475,N,12158.3416,W,0.13,309.62,120598,,*10 
UTC time    161229.487  hhmmss.sss
Status   A  A = data valid or V = data not valid
Latitude    3723.2475   ddmm.mmmm
N/S indicator  N  N = North or S = South
Longitude   12158.3416  dddmm.mmmm
E/W indicator  W  E = East or W = West
Speed over ground    0.13  knots
Course over ground   309.62   degrees
Date  120598   ddmmyy
Magnetic Variation      Degrees (E= East or W = West)
Mode  A  A = Autonomous, D = DGPS, E =DR 
*/
static void parse_rmc(
      /* in out */       gps_receiver_class_type *gps,
      /* in     */ const char *sentence,
      /* in out */       gps_receiver_output_type *out
      )
{
   int32_t idx = 6;
   read_utc_time(sentence, &idx, gps, out);
   skip_one(sentence, &idx);
   read_latitude(sentence, &idx, gps, out);
   read_longitude(sentence, &idx, gps, out);
   read_sog(sentence, &idx, gps, out);
   read_cog(sentence, &idx, gps, out);
   read_utc_date(sentence, &idx, gps, out);
}


// 
static void parse_and_publish(
      /* in out */       datap_desc_type *self,
      /* in out */       gps_receiver_class_type *gps,
      /* in     */ const uint8_t inbound_data[GPS_BLOCK_SIZE]
      )
{
   char data[GPS_BLOCK_SIZE];
   strncpy(data, (const char*) inbound_data, sizeof(data));
   if (gps->logfile) {
      fprintf(gps->logfile, "%s\n", data);
   }
   //
   char *save_ptr = NULL;
   // extract timestamp
   errno = 0;
   double t = strtod(data, &save_ptr);
   if (errno != 0) {
      log_err(gps->log, "Error extracting timestamp from '%s' : %s\n",
            data, strerror(errno));
//printf("Error extracting timestamp from '%s' : %s", data, strerror(errno));
      goto end;
   }
   char *sentence = strtok_r(save_ptr, "\n\r ", &save_ptr);
   if (sentence == NULL) {
      log_err(gps->log, "Packet at %.3f missing sentence", t);
//printf("Packet at %.3f missing sentence\n", t);
      goto end;
   }
   size_t len = strlen(sentence);
   if (len < 8) {
      log_err(gps->log, "Sentence '%s' is too short", sentence);
//printf("Sentence '%s' is too short\n", sentence);
      goto end;
   }
   nmea_message_type type = ident_nmea_message(sentence);
   /////////////////////
   // get data sink
   uint32_t dp_idx = (uint32_t) (self->elements_produced % self->queue_length);
//printf("GPS out IDX %d\n", dp_idx);
   self->ts[dp_idx] = t;
   gps_receiver_output_type *out = dp_get_object_at(self, dp_idx);
   memset(out, 0, sizeof *out);
   strncpy(out->message_type, sentence, 6);
   out->message_type[6] = 0;
   /////////////////////
   switch (type) {
      case NMEA_GGA:
         parse_gga(gps, sentence, out);
         break;
      case NMEA_RMC:
         parse_rmc(gps, sentence, out);
         break;
      case NUM_NMEA_MESSAGE_TYPES:
//printf("Unsupported message type\n");
         // unsupported message -- ignore it
         goto end;
      default:
         log_err(gps->log, "Internal errror - invalid nmea message type %d\n",
               type);
         hard_exit(__FILE__, __LINE__);
   };
   // if out has valid time and position, publish. anything else is a bonus
   uint32_t mask = GPS_REC_MIN_DATA_FOR_PUBLISH;
   if ((out->available & mask) == mask) {
//printf("GPS %.4f,%.4f\n", out->pos.x_deg, out->pos.y_deg);
      self->elements_produced++;
      dp_signal_data_available(self);
//print_nmea_message(out);
   }
end:
   ;
}

