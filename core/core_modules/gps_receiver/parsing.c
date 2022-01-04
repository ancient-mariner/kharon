/***********************************************************************
* This file is part of kharon <https://github.com/ancient-mariner/kharon>.
* Copyright (C) 2019-2022 Keith Godfrey
*
* kharon is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, version 3.
*
* kharon is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with kharon.  If not, see <http://www.gnu.org/licenses/>.
***********************************************************************/


// skip next field and advance to character beyond next comma
// if there isn't another field, idx is set to -1
static void skip_one(
      /* in     */ const char *sentence,
      /* in out */       int32_t *idx
      )
{
   int32_t i = *idx;
   if (i < 0) {
      return;
   }
   char c;
   while ((c = sentence[i]) != 0) {
      i++;  // advance to char past this one
      if (c == ',') {
         break;   // end of field
      }
   }
   if ((c == 0) || (sentence[i] == 0)) {
      *idx = -1;
   } else {
      *idx = i;
   }
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// date and time

// reads utc date from sentence starting at index
// index advanced to first char in next field
// if end of sentence is reached, idx is set to -1
static void read_utc_date(
      /* in     */ const char *sentence,
      /* in out */       int32_t *idx,
      /* in out */       gps_receiver_class_type *gps,
      /* in out */       gps_receiver_output_type *out
      )
{
   if (*idx < 0) {
      return;
   }
   const char *head = &sentence[*idx];
   if (head[0] == 0) {
      // this shouldn't happen but check just in case
      *idx = -1;
      return;
   }
   /////////////////////////////////////////////////////////////////////
   // read UTC value
   errno = 0;
   char *endptr;
   out->zulu_date = (uint32_t) strtol(head, &endptr, 10);
   if (errno != 0) {
printf("Error parsing UTC date in '%s'\n", sentence);
      log_err(gps->log, "Error parsing UTC date in '%s'", sentence);
      goto err;
   }
   // zulu time is assumed to be w/in the packet -- not the last field, so
   //    require a comma to terminate it
   if (endptr[0] != ',') {
printf("Terminating UTC date comma not detected '%s'\n", sentence);
      log_err(gps->log, "Terminating UTC date comma not detected '%s'",
            sentence);
      goto err;
   }
   out->available |= GPS_REC_AVAILABLE_DATE;
   if (endptr[1] == 0) {
      *idx = -1;
   } else {
      // advance to next character past ','
      *idx = (int32_t) (endptr - sentence) + 1;
   }
   return;
err:
   // advance to end of field -- no successful parse here
   skip_one(sentence, idx);
}

// reads utc time from sentence starting at index
// index advanced to first char in next field
// if end of sentence is reached, idx is set to -1
static void read_utc_time(
      /* in     */ const char *sentence,
      /* in out */       int32_t *idx,
      /* in out */       gps_receiver_class_type *gps,
      /* in out */       gps_receiver_output_type *out
      )
{
   if (*idx < 0) {
      return;
   }
   const char *head = &sentence[*idx];
   if (head[0] == 0) {
      // this shouldn't happen but check just in case
      *idx = -1;
      return;
   }
   /////////////////////////////////////////////////////////////////////
   // read UTC value
   errno = 0;
   char *endptr;
   out->zulu_time = strtod(head, &endptr);
//printf("Zulu time: %f\n", out->zulu_time);
//printf("end-ptr '%s'\n", endptr);
   if (errno != 0) {
printf("Error parsing UTC time in '%s'\n", sentence);
      log_err(gps->log, "Error parsing UTC time in '%s'", sentence);
      goto err;
   }
   // zulu time is assumed to be w/in the packet -- not the last field, so
   //    require a comma to terminate it
   if (endptr[0] != ',') {
printf("Terminating UTC time comma not detected '%s'\n", sentence);
      log_err(gps->log, "Terminating UTC time comma not detected '%s'",
            sentence);
      goto err;
   }
   out->available |= GPS_REC_AVAILABLE_TIME;
   if (endptr[1] == 0) {
      *idx = -1;
   } else {
      // advance to next character past ','
      *idx = (int32_t) (endptr - sentence) + 1;
   }
   return;
err:
   // advance to end of field -- no successful parse here
   skip_one(sentence, idx);
}

// date and time
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// lat and lon

// reads latitude from sentence starting at index
// index advanced to first char in next field
// if end of sentence is reached, idx is set to -1
static void read_latitude(
      /* in     */ const char *sentence,
      /* in out */       int32_t *idx,
      /* in out */       gps_receiver_class_type *gps,
      /* in out */       gps_receiver_output_type *out
      )
{
   if (*idx < 0) {
      return;
   }
   const char *head = &sentence[*idx];
   if (head[0] == 0) {
      // this shouldn't happen but check just in case
      return;
   }
   /////////////////////////////////////////////
   // read latitude ddmm.mmmm
   errno = 0;
   char *endptr;
   double lat = strtod(head, &endptr);
   if (errno != 0) {
printf("Error parsing latitude in '%s'\n", sentence);
      log_err(gps->log, "Error parsing latitude in '%s'", sentence);
      goto err;
   }
   // zulu time is assumed to be w/in the packet -- not the last field, so
   //    require a comma to terminate it
   if (endptr[0] != ',') {
printf("Terminating latitude comma not detected '%s'\n", sentence);
      log_err(gps->log, "Terminating latitude comma not detected '%s'",
            sentence);
      goto err;
   }
   /////////////////////////////////////////////////////////////////////
   // read latitude north or south
   // convert from ddmm.mmmm to dd.mmmmmm
   lat /= 100.0;
   double deg = floor(lat);
   // convert minutes to decimal
   double min = lat - deg;
   min = min * 100.0 / 60.0;
   assert(min < 1.0);
   // rebuild lat
   lat = deg + min;
   /////////////////////////////////////////////
   // now correct for north or south
   if (endptr[1] == 'N') {
      ;
   } else if (endptr[1] == 'S') {
      lat = -lat;
   } else {
      log_err(gps->log, "Failed to recognize if lat was N or S in '%s'",
            sentence);
      goto err;
   }
   // latitude is assumed to be w/in the packet -- not the last field, so
   //    require a comma to terminate it
   if (endptr[2] != ',') {
printf("Terminating latitude N/S comma not detected '%s'\n", sentence);
      log_err(gps->log, "Terminating latitude N/S comma not detected '%s'",
            sentence);
      goto err;
   }
   /////////////////////////////////////////////
   out->pos.y_deg = lat;
   out->available |= GPS_REC_AVAILABLE_LATITUDE;
   if (endptr[3] == 0) {
      *idx = -1;
   } else {
      // advance to next character past ','
      *idx = (int32_t) (endptr - sentence) + 3;
   }
   return;
err:
   // advance 2 commas -- one for lat, other for N/S
   skip_one(sentence, idx);
   skip_one(sentence, idx);
}


// reads longitude from sentence starting at index
// index advanced to first char in next field
// if end of sentence is reached, idx is set to -1
static void read_longitude(
      /* in     */ const char *sentence,
      /* in out */       int32_t *idx,
      /* in out */       gps_receiver_class_type *gps,
      /* in out */       gps_receiver_output_type *out
      )
{
   if (*idx < 0) {
      return;
   }
   const char *head = &sentence[*idx];
   if (head[0] == 0) {
      // this shouldn't happen but check just in case
      return;
   }
   /////////////////////////////////////////////
   // read longitude dddmm.mmmm
   errno = 0;
   char *endptr;
   double lon = strtod(head, &endptr);
   if (errno != 0) {
printf("Error parsing longitude in '%s'\n", sentence);
      log_err(gps->log, "Error parsing longitude in '%s'", sentence);
      goto err;
   }
   // zulu time is assumed to be w/in the packet -- not the last field, so
   //    require a comma to terminate it
   if (endptr[0] != ',') {
printf("Terminating longitude comma not detected '%s'\n", sentence);
      log_err(gps->log, "Terminating longitude comma not detected '%s'",
            sentence);
      goto err;
   }
   /////////////////////////////////////////////////////////////////////
   // read east/west
   // convert from dddmm.mmmm to dd.mmmmmm
   lon /= 100.0;
   double deg = floor(lon);
   // convert minutes to decimal
   double min = lon - deg;
   min = min * 100.0 / 60.0;
   assert(min < 1.0);
   // rebuild lon
   lon = deg + min;
   /////////////////////////////////////////////
   if (endptr[1] == 'E') {
      ;
   } else if (endptr[1] == 'W') {
      lon = -lon;
   } else {
      log_err(gps->log, "Failed to recognize if lat was N or S in '%s'",
            sentence);
      goto err;
   }
   // longitude is assumed to be w/in the packet -- not the last field, so
   //    require a comma to terminate it
   if (endptr[2] != ',') {
printf("Terminating longitude N/S comma not detected '%s'\n", sentence);
      log_err(gps->log, "Terminating longitude N/S comma not detected '%s'",
            sentence);
      goto err;
   }
   /////////////////////////////////////////////
   out->pos.x_deg = lon;
   out->available |= GPS_REC_AVAILABLE_LONGITUDE;
   if (endptr[3] == 0) {
      *idx = -1;
   } else {
      // advance to next character past ','
      *idx = (int32_t) (endptr - sentence) + 3;
   }
   return;
err:
   // advance 2 commas -- one for lat, other for N/S
   skip_one(sentence, idx);
   skip_one(sentence, idx);
}

// lat and lon
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// cog and sog

// reads course over ground from sentence starting at index
// index advanced to first char in next field
// if end of sentence is reached, idx is set to -1
static void read_cog(
      /* in     */ const char *sentence,
      /* in out */       int32_t *idx,
      /* in out */       gps_receiver_class_type *gps,
      /* in out */       gps_receiver_output_type *out
      )
{
   if (*idx < 0) {
      return;
   }
   const char *head = &sentence[*idx];
   if (head[0] == 0) {
      // this shouldn't happen but check just in case
      *idx = -1;
      return;
   }
   /////////////////////////////////////////////////////////////////////
   // read UTC value
   errno = 0;
   char *endptr;
   out->heading.degrees = strtof(head, &endptr);
   if (errno != 0) {
printf("Error parsing UTC time in '%s'\n", sentence);
      log_err(gps->log, "Error parsing UTC time in '%s'", sentence);
      goto err;
   }
   // zulu time is assumed to be w/in the packet -- not the last field, so
   //    require a comma to terminate it
   if (endptr[0] != ',') {
printf("Terminating UTC time comma not detected '%s'\n", sentence);
      log_err(gps->log, "Terminating UTC time comma not detected '%s'",
            sentence);
      goto err;
   }
   out->available |= GPS_REC_AVAILABLE_TRACK;
   if (endptr[1] == 0) {
      *idx = -1;
   } else {
      // advance to next character past ','
      *idx = (int32_t) (endptr - sentence) + 1;
   }
   return;
err:
   // advance to end of field -- no successful parse here
   skip_one(sentence, idx);
}

// reads speed over ground from sentence starting at index
// index advanced to first char in next field
// if end of sentence is reached, idx is set to -1
static void read_sog(
      /* in     */ const char *sentence,
      /* in out */       int32_t *idx,
      /* in out */       gps_receiver_class_type *gps,
      /* in out */       gps_receiver_output_type *out
      )
{
   if (*idx < 0) {
      return;
   }
   const char *head = &sentence[*idx];
   if (head[0] == 0) {
      // this shouldn't happen but check just in case
      *idx = -1;
      return;
   }
   /////////////////////////////////////////////////////////////////////
   // read UTC value
   errno = 0;
   char *endptr;
   out->speed.mps = strtod(head, &endptr) * KNOTS_TO_MPS;
   if (errno != 0) {
printf("Error parsing UTC time in '%s'\n", sentence);
      log_err(gps->log, "Error parsing UTC time in '%s'", sentence);
      goto err;
   }
   // zulu time is assumed to be w/in the packet -- not the last field, so
   //    require a comma to terminate it
   if (endptr[0] != ',') {
printf("Terminating UTC time comma not detected '%s'\n", sentence);
      log_err(gps->log, "Terminating UTC time comma not detected '%s'",
            sentence);
      goto err;
   }
   out->available |= GPS_REC_AVAILABLE_SPEED;
   if (endptr[1] == 0) {
      *idx = -1;
   } else {
      // advance to next character past ','
      *idx = (int32_t) (endptr - sentence) + 1;
   }
   return;
err:
   // advance to end of field -- no successful parse here
   skip_one(sentence, idx);
}

// cog and sog
////////////////////////////////////////////////////////////////////////

