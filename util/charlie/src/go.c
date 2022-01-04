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
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include "pinet.h"
#include "postmaster.h"

#include "charlie.h"

void print_destinations(
      /* in     */ const char *prefix
      );

// loads destination database into memory
// returns 0 on success, -1 on failure
int load_destination_database(void);

// returns destination record associated with this name, and NULL if that
//    name is not found. there is no checking for duplicate records --
//    the first match found is returned
dest_record_type * get_destination_record(
      /* in     */ const char *name
      );

////////////////////////////////////////////////////////////////////////

static dest_record_type *records_ = NULL;
static uint32_t num_records_ = 0;
static uint32_t num_allocated_records_ = 0;

// number of destination records allocated at a time
#define DB_RECORD_BLOCK_SIZE     1024


// returns destination record associated with this name, and NULL if that
//    name is not found. there is no checking for duplicate records --
//    the first match found is returned
dest_record_type * get_destination_record(
      /* in     */ const char *name
      )
{
   dest_record_type *dest = NULL;
   for (uint32_t i=0; i<num_records_; i++) {
      dest_record_type *cand = &records_[i];
      if (strcmp(cand->name, name) == 0) {
         dest = cand;
         break;
      }
   }
   return dest;
}


// prints name of all destinations that start with 'prefix'. if prefix
//    is an empty string or is NULL, all destinations are printed
void print_destinations(
      /* in     */ const char *prefix
      )
{
   size_t len = (prefix == NULL) ? 0 : strlen(prefix);
   uint32_t cnt = 0;
   printf("  Line num                Destination   Longitude    Latitude  "
         "Radius (meters)\n");
   for (uint32_t i=0; i<num_records_; i++) {
      dest_record_type *dest = &records_[i];
      if (len > 0) {
         if (strncmp(dest->name, prefix, len) != 0) {
            continue;
         }
      }
      cnt++;
      printf("  %3d  %30s %11.4f %11.4f  %.1f\n", dest->line_num, dest->name,
            dest->longitude, dest->latitude, (double) dest->radius.meters);
   }
   if (cnt == 0) {
      if (num_records_ == 0) {
         printf("  Empty database\n");
      } else {
         printf("\n");
         printf("  No destinations starting with '%s'\n", prefix);
      }
   }
}


// loads destination database into memory
// returns 0 on success, -1 on failure
int load_destination_database(void)
{
   int rc = -1;
   FILE *fp = fopen(CHARLIE_DESTINATION_DB, "r");
   char buf[STR_LEN];
   if (!fp) {
      fprintf(stderr, "Failed to open '%s': %s\n", CHARLIE_DESTINATION_DB,
            strerror(errno));
      goto end;
   }
   uint32_t line_num = 0;
   while (fgets(buf, STR_LEN, fp) != NULL) {
      line_num++;
      // advance to first non-whitespace
      char *str = trim_whitespace(buf);
      if ((str == NULL) || (str[0] == '#')) {
         // if empty line or first text is '#' then fetch next line -- nothing
         //    to see here
         continue;
      }
      if (num_records_ >= num_allocated_records_) {
         num_allocated_records_ += DB_RECORD_BLOCK_SIZE;
         records_ =
               realloc(records_, num_allocated_records_ * sizeof *records_);
      }
      dest_record_type *dest = &records_[num_records_++];
      //////////////////////////
      // fetch name. add error checking as this file will be human-modified
      uint32_t idx = 0;
      char c = 0;
      for (; idx<CHARLIE_DESTINATION_DB_NAME_LEN_MAX-1; idx++) {
         c = str[idx];
         if ((c == 0) || (c == 10) || (c == 13)) {
            fprintf(stderr, "Parse error reading destination name on line "
                  "%d of %s\n", line_num, CHARLIE_DESTINATION_DB);
            fprintf(stderr, "Destination '%s' has no coordinates\n", str);
            goto end;
         }
         if ((c == ' ') || (c == '\t')) {
            break;
         }
         dest->name[idx] = c;
      }
      dest->name[idx] = 0;
      if ((c != ' ') && (c != '\t')) {
         fprintf(stderr, "Parse error reading destination name on line "
               "%d of %s\n", line_num, CHARLIE_DESTINATION_DB);
         fprintf(stderr, "Name appears to be too long\n");
         goto end;
      }
      //////////////////////////
      // read lat and lon
      errno = 0;
      char *endptr;
      dest->longitude = strtod(&str[idx], &endptr);
      if (errno != 0) {
         fprintf(stderr, "Parse error reading longitude on line "
               "%d of %s : %s\n", line_num, CHARLIE_DESTINATION_DB,
               strerror(errno));
         goto end;
      }
      dest->latitude = strtod(endptr, &endptr);
      if (errno != 0) {
         fprintf(stderr, "Parse error reading latitude on line "
               "%d of %s : %s\n", line_num, CHARLIE_DESTINATION_DB,
               strerror(errno));
         goto end;
      }
      //////////////////////////
      // read radius
      dest->radius.meters = strtof(endptr, NULL);
      if (errno != 0) {
         fprintf(stderr, "Parse error reading longitude on line "
               "%d of %s : %s\n", line_num, CHARLIE_DESTINATION_DB,
               strerror(errno));
         goto end;
      }
      //
      dest->line_num = line_num;
   }
   /////////////////////////////////////////////
   rc = 0;
end:
   if (fp != NULL) {
      fclose(fp);
   }
   return rc;
}

////////////////////////////////////////////////////////////////////////


static void usage(
      /* in     */ const int argc,
      /* in     */ const char **argv
      )
{
   (void) argc;
   printf("  Set destination\n\n");
   printf("  Usage: %s [<dest name>]\n", argv[0]);
   printf("  Calling with no parameters will show a list of options\n");
   printf("  Calling with a partial name will restrict the list to "
         "destinations\n");
   printf("     that begin with the string\n");
   printf("  The database of destinations is at '%s'\n",
         CHARLIE_DESTINATION_DB);
   printf("\n");
   exit(1);
}


static void goto_destination(
      /* in     */ const network_id_type *id,
      /* in     */ const dest_record_type *dest
      )
{
   pm_request_type req;
   pm_response_type resp;
   memset(&req, 0, sizeof(req));
   req.request_type = PM_CMD_SET_DESTINATION;
   bam32_type lat, lon;
   CVT_DEG_TO_BAM32(dest->longitude, lon);
   CVT_DEG_TO_BAM32(dest->latitude, lat);
   req.custom_0 = lon.sangle32;
   req.custom_1 = lat.sangle32;
   req.custom_2 = (int32_t) (dest->radius.meters + 0.9);
   /////////////////////////////////////////////////////////////////////
   // init connection
   int sockfd = -1;
   if ((sockfd = connect_to_server(id)) < 0) {
      printf("Error connecting to postmaster (at %s:%d)\n", id->ip, id->port);
      goto end;
   }
   /////////////////////////////////////////////////////////////////////
   // send data
   if (send_postmaster_request(sockfd, &req, NULL) != 0) {
      printf("Error sending packet to postmaster\n");
      goto end;
   }
   if (read_postmaster_response(sockfd, &req, &resp, NULL) != 0) {
      printf("Error reading packet from postmaster\n");
      goto end;
   }
   //
   double t = atof((char*) resp.t);
   if (resp.request_type == PM_CMD_NULL) {
      printf("%.3f   ERROR setting destination to %.4f%c %.4f%c (%s)\n", t,
            dest->longitude, dest->longitude < 0.0 ? 'W' : 'E',
            dest->latitude, dest->latitude < 0.0 ? 'S' : 'N',
            dest->name);
   } else {
      printf("%.3f   Requested setting destination to %.4f%c %.4f%c "
            "r=%.1f  (%s)\n", t,
            dest->longitude, dest->longitude < 0.0 ? 'W' : 'E',
            dest->latitude, dest->latitude < 0.0 ? 'S' : 'N',
            (double) dest->radius.meters, dest->name);
   }
end:
   if (sockfd >= 0) {
      close(sockfd);
   }
}


int main(const int argc, const char **argv)
{
   int rc = -1;
   set_device_dir_path(KHARON_DEVICE_DIR);
   // load and analyze database before checking for postmaster availability
   // this allows offline querying
   if (load_destination_database() != 0) {
      goto end;
   }
   dest_record_type *dest = NULL;
   if (argc == 1) {
      print_destinations("");
   } else if (argc == 2) {
      dest = get_destination_record(argv[1]);
      if (dest == NULL) {
         print_destinations(argv[1]);
      }
   } else {
      usage(argc, argv);
   }
   // send destination to postmaster
   if (dest != NULL) {
      network_id_type id;
      if (get_postmaster_address(&id) != 0) {
         goto end;
      }
      goto_destination(&id, dest);
   }
   rc = 0;
end:
   return rc;
}

