#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <assert.h>
#include <errno.h>
#include "beacon.h"

// merge 2 beacons.bin files, e.g., beacons.bin.0-40 and beacons.bin.41-90

#define DEFAULT_MAP_FOLDER    "/opt/kharon/mapping/master/"

// first row to be listed
static char low_file_[STR_LEN] = { 0 };
static char high_file_[STR_LEN] = { 0 };
static char out_file_[STR_LEN] = { 0 };
static int32_t split_point_ = -1;


// returns 0 on success, >0 if problem detected
static uint32_t verify_setup(void)
{
   uint32_t errs = 0;
   if (split_point_ < 0) {
      fprintf(stderr, "Split point not specified\n");
      errs++;
   }
   if (low_file_[0]) {
      fprintf(stderr, "Low file not specified\n");
      errs++;
   }
   if (high_file_[0]) {
      fprintf(stderr, "High file not specified\n");
      errs++;
   }
   if (out_file_[0]) {
      fprintf(stderr, "Output file not specified\n");
      errs++;
   }
   return errs;
}


static void parse_command_line(int argc, char *argv[])
{
   int opt;
   while ((opt = getopt(argc, argv, "ha:o:s:z:")) != -1) {
      switch (opt) {
         case 'a':
         {
            const char *name = optarg;
            strncpy(low_file_, name, STR_LEN);
            break;
         }
         case 'h':
            goto usage;
         case 'o':
         {
            const char *name = optarg;
            strncpy(out_file_, name, STR_LEN);
            break;
         }
         case 's':
         {
            const char *row = optarg;
            errno = 0;
            split_point_ = (int32_t) strtol(row, NULL, 10);
            if (errno != 0) {
               goto usage;
            }
            break;
         }
         case 'z':
         {
            const char *name = optarg;
            strncpy(high_file_, name, STR_LEN);
            break;
         }
         default:
            goto usage;
      };
   };
   // check extra arguments -- there should be none
   // one way to do this: for(; optind < argc; optind++) {
   if (optind < argc) {
      goto usage;
   }
   if (verify_setup() != 0) {
      goto usage;
   }
   return;
   /////////////////////////////////////////////
usage:
   printf("Merges two beacon files, typically those that have beacon paths\n");
   printf("computed over separate\n");
   printf("ranges. The 'low file' is copied up to beacon number 's', but not "
         "including it, while becaon\n");
   printf("'s' and beyond are taken from 'high file'\n");
   printf("\n");
   printf("Usage: %s -a <low file> -r <split row> -z <high file> "
         "-o <outfile> [-h]\n", argv[0]);
   printf("\n");
   printf("where:\n");
   printf("   a   name of file containing low rows to copy\n");
   printf("   z   name of file containing high rows to copy\n");
   printf("   s   dividing beacon number. those < to this are from low file, "
         ">= from high file\n");
   printf("   o   name of output file\n");
   printf("   h   prints this output (ie, help)\n");
   exit(1);
}



int main(int argc, char **argv)
{
   parse_command_line(argc, argv);
   beacon_record_type *low_data;
   beacon_record_type *high_data = NULL;
   assert(split_point_ > 0);
   size_t offset = (size_t) split_point_ * sizeof *low_data;
   low_data = malloc(offset);
   FILE *ifpa = NULL;
   FILE *ifpb = NULL;
   FILE *ofp = NULL;
   /////////////////////////////////////////////
   // read chunk from A
   ifpa = fopen(low_file_, "rb");
   if (!ifpa) {
      fprintf(stderr, "Unable to open '%s': %s\n", low_file_, strerror(errno));
      goto end;
   }
   if (fread(low_data, offset, 1, ifpa) != 1) {
      fprintf(stderr, "Error reading low beacon data: %s\n", strerror(errno));
      goto end;
   }
   /////////////////////////////////////////////
   // read chunk from Z
   ifpb = fopen(high_file_, "rb");
   if (!ifpb) {
      fprintf(stderr, "Unable to open '%s': %s\n", high_file_, strerror(errno));
      goto end;
   }
   fseek(ifpb, 0, SEEK_END);
   long file_len = ftell(ifpb);
   if (file_len < 0) {
      fprintf(stderr, "Error getting file length: %s\n", strerror(errno));
      goto end;
   }
   assert((size_t) file_len > offset);
   size_t len = (size_t) file_len - offset;
   fseek(ifpb, (int32_t) offset, SEEK_SET);
   high_data = malloc(len);
   if (fread(high_data, len, 1, ifpb) != 1) {
      fprintf(stderr, "Error reading high beacon data: %s\n", strerror(errno));
      goto end;
   }
   /////////////////////////////////////////////
   ofp = fopen(out_file_, "wb");
   if (!ofp) {
      fprintf(stderr, "Unable to open '%s': %s\n", out_file_, strerror(errno));
      goto end;
   }
   fwrite(low_data, offset, 1, ofp);
   fwrite(high_data, len, 1, ofp);
end:
   if (ifpa)   { fclose(ifpa);   }
   if (ifpb)   { fclose(ifpb);   }
   if (ofp)    { fclose(ofp);    }
   if (low_data)  { free(low_data);    }
   if (high_data) { free(high_data);   }
}

