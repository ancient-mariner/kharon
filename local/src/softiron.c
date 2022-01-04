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
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include "lin_alg.h"
#include "dev_info.h"
#include "softiron.h"

//void apply_softiron_correction(
//      /* in     */ const matrix_type *r,
//      /* in     */ const vector_type *scale,
//      /* in     */ const vector_type *offset,
//      /* in     */ const vector_type *mag,
//      /*    out */       vector_type *corrected_mag
//      );

// opens IMU output file + mag scale, offset, softiron files
//    then applies correction to IMU data and writes new file.
//    output file either performs partial operation (offset,
//    rotation and scale) or full (partial + re-rotate)
// correctin algorithm used here should be used for attitude
//    correction

// don't include this when compiling as a standalone app -- it'll be
//    linked in


static void apply_partial_softiron_correction(
      /* in     */ const matrix_type *r,
      /* in     */ const vector_type *offset,
      /* in     */ const vector_type *mag,
      /*    out */       vector_type *corrected_mag
      )
{
   vector_type tmp;
   for (uint32_t i=0; i<3; i++) {
      tmp.v[i] = mag->v[i] - offset->v[i];
   }
   mult_matrix_vector(r, &tmp, corrected_mag);
//print_mat(r, "R");
//print_vec(mag, "orig");
//print_vec(&tmp, "offset");
//print_vec(corrected_mag, "new");
//if (mag->v[0] != 0.0f)
//   exit(0);
}

#if !defined(SOFTIRON_APP)

static void apply_scale_and_rotation(
      /* in     */ const matrix_type *r,
      /* in     */ const vector_type *mag,
      /* in     */ const vector_type *scale,
      /*    out */       vector_type *corrected_mag
      )
{
   vector_type tmp;
   for (uint32_t i=0; i<3; i++) {
      tmp.v[i] = mag->v[i] * scale->v[i];
   }
   mult_vector_matrix(&tmp, r, corrected_mag);
}

void apply_softiron_correction(
      /* in     */ const matrix_type *r,
      /* in     */ const vector_type *scale,
      /* in     */ const vector_type *offset,
      /* in     */ const vector_type *mag,
      /*    out */       vector_type *corrected_mag
      )
{
   vector_type tmp;
   apply_partial_softiron_correction(r, offset, mag, &tmp);
   apply_scale_and_rotation(r, &tmp, scale, corrected_mag);
}

#else    // SOFTIRON_APP

static char imu_log_[STR_LEN] = { 0 };
static char imu_out_[STR_LEN] = { 0 };
static char host_name_[STR_LEN] = { 0 };
static char device_name_[STR_LEN] = { 0 };
static char dev_root_[STR_LEN] = { 0 };
static int full_process_ = 1;

static void usage(const char *arg0)
{
   printf("Applies soft-iron correction to file generated from 's2'\n");
   printf("The purpose of this app is to allow iterative tweaking of \n");
   printf("   softiron correction configs, that shares the correction \n");
   printf("   logic with the kernel attitude module\n");
   printf("Input data file should be created on device using \n");
   printf("   s2 -x -f <filename>\n\n");
   printf("Usage: %s -h <host> -d <device> -i <in-name> -o <out-name> [-p]\n",
         arg0);
   printf("\n");
   printf("where:\n");
   printf("    -d    name of device to emulate\n");
   printf("    -h    name of IMU host\n");
   printf("    -i    name of input file\n");
   printf("    -o    name of output file\n");
   printf("    -p    do partial data conversion (defaults to full)\n");
   printf("    -x    path to dev directory\n");
   printf("\n");
   exit(1);
}


static void parse_command_line(int argc, char *argv[])
{
   int opt;
   while ((opt = getopt(argc, argv, "d:h:i:o:px:")) != -1) {
      switch (opt) {
         case 'd':
            {
               if (device_name_[0])
                  usage(argv[0]);
               const char * name = optarg;
               strcpy(device_name_, name);
            }
            break;
         case 'h':
            {
               if (host_name_[0])
                  usage(argv[0]);
               const char * name = optarg;
               strcpy(host_name_, name);
            }
            break;
         case 'i':
            {
               if (imu_log_[0])
                  usage(argv[0]);
               const char * name = optarg;
               strcpy(imu_log_, name);
            }
            break;
         case 'o':
            {
               if (imu_out_[0])
                  usage(argv[0]);
               const char * name = optarg;
               strcpy(imu_out_, name);
            }
            break;
         case 'p':
            full_process_ = 0;
            break;
         case 'x':
            {
               if (dev_root_[0])
                  usage(argv[0]);
               const char * name = optarg;
               strcpy(dev_root_, name);
            }
            break;
         default:
            usage(argv[0]);
      }
   }
   if (host_name_[0] == 0) {
      usage(argv[0]);
   }
   if (device_name_[0] == 0) {
      usage(argv[0]);
   }
   if (imu_log_[0] == 0) {
      usage(argv[0]);
   }
   if (imu_out_[0] == 0) {
      usage(argv[0]);
   }
   printf("host:       %s\n", host_name_);
   printf("device:     %s\n", device_name_);
   printf("imu data:   %s\n", imu_log_);
   printf("imu output: %s\n", imu_out_);
   printf("----------\n");
   if (strcmp(imu_log_, imu_out_) == 0) {
      printf("IMU data and output files must be different\n");
      exit(1);
   }
}

static uint32_t load_configuration_data(
      /*    out */       matrix_type *r,
      /*    out */       vector_type *scale,
      /*    out */       vector_type *offset
      )
{
   uint32_t errs = 0;
   FILE *fp;
   fp = open_config_file_ro2(dev_root_, host_name_, "sensors/i2c", device_name_,
         "modality/mag/softiron");
   if (fp) {
      config_read_matrix(fp, r);
      fclose(fp);
   } else {
      printf("Failed to open config file (softiron)\n");
      errs++;
   }
   //
   fp = open_config_file_ro2(dev_root_, host_name_, "sensors/i2c", device_name_,
         "modality/mag/scale");
   if (fp) {
      config_read_vector(fp, scale);
      fclose(fp);
   } else {
      printf("Failed to open config file (scale)\n");
      errs++;
   }
   //
   fp = open_config_file_ro2(dev_root_, host_name_, "sensors/i2c", device_name_,
         "modality/mag/offset");
   if (fp) {
      config_read_vector(fp, offset);
      fclose(fp);
   } else {
      printf("Failed to open config file (offset)\n");
      errs++;
   }
   return errs;
}

// loads next IMU sample from IMU log file at fp
static int load_next_imu_sample(
      /*    out */       vector_type *gyr,
      /*    out */       vector_type *acc,
      /*    out */       vector_type *mag,
      /*    out */       double *temp,
      /* in out */       FILE *fp
      )
{
   static char *lineptr = NULL;
   static size_t ptr_size = 0;
   char orig_line[STR_LEN];
   if (getline(&lineptr, &ptr_size, fp) <= 0) {
      return -1;
   }
   size_t len = sizeof orig_line;
   strncpy(orig_line, lineptr, len-1);
   orig_line[len-1] = 0;
   char *str = lineptr;
   *temp = 0.0f;
   zero_vector(gyr);
   zero_vector(acc);
   zero_vector(mag);
   uint32_t err = 0;
   for (uint32_t i=0; i<10; i++) {
      char *tok = strtok(str, " \n,");
      if (tok == NULL) {
         err = 1;
         break;
      }
      switch (i) {
         case 0: gyr->v[0] = atof(tok); break;
         case 1: gyr->v[1] = atof(tok); break;
         case 2: gyr->v[2] = atof(tok); break;
         case 3: acc->v[0] = atof(tok); break;
         case 4: acc->v[1] = atof(tok); break;
         case 5: acc->v[2] = atof(tok); break;
         case 6: mag->v[0] = atof(tok); break;
         case 7: mag->v[1] = atof(tok); break;
         case 8: mag->v[2] = atof(tok); break;
         case 9: *temp = atof(tok); break;
      }
      str = NULL;
   }
   if (err != 0) {
      printf("Parse error in line '%s'\n", orig_line);
      return -1;
   }
   return 0;
}


static void convert_file(
      /* in     */ const matrix_type *r,
      /* in     */ const vector_type *scale,
      /* in     */ const vector_type *offset
      )
{
   FILE *ifp = NULL;
   FILE *ofp = NULL;
   ifp = fopen(imu_log_, "r");
   if (!ifp) {
      fprintf(stderr, "Unable to open '%s' for reading\n", imu_log_);
      goto end;
   }
   ofp = fopen(imu_out_, "w");
   if (!ofp) {
      fprintf(stderr, "Unable to open '%s' for writing\n", imu_out_);
      goto end;
   }
   vector_type gyr, acc, mag, new_mag;
   double temp;
   while (load_next_imu_sample(&gyr, &acc, &mag, &temp, ifp) == 0) {
      if (full_process_) {
         apply_softiron_correction(r, scale, offset, &mag, &new_mag);
      } else {
         apply_partial_softiron_correction(r, offset, &mag, &new_mag);
      }
      fprintf(ofp,
            "%7.4f, %7.4f, %7.4f,  "
            "%7.4f, %7.4f, %7.4f,  "
            "%7.4f, %7.4f, %7.4f,  "
            "%5.1f\n",
            (double) gyr.v[0], (double) gyr.v[1], (double) gyr.v[2],
            (double) acc.v[0], (double) acc.v[1], (double) acc.v[2],
            (double) new_mag.v[0], (double) new_mag.v[1],
            (double) new_mag.v[2], temp);
   }
end:
   if (ifp) {
      fclose(ifp);
   }
   if (ofp) {
      fclose(ofp);
   }
}


int main(int argc, char **argv){
   parse_command_line(argc, argv);
   matrix_type rot;
   vector_type scale;
   vector_type offset;
   if (load_configuration_data(&rot, &scale, &offset) != 0) {
      return 1;
   }
   convert_file(&rot, &scale, &offset);
   return 0;
}

#endif // SOFTIRON_APP

