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
#include "dev_info.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <unistd.h>
#include "lin_alg.h"
#include "logger.h"

static char dev_[STR_LEN] = { 0 };

// use cases
// read content of file where path name is variable
//    (eg, /pinet/dev/<host>/sensors/<name>/foo)
// read content with fixed path
//    (eg, /pinet/<environment>/<host>/foo/bar
//
// return data of different types. e.g., byte, line, vector, matrix
// ignore lines starting with '#'
// allow reading of arbitrary series of types
//


////////////////////////////////////////////////////////////////////////
// utility functions

// terminate string at first newline character
static void truncate_at_newline(
      /*    out */       char *content,
      /* in     */ const uint32_t max_len
      )
{
   for (uint32_t i=0; i<max_len; i++) {
      char c = content[i];
      if (c == 0) {
         break;
      } else if ((c == 10) || (c == 13)) {
         content[i] = 0;
         break;
      }
   }
   content[max_len-1] = 0; // make sure buffer is null terminated
}

// reads next non-comment line and stores result in content
// newline is stripped if present
// on success, 'content' is returned
// when there's no more content, or on error, NULL is returned
char * get_next_line(
      /* in out */       FILE *fp,
      /*    out */       char *content,
      /* in     */ const uint32_t max_len
      )
{
   while (fgets(content, (int) max_len, fp) != NULL) {
      trim_leading_whitespace(content);
      char c = content[0];
      if ((c != '#') && (c != 0) && (c != 10) && (c != 13)) {
         // found a non-comment line with apparent content. strip trailing
         //    newline and return directly
         truncate_at_newline(content, max_len);
         return content;
      }
   }
   // if we made it outside the loop then no non-comment content was read
   // signal error by returning NULL
   return NULL;
}

void set_device_dir_path(
      /* in     */ const char *dev
      )
{
   strcpy(dev_, dev);
}

const char * get_device_dir(void)
{
   return dev_;
}

// utility functions
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// file management

// generates path string /<root-path>/<host>/<path1>/<obj>/<path2>
void build_path_string2(
      /* in     */ const char *root,
      /* in     */ const char *host,
      /* in     */ const char *path1,
      /* in     */ const char *obj,
      /* in     */ const char *path2,
      /*    out */       char *full_path,
      /* in     */ const uint32_t max_len
      )
{
   assert(max_len >= STR_LEN);   // string must have realistic size
   int32_t idx = 0;
   // root directory
   if (root != NULL) {
      if (root[0] == '/') {
         idx = snprintf(full_path, max_len, "%s", root);
      } else {
         idx = snprintf(full_path, max_len, "/%s", root);
      }
   } else {
      if (dev_[0] == 0) {
         // this is a config error that should be identified early in
         //    runtime. keep a standard assert to be debugger friendly,
         //    and print an explanation of what'd probably wrong.
         fprintf(stderr, "set_device_dir_path() was not called\n");
         assert(dev_[0] != 0);
      }
      if (dev_[0] == '/') {
         idx = snprintf(full_path, max_len, "%s", dev_);
      } else {
         idx = snprintf(full_path, max_len, "/%s", dev_);
      }
   }
   // hostname
   if (host != NULL) {
      idx += snprintf(&full_path[idx], max_len-(uint32_t)idx, "%s%s",
            full_path[idx-1] == '/' ? "" : "/", host);
   } else {
      char hostname[HOST_LEN];
      if (gethostname(hostname, HOST_LEN) != 0) {
         log_info_type *log = get_kernel_log();
         log_err(log, "Unable to read hostname (%s)", strerror(errno));
         full_path[0] = 0;
         return;
      }
      idx += snprintf(&full_path[idx], max_len-(uint32_t)idx, "%s%s",
            full_path[idx-1] == '/' ? "" : "/", hostname);
   }
   // include path1, obj and path2 in name if value(s) provided
   if (path1 != NULL) {
      idx += snprintf(&full_path[idx], max_len-(uint32_t)idx, "%s%s",
            full_path[idx-1] == '/' ? "" : "/", path1);
   }
   if (obj != NULL) {
      idx += snprintf(&full_path[idx], max_len-(uint32_t)idx, "%s%s",
            full_path[idx-1] == '/' ? "" : "/", obj);
   }
   if (path2 != NULL) {
      idx += snprintf(&full_path[idx], max_len-(uint32_t)idx, "%s%s",
            full_path[idx-1] == '/' ? "" : "/", path2);
   }
   full_path[max_len-1] = 0;
}

// opens file /<root-path>/<host>/<path1>/<obj>/<path2>
// returns resulting FILE *
// if host is NULL, gethostname is used
// if path1, obj or path2 are NULL, the path is created without using the
//    NULL field(s)
// calling function responsible for closing file
FILE * open_config_file_ro2(
      /* in     */ const char *root,
      /* in     */ const char *host,
      /* in     */ const char *path1,
      /* in     */ const char *obj,
      /* in     */ const char *path2
      )
{
   char fname[STR_LEN];
   build_path_string2(root, host, path1, obj, path2, fname, STR_LEN);
   FILE *fp = fopen(fname, "r");
   if (!fp) {
      log_info_type *log = get_kernel_log();
      log_err(log, "Unable to open file '%s' (ro) : %s", fname,
            strerror(errno));
   }
   // let calling process handle error
   return fp;
}

// opens file /<root-path>/<host>/<path1>/<obj>/<path2>
FILE * open_config_file_w2(
      /* in     */ const char *root,
      /* in     */ const char *host,
      /* in     */ const char *path1,
      /* in     */ const char *obj,
      /* in     */ const char *path2
      )
{
   char fname[STR_LEN];
   build_path_string2(root, host, path1, obj, path2, fname, STR_LEN);
   FILE *fp = fopen(fname, "w");
   if (!fp) {
      log_info_type *log = get_kernel_log();
      log_err(log, "Unable to open file '%s' (w) : %s", fname,
            strerror(errno));
   }
   // let calling process handle error
   return fp;
}

// open file
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// read and parse values

int32_t config_read_string(
      /* in out */       FILE *fp,
      /*    out */       char * content,
      /* in     */ const uint32_t max_len
      )
{
   if (get_next_line(fp, content, max_len) == NULL)
      goto err;
   return 0;
err:
   content[0] = 0;
   log_info_type *log = get_kernel_log();
   log_err(log, "Failed to read text line from config file");
   return -1;
}


// reads one line of config file and converts value to integer
int32_t config_read_int(
      /* in out */       FILE *fp,
      /*    out */       int32_t *i
      )
{
   char buf[STR_LEN];
   log_info_type *log = get_kernel_log();
   if (get_next_line(fp, buf, STR_LEN) == NULL)
      goto err;
   errno = 0;
   *i = (int32_t) strtol(buf, NULL, 10);
   if (errno != 0) {
      log_err(log, "Failed to parse integer (errno=%d, val='%s')", errno, buf);
      goto err;
   }
   return 0;
err:
   // on inability to read, set values to zero
   *i = 0;
   log_err(log, "Failed to read integer");
   return -1;
}

// reads one line of config file and converts first 2 chars from hex
//    to byte
int32_t config_read_byte(
      /* in out */       FILE *fp,
      /*    out */       uint8_t *b
      )
{
   char buf[STR_LEN];
   if (get_next_line(fp, buf, STR_LEN) == NULL)
      goto err;
   // convert first 2 chars from hex to binary byte
   int32_t val = 0;
   for (uint32_t i=0; i<2; i++) {
      val <<= 4;
      char c = buf[i];
      if ((c >= '0') && (c <= '9')) {
         val += c - '0';
      } else if ((c >= 'A') && (c <= 'F')) {
         val += c - 'A' + 10;
      } else if ((c >= 'a') && (c <= 'f')) {
         val += c - 'a' + 10;
      } else {
         goto err;
      }
   }
   *b = (uint8_t) val;
   return 0;
err:
   // on inability to read, set values to zero
   *b = 0;
   log_info_type *log = get_kernel_log();
   log_err(log, "Failed to read hex byte");
   return -1;
}

int32_t config_read_vector(
      /* in out */       FILE *fp,
      /*    out */       vector_type *vec
      )
{
   char buf[STR_LEN];
   if (get_next_line(fp, buf, STR_LEN) == NULL)
      goto err;
   // line loaded from config file. now parse it
   char *str = buf;
   errno = 0;
   vec->v[0] = strtof(str, &str);
   vec->v[1] = strtof(str, &str);
   vec->v[2] = strtof(str, &str);
   if (errno != 0)
      goto err;
   return 0;
err:
   zero_vector(vec);
   log_info_type *log = get_kernel_log();
   log_err(log, "Failed to parse vector from config. Read '%s'", buf);
   return -1;
}


int32_t config_read_matrix(
      /* in out */       FILE *fp,
      /*    out */       matrix_type *mat
      )
{
   char buf[STR_LEN];
   uint32_t idx = 0;
   for (uint32_t row=0; row<3; row++) {
      if (get_next_line(fp, buf, STR_LEN) == NULL)
         goto err;
      char *str = buf;
      errno = 0;
      for (uint32_t col=0; col<3; col++) {
         mat->m[idx++] = strtof(str, &str);
      }
      if (errno != 0) {
         goto err;
      }
   }
   return 0;
err:
   for (uint32_t i=0; i<9; i++) {
      mat->m[i] = 0.0f;
   }
   log_info_type *log = get_kernel_log();
   log_err(log, "Failed to read matrix from config");
   return -1;
}


// read config
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
//

int32_t config_write_vector2(
      /* in     */ const char *root,
      /* in     */ const char *host,
      /* in     */ const char *path1,
      /* in     */ const char *obj,
      /* in     */ const char *path2,
      /* in     */ const vector_type *vec
      )
{
   // if config file present, read values
   FILE *fp = open_config_file_w2(root, host, path1, obj, path2);
   if (!fp) {
      log_info_type *log = get_kernel_log();
      log_err(log, "Unable to write config file (%s)", strerror(errno));
      return -1;
   }
   fprintf(fp, "%f\t%f\t%f\n", (double) vec->v[0],
         (double) vec->v[1], (double) vec->v[2]);
   fclose(fp);
   return 0;
}


// resolve endpoint target of sensor device for specified host
// endpoint target file stores hostname and endpoint name of target
//    eg, "ghost imu_1"
// the endpoint target <env>/<host>/endpoints/<name> stores the port number
//    for the endpoint for <name>
// the file dev/<host>/ip_addr stores the IP address of <host>
int32_t resolve_endpoint_for_host(
      /* in     */ const char *endpoint_name,
      /* in     */ const char *endpoint_loc,
      /* in     */ const char *host_name,
      /*    out */       network_id_type *id
      )
{
   assert(dev_[0] != 0);
   char remotehost[STR_LEN];  // hostname where endpoint resides
   char ep[STR_LEN];    // name of service hosting endpoint
   char buf[STR_LEN];   // temporary storage
   /////////////////////////////////////////////
   // get endpoint info (e.g., "ghost imu_1")
   FILE *fp = open_config_file_ro2(dev_, host_name, endpoint_loc,
         endpoint_name, NULL);
   log_info_type *log = get_kernel_log();
   if (!fp) {
      log_err(log, "Failed to get config for host %s, endpoint %s, name %s",
            host_name, endpoint_loc, endpoint_name);
      goto err;
   }
   if (config_read_string(fp, buf, STR_LEN) != 0) {
      log_err(log, "Unable to read config string for %s", buf);
      goto err;
   }
   fclose(fp);
   fp = NULL;
   // parse out hostname
   char *tok = strtok(buf, " \t");
   if (tok == NULL) {
      goto err;
   }
   strcpy(remotehost, tok);
   // parse out endpoint name
   tok = strtok(NULL, " \t");
   if (tok == NULL) {
      goto err;
   }
   strcpy(ep, tok);
   /////////////////////////////////////////////
   // use host and endpoint name to get port and IP
   fp = open_config_file_ro2(dev_, remotehost, "endpoints", ep, NULL);
   if (!fp) {
      log_err(log, "Error opening remote host endpoint file (host=%s, name=%s)",
            remotehost, ep);
      goto err;
   }
   if (config_read_string(fp, buf, STR_LEN) != 0)
      goto err;
   fclose(fp);
   fp = NULL;
   id->port = (uint16_t) atoi(buf);
   // IP
   assert(dev_[0] != 0);
   fp = open_config_file_ro2(dev_, remotehost, "ip_addr", NULL, NULL);
   if (!fp) {
      log_err(log, "Error opening remote host ip_addr (host=%s)",
            remotehost);
      goto err;
   }
   if (config_read_string(fp, id->ip, MAX_IP_LEN) != 0)
      goto err;
   fclose(fp);
   fp = NULL;
   // use host to get IP address
   log_info(log, "%s targets %s:%d", endpoint_name, id->ip, id->port);
   return 0;
err:
   if (fp)
      fclose(fp);
   return -1;
}

// resolve endpoint target of sensor device for specified host
// endpoint target file stores hostname and endpoint name of target
//    eg, "ghost imu_1"
// the endpoint target <env>/<host>/endpoints/<name> stores the port number
//    for the endpoint for <name>
// the file dev/<host>/ip_addr stores the IP address of <host>
// this is for use by emulation
int32_t resolve_sensor_endpoint_for_host(
      /* in     */ const char *endpoint_name,
      /* in     */ const char *host_name,
      /*    out */       network_id_type *id
      )
{
   char remotehost[STR_LEN];  // hostname where endpoint resides
   char ep[STR_LEN];    // name of service hosting endpoint
   char buf[STR_LEN];   // temporary storage
   /////////////////////////////////////////////
   // get endpoint info (e.g., "ghost imu_1")
   assert(dev_[0] != 0);
   FILE *fp = open_config_file_ro2(dev_, host_name, "sensors",
         endpoint_name, NULL);
   log_info_type *log = get_kernel_log();
   if (!fp)
      goto err;
   if (config_read_string(fp, buf, STR_LEN) != 0)
      goto err;
   fclose(fp);
   fp = NULL;
   // parse out hostname
   char *tok = strtok(buf, " \t");
   if (tok == NULL) {
      goto err;
   }
   strcpy(remotehost, tok);
   // parse out endpoint name
   tok = strtok(NULL, " \t");
   if (tok == NULL) {
      goto err;
   }
   strcpy(ep, tok);
   /////////////////////////////////////////////
   // use host and endpoint name to get port and IP
   assert(dev_[0] != 0);
   fp = open_config_file_ro2(dev_, remotehost, "endpoints", ep, NULL);
   if (!fp) {
      log_err(log, "Error opening remote host endpoint file (host=%s, name=%s)",
            remotehost, ep);
      goto err;
   }
   if (config_read_string(fp, buf, STR_LEN) != 0)
      goto err;
   fclose(fp);
   fp = NULL;
   id->port = (uint16_t) atoi(buf);
   // IP
   fp = open_config_file_ro2(dev_, remotehost, "ip_addr", NULL, NULL);
   if (!fp) {
      log_err(log, "Error opening remote host ip_addr (host=%s)",
            remotehost);
      goto err;
   }
   if (config_read_string(fp, id->ip, MAX_IP_LEN) != 0)
      goto err;
   fclose(fp);
   fp = NULL;
   // use host to get IP address
   log_info(log, "%s targets %s:%d", endpoint_name, id->ip, id->port);
   return 0;
err:
   if (fp)
      fclose(fp);
   return -1;
}

//// resolve endpoint target of sensor device for specified host
//// endpoint target file stores hostname and endpoint name of target
////    eg, "ghost imu_1"
//// the endpoint target <env>/<host>/endpoints/<name> stores the port number
////    for the endpoint for <name>
//// the file dev/<host>/ip_addr stores the IP address of <host>
//int32_t resolve_sensor_endpoint_for_host(
//      /* in     */ const char *endpoint_name,
//      /* in     */ const char *host_name,
//      /*    out */       network_id_type *id
//      )
//{
//   return resolve_endpoint_for_host(endpoint_name, "sensor", host_name, id);
//}

// resolve endpoint target of sensor device for this host
// endpoint target file stores hostname and endpoint name of target
//    eg, "ghost imu_1"
// the endpoint target dev/<host>/endpoints/<name> stores the port number
//    for the endpoint for <name>
// the file dev/<host>/ip_addr stores the IP address of <host>
int32_t resolve_sensor_endpoint(
      /* in     */ const char *endpoint_name,
      /*    out */       network_id_type *id
      )
{
   return resolve_endpoint_for_host(endpoint_name, "sensors", NULL, id);
}

#if defined(DEV_INFO_TEST)

#define TEST_FILE "/tmp/test.txt"
#define TEST_TEXT "mary had a little lamb"

static uint32_t test_build_path_string(void)
{
   uint32_t errs = 0;
   printf("Testing build_path_string\n");
   //////////////////
   char buf[STR_LEN];
   {
      build_path_string2("tmp", "a", "b", "c", "d", buf, STR_LEN);
      const char ABCD[] = "/tmp/a/b/c/d";
      if (strcmp(buf, ABCD) != 0) {
         fprintf(stderr, "Bad path. Got '%s', expected '%s\n", buf, ABCD);
         errs++;
      }
   }
   {
      build_path_string2("tmp/", "a", "b", "c", NULL, buf, STR_LEN);
      const char ABC[] = "/tmp/a/b/c";
      if (strcmp(buf, ABC) != 0) {
         fprintf(stderr, "Bad path. Got '%s', expected '%s\n", buf, ABC);
         errs++;
      }
   }
   {
      build_path_string2("/tmp/", "a", NULL, "c", "d", buf, STR_LEN);
      const char ACD[] = "/tmp/a/c/d";
      if (strcmp(buf, ACD) != 0) {
         fprintf(stderr, "Bad path. Got '%s', expected '%s\n", buf, ACD);
         errs++;
      }
   }
   {
      build_path_string2("/tmp", "a", "b", NULL, NULL, buf, STR_LEN);
      const char AB[] = "/tmp/a/b";
      if (strcmp(buf, AB) != 0) {
         fprintf(stderr, "Bad path. Got '%s', expected '%s\n", buf, AB);
         errs++;
      }
   }
   {
      build_path_string2("tmp", "a", NULL, "c", NULL, buf, STR_LEN);
      const char AC[] = "/tmp/a/c";
      if (strcmp(buf, AC) != 0) {
         fprintf(stderr, "Bad path. Got '%s', expected '%s\n", buf, AC);
         errs++;
      }
   }
   {
      build_path_string2("tmp/", "a", NULL, NULL, "d", buf, STR_LEN);
      const char AD[] = "/tmp/a/d";
      if (strcmp(buf, AD) != 0) {
         fprintf(stderr, "Bad path. Got '%s', expected '%s\n", buf, AD);
         errs++;
      }
   }
   {
      char hostname[HOST_LEN];
      gethostname(hostname, HOST_LEN);
      build_path_string2("tmp", NULL, NULL, NULL, "d", buf, STR_LEN);
      char ad_buf[STR_LEN];
      sprintf(ad_buf, "/tmp/%s/d", hostname);
      if (strcmp(buf, ad_buf) != 0) {
         fprintf(stderr, "Bad path. Got '%s', expected '%s\n", buf, ad_buf);
         errs++;
      }
   }
   /////////////////////////////////////////////////////////////
   if (errs == 0) {
      printf("    passed\n");
   } else {
      printf("    %d errors\n", errs);
   }
   return errs;
}

static uint32_t write_test_file(void)
{

   FILE *fp = fopen(TEST_FILE, "w");
   if (!fp) {
      fprintf(stderr, "Unable to create test file '%s'\n", TEST_FILE);
      return 1;
   }
   fprintf(fp, "# text test\n");
   fprintf(fp, "#\n\n%s\n", TEST_TEXT);
   fprintf(fp, "# hex test\n");
   fprintf(fp, "1f\n");
   fprintf(fp, "# vector test\n");
   fprintf(fp, "1\t2.0 -3.0\n");
   fprintf(fp, "# matrix test\n");
   fprintf(fp, "1\t\t0\t0\n");
   fprintf(fp, "0\t\t1\t0\n");
   fprintf(fp, "0\t0\t1\n");
   fclose(fp);
   return 0;
}

static uint32_t test_file_read(void)
{
   uint32_t errs = 0;
   printf("Testing file reading\n");
   ///////////////////
   FILE *fp = fopen(TEST_FILE, "r");
   if (!fp) {
      fprintf(stderr, "Unable to read test file '%s'\n", TEST_FILE);
      return 1;
   }
   /////////////////////////////////////////////////////////////
   // text test
   char buf[STR_LEN];
   if (config_read_string(fp, buf, STR_LEN) != 0) {
      fprintf(stderr, "Error reading text from test file\n");
      errs++;
   }
   if (strcmp(buf, TEST_TEXT) != 0) {
      fprintf(stderr, "Incorrect text read from file.\n");
      fprintf(stderr, "Expected: '%s'\n", TEST_TEXT);
      fprintf(stderr, "Got:      '%s'\n", buf);
      errs++;
   }
   /////////////////////////////////////////////////////////////
   // hext test
   uint8_t b = 0;
   if (config_read_byte(fp, &b) != 0) {
      fprintf(stderr, "Error reading byte from test file\n");
      errs++;
   }
   const uint8_t expected_byte = 31;
   if (b != expected_byte) {
      fprintf(stderr, "Incorrect byte from test file - expected 0x%02x, "
            "got 0x%2x\n", expected_byte, b);
      errs++;
   }
   /////////////////////////////////////////////////////////////
   // vector test
   vector_type vec;
   if (config_read_vector(fp, &vec) != 0) {
      fprintf(stderr, "Error reading vector from test file\n");
      errs++;
   }
   const double v0 = 1.0;
   const double v1 = 2.0;
   const double v2 = -3.0;
   if ((v0 != vec.v[0]) || (v1 != vec.v[1]) || (v2 != vec.v[2])) {
      fprintf(stderr, "Incorrect vector from test file\n");
      fprintf(stderr, "Expected: %f,%f,%f\n", (double) v0, (double) v1,
            (double) v2);
      fprintf(stderr, "Got:      %f,%f,%f\n", (double) vec.v[0],
            (double) vec.v[1], (double) vec.v[2]);
      errs++;
   }
   /////////////////////////////////////////////////////////////
   // matrix test
   matrix_type mat;
   if (config_read_matrix(fp, &mat) != 0) {
      fprintf(stderr, "Error reading matrix from test file\n");
      errs++;
   }
   if ((mat.m[0] != 1.0) || (mat.m[1] != 0.0) || (mat.m[2] != 0.0) ||
         (mat.m[3] != 0.0) || (mat.m[4] != 1.0) || (mat.m[5] != 0.0) ||
         (mat.m[6] != 0.0) || (mat.m[7] != 0.0) || (mat.m[8] != 1.0)) {
      fprintf(stderr, "Incorrect matrix from test file. "
            "Expected identity matrix\n");
      print_mat(&mat, "Got this matrix");
      errs++;
   }
   /////////////////////////////////////////////////////////////
   fclose(fp);
   if (errs == 0) {
      printf("    passed\n");
   } else {
      printf("    %d errors\n", errs);
   }
   return errs;
}


int main(int argc, char **argv)
{
   (void) argc;
   (void) argv;
   printf("Testing dev_info\n");
   uint32_t errs = 0;
   errs += write_test_file();
   errs += test_file_read();
   errs += test_build_path_string();
   //
   if (errs == 0) {
      printf("--------------------\n");
      printf("--  Tests passed  --\n");
      printf("--------------------\n");
   } else {
      printf("---------------------------------\n");
      printf("***  One or more tests failed ***\n");
   }
   return (int) errs;
}

#endif   // DEV_INFO_TEST


