#if !defined(DEV_INFO_H)
#define  DEV_INFO_H
#include "pin_types.h"
#include <stdio.h>

#define DEFAULT_DEVICE_DIR_PATH  "/opt/kharon/data/dev/"

// called during initialization (via init script) to set device directory
void set_device_dir_path(
      /* in     */ const char *env
      );

// returns device directory path
const char * get_device_dir(void);


// generates path string /<root-path>/<host>/<path1>/<obj>/<path2>
// if root is NULL then it defaults to get_device_dir()
// if hostname is NULL then the value from gethostname() is used
// if path1, obj, and/or path2 is NULL, the value is not used to create
//    the path
void build_path_string2(
      /* in     */ const char *root,
      /* in     */ const char *host,
      /* in     */ const char *path1,
      /* in     */ const char *obj,
      /* in     */ const char *path2,
      /*    out */       char *full_path,
      /* in     */ const uint32_t max_len
      );

//
// opens file /<root-path>/<host>/<path1>/<obj>/<path2> for reading / writing
// this is a simple wrapper around build_path_string() and fopen()
// calling function needs to call fclose() on returned FILE *
//
FILE * open_config_file_ro2(
      /* in     */ const char *root,
      /* in     */ const char *host,
      /* in     */ const char *path1,
      /* in     */ const char *obj,
      /* in     */ const char *path2
      );

FILE * open_config_file_w2(
      /* in     */ const char *root,
      /* in     */ const char *host,
      /* in     */ const char *path1,
      /* in     */ const char *obj,
      /* in     */ const char *path2
      );


////////////////////////////////////////////////
// read data out of opened FILE

int32_t config_read_string(
      /* in out */       FILE *fp,
      /*    out */       char * content,
      /* in     */ const uint32_t max_len
      );

int32_t config_read_byte(
      /* in out */       FILE *fp,
      /*    out */       uint8_t *b
      );

int32_t config_read_vector(
      /* in out */       FILE *fp,
      /*    out */       vector_type *vec
      );

int32_t config_read_matrix(
      /* in out */       FILE *fp,
      /*    out */       matrix_type *mt
      );

// reads one line of config file and converts value to integer
int32_t config_read_int(
      /* in out */       FILE *fp,
      /*    out */       int32_t *i
      );

// reads next non-comment line and stores result in content
// newline is stripped if present
// on success, 'content' is returned
// when there's no more content, or on error, NULL is returned
char * get_next_line(
      /* in out */       FILE *fp,
      /*    out */       char *content,
      /* in     */ const uint32_t max_len
      );


//////////////////////////////////////////////////////

// resolve endpoint target of sensor device for this host
// endpoint target file stores hostname and endpoint name of target
//    eg, "ghost imu_1"
// the endpoint target <env>/<host>/endpoints/<name> stores the port number
// the file <env>/<host>/ip_addr stores the IP address
int32_t resolve_sensor_endpoint(
      /* in     */ const char *endpoint_name,
      /*    out */       network_id_type *id
      );

// this is for emulating running on a different host
int32_t resolve_sensor_endpoint_for_host(
      /* in     */ const char *endpoint_name,
      /* in     */ const char *host_name,
      /*    out */       network_id_type *id
      );

// resolve endpoint for a given host, with endpoint not necessarily
//    for sensors. 
// endpoint_loc is the name of the directory under /dev/host where
//    the endpoint is stored (e.g., 'sensors' or 'data_source')
int32_t resolve_endpoint_for_host(
      /* in     */ const char *endpoint_name,
      /* in     */ const char *endpoint_loc, 
      /* in     */ const char *host_name,
      /*    out */       network_id_type *id
      );

////////////////////////////////////////////////////////////////////////

// writes a vector file to specified file
// name generated using build_path_string()
int32_t config_write_vector2(
      /* in     */ const char *root,
      /* in     */ const char *host,
      /* in     */ const char *path1,
      /* in     */ const char *obj,
      /* in     */ const char *path2,
      /* in     */ const vector_type *vec
      );

#endif   // DEV_INFO_H
