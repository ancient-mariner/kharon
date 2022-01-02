#include "core_modules/attitude.h"
#include "core_modules/frame_sync.h"
#include "core_modules/gps_receiver.h"
#include "core_modules/imu_receiver.h"
#include "core_modules/optical_up.h"
#include "core_modules/panorama.h"
#include "core_modules/vy_receiver.h"

////////////////////////////////////////////////////////////////////////
// attitude 

static int32_t create_attitude(lua_State *L)
{
   globals_accessed_ = 1;
   int32_t argc = lua_gettop(L);
   if (argc != 2)
   {
      fprintf(stderr, "Lua syntax error\n");
      fprintf(stderr, "%s requires 2 arguments\n", __func__);
      fprintf(stderr, "arg1 is attitude module name\n");
      fprintf(stderr, "arg2 is logging indicator ['log' | 'no-log']\n");
      fprintf(stderr, "encountered: %s(", __func__);
      for (int32_t i=1; i<=argc; i++)
         fprintf(stderr, "%s%s", lua_tostring(L, i), i==argc?"":", ");
      fprintf(stderr, ")\n");
      errs_++;
      return 1;
   }
   const char * str1 = get_string(L, __func__, 1);
   const char * str2 = get_string(L, __func__, 2);
   // allocated here -- must be freed in attitude object
   attitude_setup_type *att_setup = malloc(sizeof *att_setup);
   att_setup->logging = determine_logging_state(str2);
   launch_thread(str1, ATTITUDE_CLASS_NAME, attitude_class_init, att_setup);
   return 0;
}


static int32_t set_imu_priority_(lua_State *L)
{
   int32_t argc = lua_gettop(L);
   if (argc != 4)
   {
      fprintf(stderr, "Lua syntax error\n");
      fprintf(stderr, "%s requires 4 arguments\n", __func__);
      fprintf(stderr, "arg1 is imu module name\n");
      fprintf(stderr, "arg2 is priority of GYR\n");
      fprintf(stderr, "arg3 is priority of ACC\n");
      fprintf(stderr, "arg4 is priority of MAG\n");
      fprintf(stderr, "Priorities are between %d and %d; %d for absent\n",
            IMU_PRI_1+1, IMU_PRI_NULL, IMU_PRI_NULL+1);
      fprintf(stderr, "encountered: %s(", __func__);
      for (int32_t i=1; i<=argc; i++)
         fprintf(stderr, "%s%s", lua_tostring(L, i), i==argc?"":", ");
      fprintf(stderr, ")\n");
      errs_++;
      return 1;
   }
   const char * imu = get_string(L, __func__, 1);
   datap_desc_type *imu_prod = find_source(imu);
   // subtract 1. internal values are 0-based, script values are 1-based
   uint32_t gyr = (uint32_t) lua_tointeger(L, 2) - 1;
   uint32_t acc = (uint32_t) lua_tointeger(L, 3) - 1;
   uint32_t mag = (uint32_t) lua_tointeger(L, 4) - 1;
   uint32_t low = (acc < gyr) ? acc : gyr;
   uint32_t high = (acc > gyr) ? acc : gyr;
   low = low < mag ? low : mag;
   high = high > mag ? high : mag;
   if ((low < IMU_PRI_1+1) || (high > IMU_PRI_NULL)) {
   }
   if (argc != 4)
   {
      fprintf(stderr, "Configuration error\n");
      fprintf(stderr, "Priorities must be between %d and %d; %d for absent\n",
            IMU_PRI_1+1, IMU_PRI_NULL, IMU_PRI_NULL+1);
      fprintf(stderr, "encountered: %s(", __func__);
      for (int32_t i=1; i<=argc; i++)
         fprintf(stderr, "%s%s", lua_tostring(L, i), i==argc?"":", ");
      fprintf(stderr, ")\n");
      errs_++;
      return 1;
   }
   set_imu_priority(imu_prod, acc, gyr, mag);
   return 0;
}

// attitude 
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// frame_sync 

static int32_t create_frame_sync(lua_State *L)
{
   int32_t argc = lua_gettop(L);
   globals_accessed_ = 1;
   if (argc != 2)
   {
      fprintf(stderr, "Lua syntax error\n");
      fprintf(stderr, "%s requires 2 arguments\n", __func__);
      fprintf(stderr, "arg1 is object name (e.g., 'frame_sync')\n");
      fprintf(stderr, "arg2 is logging indicator ('log', 'no-log')\n");
      fprintf(stderr, "encountered: %s(", __func__);
      for (int32_t i=1; i<=argc; i++)
         fprintf(stderr, "%s%s", lua_tostring(L, i), i==argc?"":", ");
      fprintf(stderr, ")\n");
      errs_++;
      return 1;
   }
   const char * str1 = get_string(L, __func__, 1);
   const char * str2 = get_string(L, __func__, 2);
   frame_sync_setup_type *sync_setup = malloc(sizeof *sync_setup);
   sync_setup->logging = determine_logging_state(str2);
   launch_thread(str1, FRAME_SYNC_CLASS_NAME, frame_sync_class_init, 
         sync_setup);
   return 0;
}

// frame_sync 
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// gps

static int32_t create_gps_receiver(lua_State *L)
{
   int32_t argc = lua_gettop(L);
   globals_accessed_ = 1;
   if (argc != 2)
   {
      fprintf(stderr, "Lua syntax error\n");
      fprintf(stderr, "%s requires 2 arguments\n", __func__);
      fprintf(stderr, "arg1 is object name (e.g., 'gps')\n");
      fprintf(stderr, "arg2 is logging indicator ('log', 'no-log')\n");
      fprintf(stderr, "encountered: %s(", __func__);
      for (int32_t i=1; i<=argc; i++)
         fprintf(stderr, "%s%s", lua_tostring(L, i), i==argc?"":", ");
      fprintf(stderr, ")\n");
      errs_++;
      return 1;
   }
   const char * str1 = get_string(L, __func__, 1);
   const char * str2 = get_string(L, __func__, 2);
   gps_receiver_setup_type *gps_setup = malloc(sizeof *gps_setup);
   gps_setup->logging = determine_logging_state(str2);
   strcpy(gps_setup->device_name, str1);
   launch_thread(str1, GPS_RECEIVER_CLASS_NAME, gps_receiver_init, gps_setup);
   return 0;
}

//static int32_t create_gps_reader(lua_State *L)
//{
//   int32_t argc = lua_gettop(L);
//   globals_accessed_ = 1;
//   if (argc != 2)
//   {
//      fprintf(stderr, "Lua syntax error\n");
//      fprintf(stderr, "%s requires 2 arguments\n", __func__);
//      fprintf(stderr, "arg1 is object name (e.g., 'gps')\n");
//      fprintf(stderr, "arg2 is logging indicator ('log', 'no-log')\n");
//      fprintf(stderr, "encountered: %s(", __func__);
//      for (int32_t i=1; i<=argc; i++)
//         fprintf(stderr, "%s%s", lua_tostring(L, i), i==argc?"":", ");
//      fprintf(stderr, ")\n");
//      errs_++;
//      return 1;
//   }
//   const char * str1 = get_string(L, __func__, 1);
//   const char * str2 = get_string(L, __func__, 2);
//   gps_reader_setup_type *gps_setup = malloc(sizeof *gps_setup);
//   gps_setup->logging = determine_logging_state(str2);
//   strcpy(gps_setup->device_name, str1);
//   launch_thread(str1, GPS_READER_CLASS_NAME, gps_reader_init, gps_setup);
//   return 0;
//}

// gps
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// imu_receiver 

static int32_t create_imu_receiver(lua_State *L)
{
   int32_t argc = lua_gettop(L);
   globals_accessed_ = 1;
   if (argc != 3)
   {
      fprintf(stderr, "Lua syntax error\n");
      fprintf(stderr, "%s requires 3 arguments\n", __func__);
      fprintf(stderr, "arg1 is module name (arbitrary)\n");
      fprintf(stderr, "arg2 is name of the hosting device (under dev/)\n");
      fprintf(stderr, "arg3 is logging indicator ('log', 'no-log')\n");
      fprintf(stderr, "encountered: create_imu_receiver(");
      for (int32_t i=1; i<=argc; i++)
         fprintf(stderr, "%s%s", lua_tostring(L, i), i==argc?"":", ");
      fprintf(stderr, ")\n");
      errs_++;
      return 1;
   }
   const char * str1 = get_string(L, __func__, 1);
   const char * str2 = get_string(L, __func__, 2);
   const char * str3 = get_string(L, __func__, 3);
   // allocated here -- must be freed in imu receiver
   imu_setup_type *imu_setup = malloc(sizeof *imu_setup);
   strncpy(imu_setup->device_name, str2, MAX_NAME_LEN-1);
   imu_setup->logging = determine_logging_state(str3);
   launch_thread(str1, IMU_CLASS_NAME, imu_class_init, imu_setup);
   return 0;
}

// imu_receiver 
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// optical_up 

static int32_t create_optical_up(lua_State *L)
{
   int32_t argc = lua_gettop(L);
   globals_accessed_ = 1;
   if (argc != 2)
   {
      fprintf(stderr, "Lua syntax error\n");
      fprintf(stderr, "%s requires 2 arguments\n", __func__);
      fprintf(stderr, "arg1 is object name (e.g., 'up_1')\n");
      fprintf(stderr, "arg2 is logging indicator ('log', 'no-log')\n");
      fprintf(stderr, "encountered: %s(", __func__);
      for (int32_t i=1; i<=argc; i++)
         fprintf(stderr, "%s%s", lua_tostring(L, i), i==argc?"":", ");
      fprintf(stderr, ")\n");
      errs_++;
      return 1;
   }
   const char * str1 = get_string(L, __func__, 1);
   const char * str2 = get_string(L, __func__, 2);
   // allocated here -- must be freed in imu receiver
   optical_up_setup_type *optical_setup = malloc(sizeof *optical_setup);
   optical_setup->logging = determine_logging_state(str2);
   launch_thread(str1, OPTICAL_UP_CLASS_NAME, optical_up_class_init, 
         optical_setup);
   return 0;
}

// optical_up 
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// panorama 

static int32_t create_panorama(lua_State *L)
{
   int32_t argc = lua_gettop(L);
   globals_accessed_ = 1;
   if (argc != 6)
   {
      fprintf(stderr, "Lua syntax error\n");
      fprintf(stderr, "%s requires 6 arguments\n", __func__);
      fprintf(stderr, "arg1 is object name (e.g., 'panorama')\n");
      fprintf(stderr, "arg2 is logging indicator ('log', 'no-log')\n");
      fprintf(stderr, "arg3 is output mode ('gray', 'color')\n");
      fprintf(stderr, "arg4 is height of camera above water, in meters\n");
      fprintf(stderr, "arg5 is distance of camera forward of ship's "
            "rotational axis, in meters\n");
      fprintf(stderr, "arg6 is distance of camera starboard of "
            "ship's centerline, in meters\n");
      fprintf(stderr, "encountered: %s(", __func__);
      for (int32_t i=1; i<=argc; i++)
         fprintf(stderr, "%s%s", lua_tostring(L, i), i==argc?"":", ");
      fprintf(stderr, ")\n");
      errs_++;
      return 1;
   }
   const char * str1 = get_string(L, __func__, 1);
   const char * str2 = get_string(L, __func__, 2);
   const char * str3 = get_string(L, __func__, 3);
   const char * str4 = get_string(L, __func__, 4);
   const char * str5 = get_string(L, __func__, 5);
   const char * str6 = get_string(L, __func__, 6);
   panorama_setup_type *pan_setup = malloc(sizeof *pan_setup);
   pan_setup->logging = determine_logging_state(str2);
   pan_setup->output_type = determine_color_state(str3);
   pan_setup->camera_height_meters.meters = atof(str4);
   if (pan_setup->camera_height_meters.meters <= 0.1) {
      fprintf(stderr, "Camera height for %s was %s, which is too low\n",
            str1, str4);
      errs_++;
      return 1;
   }
   pan_setup->camera_position_forward.meters = atof(str5);
   pan_setup->camera_position_starboard.meters = atof(str6);
   launch_thread(str1, PANORAMA_CLASS_NAME, panorama_init, pan_setup);
   return 0;
}

//static int32_t define_phantom_image(lua_State *L)
//{
//   int32_t argc = lua_gettop(L);
//   if (argc != 2)
//   {
//      fprintf(stderr, "Lua syntax error\n");
//      fprintf(stderr, "%s requires 2 arguments\n", __func__);
//      fprintf(stderr, "arg1 is panorama module name\n");
//      fprintf(stderr, "arg2 is config file describing phantom\n");
//      fprintf(stderr, "see include/modules/panorama.h for config format\n");
//      fprintf(stderr, "encountered: %s(", __func__);
//      for (int32_t i=1; i<=argc; i++)
//         fprintf(stderr, "%s%s", lua_tostring(L, i), i==argc?"":", ");
//      fprintf(stderr, ")\n");
//      errs_++;
//      return 1;
//   }
//   const char * pan = get_string(L, __func__, 1);
//   const char * config = get_string(L, __func__, 2);
//   datap_desc_type *pan_prod = find_source(pan);
//   insert_phantom_image(pan_prod, config);
//   return 0;
//}


// panorama 
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// vy_receiver 

static int32_t create_vy_receiver(lua_State *L)
{
   int32_t argc = lua_gettop(L);
   globals_accessed_ = 1;
   if (argc != 4)
   {
      fprintf(stderr, "Lua syntax error\n");
      fprintf(stderr, "%s requires 4 arguments\n", __func__);
      fprintf(stderr, "arg1 is camera producer name\n");
      fprintf(stderr, "arg2 is camera device name\n");
      fprintf(stderr, "arg3 is camera number, on [0,%d)\n", MAX_NUM_CAMERAS);
      fprintf(stderr, "arg4 is logging indicator ('log', 'no-log')\n");
      goto err;
   }
   const char * str1 = get_string(L, __func__, 1);
   const char * str2 = get_string(L, __func__, 2);
   const char * str3 = get_string(L, __func__, 3);
   const char * str4 = get_string(L, __func__, 4);
   // setup struct allocated here -- must be freed in vy receiver
   vy_setup_type *vy_setup = malloc(sizeof *vy_setup);
   strncpy(vy_setup->device_name, str2, MAX_NAME_LEN);
   int32_t num = atoi(str3);
   if ((num < 0) || (num > MAX_NUM_CAMERAS)) {
      fprintf(stderr, "Camera number not valid. Must be on [0,%d)\n",
            MAX_NUM_CAMERAS);
      goto err;
   }
   vy_setup->camera_num = (uint8_t) num;
   vy_setup->logging = determine_logging_state(str4);
   launch_thread(str1, VY_CLASS_NAME, vy_class_init, vy_setup);
   return 0;
err:
   errs_++;
   fprintf(stderr, "Function call: %s(", __func__);
   for (int32_t i=1; i<=argc; i++)
      fprintf(stderr, "%s%s", lua_tostring(L, i), i==argc?"":", ");
   fprintf(stderr, ")\n");
   return 1;
}

