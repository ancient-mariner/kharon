#include <errno.h>
#include "routing/beeper.h"
#include "routing/driver.h"


////////////////////////////////////////////////////////////////////////
// beeper

static int32_t create_beeper(lua_State *L)
{
   int32_t argc = lua_gettop(L);
   globals_accessed_ = 1;
   if (argc != 1)
   {
      fprintf(stderr, "Lua syntax error\n");
      fprintf(stderr, "%s requires 1 arguments\n", __func__);
      fprintf(stderr, "arg1 is object name (e.g., 'beeper')\n");
      fprintf(stderr, "encountered: %s(", __func__);
      for (int32_t i=1; i<=argc; i++)
         fprintf(stderr, "%s%s", lua_tostring(L, i), i==argc?"":", ");
      fprintf(stderr, ")\n");
      errs_++;
      return 1;
   }
   beeper_setup_type *setup = malloc(sizeof *setup);
   memset(setup, 0, sizeof *setup);
   //
   const char * str1 = get_string(L, __func__, 1);
   launch_thread(str1, BEEPER_CLASS_NAME, beeper_init, setup);
   return 0;
}

// beeper
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
// driver

static int32_t create_driver(lua_State *L)
{
   int32_t argc = lua_gettop(L);
   globals_accessed_ = 1;
   if (argc != 1)
   {
      fprintf(stderr, "Lua syntax error\n");
      fprintf(stderr, "%s requires 1 arguments\n", __func__);
      fprintf(stderr, "arg1 is object name (e.g., 'driver')\n");
      fprintf(stderr, "encountered: %s(", __func__);
      for (int32_t i=1; i<=argc; i++)
         fprintf(stderr, "%s%s", lua_tostring(L, i), i==argc?"":", ");
      fprintf(stderr, ")\n");
      errs_++;
      return 1;
   }
   driver_setup_type *setup = malloc(sizeof *setup);
   memset(setup, 0, sizeof *setup);
   //
   const char * str1 = get_string(L, __func__, 1);
   launch_thread(str1, DRIVER_CLASS_NAME, driver_init, setup);
   return 0;
}


static int32_t set_destination_(lua_State *L)
{
   int32_t errs = 0;
   world_coordinate_type dest;
   meter_type radius;
   int32_t argc = lua_gettop(L);
   if (argc != 3)
   {
      errs++;
      fprintf(stderr, "Invalid number of parameters in %s\n", __func__);
      goto end;
   }
   const char * x_deg = get_string(L, __func__, 1);
   const char * y_deg = get_string(L, __func__, 2);
   const char * rad_str = get_string(L, __func__, 3);
   //
   /////////////////////////////
   errno = 0;
   dest.x_deg = strtof(x_deg, NULL);
   if (errno != 0) {
      fprintf(stderr, "Error parsing longitude in '%s'. Value='%s'\n",
            __func__, x_deg);
      errs++;
   }
   // push longitude to (0,360]
   if (dest.x_deg < 0.0) {
      dest.x_deg += 360.0;
   }
   /////////////////////////////
   errno = 0;
   dest.y_deg = strtof(y_deg, NULL);
   if (errno != 0) {
      fprintf(stderr, "Error parsing latitude in '%s'. Value='%s'\n",
            __func__, y_deg);
      errs++;
   }
   if (fabs(dest.y_deg) > 90.0) {
      fprintf(stderr, "Error setting latitude in '%s'. Value='%s'. Did "
            "lat and lon get reversed?\n",
            __func__, y_deg);
      errs++;
   }
   /////////////////////////////
   errno = 0;
   radius.meters = strtof(rad_str, NULL);
   if (errno != 0) {
      fprintf(stderr, "Error parsing radius in '%s'. Value='%s'\n",
            __func__, rad_str);
      errs++;
   }
   /////////////////////////////
   if (errs == 0) {
      set_destination(dest, radius);
   }
end:
   if (errs != 0) {
      fprintf(stderr, "\n");
      fprintf(stderr, "%s requires 3 arguments\n", __func__);
      fprintf(stderr, "arg1 is destination longitude (0,360)\n");
      fprintf(stderr, "arg2 is destination latitude (-90,90)\n");
      fprintf(stderr, "arg3 is destination radius in meters\n");
      fprintf(stderr, "encountered: %s(", __func__);
      for (int32_t i=1; i<=argc; i++)
         fprintf(stderr, "%s%s", lua_tostring(L, i), i==argc?"":", ");
      fprintf(stderr, ")\n");
      fprintf(stderr, "Function sets driver's destination\n");
      errs_++;
      return 1;
   }
   return errs;
}

//static int32_t set_position_hint_(lua_State *L)
//{
//   int32_t errs = 0;
//   world_coordinate_type pos;
//   int32_t argc = lua_gettop(L);
//   if (argc != 2)
//   {
//      errs++;
//      fprintf(stderr, "Invalid number of parameters in %s\n", __func__);
//      goto end;
//   }
//   const char * x_deg = get_string(L, __func__, 1);
//   const char * y_deg = get_string(L, __func__, 2);
//   //
//   /////////////////////////////
//   errno = 0;
//   pos.x_deg = strtof(x_deg, NULL);
//   if (errno != 0) {
//      fprintf(stderr, "Error parsing longitude in '%s'. Value='%s'\n",
//            __func__, x_deg);
//      errs++;
//   }
//   /////////////////////////////
//   errno = 0;
//   pos.y_deg = strtof(y_deg, NULL);
//   if (errno != 0) {
//      fprintf(stderr, "Error parsing latitude in '%s'. Value='%s'\n",
//            __func__, y_deg);
//      errs++;
//   }
//   if (fabs(pos.y_deg) > 90.0) {
//      fprintf(stderr, "Error setting latitude in '%s'. Value='%s'. Did "
//            "lat and lon get reversed?\n",
//            __func__, y_deg);
//      errs++;
//   }
//   /////////////////////////////
//   if (errs == 0) {
//      set_position_hint(pos);
//   }
//end:
//   if (errs != 0) {
//      fprintf(stderr, "\n");
//      fprintf(stderr, "%s requires 2 arguments\n", __func__);
//      fprintf(stderr, "arg1 is position hint longitude (-180,180)\n");
//      fprintf(stderr, "arg2 is position hint latitude (-90,90)\n");
//      fprintf(stderr, "encountered: %s(", __func__);
//      for (int32_t i=1; i<=argc; i++)
//         fprintf(stderr, "%s%s", lua_tostring(L, i), i==argc?"":", ");
//      fprintf(stderr, ")\n");
//      fprintf(stderr, "Function sets driver's approximate start location\n");
//      errs_++;
//      return 1;
//   }
//   return errs;
//}

// turn autotracking (self-driving) on and off
static int32_t set_autotracking_(lua_State *L)
{
   int32_t errs = 0;
   int32_t argc = lua_gettop(L);
   if (argc != 1)
   {
      errs++;
      fprintf(stderr, "Invalid number of parameters in %s\n", __func__);
      fprintf(stderr, "Usage: %s(1 | 0)\n", __func__);
      fprintf(stderr, "Where 1 is turn autopilot on, 0 off\n");
      errs++;
      goto end;
   }
   const char * param = get_string(L, __func__, 1);
   uint32_t on_off = (uint32_t) atol(param);
   set_autotracking(on_off);
end:
   return errs;
}

// driver
////////////////////////////////////////////////////////////////////////

