#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <string.h>
#include <math.h>
#include "world_map.h"

#include "../declination.c"

static int32_t test_load_declination(void)
{
   int32_t errs = 0;
   printf("testing load_declination\n");
   /////////////////////////////////////////////////////////////
   world_coordinate_type latlon;
   const char * fname = "/pinet/mapping/master/magnetic.txt";
   double declination, inclination, expected, expected2;
   //
   latlon.latitude = 90.0;
   latlon.longitude = 0.0;
   errs += load_declination(fname, latlon, &declination, &inclination);
   expected = 4.23;
   if (fabs(declination - expected) > 0.001) {
      fprintf(stderr, "A declination %.1f,%.1f: Got %f, expected %f\n", 
            (double) latlon.lon, (double) latlon.lat, (double) declination, 
            (double) expected);
      errs++;
   }
   expected2 = 87.66;
   if (fabs(inclination - expected2) > 0.001) {
      fprintf(stderr, "A inclination %.1f,%.1f: Got %f, expected %f\n", 
            (double) latlon.lon, (double) latlon.lat, (double) inclination, 
            (double) expected2);
      errs++;
   }
   //
   latlon.latitude = 49.0;
   latlon.longitude = -123.0;
   errs += load_declination(fname, latlon, &declination, &inclination);
   expected = 15.97;
   if (fabs(declination - expected) > 0.001) {
      fprintf(stderr, "B declination %.1f,%.1f: Got %f, expected %f\n", 
            (double) latlon.lon, (double) latlon.lat, (double) declination, 
            (double) expected);
      errs++;
   }
   expected2 = 69.89;
   if (fabs(inclination - expected2) > 0.001) {
      fprintf(stderr, "B inclination %.1f,%.1f: Got %f, expected %f\n", 
            (double) latlon.lon, (double) latlon.lat, (double) inclination, 
            (double) expected2);
      errs++;
   }
   //
   latlon.latitude = 49.0;
   latlon.longitude = 237.0;
   errs += load_declination(fname, latlon, &declination, &inclination);
   if (fabs(declination - expected) > 0.001) {
      fprintf(stderr, "C declination %.1f,%.1f: Got %f, expected %f\n", 
            (double) latlon.lon, (double) latlon.lat, (double) declination, 
            (double) expected);
      errs++;
   }
   if (fabs(inclination - expected2) > 0.001) {
      fprintf(stderr, "C inclination %.1f,%.1f: Got %f, expected %f\n", 
            (double) latlon.lon, (double) latlon.lat, (double) inclination, 
            (double) expected2);
      errs++;
   }
   /////////////////////////////////////////////////////////////
   if (errs == 0) {
      printf("    passed\n");
   } else {
      printf(" ** %d error(s)\n", errs);
   }
   return errs;
}


int main(int argc, char** argv)
{
   (void) argc;
   (void) argv;
   int32_t errs = 0;
   errs += test_load_declination();
   //////////////////
   printf("\n");
   if (errs == 0) {
      printf("--------------------\n");
      printf("--  Tests passed  --\n");
      printf("--------------------\n");
   } else {
      printf("**********************************\n");
      printf("**** ONE OR MORE TESTS FAILED ****\n");
      printf("**********************************\n");
      fprintf(stderr, "%s failed\n", argv[0]);
   }
   return (int) errs;
}

