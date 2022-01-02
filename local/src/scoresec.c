#include "pin_types.h"
#include <stdio.h>

scoresec_type degrees_to_scoresec(
      /* in     */ const float degrees
      )
{
   int32_t tmp = (int32_t) (degrees * 65536.0f / 360.0f);
   //int32_t tmp = (int32_t) ((degrees + 0.002778f) * 65536.0f / 360.0f);
   scoresec_type ss = { .uscore = (uint16_t) (tmp & 0x0000ffff) };
   return ss;
}

degree_type scoresec_to_degrees(
      /* in     */ const scoresec_type theta
      )
{
   degree_type deg = { .degrees = (float) theta.uscore * 360.0f / 65536.0f };
   return deg;
}

// calculate midpoint of two scoresecs A and B. if B<A it is assumed that
//    this is a wrapping situation
scoresec_type average_of_scoresecs(
      /* in     */ const scoresec_type a,
      /* in     */ const scoresec_type b
      )
{
   uint16_t delta = (uint16_t) (b.uscore - a.uscore);
   uint16_t midpoint = (uint16_t) (a.uscore + delta/2);
   scoresec_type ss = { .uscore = midpoint };
   return ss;
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

#if defined(TEST_SCORESEC)

static uint32_t test_scoresec_to_degrees(void)
{
   uint32_t errs = 0;
   printf("Testing scoresec_to_degrees\n");
   /////////////////////////////////////////////
   scoresec_type ss = { .uscore = 0x8000 };
   degree_type deg = scoresec_to_degrees(ss);
   float expected = 180.0f;
   if (deg.degrees != expected) {
      fprintf(stderr, "Incorrect value. Expected %.4f, got %.4f", 
            (double) expected, (double) deg.degrees);
      errs++;
   }
   /////////////////////////////////////////////
   if (errs == 0) {
      printf("    passed\n");
   } else {
      printf("    %d errors\n", errs);
   }
   return errs;
}

static uint32_t test_degrees_to_scoresec(void)
{
   uint32_t errs = 0;
   printf("Testing degrees_to_scoresec\n");
   /////////////////////////////////////////////
   float deg = 180.0f;
   scoresec_type ss = degrees_to_scoresec(deg);
   uint16_t expected = 0x8000;
   if (ss.uscore != expected) {
      fprintf(stderr, "Incorrect value. Expected %d, got %d\n", 
            expected, ss.uscore);
      errs++;
   }
   /////////////////////////////////////////////
   if (errs == 0) {
      printf("    passed\n");
   } else {
      printf("    %d errors\n", errs);
   }
   return errs;
}

static uint32_t test_wrap(void)
{
   uint32_t errs = 0;
   printf("Testing wrapping behavior\n");
   // note that there are unresolved off-by-one rounding issues
   /////////////////////////////////////////////
   // 340 degrees is 61895(.1111) scoresec
   // 20 degrees is 3640(.8888) scoresec
   {
      float a = 10.0f;
      float b = 350.0f;
      scoresec_type ab = degrees_to_scoresec(a - b);
      uint16_t expected_ab = 3641;
      if (ab.uscore != expected_ab) {
         fprintf(stderr, "Incorrect %.1f-%.1f value. Expected %d, got %d\n", 
               (double) a, (double) b, expected_ab, ab.uscore);
         errs++;
      }
      scoresec_type ba = degrees_to_scoresec(b - a);
      uint16_t expected_ba = 61895;
      if (ba.uscore != expected_ba) {
         fprintf(stderr, "Incorrect %.1f-%.1f value. Expected %d, got %d\n", 
               (double) b, (double) a, expected_ba, ba.uscore);
         errs++;
      }
   }
   {
      float a = 30.0f;
      float b = 10.0f;
      scoresec_type ab = degrees_to_scoresec(a - b);
      uint16_t expected_ab = 3640;
      if (ab.uscore != expected_ab) {
         fprintf(stderr, "Incorrect %.1f-%.1f value. Expected %d, got %d\n", 
               (double) a, (double) b, expected_ab, ab.uscore);
         errs++;
      }
      scoresec_type ba = degrees_to_scoresec(b - a);
      uint16_t expected_ba = 61896;
      if (ba.uscore != expected_ba) {
         fprintf(stderr, "Incorrect %.1f-%.1f value. Expected %d, got %d\n", 
               (double) b, (double) a, expected_ba, ba.uscore);
         errs++;
      }
   }
   /////////////////////////////////////////////
   if (errs == 0) {
      printf("    passed\n");
   } else {
      printf("    %d errors\n", errs);
   }
   return errs;
}

static uint32_t test_average_of_scoresecs(void)
{
   uint32_t errs = 0;
   printf("Testing average_of_scoresecs\n");
   /////////////////////////////////////////////
   // easy case
   {
      scoresec_type a = { .uscore = 1000 };
      scoresec_type b = { .uscore = 5000 };
      scoresec_type c = average_of_scoresecs(a, b);
      uint16_t expected = 3000;
      if (expected != c.uscore) {
         fprintf(stderr, "Incorrect %d,%d midpoint. Expected %d, got %d\n", 
               a.uscore, b.uscore, expected, c.uscore);
         errs++;
      }
      scoresec_type d = average_of_scoresecs(b, a);
      expected = 35768;
      if (expected != d.uscore) {
         fprintf(stderr, "Incorrect %d,%d midpoint. Expected %d, got %d\n", 
               a.uscore, b.uscore, expected, d.uscore);
         errs++;
      }
   }
   /////////////////////////////////////////////
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
   uint32_t errs = 0;
   errs += test_scoresec_to_degrees();
   errs += test_degrees_to_scoresec();
   errs += test_wrap();
   errs += test_average_of_scoresecs();
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
#endif   // TEST_SCORESEC
