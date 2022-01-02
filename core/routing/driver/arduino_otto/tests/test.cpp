#include <stdio.h>
#include <stdint.h>
#include <assert.h>

static int32_t analog_read_val_ = 0;
// fragile define -- should work on limited basis
#define analogRead(a)     (analog_read_val_)

#define analogWrite(a, b)     { (void) a; (void) b; }
#define pinMode(a, b)     { (void) a; (void) b; }

#define INPUT     1
#define OUTPUT    2
#define A0        0

// delay shouldn't be executed during test
#define delay(n)     assert(1 == 0) 

class SerialShell {
public:
   static void write(uint32_t val)  { printf("Serial write %d\n", val); }
   static void print(float val)     { printf("Serial print %.4f\n", (double) val); }
   static void print(int32_t val)   { printf("Serial print %d\n", val); }
   static void print(const char *str) { printf("Serial print '%s'\n", str); }
   static void begin(int32_t baud)  { (void) baud; }
   static uint8_t read()            { return 0; }
   static int32_t available()       { return 0; }
};

SerialShell Serial;

typedef int32_t elapsedMillis;

#define TEST_HARNESS    1
#include "../arduino_otto.ino"


static uint32_t test_limit_piston_deflection_by_dps(void)
{
   uint32_t errs = 0;
   printf("Testing limit_piston_deflection_by_dps\n");
   /////////////////////////////////////////////////////////////////////
   init_globals();
   float expected;
   // right turn, tiller fully extended, slow turn, large delta
   turn_rate_dps_ = DPS_PASS_LIMIT;
   target_position_ = 1.0f;
   limit_piston_deflection_by_dps(45.0);
   expected = 1.0f;
   if (fabsf(target_position_ - expected) > 0.001f) {
      fprintf(stderr, "A Piston deflection unexpectedly limited to %.3f "
            "expected %.3f\n", (double) target_position_, (double) expected);
      errs++;
   }
   // right turn, tiller fully extended, slow turn, small delta
   //    (artificial case -- tiller can't be here for small delta)
   target_position_ = 1.0f;
   limit_piston_deflection_by_dps(1.0);
   expected = 1.0f;
   if (fabsf(target_position_ - expected) > 0.001f) {
      fprintf(stderr, "B Piston deflection unexpectedly limited to %.3f "
            "expected %.3f\n", (double) target_position_, (double) expected);
      errs++;
   }
   /////////////////////////////////////////////////////////////////////
   // right turn, tiller fully extended, medium turn, large delta
   turn_rate_dps_ = 2.0f * DPS_PASS_LIMIT;
   target_position_ = 1.0f;
   limit_piston_deflection_by_dps(45.0);
   expected = 1.0f;
   if (fabsf(target_position_ - expected) > 0.001f) {
      fprintf(stderr, "C Piston deflection unexpectedly limited to %.3f "
            "expected %.3f\n", (double) target_position_, (double) expected);
      errs++;
   }
   // right turn, tiller fully extended, medium turn, medium delta
   //    (artificial case -- tiller can't be here for this delta)
   target_position_ = 1.0f;
   limit_piston_deflection_by_dps(4.0);
   expected = 0.8f;
   if (fabsf(target_position_ - expected) > 0.001f) {
      fprintf(stderr, "D Piston deflection unexpectedly limited to %.3f "
            "expected %.3f\n", (double) target_position_, (double) expected);
      errs++;
   }
   // right turn, tiller fully extended, medium turn, medium delta
   //    (artificial case -- tiller can't be here for this delta)
   target_position_ = 1.0f;
   limit_piston_deflection_by_dps(3.0);
   expected = 0.6f;
   if (fabsf(target_position_ - expected) > 0.001f) {
      fprintf(stderr, "E Piston deflection unexpectedly limited to %.3f "
            "expected %.3f\n", (double) target_position_, (double) expected);
      errs++;
   }
   // right turn, tiller fully extended, medium turn, small delta
   //    (artificial case -- tiller can't be here for this delta)
   target_position_ = 1.0f;
   limit_piston_deflection_by_dps(2.0);
   expected = 0.5f;
   if (fabsf(target_position_ - expected) > 0.001f) {
      fprintf(stderr, "F Piston deflection unexpectedly limited to %.3f "
            "expected %.3f\n", (double) target_position_, (double) expected);
      errs++;
   }
   // right turn, tiller fully extended, medium turn, small delta
   // active center to port -> turning right
   //    (artificial case -- tiller can't be here for this delta)
   target_position_ = 1.0f;
   active_center_ = 0.75f;
   limit_piston_deflection_by_dps(2.0);
   expected = 0.5f;
   if (fabsf(target_position_ - expected) > 0.001f) {
      fprintf(stderr, "G Piston deflection unexpectedly limited to %.3f "
            "expected %.3f\n", (double) target_position_, (double) expected);
      errs++;
   }
   // right turn, tiller fully extended, medium turn, small delta
   // active center to sb -> turning left
   //    (artificial case -- tiller can't be here for this delta)
   target_position_ = 1.0f;
   active_center_ = 0.25f;
   limit_piston_deflection_by_dps(2.0);
   expected = 0.25f;
   if (fabsf(target_position_ - expected) > 0.001f) {
      fprintf(stderr, "G Piston deflection unexpectedly limited to %.3f "
            "expected %.3f\n", (double) target_position_, (double) expected);
      errs++;
   }
   /////////////////////////////////////////////////////////////////////
   // right turn, tiller fully extended, hard turn, medium delta
   // active center to port -> turning right
   //    (artificial case -- tiller can't be here for this delta)
   turn_rate_dps_ = 10.0f * DPS_PASS_LIMIT;
   target_position_ = 1.0f;
   active_center_ = 0.75f;
   limit_piston_deflection_by_dps(13.0);
   expected = 0.52f;
   if (fabsf(target_position_ - expected) > 0.001f) {
      fprintf(stderr, "H Piston deflection unexpectedly limited to %.3f "
            "expected %.3f\n", (double) target_position_, (double) expected);
      errs++;
   }
   // right turn, tiller fully extended, hard turn, medium delta
   // active center to port -> turning right
   //    (artificial case -- tiller can't be here for this delta)
   turn_rate_dps_ = 10.0f * DPS_PASS_LIMIT;
   target_position_ = 1.0f;
   active_center_ = 0.75f;
   limit_piston_deflection_by_dps(12.0);
   expected = 0.5f;
   if (fabsf(target_position_ - expected) > 0.001f) {
      fprintf(stderr, "I Piston deflection unexpectedly limited to %.3f "
            "expected %.3f\n", (double) target_position_, (double) expected);
      errs++;
   }
   // right turn, tiller fully extended, hard turn, medium delta
   // active center to sb -> turning left
   //    (artificial case -- tiller can't be here for this delta)
   turn_rate_dps_ = 10.0f * DPS_PASS_LIMIT;
   target_position_ = 1.0f;
   active_center_ = 0.25f;
   limit_piston_deflection_by_dps(13.0);
   expected = 0.28f;
   if (fabsf(target_position_ - expected) > 0.001f) {
      fprintf(stderr, "J Piston deflection unexpectedly limited to %.3f "
            "expected %.3f\n", (double) target_position_, (double) expected);
      errs++;
   }
   // right turn, tiller fully extended, hard turn, medium delta
   // active center to sb -> turning left
   //    (artificial case -- tiller can't be here for this delta)
   turn_rate_dps_ = 10.0f * DPS_PASS_LIMIT;
   target_position_ = 1.0f;
   active_center_ = 0.25f;
   limit_piston_deflection_by_dps(12.0);
   expected = 0.25f;
   if (fabsf(target_position_ - expected) > 0.001f) {
      fprintf(stderr, "K Piston deflection unexpectedly limited to %.3f "
            "expected %.3f\n", (double) target_position_, (double) expected);
      errs++;
   }
   /////////////////////////////////////////////////////////////////////
   if (errs == 0) {
      printf("    passed\n");
   } else {
      printf("    %d errors\n", errs);
   }
   return errs;
}


static uint32_t test_update_actuator(void)
{
   uint32_t errs = 0;
   printf("Testing update_actuator\n");
   /////////////////////////////////////////////////////////////////////
   init_globals();
   float expected;
   course_ = 512;
   update_actuator();
   expected = 0.5f;
   if (fabsf(target_position_ - expected) > 0.001f) {
      fprintf(stderr, "A Invalid course, rudder at %.3f "
            "expected %.3f\n", (double) target_position_, (double) expected);
      errs++;
   }
   // no I, just PD
   init_globals();
   course_ = 350;
   heading_ = 5;
   update_actuator();
   expected = 0.333f;
   if (fabsf(target_position_ - expected) > 0.001f) {
      fprintf(stderr, "B 15-deg left turn problem, rudder at %.3f "
            "expected %.3f\n", (double) target_position_, (double) expected);
      errs++;
   }
   init_globals();
   course_ = 5;
   heading_ = 350;
   update_actuator();
   expected = 0.667f;
   if (fabsf(target_position_ - expected) > 0.001f) {
      fprintf(stderr, "B 15-deg right turn problem, rudder at %.3f "
            "expected %.3f\n", (double) target_position_, (double) expected);
      errs++;
   }
   // use I
   // enable integration but don't alter heading
   init_globals();
   course_ = 5;
   heading_ = 350;
   for (uint32_t i=0; i<130; i++) {
      update_actuator();
   }
   expected = 0.8667f;
   if (fabsf(target_position_ - expected) > 0.001f) {
      fprintf(stderr, "C 15-deg right, I active, rudder at %.4f "
            "expected %.4f\n", (double) target_position_, (double) expected);
      errs++;
   }
   // enable integration, slowly alter heading
   init_globals();
   course_ = 90;
   heading_ = 80;
   for (uint32_t i=0; i<150; i++) {
      if ((i < 100) && ((i % 10) == 0)) {
         heading_++;
      }
      update_actuator();
   }
   expected = 0.5625f;
   if (fabsf(active_center_ - expected) > 0.001f) {
      fprintf(stderr, "D 10-deg right, I active, heading update, "
            "center at %.4f expected %.4f\n", 
            (double) target_position_, (double) expected);
      errs++;
   }
   /////////////////////////////////////////////////////////////////////
   if (errs == 0) {
      printf("    passed\n");
   } else {
      printf("    %d errors\n", errs);
   }
   return errs;
}

//static uint32_t test_parse_heading_data(void)
//{
//   uint32_t errs = 0;
//   printf("Testing parse_heading_data\n");
//   /////////////////////////////////////////////////////////////////////
//   errs++;
//   /////////////////////////////////////////////////////////////////////
//   if (errs == 0) {
//      printf("    passed\n");
//   } else {
//      printf("    %d errors\n", errs);
//   }
//   return errs;
//}


int main(int argc, char** argv) {
   (void) argc;
   uint32_t errs = 0;
   errs += test_limit_piston_deflection_by_dps();
   errs += test_update_actuator();
//   errs += test_parse_heading_data();
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
