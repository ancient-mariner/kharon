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
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

// test drift correction addition to complementary filter
//    (clone of algorithm only)
// create sine wave, measure difference between samples. use this
//    as analog of gyro signal (ie, measured change). add noise
//    to each sine sample and use that to mix with gyro for complementary
//    filter (ie, noisy sine as analog of acc signal)
// add constant offset ('drift') to delta and check correction ability

#define SIM_DUR_SEC     200.0
#define DT_SEC          0.01

#define TAU_COMP_SEC         20.0
#define TAU_COMP             (DT_SEC / TAU_COMP_SEC)

#define TAU_CORRECTED_SEC         10.0
#define TAU_CORRECTED             (DT_SEC / TAU_CORRECTED_SEC)

#define SIN_PERIOD_SEC     2.0

#define NOISE_MAG    0.1

#define DRIFT    0.0005

int main(int argc, const char **argv)
{
   (void) argc;
   (void) argv;
   double est_val = 0.0;
   double est_err = 0.0;
   double t = 0.0;
   double last_v = 0.0;
   while (t < SIM_DUR_SEC) {
      double actual = sin(t * 2.0 * M_PI / SIN_PERIOD_SEC);
      double noise = 2.0 * drand48() - 1.0;
      noise *= fabs(noise);
      double delta = actual - last_v + DRIFT;
      double observed = actual + NOISE_MAG * noise;
      est_val = (1.0 - TAU_COMP) * (est_val + delta) + TAU_COMP * observed;
      double err = est_val - observed;
      est_err = (1.0 - TAU_CORRECTED) * est_err + TAU_CORRECTED * err;
printf("%.3f   %7.3f   %7.3f   %7.3f   %7.3f    %7.4f   %7.5f\n", t, actual, observed, est_val, est_err, est_val - est_err, (est_val - est_err) - actual);
      t += DT_SEC;
      last_v = actual;
   }
   return 0;
}


