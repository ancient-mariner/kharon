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
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <alsa/asoundlib.h>
#include <unistd.h>
#include <errno.h>
#include "datap.h"

#include "routing/beeper.h"
#include "../sound.c"


int main()
{
   /////////////////////////////////////////////
   // setup
   initialize_sound_buffers();
   if (establish_sound_interface() != 0) {
      fprintf(stderr, "Failed to establish sound interface\n");
      return 1;
   }
   /////////////////////////////////////////////
   // play sounds
   pthread_t tid;
   datap_desc_type dp;
   dp.run_state = 0;
   pthread_create(&tid, NULL, thread_main, &dp);
   sleep(1);
   requests_.signed_all = -1;
   sleep(7);
   dp.run_state = DP_STATE_DONE;
   pthread_join(tid, NULL);
   /////////////////////////////////////////////
   // shutdown
   shutdown_sound_interface();
   return 0;
}


