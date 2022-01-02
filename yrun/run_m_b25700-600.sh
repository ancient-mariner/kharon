#!/bin/bash

DATA_DIR=/pinet/yrun/2021-05-29_25700_600

T="-t 13699.000"

rpi_emulator $T -d hedgehog -i $DATA_DIR/hedgehog_imu &
rpi_emulator $T -d mercury1 -i $DATA_DIR/mercury1_gyrmag -c $DATA_DIR/mercury1_cam > out_m1 &
rpi_emulator $T -d mercury2                              -c $DATA_DIR/mercury2_cam > out_m2 &
rpi_emulator $T -d mercury3                              -c $DATA_DIR/mercury3_cam > out_m3 &
rpi_emulator $T -d mercury4 -i $DATA_DIR/mercury4_acc    -c $DATA_DIR/mercury4_cam > out_m4 &
rpi_emulator $T -d gopher -g $DATA_DIR/gopher_gps > out_g &
sleep 1
em_start.sh
#sleep 30
#sleep 60
#sleep 150
sleep 240
#sleep 600
em_kill.sh
sleep 1
killall -s SIGINT bob

