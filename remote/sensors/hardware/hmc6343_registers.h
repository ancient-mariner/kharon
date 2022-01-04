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
#ifndef MAX21K_REGISTERS_H
#define MAX21K_REGISTERS_H

// shift due 7-bit default
//#define ACC_ADDRESS   0x32
//#define MAG_ADDRESS   0x32
//#define TEMP_ADDRESS   0x32

#define SLAVE_ADDR      0x00
#define OP_MODE1        0x04
#define OP_MODE2        0x05


#define DEVIAT_LSB      0x0A
#define DEVIAT_MSB      0x0B
#define VAR_LSB         0x0C
#define VAR_MSB         0x0D

#define X_OFF_LSB       0x0E
#define X_OFF_MSB       0x0F
#define Y_OFF_LSB       0x10
#define Y_OFF_MSB       0x11
#define Z_OFF_LSB       0x12
#define Z_OFF_MSB       0x13
#define FILTER_LSB      0x14
#define FILTER_MSB      0x15

#define ACC_XYZ_BIG     0x40
#define MAG_XYZ_BIG     0x45
#define HEAD_XYZ_BIG    0x50
#define TILT_XYZ_BIG    0x55

#define ENTER_CALIB     0x71
#define ORI_Z_UP        0x72
#define ORI_Y_UP        0x73
#define ORI_X_UP        0x74
#define ENTER_RUN       0x75
#define ENTER_STDBY     0x76
#define EXIT_CALIB      0x7E
#define RESET           0x82
#define ENTER_SLEEP     0x83
#define EXIT_SLEEP      0x84
//
//#define DSYNC_CFG     0x1A
//#define DSYNC_CNT     0x1B

#define WHO_AM_I      0x20

#define BANK_SELECT   0x21
#define SYSTEM_STATUS 0x22

#define GYRO_X_H      0x23
#define GYRO_X_L      0x24
#define GYRO_Y_H      0x25
#define GYRO_Y_L      0x26
#define GYRO_Z_H      0x27
#define GYRO_Z_L      0x28

#define TEMP_H        0x29
#define TEMP_L        0x2A

#define HP_RST        0x3B
#define FIFO_COUNT    0x3C
#define FIFO_STATUS   0x3D

#define FIFO_DATA     0x3E
#define PAR_RST       0x3F

#define INT1_CFG      0x30
#define INT1_SRC      0x31
#define INT1_THS_XH   0x32
#define INT1_THS_XL   0x33
#define INT1_THS_YH   0x34
#define INT1_THS_YL   0x35
#define INT1_THS_ZH   0x36
#define INT1_THS_ZL   0x37
#define INT1_DURATION 0x38

#endif   // MAX21K_REGISTERS_H
