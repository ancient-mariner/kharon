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
#ifndef L3G_REGISTERS_H
#define L3G_REGISTERS_H

// shift due 7-bit default
//#define L3G_GYR_ADDRESS   0x6b

#define WHO_AM_I      0x0F

#define CTRL_REG1     0x20
#define CTRL_REG2     0x21
#define CTRL_REG3     0x22
#define CTRL_REG4     0x23
#define CTRL_REG5     0x24

#define REFERENCE     0x25
#define OUT_TEMP      0x26
#define STATUS_REG    0x27

#define OUT_X_L       0x28
#define OUT_X_H       0x29
#define OUT_Y_L       0x2A
#define OUT_Y_H       0x2B
#define OUT_Z_L       0x2C
#define OUT_Z_H       0x2D

#define FIFO_CTRL_REG 0x2E
#define FIFO_SRC_REG  0x2F

#define INT1_CFG      0x30
#define INT1_SRC      0x31
#define INT1_THS_XH   0x32
#define INT1_THS_XL   0x33
#define INT1_THS_YH   0x34
#define INT1_THS_YL   0x35
#define INT1_THS_ZH   0x36
#define INT1_THS_ZL   0x37
#define INT1_DURATION 0x38

#endif   // L3G_REGISTERS_H
