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
#if !defined(LIS3MDL_REGISTERS_H)
#define  LIS3MDL_REGISTERS_H

#define WHO_AM_I                    0x0F
#define WHO_AM_I_VALUE              0x3D

#define CTRL_REG1                   0x20
#define CTRL_REG2                   0x21
#define CTRL_REG3                   0x22
#define CTRL_REG4                   0x23
#define CTRL_REG5                   0x24

#define STATUS_REG                  0x27
#define OUT_X_L                     0x28
#define OUT_X_H                     0x29
#define OUT_Y_L                     0x2A
#define OUT_Y_H                     0x2B
#define OUT_Z_L                     0x2C
#define OUT_Z_H                     0x2D

#define TEMP_OUT_L                  0x2E
#define TEMP_OUT_H                  0x2F

//#define INT_CFG                     0x30
//#define INT_SRC                     0x31
//#define INT_THS_L                   0x32
//#define INT_THS_H                   0x33

#endif   //  LIS3MDL_REGISTERS_H
