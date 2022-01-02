#if !defined(A3G4250D_REGISTERS_H)
#define  A3G4250D_REGISTERS_H

#define WHO_AM_I                 0x0F
#define WHO_AM_I_VALUE           0xD3

#define CTRL_REG1                0x20
#define CTRL_REG2                0x21
#define CTRL_REG3                0x22
#define CTRL_REG4                0x23
#define CTRL_REG5                0x24

#define REFERENCE                0x25
#define OUT_TEMP                 0x26

#define STATUS_REG               0x27
#define OUT_X_L                  0x28
#define OUT_X_H                  0x29
#define OUT_Y_L                  0x2A
#define OUT_Y_H                  0x2B
#define OUT_Z_L                  0x2C
#define OUT_Z_H                  0x2D

#define FIFO_CTRL_REG            0x2E
#define FIFO_SRC_REG             0x2F

#endif   //  A3G4250D_REGISTERS_H
