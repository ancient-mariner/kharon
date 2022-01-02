#if !defined(BMG160_REGISTERS_H)
#define  BMG160_REGISTERS_H

#define BMG160_CHIP_ID_ADDR						   0x00

#define RATE_X_LSB_ADDR                  0x02
#define RATE_X_MSB_ADDR                  0x03
#define RATE_Y_LSB_ADDR                  0x04
#define RATE_Y_MSB_ADDR                  0x05
#define RATE_Z_LSB_ADDR                  0x06
#define RATE_Z_MSB_ADDR                  0x07
#define TEMP_ADDR                        0x08

#define INTR_STAT0_ADDR                  0x09
#define INTR_STAT1_ADDR                  0x0A
#define INTR_STAT2_ADDR                  0x0B
#define INTR_STAT3_ADDR                  0x0C
#define FIFO_STAT_ADDR                   0x0E

#define RANGE_ADDR                       0x0F
#define BW_ADDR                          0x10
#define MODE_LPM1_ADDR                   0x11
#define MODE_LPM2_ADDR                   0x12
#define RATE_HBW_ADDR                    0x13
#define BGW_SOFT_RST_ADDR                0x14

#define INTR_ENABLE0_ADDR                0x15
#define INTR_ENABLE1_ADDR                0x16
#define INTR_MAP_ZERO_ADDR               0x17
#define INTR_MAP_ONE_ADDR                0x18
#define INTR_MAP_TWO_ADDR                0x19
#define INTR_ZERO_ADDR                   0x1A
#define INTR_ONE_ADDR                    0x1B
#define INTR_TWO_ADDR                    0x1C
#define INTR_4_ADDR                      0x1E
#define INTR_RST_LATCH_ADDR              0x21

#define HIGHRATE_THRES_X_ADDR            0x22
#define HIGHRATE_DURN_X_ADDR             0x23
#define HIGHRATE_THRES_Y_ADDR            0x24
#define HIGHRATE_DURN_Y_ADDR             0x25
#define HIGHRATE_THRES_Z_ADDR            0x26
#define HIGHRATE_DURN_Z_ADDR             0x27
#define SOC_ADDR                         0x31

#define A_FOC_ADDR                       0x32

#define TRIM_NVM_CTRL_ADDR               0x33
#define BGW_SPI3_WDT_ADDR                0x34

#define OFFSET_OFC1_ADDR                 0x36
#define OFC2_ADDR                        0x37
#define OFC3_ADDR                        0x38
#define OFC4_ADDR                        0x39
#define TRIM_GP0_ADDR                    0x3A
#define TRIM_GP1_ADDR                    0x3B

#define SELFTEST_ADDR                    0x3C

#define FIFO_CONFIG_0_ADDR                   0x3D
#define FIFO_CONFIG_1_ADDR                   0x3E

#define FIFO_DATA_ADDR                   0x3F


#endif   //  BMG160_REGISTERS_H
