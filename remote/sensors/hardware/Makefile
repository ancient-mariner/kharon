include ../../../util/set_env_base.make
include ../../../util/remote_set_env.make

LIB_DIR = ../

#HDRS = lsm9ds0.h lsm9ds0_registers.h \
#		 lsm303.h lsm303_registers.h \
#		 l3g.h l3g_registers.h	\
#		 bmg160.h bmg160_registers.h \
#		 hmc6343.h hmc_6343_registers.h  \
#		 lis3mdl.h lis3mdl_registers.h   \
#		 lis3dh.h lis3dh_registers.h     \
#       a3g4250d.h a3g4250d_registers.h \
#       mag3110.h 
OBJS = lsm9ds0.o lsm303.o l3g.o bmg160.o hmc6343.o lis3mdl.o \
       lis3dh.o a3g4250d.o mag3110.o


all: lib

lib: $(OBJS)
	rm -f $(LIB_DIR)/$(SENS_I2C_LIB)
	ar -cvq $(SENS_I2C_LIB) $(OBJS)
	cp $(SENS_I2C_LIB) $(LIB_DIR)

%.o: %.c 
	$(CC) $< -c -o $@ -I../ $(CFLAGS)

xmag_3110: xmag_3110.c
	gcc -std=gnu11 xmag_3110.c -o xmag_3110

acc_hmc: acc_hmc.c
	gcc -std=gnu11 acc_hmc.c -o acc_hmc

xgyr_bmg: xgyr_bmg.c
	gcc -std=gnu11 xgyr_bmg.c -o xgyr_bmg

refresh: clean all

clean:
	rm -f *.o *.a

