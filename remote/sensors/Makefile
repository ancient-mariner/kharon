include ../../util/set_env_base.make
include ../../util/remote_set_env.make

LIBS = $(LOCAL_LIB) $(SENS_I2C_LIB) -lpthread -lm -ldl -rdynamic 

COMMON_OBJS = sens_lib.o sens_db.o sens_net.o


all: s2 emulator
	./buildnum.py

s2: s2.o $(COMMON_OBJS) subdirs
	$(CC) s2.o -o s2 $(CFLAGS) $(COMMON_OBJS) $(LIBS) 
	cp s2 /pinet/local/bin_rpi/s2

emulator: emulator.o $(COMMON_OBJS) 
	$(CC) emulator.o -o emulator $(CFLAGS) $(COMMON_OBJS) $(LIBS) 
	cp emulator $(LOCAL_BIN_DIR)/rpi_emulator


subdirs:
	cd hardware; make



%.o: %.c $(HDRS)
	$(CC) $< -c -o $@ $(CFLAGS)

refresh: clean all

clean:
	rm -f *.o *.a s2 emulator
	cd hardware; make clean

