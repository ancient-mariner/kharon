include ../../../../util/set_env_base.make
include ../../../../util/core_set_env.make


CORE_LIBS = $(ROOT)local/lib/liblocal.a
EXT_LIBS = $(ROOT)external/lib/liblua.a 


LIB = $(CORE_LIBS) $(EXT_LIBS) -lm 

HDRS = $(ROOT)brain/include/modules/attitude.h 

TESTS = test_imu_streams test_drift_correction

all: $(TESTS)

test_drift_correction: drift_correction.c
	$(CC) drift_correction.c $(CFLAGS) -o test_drift_correction $(LIB)

test_imu_streams: ../imu_streams.c
	$(CC) ../imu_streams.c -DIMU_STREAMS_TEST $(CFLAGS) -o test_imu_streams $(LIB)

%.o: %.c $(HDRS)
	$(CC) $< -c -o $@ $(CFLAGS)

refresh: clean all

clean:
	rm -f *.o $(TESTS) _log*

