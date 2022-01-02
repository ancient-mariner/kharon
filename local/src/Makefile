include ../../util/set_env_base.make
include ../../util/local_set_env.make


LIB = -L$(LOCAL_LIB_DIR) -lm -lpthread -ldl

OBJS = pinet.o sensor_packet.o lin_alg.o mem.o timekeeper.o udp_sync_receiver.o image.o iatan2.o blur.o time_lib.o dev_info.o logger.o softiron.o 

APPS = yuv2pgm calc_softiron softiron

########################################################################
#
all: lib testing apps

apps: $(APPS)


lib: $(OBJS)
	rm -f liblocal.a
	gcc-ar rcsv liblocal.a $(OBJS)
	cp liblocal.a $(LOCAL_LIB_DIR)


%.o: %.c $(HDRS)
	$(CC) $< -c -o $@ $(CFLAGS)


yuv2pgm: yuv2pgm.c
	$(CC) -o yuv2pgm yuv2pgm.c $(CFLAGS) $(LIB) 
	cp yuv2pgm ../bin

calc_softiron: calc_softiron.c lib
	$(CC) -o calc_softiron calc_softiron.c $(CFLAGS) $(LIB) liblocal.a
	cp calc_softiron ../bin

softiron: softiron.c lib
	$(CC) -o softiron softiron.c $(CFLAGS) $(LIB) liblocal.a -DSOFTIRON_APP
	cp softiron ../bin

testing: test_linalg \
         test_blur \
         test_time_lib \
         test_dev_info \
         test_iatan2 \
         test_mem \
         test_image \
         test_timekeeper \
         test_sanity 

test_linalg: lin_alg.c
	$(CC) -o test_linalg lin_alg.c $(CFLAGS) $(LIB) liblocal.a -DLIN_ALG_TEST

test_image: image.c
	$(CC) -o test_image image.c $(CFLAGS) -DIMAGE_TEST $(LIB) liblocal.a

test_mem: mem.c
	$(CC) -o test_mem mem.c $(CFLAGS) -DTEST_MEM $(LIB) liblocal.a

test_blur: blur.c
	$(CC) -o test_blur blur.c $(CFLAGS) -DTEST_BLUR $(LIB) liblocal.a

test_iatan2: iatan2.c
	$(CC) -o test_iatan2 iatan2.c $(CFLAGS) -DIATAN2_TEST $(LIB) liblocal.a

test_dev_info: dev_info.c
	$(CC) -o test_dev_info dev_info.c logger.c timekeeper.c pinet.c lin_alg.c $(CFLAGS) -DDEV_INFO_TEST $(LIB) 

test_sanity: sanity.c 
	$(CC) -o test_sanity sanity.c liblocal.a $(CFLAGS) $(LIB)

test_timekeeper: timekeeper.c
	$(CC) -o test_timekeeper timekeeper.c logger.c pinet.c $(CFLAGS) -DTEST_TIMEKEEPER $(LIB)

#test_downsample: downsample.c
#	$(CC) -o test_downsample downsample.c $(CFLAGS) -DTEST_DOWNSAMPLE $(LIB)

test_time_lib: time_lib.c
	$(CC) -o test_time_lib time_lib.c $(CFLAGS) -DTEST_TIME_LIB $(LIB)

#test_logger: logger.c
#	$(CC) -o test_logger logger.c timekeeper.c pinet.c lin_alg.c $(CFLAGS) -DTEST_LOGGER $(LIB)



clean:
	rm -f *.o *.a test_* *.pnm *.pgm _log.*
	rm -f $(APPS)

refresh: clean all

