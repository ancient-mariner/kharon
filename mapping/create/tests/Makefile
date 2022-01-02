include ../../../util/set_env_base.make
include ../../../util/mapping_set_env.make

LIB = $(MOD_ROUTING_LIB) $(MAPPING_LIB) $(LOCAL_LIB) -lm

OBJS = common.o declination.o
TARGETS = test_declination    \
         test_common

UTILS = ztest zcheck_noaa

all: $(TARGETS) $(UTILS)

test_declination: declination.c
	$(CC) -o test_declination declination.c $(CFLAGS) $(LIB) -DUNIT_TEST

test_common: common.c
	$(CC) -o test_common common.c $(CFLAGS) $(LIB) -DUNIT_TEST

zcheck_noaa: zcheck_noaa.c
	$(CC) -o zcheck_noaa zcheck_noaa.c ../common.c $(CFLAGS) $(LIB) -lz

ztest: ztest.c
	$(CC) -o ztest ztest.c ../common.c $(CFLAGS) $(LIB)


clean:
	rm -f *.o test_* $(TARGETS) $(UTILS)

refresh: clean all

