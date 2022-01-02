include ../../util/set_env_base.make
include ../../util/core_set_env.make

TARGETS = rgb2yuv check_associator disp_zone

LIB: -L(LOCAL_LIB)


all: $(TARGETS)

rgb2yuv:
	$(CC) $(CFLAGS) $(INC) rgb2yuv.c -o rgb2yuv $(LIB)
	cp rgb2yuv $(ROOT)/local/bin/rgb2yuv

disp_zone: disp_zone.c
	$(CC) $(CFLAGS) disp_zone.c -o disp_zone $(LIB)

check_associator: check_associator.c
	$(CC) $(CFLAGS) check_associator.c -o check_associator $(LIB)

#yuv2vy:
#	$(CC) $(CFLAGS) yuv2vy.c -o yuv2vy $(LIB)
#	cp yuv2vy $(ROOT)/local/bin/yuv2vy

refresh: clean all

clean:
	rm -f *.o $(TARGETS)

