include ../../../../util/set_env_base.make
include $(ROOT)/util/core_set_env.make


LIB = $(ROOT)/local/lib/liblocal.a -lm -ldl -lpthread


all: test_align_frames 

test_align_frames: align_frames.c ../align_frames.c
	$(CC) $(TEST_CFLAGS) align_frames.c -o test_align_frames $(LIB)


%.o: %.c $(HDRS)
	$(CC) $< -c -o $@ $(TEST_CFLAGS)

refresh: clean all

clean:
	rm -f *.o test_* 

