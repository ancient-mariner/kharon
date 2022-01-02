include ../../../../util/set_env_base.make
include ../../../../util/core_set_env.make

LIB = $(LOCAL_LIB) -lm -ldl -lpthread -lasound


all: test_sound

test_sound: sound.c
	$(CC) $(TEST_CFLAGS) sound.c -o test_sound $(LIB)


%.o: %.c $(HDRS)
	$(CC) $< -c -o $@ $(TEST_CFLAGS)

refresh: clean all

clean:
	rm -f *.o test_* 

