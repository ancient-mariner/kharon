include ../../../../util/set_env_base.make
include ../../../../util/core_set_env.make


LIB = $(LOCAL_LIB) -lm -ldl -lpthread

all: test_insert_phantom

test_insert_phantom: insert_phantom.c
	$(CC) $(TEST_CFLAGS) insert_phantom.c -o test_insert_phantom -DUNIT_TEST_MODE $(LIB)

refresh: clean all

clean:
	rm -f *.o test_* 

