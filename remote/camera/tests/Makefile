include ../../../util/set_env_base.make
include ../../../util/remote_set_env.make

LIB = -lm -ldl 

all: clean all_tests

all_tests: test_accumulator

test_accumulator: clean
	$(CC) $(TEST_CFLAGS) accumulator.c ../accumulator.c -o test_accumulator $(LIB) 
refresh: clean all_tests

clean:
	rm -f *.o test_*

