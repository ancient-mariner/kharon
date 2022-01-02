include ../../../../util/set_env_base.make
include ../../../../util/core_set_env.make

all: clean all_tests

all_tests: test_accumulator

test_down:
	$(CC) $(TEST_CFLAGS) -I../ ../down.c -o test_down $(LIB) -DTEST_DOWNSAMPLE

test_accumulator: clean
	$(CC) $(TEST_CFLAGS) -I../ accumulator.c -o test_accumulator $(LIB) 

%.o: %.c
	$(CC) $< -c -o $@ $(CFLAGS)

refresh: clean all_tests

clean:
	rm -f *.o test_* *-y.pnm

