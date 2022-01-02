include ../../../../util/set_env_base.make
include ../../../../util/core_set_env.make

LIBS = $(MAPPING_LIB) $(LOCAL_LIB) -lm -ldl -lpthread

FLAGS = $(TEST_CFLAGS) $(LIBS)

all: test_path_map \
      test_share  \
      test_world_map

test_path_map: path_map.c
	$(CC) path_map.c -o test_path_map -DUNIT_TEST_MODE $(FLAGS)

test_share: share.c
	$(CC) share.c -o test_share -DUNIT_TEST_MODE $(FLAGS)

test_world_map: world_map.c
	$(CC) world_map.c -o test_world_map -DUNIT_TEST_MODE $(FLAGS)


#%.o: %.c $(HDRS)
#	$(CC) $< -c -o $@ $(FLAGS)

refresh: clean all

clean:
	rm -f *.o test_* *.pnm _log*

