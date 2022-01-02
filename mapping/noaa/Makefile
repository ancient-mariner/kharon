include ../../util/set_env_base.make
include ../../util/mapping_set_env.make


########################################################################
#

LIB = $(MOD_ROUTING_LIB) $(MAPPING_LIB) $(LOCAL_LIB) -lm

TARGETS = read_gz

all: $(TARGETS)
	

%.o: %.c $(HDRS)
	$(CC) $< -c -o $@ $(CFLAGS)
	@touch $(COMPILE_FLAG)

read_gz: read_gz.c  
	$(CC) -o read_gz read_gz.c $(CFLAGS) $(LIB) -lz


clean:
	rm -f *.o $(TARGETS)

refresh: clean all

