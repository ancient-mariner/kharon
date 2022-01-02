include ../../util/set_env_base.make
include ../../util/core_set_env.make

INC = -I$(EXTERNAL_INCLUDE_DIR)

LIB = $(MOD_ROUTING_LIB) \
      $(MOD_TRACKING_LIB) \
      $(MOD_OPT_TRACKING_LIB) \
      $(MOD_CORE_LIB) \
      $(LOCAL_LIB) \
      $(MAPPING_LIB) \
      $(EXTERNAL_LIB_DIR)liblua.a    \
      -lm -ldl -lpthread -lasound

IFACE_OBJS = script_iface.o

OBJS = datap.o postmaster.o udp_sync.o kernel.o 

TARGET = bob

all: tests bob

bob: $(OBJS) $(IFACE_OBJS) runtime.o
	$(CC) $(CFLAGS) runtime.c -o $(TARGET) $(OBJS) $(IFACE_OBJS) $(LIB) $(INC)
	cp $(TARGET) $(ROOT)local/bin/


tests: test_postmaster

test_postmaster: $(OBJS) postmaster.c 
	$(CC) $(CFLAGS) postmaster.c -o test_postmaster kernel.o datap.o udp_sync.o \
		$(LIB) -DTEST_POSTMASTER


%.o: %.c 
	$(CC) $< -c $(CFLAGS) $(INC)

refresh: clean all

clean:
	rm -f *.o *.a bob test_postmaster

