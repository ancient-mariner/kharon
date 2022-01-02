#if !defined(UDP_SYNC_RECEIVER_H)
#define UDP_SYNC_RECEIVER_H

struct udp_sync_packet;

// launches sync receiver thread
// argument is handler for received packets
// if handler is NULL, the packet is still used for clock synchronization
int create_sync_receiver(void);

// release sync resources
void shutdown_sync_receiver(void);

void register_camera(void);

#endif // UDP_SYNC_RECEIVER_H
