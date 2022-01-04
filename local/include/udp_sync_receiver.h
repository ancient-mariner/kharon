/***********************************************************************
* This file is part of kharon <https://github.com/ancient-mariner/kharon>.
* Copyright (C) 2019-2022 Keith Godfrey
*
* kharon is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, version 3.
*
* kharon is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with kharon.  If not, see <http://www.gnu.org/licenses/>.
***********************************************************************/
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
