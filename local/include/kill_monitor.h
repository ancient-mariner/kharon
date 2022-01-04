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
#if !defined(KILL_MONITOR_H)
#define KILL_MONITOR_H

// constants used by kill monitor service on devices
// if a device has the kill_monitor process running, and it receives
//    a kill message, the device is halted or rebooted
// this processes are normally located on RPi devices. the header is
//    in a central location so other processes can utilize this
//    communication pathway

#define KILL_PORT    9110

#define KILL_VERSION_BYTES    4
#define KILL_PAYLOAD_BYTES    256
#define KILL_PACKET_BYTES     (KILL_VERSION_BYTES + KILL_PAYLOAD_BYTES)

#define KILL_PAYLOAD_OFFSET   KILL_VERSION_BYTES
#define KILL_VERSION          "001"

#define KILL_RESPONSE_BYTES   KILL_PAYLOAD_BYTES

#define KILL_COMMAND_ALIVE            "alive?"
#define KILL_COMMAND_ALIVE_RESPONSE   "not dead"

#define KILL_COMMAND_SHUTDOWN         "halt"
#define KILL_COMMAND_REBOOT           "reboot"

#endif   // KILL_MONITOR_H
