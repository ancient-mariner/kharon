#!/bin/sh
# kFreeBSD do not accept scripts as interpreters, using #!/bin/sh and sourcing.
if [ true != "$INIT_D_SCRIPT_SOURCED" ] ; then
    set "$0" "$@"; INIT_D_SCRIPT_SOURCED=true . /lib/init/init-d-script
fi
### BEGIN INIT INFO
# Provides:          kill_monitor
# Required-Start:    $remote_fs $syslog
# Required-Stop:     $remote_fs $syslog
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: Remote shutdown control via network packet
# Description:       Launches pinet's kill_monitor
### END INIT INFO

# Author: Keith Godfrey

DESC="Remote shutdown control"
DAEMON=/pinet/local/bin_rpi/kill_monitor

do_start() {
        /pinet/local/bin_rpi/kill_monitor > /home/pi/kill.txt 2> /home/pi/kill.err &
}

do_stop() {
        killall -s SIGKILL kill_monitor
}

case "$1" in
  start|"")
        do_start
        ;;
  restart)
        do_stop
        do_start
        ;;
  stop)
        do_stop
        ;;
  *)
        echo "Usage: kill_monitor.sh [start|stop|restart]" >&2
        exit 3
        ;;
esac


exit 0

