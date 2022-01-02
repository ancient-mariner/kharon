#!/bin/sh
# kFreeBSD do not accept scripts as interpreters, using #!/bin/sh and sourcing.
if [ true != "$INIT_D_SCRIPT_SOURCED" ] ; then
    set "$0" "$@"; INIT_D_SCRIPT_SOURCED=true . /lib/init/init-d-script
fi
### BEGIN INIT INFO
# Provides:          pi_super
# Required-Start:    $remote_fs $syslog
# Required-Stop:     $remote_fs $syslog
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: pinet supervisor process
# Description:       Launches pinet's pi_super
### END INIT INFO

# Author: Keith Godfrey

DESC="Remote shutdown control"
DAEMON=/pinet/local/bin_rpi/pi_super

do_start() {
        /pinet/local/bin_rpi/pi_super > /home/pi/pi_super.txt 2> /home/pi/pi_super.err &
}

do_stop() {
        killall -s SIGKILL pi_super
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
        echo "Usage: pi_super.sh [start|stop|restart]" >&2
        exit 3
        ;;
esac


exit 0

