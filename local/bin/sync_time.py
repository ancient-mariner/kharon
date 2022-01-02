#!/usr/bin/env python3
import datetime
import subprocess
import sys
import time

if len(sys.argv) < 2:
    print("Usage: %s <node 1> [node 2] ...")
    print("")
    print("Sends clock-set signal to networked nodes")
    sys.exit(0)

def sync_time(targets):
    for host in targets:
        print("Syncing time with " + host)
        d = datetime.datetime.utcnow()
        d_string = "%d-%d-%d" % (d.year, d.month, d.day)
        t_string = "%02d:%02d:%02d" % (d.hour, d.minute, d.second)

        subprocess.Popen(["ssh", "-i", "/home/keith/.ssh/id_rsa", "pi@%s" % host, "sudo date -s %s" % d_string])
        time.sleep(1)
        subprocess.Popen(["ssh", "-i", "/home/keith/.ssh/id_rsa", "pi@%s" % host, "sudo date -s %s" % t_string])
        time.sleep(1)
        #subprocess.Popen(["ssh", "pi@%s" % host, "sudo date -s %s" % t_string])

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: %s <node 1> [node 2] ...")
        print("")
        print("Sends clock-set signal to networked nodes")
        sys.exit(0)

    sync_time(sys.argv[1:])

