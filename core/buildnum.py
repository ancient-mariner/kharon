#!/usr/bin/python
import socket
import os
import datetime

host = socket.gethostname()
today = datetime.date.today()

vers_num_file = "../private/build_num.txt"
compile_num_file = "kernel/compile_num.txt"
build_file = "./include/kernel/build_version_kernel.h"

try:
    with open(vers_num_file, "r") as f:
        # -1 as build_num stores number of next build. we want this one
        build_num = int(f.readline()) - 1
except:
    print("Unable to find version file. Assuming '1'")
    build_num = 1

try:
    with open(compile_num_file, "r") as f:
        compile_num = int(f.readline())
except:
    print("Unable to find compile-num file. Assuming '1'")
    compile_num = 1

try:
    version_str = "%s_%d.%d-bob_%s" % (host, build_num, compile_num, today)
    print("Version: " + version_str)
    with open(build_file, "w") as f:
        f.write('#define KERNEL_BUILD_VERSION "%s"\n' % version_str);
except:
    print("Unable to write to build version file")
    raise

with open(compile_num_file, "w") as f:
    f.write("%d\n" % (compile_num+1))

