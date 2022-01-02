#!/bin/bash

if [ $# != 2 ]; then
   echo "Usage: $0 <archive-num> <host>"
   exit 1
fi

num=$1
host=$2

archive_root=/data/archive/
boot=bob_boot.shadow.$1.tgz 
master=bob_master.shadow.$1.tgz 
scp $archive_root$boot pi@$2:$boot
scp $archive_root$master pi@$2:$master

