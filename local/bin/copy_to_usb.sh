#!/bin/bash

if [ $# != 1 ]; then
   echo "Usage: $0 <version-num>"
   exit 1
fi

VERSION=$1

cp /pinet/archive/bob_boot.shadow.$VERSION.tgz /media/keith/KINGSTON/
cp /pinet/archive/bob_master.shadow.$VERSION.tgz.mag* /media/keith/KINGSTON/
cp /pinet/archive/bob_rpi.shadow.$VERSION.tgz.mag* /media/keith/KINGSTON/
cp -n /pinet/ext/packages/lua-5.3.3.tar.gz /media/keith/KINGSTON/


