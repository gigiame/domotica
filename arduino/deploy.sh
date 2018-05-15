#!/bin/bash -e

binDir=`dirname $0`
rm -f homeautomation/src/._*
/home/openhabian/serial2mqtt/bin/stop.sh
cd homeautomation
ino build
ino upload
cd ..
/home/openhabian/serial2mqtt/bin/start.sh
echo "done!"
