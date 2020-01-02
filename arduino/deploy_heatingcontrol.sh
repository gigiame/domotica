#!/bin/bash -ex

binDir=`dirname $0`
rm -f heatingcontrol/src/._*
/home/openhabian/serial2mqtt_nano/bin/stop.sh
cd heatingcontrol
ino build -m nano328
ino upload -m nano328 -p /dev/ttyUSB0
cd ..
/home/openhabian/serial2mqtt_nano/bin/start.sh
echo "done!"
