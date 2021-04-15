#!/bin/bash

echo "Setting CAM on Nano"
cd ~/workspace/src
python comptoraw.py

#!/bin/bash
echo "Setting CAM on ERP and start lane detection and obj detection ..."
cd ~/catkin_ws/src/vip_gui
python2 comptoraw.py
python2 lanepub.py
python2 findobj.py

#!/bin/bash
sh cam.sh 01 &
sh gps.sh 02 &
sh imu.sh 03 &
sh lidar.sh 04 &



