#!/bin/bash

git clone https://github.com/ArduPilot/ardupilot
cd ardupilot
git checkout 1fe146c4092cd810b179ca0c1744c233cdb8c649
./Tools/environment_install/install-prereqs-ubuntu.sh -y
source ~/.profile
./waf configure --board sitl
./waf copter
cd ..
git clone https://github.com/khancyr/ardupilot_gazebo
cd ardupilot_gazebo
git checkout 9bae0ead65f3a20f6a1c06f3887f92172abf79a9
mkdir build
cd build
cmake ..
make -j
sudo make install
cd ..