#!/bin/bash

git clone https://github.com/ArduPilot/ardupilot
cd ardupilot
git checkout 1fe146c4092cd810b179ca0c1744c233cdb8c649
./Tools/environment_install/install-prereqs-ubuntu.sh -y
source ~/.profile
./waf configure --board sitl
./waf copter
cd ..
