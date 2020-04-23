#!/bin/bash
sim_vehicle.py -v ArduCopter -f gazebo-iris -I0 --no-extra-ports --rgbled -m --mav10 --out=udp:127.0.0.1:14560
