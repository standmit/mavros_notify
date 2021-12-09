# MAVROS Notify test package
This packagee consists hand-test with FCU SITL environment.

### RGB blinking

##### Description
Allows testing RGB led control. Current test implements code to blink certain sequence by all RGB leds connected to FCU.

##### How to use
Launch file `blinking.launch`. At first time it will take several minutes to build SITL. After that small colored window will appeared simulating real RGB led.
Default sequence is RED-RED-GREEN-GREEN-BLUE-BLUE. You can change sequence in `src/blinking.cpp`.