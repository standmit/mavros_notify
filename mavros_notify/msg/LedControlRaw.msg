# Control vehicle LEDs

# https://mavlink.io/en/messages/ardupilotmega.html#LED_CONTROL_PATTERN
uint8 LED_CONTROL_PATTERN_OFF=0
uint8 LED_CONTROL_PATTERN_FIRMWAREUPDATE=1
uint8 LED_CONTROL_PATTERN_CUSTOM=255

# https://mavlink.io/en/messages/ardupilotmega.html#LED_CONTROL
uint8 target_system
uint8 target_component
uint8 instance # 255 for all LEDs
uint8 pattern
uint8[] custom_bytes