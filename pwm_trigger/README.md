# pwm_trigger
A small program to monitor an RC channel and an internal trigger and operate a PWM switch.

This package relies on the "pigpio" package to provide access to the GPIO ports without running the node as root.

## Hardware Setup
Ensure there is a GPIO port that the servo can be used on.

## Published Topics
### ~pwm
Publishes the current PWM being written to the GPIO.

## Subscribed Topics
### ~rc_in
Subscribes to a mavros_msgs/RCIn and listens for a PWM signal to use as a threshold for the manual override.

### ~trigger
Subscribes to a std_msgs/Bool to use as the automatic trigger for setting the output PWM to low or high.


## Services
None
