# UAV Configuration
The plume_sampler package is designed with a fairly specific device setup. The basis of this stack is to use the following hardware:
- A SprintIR CO2 sensor (for the co2_monitor package)
- A PWM-based MOSFET (switch to control the sampling equipment to be used with the pwm_trigger package)
- A px4-based autopilot with a 5+ channel RC controller (for the manual override in the pwm_trigger package)

## On-Board Computer
The on-board computer is expected to be a Raspberry Pi 2, running a ros-compatible OS (Ubuntu). It will need at least 2 (USB) serial ports for it to interface with the autopilot, and also at least 1 free GPIO for the PWM output.

The setup for interfacing the serial ports with the appropriate devices should be fairly standard. By default, the link used with the SprintIR sensor uses a 115200 baud rate, while the autopilot should be configured as an offboard link, which most likely uses a baud rate of 921600.

While the co2_monitor package uses only standard python libraries, a special library will be needed to interface with the GPIOs on the RPi2. To do this, we use the the [pigpio](http://abyz.co.uk/rpi/pigpio/download.html) library, as it allows us to run programs as a standard user while interfacing with the GPIOs.

It is recommened that you add a systemd service to automatically start the pigpiod daemon on boot. As an example, create the file "/usr/lib/systemd/system/gpio.service" with the following contents:
```
[Unit]
Description=RPi GPIO interface
After=syslog.target

[Service]
Type=forking
PIDFile=/var/run/pigpio.pid
ExecStart=/usr/local/bin/pigpiod
ExecStop=/usr/bin/killall pigpiod

[Install]
WantedBy=multi-user.target
```
You will then need to update systemd, and enable the service (and start it to begin testing without a reboot:
```
systemctl daemon-reload
systemctl enable gpio.service
```
To run this session without restarting:
```
systemctl start gpio.service
```
If you are using USB-Serial converters, it is highly recommended that you create static names for your serial devices so they are predictable each boot. You can find more information on this from guides [like this one from HintShop](http://hintshop.ludvig.co.nz/show/persistent-names-usb-serial-devices/).

## Preparing Launch Files
There is a base example launch file available in the "launch" folder of this rerpository. It will need to be changed to suit your specific needs, but should show the basic intention on how these nodes were designed to be used.
