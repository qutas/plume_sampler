# co2_monitor
A small program to interface with a SprintIR CO2 sensor to log data or act as a trigger.

## Hardware Setup
The device should be set to be in Streaming Mode, otherwise this node will note work

## Published Topics
### ~reading/filtered
Publishes the filtered CO2 measurements from the device as a std_msgs/UInt64.

### ~reading/raw
Publishes the raw CO2 measurements from the device as a std_msgs/UInt64.

### ~trigger
Publishes a std_msgs/Bool as True/False to act as a even trigger once the reading is passes a trigger value.

## Subscribed Topics
None

## Services
### ~calibrate_known_value
This service takes a std_msgs/UInt64 as the data input, and sends a "known value zero-point claibration" request to the device.

Returns a True or False depending on the success of the calibration.
