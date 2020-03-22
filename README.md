# ROS driver for Roboteq brushless motor controllers in Modbus-ASCII mode

This is a ROS node for Roboteq brushless motor controllers used in Modbus-ASCII mode. I have only tested it on the SBL2360 but it should work on FBL2360 and other devices that support Modbus-ASCII. It requires Firmware 2.0 or later.

## Parameters:

* **port** -- serial port device. Default is /dev/roboteq0. Note that Modbus-ASCII *must* be plugged into the RS232 port on the Roboteq. The Roboteq does not respond to Modbus commands over USB.
* **baud** -- baud rate. Default is 115200.
* **num\_channels** -- number of channels in the Roboteq. Default is 2.

## Subscribers:
* **/command** (std\_msgs/Int32MultiArray) -- motor command. Array values must have the same number of values as the num\_channels parameter above. Each value must be between -1000 and 1000 inclusive.

## Publishers:
* **/brushless\_count** (std\_msgs/Int32MultiArray) -- brushless count, per channel
* **/brushless\_speed** (std\_msgs/Float32MultiArray) -- brushless speed, per channel
* **/closed\_loop\_error** (std\_msgs/Int32MultiArray) -- closed loop error, per channel
* **/current** (std\_msgs/Float32MultiArray) -- current in amps, per channel
* **/flags\_fault** (std\_msgs/Int16) -- fault flags
* **/flags\_runtime** (std\_msgs/Int16MultiArray) -- runtime flags, per channel
* **/flags\_status** (std\_msgs/Int16) -- status flags
* **/rotor\_angle** (std\_msgs/Float32MultiArray) -- rotor angle, per channel
* **/temperature** (sensor\_msgs/Temperature) -- temperature of system
* **/voltage** (std\_msgs/Float32) -- battery voltage

## Service calls:
None

