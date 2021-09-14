# FC calibration tool

This is a tool that helps calibrate INAV/ArduPilot FCs by using a known load. You will
need some way to generate load for your FC (a configurable load is best, throttling up
the motor will also probably work).

## Installation

To install and run this tool, you will need to have a recent version of Ruby installed.
You then need to clone this repository and all its submodules:

```
$ git clone --recursive https://github.com/shellixyz/fc_calibration_tool.git
$ cd fc_calibration_tool
$ bundle install
$ ./calibrate <your FC's port>
```

Keep in mind that you need to connect to the FC **without connecting power**, otherwise
your measurements will be influenced and your calibrated values will be useless. You
can easily do this with a piece of paper on the voltage pin of the USB cable (see a
pinout diagram for which one that pin is). Then, follow the instructions.
