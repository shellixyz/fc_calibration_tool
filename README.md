# FC calibration tool

This is a tool that helps calibrate INAV FCs.

## Installation

To install and run this tool, you need to clone this repository and all its submodules:

```
$ git clone --recursive https://github.com/shellixyz/fc_calibration_tool.git
$ sudo gem install serialport crc
$ cd fc_calibration_tool
$ ./calibrate <your FC's port>
```

Keep in mind that you need to connect to the FC **without connecting power**. You can
easily do this with a piece of paper on the voltage pin of the USB cable (see a pinout
diagram for which one that pin is). Then, follow the instructions.
