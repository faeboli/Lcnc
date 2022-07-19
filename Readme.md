Lcnc is an fpga firmware intended for transforming colorlight 
boards such as 5A-75B or 5A-75E in linuxcnc peripheral boards.
The communication with the host pc is trough a gigabit ethernet.
The protocol used is etherbone.
The driver is written in migen, using Litex framework.

Lcnc is my own interpretation of the work of colorcnc form 
Roman Pechenko <romanetz4@gmail.com> "romanetz"
https://forum.linuxcnc.org/27-driver-boards/44422-colorcnc

Usage:
- be sure to have Litex installed and working, see https://github.com/enjoy-digital/litex
- clone the repo on your pc
- edit Lcnc.py:
  modify what is between
  "Devices configuration start"
  and
  "Devices configuration end "
  where you will list the peripherals you want to include in the build.
  The default driver contains a basic example with several inputs, outputs, pwm generators
  encoders, step generators. You have to choose how many of each and what pins to use for each.
- execute Lcnc.py:
  the command detail depends on the particular board to be used as target, and the ip address to   configure.
  typing "./Lcnc.py --build --doc" will build the default target that is a colorlight 5A-75E V6.0 with ip=192.168.2.50
  this can be changed adding for example --board=5a-75b --version=8.0 --ip-addr=192.168.1.5
- once the script is terminated there are 3 files useful:
- bit file to be loaded to board under /build/Lcnc/gateware/Lcnc.bit
- etherbone registers definition under /build/software/include/csr.h
- driver configuration configuration_auto.h
- Flash the first file on the board
- Copy last 2 files under the driver folder
- now copy the driver folder on the machine that runs Linuxcnc, in my case I use it on a raspberry
- build the driver with the command ./halcompile Lcnc.c
- connect the board, ping it to make sure it is connected to ethernet
- start the driver

Enjoy

Fabio Eboli