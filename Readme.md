Lcnc is an fpga firmware to be used in LinuxCNC.

Lcnc is my own interpretation of colorcnc form 
Roman Pechenko <romanetz4@gmail.com> "romanetz"
https://forum.linuxcnc.org/27-driver-boards/44422-colorcnc

It implements stepgenerators, general purpose in/out, encoder inputs, pwm
in the fpga, and a gigabit ethernet link used to talk to the host pc running LinuxCNC.

There are several FPGA configurations available, ready to be uploaded
on FPGA boards, also eventually new user defined configurations can be generated.

The work now is focused on Colorligh 5a-75b and 5a-65e boards.

The HOST side LinuxCNC driver is provided, it will interface
with FPGA firmware, and provide pins and parameters that will connect
fpga peripherals to LinuxCNC. The driver is unique and will be able
to talk to every possible configuration.

The protocol used is etherbone.
The firmware is written in migen, using Litex framework.

# Getting started:
- you will need a colorlight 5a 75b or 75e board
- jtag adapter for loading and flashing bitfiles
- boards hw info and pin maps for jtag are available here https://github.com/q3k/chubby75
- Choose the firmware you want to try, the names reflect the peripherals available, for example 75b_v6_14o11i6s6e6p will go to a colorlight 5a75b V6.0 and will contain 14 outputs 11 inputs 6 stepgens 6 encoders interfaces 6 pwm generators.
- Upload the firmware on the board, I'm using openFPGAloader but any such tool is good.
- Connect the board to the host pc and try to ping the board, the default ip address is 192.168.2.50 a Gigabit ethernet port is needed.
- If the board responds to pings then you can build the driver in LinuxCNC wih the command "sudo halcompile --install Lcnc.c" and start to work with the board.

# Communication and first enabling of the the board:
- ensure that the RESET pin on the board is grounded, you can find it's position on the 
  pinout document "Lcnc_configurations_pinout.ods", for example for 75b_v6_14o11i6s6e6p the RESET pin is J8 pin7

I have attached a very basic HAL configuration for initial testing, the basic checks can be executed trough halshow interface. 

**Basic functionality check:**

- in the terminal start the HAL configuration: "halrun -I -V  HAL.hal"
- open a second terminal and start "halshow"
- Lcnc.00.update.time: this will show the update time for the component, in my system is around 200000ns
- Lcnc.00.watchdog-write: this is the board watchdog time, in seconds, I have it set to 0.01s (10ms) as default
- Lcnc.00.watchdog-read: this is the watchdog remaining time when petted, in seconds, check the value to be sure that is far from zero, and near to Lcnc.
- The above values should be updated continuosly if the board and driver are working correctly.
- set Lcnc.00.watchdog-write to 1
- Lcnc.00.enable to 1
- toggle Lcnc.00.enable-request from 0 to 1, then back to 0
- Lcnc.00.enabled should become 1, in this case the board is enabled and ready
- Lcnc.00.enable is the global enable flag for board, if set to true, the board can become ready to be enabled, the effective enabling will be requested by Lcnc.00.enable-request.
- Lcnc.00.enabled is the feedback confirmation that the board is enabled, as soon as Lcnc.00.enable will be set to false, the board will be disabled. The board can be disabed also if the onboard watchdog bites (to test this try to set Lcnc.00.watchdog-write to lower values until the board spontaneously disables). The board will disable also if it's hardware reset pin will be set to high (see "_ext_reset_in" in Lcnc.py for it's mapping).
- Set Lcnc.00.enable-request this interface is checked only on it's rising edge from False to True, then it is ignored, if there are no watchdog problems or hw reset request, the board will enable and set back Lcnc.00.enabled to True.


# Create your own configuration:
- Litex installed and working, see https://github.com/enjoy-digital/litex
- clone Lcnc repo on your pc
- edit Lcnc.py:
  modify what is between
  "Devices configuration start"
  and
  "Devices configuration end"
  This part of the firmware script file contains the list of the peripherals you want to include in the build, 
  and the board pins assigned to each. 
  The default driver contains a basic example with several inputs, outputs, pwm generators
  encoders, step generators.
- execute Lcnc.py:
  the command used to execute the firmware generation will accept arguments that will define
  the particular board to be used as target, and the ip address to assign to the board.
  the script can be run with default parameters, simply typing "./Lcnc.py --build --doc"
  this will generate a firmware for a colorlight 5A-75E V6.0 with ip=192.168.2.50
- the target board can be changed adding to the command "board" and "revision" parameters, 
  for example "--board=5a-75b --revision=8.0"
- ip address can be configured with "eth-ip" parameter, for example "--eth-ip=192.168.1.100"
- the script, if succesful, will generate many files, bitfile is located in /build/colorlight_5a_75e/gateware/Lcnc.bit
- upload the bitfile on the board
- connect the board, ping it to make sure that the the board is alive and connected


# Working with peripherals:
-- doc in construction, the peripherals are what they seem, you can play with them, only to be noted that stepgen is velocity mode only.

Digital inputs to the board, the pins are defined in "_gpios_in" list un Lcnc.py script, maximum number of 32 inputs can be defined:
- gpio in
  - linuxcnc -> driver
    - parameters: none
    - inputs: none
    - outputs:
      - Lcnc.xx.input.yy.in true if input driven high (needed 100ua in fpga pin)
      - Lcnc.xx.input.yy.in-n false if input driven high
  - driver -> etherbone registers: none
  - etherbone registers -> driver:
    - MMIO_INST_GPIOS_IN 32bit

Digital outputs from the board, the pins are defined in "_gpios_out" list un Lcnc.py script, maximum number of 32 outputs can be defined:
- gpio out
  - linuxcnc -> driver
    - parameters:
      - Lcnc.xx.output.yy.inv if true output is inverted
    - inputs:
      - Lcnc.xx.output.yy.out if true output is driven high or low depending on inversion parameter
    - outputs: none
  - driver -> registers:
    - MMIO_INST_GPIOS_OUT 32bit
  - registers -> driver: none

Encoder inputs, the pins are defined in "encoders" list un Lcnc.py script, maximum 16 encoders can be defined:
- encoder inputs
  - linuxcnc -> driver
    - parameters:
      - Lcnc.xx.encoder.yy.scale scale to be applied to position read
      - Lcnc.xx.encoder.yy.inv count direction inversion
    - inputs:
      - Lcnc.xx.encoder.yy.reset reset position to zero
      - Lcnc.xx.encoder.yy.enable enable counting
    - outputs:
      - Lcnc.xx.encoder.yy.pos-fb position read, scaled
      - Lcnc.xx.encoder.yy.vel-fb velocity feedback, is difference in positions between two valld to driver, so can be imprecise at low speeds
  - driver -> registers:
    - MMIO_INST_ENC_RES_EN (16 bit enable e 16bit reset)
  - registers -> driver:
    - MMIO_INST_ENC_COUNT (32bit)

PWM outputs,the pins are defined in "_pwm_out" list un Lcnc.py script:
- pwm
  - linuxcnc -> driver
    - parameters:
      - Lcnc.xx.pwm.yy.scale full scale value
      - Lcnc.xx.pwm.yy.offs offset
      - Lcnc.xx.pwm.yy.inv inversion flag
    - inputs:
      - Lcnc.xx.pwm.yy.freq frequency in Hz, acceptable value can be 765Hz to 2.5MHz, the higher frequency, the lower is the duty resolution
      - Lcnc.xx.pwm.yy.value value, from 0.0 to full scale value
      - Lcnc.xx.pwm.yy.enable ebable flag
    - outputs: none
  - driver -> registers:
    - MMIO_INST_PWM_0 (16bit period MSB,16bit width LSB)
  - registers -> driver: none

Step generator, the pins are defined in "stepgens" list un Lcnc.py script, maximum 16 instances possible:
- step generator (sg)
  - linuxcnc -> driver
    - parameters:
      - Lcnc.xx.stepgen.yy.step_inv step inversion
      - Lcnc.xx.stepgen-step_width minimum step pulse width in us (parameter is applied to all stepgens)
      - Lcnc.xx.stepgen-step_space minimum interval between two steps pulses in us (parameter is applied to all stepgens)
      - Lcnc.xx.stepgen.yy.dir_inv dir inversion
      - Lcnc.xx.stepgen-dir_width minimum dir pulse width in us (parameter is applied to all stepgens)
      - Lcnc.xx.stepgen-setup_time minimum interval between dir change and next step in us (parameter is applied to all stepgens)
      - Lcnc.xx.stepgen.yy.scale scale in step/mm
    - inputs:
      - Lcnc.xx.stepgen.yy.reset reset internal state to default
      - Lcnc.xx.stepgen.yy.enable enables step generator
      - Lcnc.xx.stepgen.yy.vel-cmd velocity command in mm/s
      - Lcnc.xx.stepgen.yy.acc-lim acceleration limit in mm/s^2
    - outputs:
      - Lcnc.xx.stepgen.yy.vel-fb feedback velocity in mm/s
      - Lcnc.xx.stepgen.yy.pos-fb feedback position in mm
  - driver -> registers:
    - MMIO_INST_STEP_RES_EN (16 bit enable e 16bit reset)
    - MMIO_INST_STEPDIRINV (16 bit step inversion e 16bit dir inversion)
    - MMIO_INST_STEPTIMES (9bits step width 9bits dir width 14bits setup time)
    - MMIO_INST_STEPSIGN (16bit) velocity command sign
    - MMIO_INST_VELOCITY_n (32bit)
  - registers -> driver:
    - MMIO_INST_POSITION_n (32bit)

Timer downcounting, when reaches zero the board is reset and peripherals are reset to default state, the driver reloads the counter value at each trasmission
- watchdog:
  - linuxcnc -> driver
    - outputs:
      - Lcnc.00.enabled if true the driver is working and enabled, goes to false in case of watchdog biting or externa hardware reset
    - inputs:
      - Lcnc.00.enable-request when goes from false to true, the driver is started
      - Lcnc.00.enable if true the enable request will be evaluated, if is false the driver is disabled
  - registers <-> driver:
    - MMIO_INST_RES_ST_REG (bits 10 to 32)

This is a clock running on fpga, and read from driver, used for time related calculations
- wallclock
  - linuxcnc -> driver
    - outputs:
      - Lcnc.xx.wallclock.yy.value u32 free running counter
      - Lcnc.xx.wallclock.yy.interval float last interval in seconds
  - registers -> driver:
    - MMIO_INST_WALLCLOCK 32bit


Enjoy

Fabio Eboli