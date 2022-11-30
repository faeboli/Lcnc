Configuration for a colorlight 5a75e V7.1 with
- 30 outputs
- 29 inputs
- 9 stepgens
- 9 encoders
- 8 pwm generators

Defaults: ip address 192.168.2.50 port 1234 MAC 0x10e2d5000000
Pinout can be found in file Lcnc_configurations_pinout.ods
- reset port (to be held to zero): J4 pin 1 (starting to count from pin 1)
- if you want to rebuild the bitfile in litex: ./Lcnc_75e_v7_30o29i9s9e8p.py --build --doc
- if you want to change board revision/ip/port/MAC: ./Lcnc_75e_v7_30o29i9s9e8p.py --revision=8.0 --eth-ip=192.168.2.51 --eth-port=1235 --mac-address=0x10e2d5000001 --build --doc
- load the bitfile to fpga with openFPGAloader in oss-cad-suite: ~/oss-cad-suite/bin/openFPGALoader --unprotect-flash -c ft232 -f ./Lcnc_75e_v7_30o29i9s9e8p.bit
