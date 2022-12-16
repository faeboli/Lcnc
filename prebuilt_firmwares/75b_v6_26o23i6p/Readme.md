Configuration for a colorlight 5a75b V6.1 with
- 26 outputs
- 23 inputs
- 6 pwm generators

- ip address 192.168.2.50 port 1234 MAC 0x10e2d5000000
- if you want to try dual board configuration use Lcnc_75b_v6_26o23i6p_1.bit for the second board, it was built with the following options in order to use it as second board
    - ip address 192.168.2.51 port 1235 MAC 0x10e2d5000001
- pinout can be found in file Lcnc_configurations_pinout.ods
- reset port (to be held to zero): J8 pin 7 (starting to count from pin 1)
- load the bitfile to fpga with openFPGAloader in oss-cad-suite: ~/oss-cad-suite/bin/openFPGALoader --unprotect-flash -c ft232 -f ./Lcnc_75b_v6_26o23i6p.bit
