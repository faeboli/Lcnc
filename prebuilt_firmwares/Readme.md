The folders contain ready to use configurations for some peripheral sets.
The configurations are also already transalted in bit files to be loaded on the boards.
Every configuration is build with two options set, so there are two bitfiles for each configuration.
The difference is in ip address, ip port, MAC address.
This is useful in case you want to use more than one board in the system: in this case ip address port and MAC shall not conflict.
I have build only 2 sets of bitfiles, so you can use at most two boards with the ready bitfiles: use one that ends with _1 as second board.
The driver will accept up to 4 boards, but in that case you will need to rebuild the configuration changing ip port and MAC for the third and fourth board. Don't forget to fill the ip and port addresses parameters in hal file in loadrt command:
loadrt Lcnc ipaddr="192.168.2.50","192.168.2.51" udpport="1234","1235"