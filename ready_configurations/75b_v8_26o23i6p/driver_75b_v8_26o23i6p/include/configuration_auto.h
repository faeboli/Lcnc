//---Board configuration---
#define IP_ADDR  "192.168.2.50"
#define UDP_PORT "1234"
#define F_FPGA (50000000.0)
//---Devices configuration---
#define N_INPUTS 23
#define N_OUTPUTS 26
#define N_ENCODERS 0
#define N_PWM 6
#define N_STEPGENS 0
#define ACC_MULT_EXP 3
//---Registers definition, all are 32bit wide---
// --transmitted registers: board to fpga
// - start of STEPGEN related regs
//  - velocity registers, one per stepgen
//  - other registers, only one each needed, max 16 STEPGENS allowed
#define TX_POS_STEP_RES_EN (TX_POS_MAX_ACC-1 + 4)
#define TX_POS_STEPDIRINV (TX_POS_STEP_RES_EN + 4)
#define TX_POS_STEPTIMES (TX_POS_STEPDIRINV + 4)
// - end of STEPGEN related regs
// OUTPUTS, only one register needed, max 32 outputs allowed
#define TX_POS_GPIOS_OUT (TX_POS_STEPTIMES + 4)
// PWM related
//   - pwm control registers, one per PWM
#define TX_POS_PWM0 (TX_POS_GPIOS_OUT + 4)
#define TX_POS_PWM1 (TX_POS_PWM0 + 4)
#define TX_POS_PWM2 (TX_POS_PWM1 + 4)
#define TX_POS_PWM3 (TX_POS_PWM2 + 4)
#define TX_POS_PWM4 (TX_POS_PWM3 + 4)
#define TX_POS_PWM5 (TX_POS_PWM4 + 4)
// end of PWM related regs
// Encoder control, one register needed, maximum 16 encoders allowed
#define TX_POS_ENC_RES_EN (TX_POS_PWM5 + 4)
// Board control and reset register, containing watchdog, only one needed
#define TX_POS_RES_STAT_REG (TX_POS_ENC_RES_EN + 4)
// number of regiters to be sent to fpga, sum of all the ones so far
#define TX_PAYLOAD_SIZE (TX_POS_RES_STAT_REG + 4)
// received register: fpga to board
// Board control and reset register, containing watchdog, only one needed
#define RX_POS_RES_ST_REG 0
// end of ENCODER related regs
//  - step counters feedback, one per stepgen
// end of STEPGEN related regs
// WALLCLOCK feedback register, only one needed
#define RX_POS_WALLCLOCK (RX_POS_STEPVEL-1 + 4)
// INPUTS register, only one needed, max 32 inputs allowed
#define RX_POS_GPIOS_IN (RX_POS_WALLCLOCK + 4)
// ENCODER related:
//  - encoder counters feedback, one per ecoder
// end of ENCODER related regs
// number of registers to be sent read from fpga, sum of all the ones so far
#define RX_PAYLOAD_SIZE (RX_POS_ENCCOUNT-1 + 4)
// ---end of configuration
