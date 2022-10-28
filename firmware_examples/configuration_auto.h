//---Board configuration---
#define IP_ADDR  "192.168.2.50"
#define UDP_PORT "1234"
#define F_FPGA (50000000.0)
//---Devices configuration---
#define N_INPUTS 11
#define N_OUTPUTS 15
#define N_ENCODERS 3
#define N_PWM 3
#define N_STEPGENS 3
//---Registers definition, all are 32bit wide---
// --transmitted registers: board to fpga
// - start of STEPGEN related regs
//  - velocity registers, one per stepgen
#define TX_POS_VELOCITY0 0
#define TX_POS_VELOCITY1 (TX_POS_VELOCITY0 + 4)
#define TX_POS_VELOCITY2 (TX_POS_VELOCITY1 + 4)
#define TX_POS_MAX_ACC0 (TX_POS_VELOCITY2 + 4)
#define TX_POS_MAX_ACC1 (TX_POS_MAX_ACC0 + 4)
#define TX_POS_MAX_ACC2 (TX_POS_MAX_ACC1 + 4)
//  - other registers, only one each needed, max 16 STEPGENS allowed
#define TX_POS_STEP_RES_EN (TX_POS_MAX_ACC2 + 4)
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
// end of PWM related regs
// Encoder control, one register needed, maximum 16 encoders allowed
#define TX_POS_ENC_RES_EN (TX_POS_PWM2 + 4)
// Board control and reset register, containing watchdog, only one needed
#define TX_POS_RES_STAT_REG (TX_POS_ENC_RES_EN + 4)
// number of regiters to be sent to fpga, sum of all the ones so far
#define TX_PAYLOAD_SIZE (TX_POS_RES_STAT_REG + 4)
// received register: fpga to board
// Board control and reset register, containing watchdog, only one needed
#define RX_POS_RES_ST_REG 0
// end of ENCODER related regs
//  - step counters feedback, one per stepgen
#define RX_POS_STEPCOUNT0 (RX_POS_RES_ST_REG + 4)
#define RX_POS_STEPCOUNT1 (RX_POS_STEPCOUNT0 + 4)
#define RX_POS_STEPCOUNT2 (RX_POS_STEPCOUNT1 + 4)
#define RX_POS_STEPVEL0 (RX_POS_STEPCOUNT2 + 4)
#define RX_POS_STEPVEL1 (RX_POS_STEPVEL0 + 4)
#define RX_POS_STEPVEL2 (RX_POS_STEPVEL1 + 4)
// end of STEPGEN related regs
// WALLCLOCK feedback register, only one needed
#define RX_POS_WALLCLOCK (RX_POS_STEPVEL2 + 4)
// INPUTS register, only one needed, max 32 inputs allowed
#define RX_POS_GPIOS_IN (RX_POS_WALLCLOCK + 4)
// ENCODER related:
//  - encoder counters feedback, one per ecoder
#define RX_POS_ENCCOUNT0 (RX_POS_GPIOS_IN + 4)
#define RX_POS_ENCCOUNT1 (RX_POS_ENCCOUNT0 + 4)
#define RX_POS_ENCCOUNT2 (RX_POS_ENCCOUNT1 + 4)
// end of ENCODER related regs
// number of registers to be sent read from fpga, sum of all the ones so far
#define RX_PAYLOAD_SIZE (RX_POS_ENCCOUNT2 + 4)
// ---end of configuration
