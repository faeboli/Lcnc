#define SEND_TIMEOUT_US 1
#define RECV_TIMEOUT_US 100
#define DRIVER_NAME "Lcnc"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <stdint.h>
#include <stdlib.h>

//---Board configuration---
#define IP_ADDR  "192.168.2.50"
#define UDP_PORT "1234"
#define F_FPGA (40000000.0)

//---Registers fields size and offsets---
#define CSR_MMIO_INST_MAX_ACC_0_ACC_SIZE 30
#define CSR_MMIO_INST_MAX_ACC_0_ACC_OFFSET 0
#define CSR_MMIO_INST_MAX_ACC_0_ACC_MULT_SIZE 2
#define CSR_MMIO_INST_MAX_ACC_0_ACC_MULT_OFFSET 30
#define CSR_MMIO_INST_STEP_RES_EN_SGENABLE_OFFSET 16
#define CSR_MMIO_INST_STEP_RES_EN_SGENABLE_SIZE 16
#define CSR_MMIO_INST_STEP_RES_EN_SGRESET_OFFSET 0
#define CSR_MMIO_INST_STEP_RES_EN_SGRESET_SIZE 16
#define CSR_MMIO_INST_STEPDIRINV_DIR_INV_OFFSET 0
#define CSR_MMIO_INST_STEPDIRINV_DIR_INV_SIZE 16
#define CSR_MMIO_INST_STEPDIRINV_STEP_INV_OFFSET 16
#define CSR_MMIO_INST_STEPDIRINV_STEP_INV_SIZE 16
#define CSR_MMIO_INST_STEPTIMES_DIR_SETUP_OFFSET 0
#define CSR_MMIO_INST_STEPTIMES_DIR_SETUP_SIZE 14
#define CSR_MMIO_INST_STEPTIMES_DIR_WIDTH_OFFSET 14
#define CSR_MMIO_INST_STEPTIMES_DIR_WIDTH_SIZE 9
#define CSR_MMIO_INST_STEPTIMES_STEP_WIDTH_OFFSET 23
#define CSR_MMIO_INST_STEPTIMES_STEP_WIDTH_SIZE 9
#define CSR_MMIO_INST_ENC_RES_EN_RESET_OFFSET 0
#define CSR_MMIO_INST_ENC_RES_EN_RESET_SIZE 16
#define CSR_MMIO_INST_ENC_RES_EN_ENABLE_OFFSET 16
#define CSR_MMIO_INST_ENC_RES_EN_ENABLE_SIZE 16
#define CSR_MMIO_INST_RES_ST_REG_WATCHDOG_OFFSET 10
#define CSR_MMIO_INST_RES_ST_REG_WATCHDOG_SIZE 22
#define ACC_MULT_EXP 3

/*
 * registers address is function of number of peripherals in the configuration:
 * SIZE             register        R/W
 * init registers
 * 0x04             INIT_WRITE       W 
 * 0x04             CONFIGURATION    R 
 * stepgen related registers
 * 0x04*n_sg        VELOCITYn        W 
 * 0x04*n_sg        MAX_ACCn         W 
 * 0x04             STEP_RES_EN      W 
 * 0x04             STEPDIRINV       W 
 * 0x04             STEPTIMES        W 
 * gpio out register
 * 0x04             GPIOS_OUT        W 
 * pwm registers
 * 0x04*n_pwm       PWM_n            W 
 * encoder related regs
 * 0x04             ENC_RES_EN       W 
 * reset status register
 * 0x04             RES_ST_REG       R+W
 * stepgen related registers
 * 0x04*n_sg        SG_COUNT_n       R  
 * 0x04*n_sg        SG_VEL_n         R  
 * wallclock
 * 0x04             WALLCLOCK        R  
 * gpio in
 * 0x04             GPIOS_IN         R  
 * encoder related regs
 * 0x04*n_en-1      ENC_COUNT_n      R  
 * */

//------------------------------------------------------------------
#define F_FPGA_TIME_US (1.0E6/F_FPGA) // fpga clock time in us
#define WDT_SCALE (64/F_FPGA) // 
#define VEL_SIZE_BITS 32 // width of command word in bits
#define EB_HEADER_SIZE 16  // etherbone header size in bytes
#define INIT_RX_PAYLOAD_SIZE 4
#define INIT_TX_PAYLOAD_SIZE 4
#define RX_PAYLOAD_MAX_SIZE 256
#define TX_PAYLOAD_MAX_SIZE 256
#define MAX_N_INPUTS 32
#define MAX_N_OUTPUTS 32
#define MAX_N_STEPGENS 16
#define MAX_N_ENCODERS 16
#define MAX_N_PWM 16

/*

The EtherBone record has a struct that looks like this:

struct etherbone_record {
	// 1...
	uint8_t bca : 1;
	uint8_t rca : 1;
	uint8_t rff : 1;
	uint8_t ign1 : 1;
	uint8_t cyc : 1;
	uint8_t wca : 1;
	uint8_t wff : 1;
	uint8_t ign2 : 1;

	uint8_t byte_enable;

	uint8_t wcount;

	uint8_t rcount;

	uint32_t write_addr;
    union {
    	uint32_t value;
        read_addr;
    };
} __attribute__((packed));

This is wrapped inside of an EtherBone network packet header:

struct etherbone_packet {
	uint8_t magic[2]; // 0x4e 0x6f
	uint8_t version : 4;
	uint8_t ign : 1;
	uint8_t no_reads : 1;
	uint8_t probe_reply : 1;
	uint8_t probe_flag : 1;
	uint8_t port_size : 4;
	uint8_t addr_size : 4;
	uint8_t padding[4];

	struct etherbone_record records[0];
} __attribute__((packed));

LiteX only supports a single record per packet, so either wcount or rcount
is set to 1.  For a read, the read_addr is specified.  For a write, the
write_addr is specified along with a value.

The same type of record is returned, so your data is at offset 16.
*/

struct eb_connection {
    int fd;
    int read_fd;
    struct addrinfo* addr;
};

struct eb_connection *eb_connect(const char *addr, const char *port);
void eb_disconnect(struct eb_connection **conn);
int eb_fill_header(uint8_t wb_buffer[20],int is_read, uint8_t num, uint32_t base_addr);

#ifdef __cplusplus
};
#endif /* __cplusplus */

// structura driver data

typedef struct {
	// hal work signals
  	hal_bit_t *digital_in[MAX_N_INPUTS];
	hal_bit_t *digital_in_n[MAX_N_INPUTS];
  	hal_bit_t *digital_out[MAX_N_OUTPUTS]; 
  	hal_bit_t digital_out_inv[MAX_N_OUTPUTS]; 
  	hal_u32_t *wallclock;
  	hal_float_t *watchdog_rd;
  	hal_float_t watchdog_rd_old;
  	hal_float_t watchdog_wr;
	hal_u32_t wallclock_old;
	hal_float_t *wallclock_intvl;
	hal_float_t *enc_pos_fb[MAX_N_ENCODERS];
	hal_float_t enc_pos_fb_old[MAX_N_ENCODERS];
	hal_bit_t *enc_res[MAX_N_ENCODERS];
	hal_bit_t *enc_en[MAX_N_ENCODERS];
	hal_float_t *enc_vel_fb[MAX_N_ENCODERS];
	hal_float_t enc_scale[MAX_N_ENCODERS];
	hal_bit_t enc_inv[MAX_N_ENCODERS];
	hal_float_t *stepgen_velocity_cmd[MAX_N_STEPGENS];
	hal_float_t *stepgen_acc_lim[MAX_N_STEPGENS];
	hal_float_t *stepgen_velocity_fb[MAX_N_STEPGENS];
	hal_float_t *stepgen_position_fb[MAX_N_STEPGENS];
	hal_bit_t *stepgen_enable[MAX_N_STEPGENS];
	hal_bit_t *stepgen_reset[MAX_N_STEPGENS];
	hal_float_t stepgen_scale[MAX_N_STEPGENS];
	hal_bit_t stepgen_step_inv[MAX_N_STEPGENS];
	hal_bit_t stepgen_dir_inv[MAX_N_STEPGENS];
	hal_float_t stepgen_step_width;
	hal_float_t stepgen_step_space;
	hal_float_t stepgen_dir_width;
	hal_float_t stepgen_setup;
	hal_float_t *pwm_freq[MAX_N_PWM];
	hal_float_t *pwm_value[MAX_N_PWM];
	hal_bit_t *pwm_enable[MAX_N_PWM];
	hal_float_t pwm_scale[MAX_N_PWM];
	hal_float_t pwm_offs[MAX_N_PWM];
	hal_bit_t pwm_inv[MAX_N_PWM];
	hal_bit_t *enable;
	hal_bit_t *enable_req;
	hal_bit_t *enabled;
	hal_bit_t enable_req_old;
	hal_u32_t tx_max_retries;
	hal_u32_t num_errors_reported;
	struct eb_connection* eb;
} data_hal;



