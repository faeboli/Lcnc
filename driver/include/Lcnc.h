#define SEND_TIMEOUT_US 10
//#define RECV_TIMEOUT_US 10
//#define READ_PCK_DELAY_NS 10000
#define DRIVER_NAME "Lcnc"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <stdint.h>
#include <stdlib.h>

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
    int is_direct;
    struct addrinfo* addr;
};

struct eb_connection *eb_connect(const char *addr, const char *port, int is_direct);
void eb_disconnect(struct eb_connection **conn);
int eb_fill_header(uint8_t wb_buffer[20],int is_read, uint8_t num, uint8_t base_addr);

#ifdef __cplusplus
};
#endif /* __cplusplus */

// structura driver data

typedef struct {
	// hal work signals
  	hal_bit_t *digital_in[N_INPUTS];
	hal_bit_t *digital_in_n[N_INPUTS];
  	hal_bit_t *digital_out[N_OUTPUTS]; 
  	hal_bit_t digital_out_inv[N_OUTPUTS]; 
  	hal_u32_t *wallclock;
  	hal_float_t *watchdog_rd;
  	hal_float_t watchdog_wr;
	hal_u32_t wallclock_old;
	hal_float_t *wallclock_intvl;
	hal_float_t *enc_pos_fb[N_ENCODERS];
	hal_float_t enc_pos_fb_old[N_ENCODERS];
	hal_bit_t *enc_res[N_ENCODERS];
	hal_bit_t *enc_en[N_ENCODERS];
	hal_float_t *enc_vel_fb[N_ENCODERS];
	hal_float_t enc_scale[N_ENCODERS];
	hal_bit_t enc_inv[N_ENCODERS];
	hal_float_t *stepgen_velocity_cmd[N_STEPGENS];
	hal_float_t *stepgen_acc_lim[N_STEPGENS];
	hal_float_t *stepgen_velocity_fb[N_STEPGENS];
	hal_float_t *stepgen_position_fb[N_STEPGENS];
	hal_bit_t *stepgen_enable[N_STEPGENS];
	hal_bit_t *stepgen_reset[N_STEPGENS];
	hal_float_t stepgen_scale[N_STEPGENS];
	hal_bit_t stepgen_step_inv[N_STEPGENS];
	hal_bit_t stepgen_dir_inv[N_STEPGENS];
	hal_float_t stepgen_step_width;
	hal_float_t stepgen_step_space;
	hal_float_t stepgen_dir_width;
	hal_float_t stepgen_setup;
	hal_float_t *pwm_freq[N_PWM];
	hal_float_t *pwm_value[N_PWM];
	hal_bit_t *pwm_enable[N_PWM];
	hal_float_t pwm_scale[N_PWM];
	hal_float_t pwm_offs[N_PWM];
	hal_bit_t pwm_inv[N_PWM];
	hal_bit_t *enable;
	hal_bit_t *enable_req;
	hal_bit_t *enabled;
	hal_bit_t enable_req_old;
	hal_u32_t tx_max_retries;
	hal_u32_t num_errors_reported;
	struct eb_connection* eb;
} data_hal;



