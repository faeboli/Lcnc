
#include "rtapi_ctype.h"	/* isspace() */
#include "rtapi.h"			/* RTAPI realtime OS API */
#include "rtapi_app.h"		/* RTAPI realtime module decls */
#include "hal.h"			/* HAL public API decls */
#include <linux/types.h>
#include "rtapi_math64.h"
#include "rtapi_math.h"
#if defined(__FreeBSD__)
#include <sys/endian.h>
#else
#include <endian.h>
#endif
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <errno.h>
#include <linux/errno.h>
#include <stdlib.h>
#include <netdb.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include "include/configuration_auto.h"
#include "include/Lcnc.h"
#include "include/csr.h"

/* module information */
MODULE_AUTHOR("");
MODULE_DESCRIPTION("");
MODULE_LICENSE("");


//------------------------------------------------------------------
#define F_FPGA_TIME_US (1.0E6/F_FPGA) // fpga clock time in us
#define WDT_SCALE (64/F_FPGA) // 
#define VEL_SIZE_BITS 32 // width of command word in bits
#define EB_HEADER_SIZE 16  // etherbone header size in bytes
/***********************************************************************
*                STRUCTURES AND GLOBAL VARIABLES                       *
************************************************************************/

/* globals */
data_hal* device_data=0;
static int comp_id;		/* component ID */
static int num_ports;		/* number of ports configured */

/***********************************************************************
*                  LOCAL FUNCTION DECLARATIONS                         *
************************************************************************/
/* These is the functions that actually do the I/O
   everything else is just init code
*/
static void update_port(void *arg, long period);


/* buids an etherbone network packet
 * wb_buffer is the container of the packet
 * data to be wirren in case is a write operation
 * address is the address from wich to read or write
 * is_read selects a read or write operation */

int eb_fill_header(uint8_t wb_buffer[20],int is_read, uint8_t num, uint8_t base_addr)
{
    memset(wb_buffer, 0, 20);
    // etherbone packet header start
    wb_buffer[0] = 0x4e;	// Magic byte 0
    wb_buffer[1] = 0x6f;	// Magic byte 1
    wb_buffer[2] = 0x10;	// Version 1, all other flags 0
    wb_buffer[3] = 0x44;	// Address is 32-bits, port is 32-bits
    wb_buffer[4] = 0;		// Padding
    wb_buffer[5] = 0;		// Padding
    wb_buffer[6] = 0;		// Padding
    wb_buffer[7] = 0;		// Padding
    // etherbone packet header end
    
    // etherbone record start 
    wb_buffer[8] = 0;		// No Wishbone flags are set (cyc, wca, wff, etc.)
    wb_buffer[9] = 0x0f;	// Byte enable

    if (is_read) {
        wb_buffer[10] = 0;  // Write count
        wb_buffer[11] = num;	// Read count
    }
    else {
        wb_buffer[10] = num;	// Write count
        wb_buffer[11] = 0;  // Read count
    	}
    wb_buffer[12]= base_addr;
    return 20;
} 

int eb_send(struct eb_connection *conn, const void *bytes, size_t len) {
    if (conn->is_direct)
        return sendto(conn->fd, bytes, len, 0, conn->addr->ai_addr, conn->addr->ai_addrlen);
    return write(conn->fd, bytes, len);
}

int eb_recv(struct eb_connection *conn, void *bytes, size_t max_len) {
    if (conn->is_direct)
        return recvfrom(conn->read_fd, bytes, max_len, 0, NULL, NULL);
    return read(conn->fd, bytes, max_len);
}

struct eb_connection *eb_connect(const char *addr, const char *port, int is_direct) {

    struct addrinfo hints;
    struct addrinfo* res = 0;
    int err;
    int sock;

    struct eb_connection *conn = malloc(sizeof(struct eb_connection));
    if (!conn) {
        rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: couldn't allocate memory for eb_connection");
        return NULL;
    }

    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = is_direct ? SOCK_DGRAM : SOCK_STREAM;
    hints.ai_protocol = is_direct ? IPPROTO_UDP : IPPROTO_TCP;
    hints.ai_flags = AI_ADDRCONFIG;
    err = getaddrinfo(addr, port, &hints, &res);
    if (err != 0) {
        rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: failed to resolve remote socket address (err=%d / %s)\n", err, gai_strerror(err));
        free(conn);
        return NULL;
    }

    conn->is_direct = is_direct;

    if (is_direct) {
        // Rx half
        struct sockaddr_in si_me;

        memset((char *) &si_me, 0, sizeof(si_me));
        si_me.sin_family = res->ai_family;
        si_me.sin_port = ((struct sockaddr_in *)res->ai_addr)->sin_port;
        si_me.sin_addr.s_addr = htobe32(INADDR_ANY);

        int rx_socket;
        if ((rx_socket = socket(res->ai_family, res->ai_socktype, res->ai_protocol)) == -1) {
            rtapi_print_msg(RTAPI_MSG_ERR, "Lcnc: Unable to create Rx socket: %s\n", strerror(errno));
            freeaddrinfo(res);
            free(conn);
            return NULL;
        }
        if (bind(rx_socket, (struct sockaddr*)&si_me, sizeof(si_me)) == -1) {
            rtapi_print_msg(RTAPI_MSG_ERR, "Lcnc: Unable to bind Rx socket to port: %s\n", strerror(errno));
            close(rx_socket);
            freeaddrinfo(res);
            free(conn);
            return NULL;
        }
		struct timeval timeout;
		timeout.tv_sec = 0;
		timeout.tv_usec = 10000; //FIRST_PACKETS
		err = setsockopt(rx_socket, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(timeout));
		if (err < 0) {
			rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: could NOT to set setsockopt for tx\n");
            free(conn);
            return NULL;
		}
        // Tx half
        int tx_socket = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
        if (tx_socket == -1) {
            rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: Unable to create socket: %s\n", strerror(errno));
            close(rx_socket);
            close(tx_socket);
            freeaddrinfo(res);
            rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: unable to create socket: %s\n", strerror(errno));
            free(conn);
            return NULL;
        }

		timeout.tv_usec = SEND_TIMEOUT_US;
		err = setsockopt(tx_socket, SOL_SOCKET, SO_SNDTIMEO, (char *)&timeout, sizeof(timeout));
		if (err < 0) {
			rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: could NOT to set setsockopt for tx\n");
            free(conn);
            return NULL;
		}

        conn->read_fd = rx_socket;
        conn->fd = tx_socket;
        conn->addr = res;
    }
    else {
        sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock == -1) {
            rtapi_print_msg(RTAPI_MSG_ERR, "Lcnc: failed to create socket: %s\n", strerror(errno));
            freeaddrinfo(res);
            free(conn);
            return NULL;
        }

        int connection = connect(sock, res->ai_addr, res->ai_addrlen);
        if (connection == -1) {
            close(sock);
            freeaddrinfo(res);
            rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: unable to create socket: %s\n", strerror(errno));
            free(conn);
            return NULL;
        }

        conn->fd = sock;
        conn->addr = res;
    }

    return conn;
}

void eb_disconnect(struct eb_connection **conn) {
    if (!conn || !*conn)
        return;

    freeaddrinfo((*conn)->addr);
    close((*conn)->fd);
    if ((*conn)->read_fd)
        close((*conn)->read_fd);
    free(*conn);
    *conn = NULL;
    return;
}


/***********************************************************************
*                       INIT AND EXIT CODE                             *
************************************************************************/

int rtapi_app_main(void)
{
    char name[HAL_NAME_LEN + 1];
    int i,r ;

    num_ports = 1;

    // STEP 1: initialise the driver 
    comp_id = hal_init(DRIVER_NAME);
    if (comp_id < 0) 
    {
    	rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: ERROR: hal_init() failed\n");
    	goto fail0;
    }
    else rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: hal_init() ok\n");

    // STEP 2: allocate shared memory for to_hal data
    device_data = hal_malloc(sizeof(data_hal));
    if (device_data == 0) 
    {
    	rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: ERROR: hal_malloc() failed\n");
		r = -1;
		goto fail0;
    }
    else rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: hal_malloc() ok\n");

//###################################################
///// INIT ETH BOARD ; OPEN SOCKET
//###################################################
    device_data->eb = eb_connect(IP_ADDR, UDP_PORT, 1);

	if (!device_data->eb)
    {
        rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: ERROR: failed to connect to board\n");
        goto fail1;
    }
    else rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: connected to board \n");

//######################################################
//######### EXPORT SIGNALS, PIN, FUNCTION
//######################################################

// enable and reset
	r = hal_pin_bit_newf(HAL_IN, &(device_data->enable),comp_id, "Lcnc.%02d.enable", 0);
      	if (r < 0) 
      	{
			rtapi_print_msg(RTAPI_MSG_ERR, "Lcnc: ERROR: input %d export failed with err=%i\n", i ,r);
			r = -1;
			goto fail1;
	}
	r = hal_pin_bit_newf(HAL_IN, &(device_data->enable_req),comp_id, "Lcnc.%02d.enable-request", 0);
      	if (r < 0) 
      	{
			rtapi_print_msg(RTAPI_MSG_ERR, "Lcnc: ERROR: input %d export failed with err=%i\n", i ,r);
			r = -1;
			goto fail1;
	}
	r = hal_pin_bit_newf(HAL_OUT, &(device_data->enabled),comp_id, "Lcnc.%02d.enabled", 0);
      	if (r < 0) 
      	{
			rtapi_print_msg(RTAPI_MSG_ERR, "Lcnc: ERROR: input %d export failed with err=%i\n", i ,r);
			r = -1;
			goto fail1;
	}
// tx retries timeout
	r = hal_param_u32_newf(HAL_RW, &(device_data->tx_max_retries),comp_id, "Lcnc.%02d.tx-max-retries", 0);
      	if (r < 0) 
      	{
			rtapi_print_msg(RTAPI_MSG_ERR, "Lcnc: ERROR: input %d export failed with err=%i\n", i ,r);
			r = -1;
			goto fail1;
	}
	device_data->tx_max_retries=(hal_u32_t)5;
// watchdog
	r = hal_pin_float_newf(HAL_OUT, &(device_data->watchdog_rd),comp_id, "Lcnc.%02d.watchdog-read", 0);
      	if (r < 0) 
      	{
			rtapi_print_msg(RTAPI_MSG_ERR, "Lcnc: ERROR: input %d export failed with err=%i\n", i ,r);
			r = -1;
			goto fail1;
	}
	r = hal_param_float_newf(HAL_RW, &(device_data->watchdog_wr),comp_id, "Lcnc.%02d.watchdog-write", 0);
      	if (r < 0) 
      	{
			rtapi_print_msg(RTAPI_MSG_ERR, "Lcnc: ERROR: input %d export failed with err=%i\n", i ,r);
			r = -1;
			goto fail1;
	}
	device_data->watchdog_wr=(hal_float_t)0.01;
// Inputs
    for ( i=0; i<N_INPUTS;i++) 
    {
		r = hal_pin_bit_newf(HAL_OUT, &(device_data->digital_in[i]),comp_id, "Lcnc.%02d.input.%02d.in", 0, i);
      	if (r < 0) 
      	{
			rtapi_print_msg(RTAPI_MSG_ERR, "Lcnc: ERROR: input %d export failed with err=%i\n", i ,r);
			r = -1;
			goto fail1;
	}

      	r = hal_pin_bit_newf(HAL_OUT, &(device_data->digital_in_n[i]),comp_id, "Lcnc.%02d.input.%02d.in-n", 0, i);
      	if (r < 0) 
      	{
        	rtapi_print_msg(RTAPI_MSG_ERR, "Lcnc: ERROR: input-n %d export failed with err=%i\n", i,r);
        	r = -1;
        	goto fail1;
      	}

    }

// outputs
    for ( i=0; i<N_OUTPUTS;i++) 
    {
		r = hal_pin_bit_newf(HAL_IN, &(device_data->digital_out[i]),comp_id, "Lcnc.%02d.output.%02d.out", 0, i);
		if (r < 0) 
		{
			rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: ERROR: output %d export failed with err=%i\n", i,r);
			r = -1;
			goto fail1;
		}
		r = hal_param_bit_newf(HAL_RW, &(device_data->digital_out_inv[i]),comp_id, "Lcnc.%02d.output.%02d.inv", 0, i);
		if (r < 0) 
		{
			rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: ERROR: output inversion %d export failed with err=%i\n", i,r);
			r = -1;
			goto fail1;
		}
    }

// wallclock
    r = hal_pin_u32_newf(HAL_OUT, &(device_data->wallclock), comp_id, "Lcnc.%02d.wallclock.%02d.value", 0,0);
	if (r < 0) 
	{
		rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: ERROR: board_wallclock var export failed with err=%i\n",r);
		r = -1;
		goto fail1;
	}
    r = hal_pin_float_newf(HAL_OUT, &(device_data->wallclock_intvl), comp_id, "Lcnc.%02d.wallclock.%02d.interval", 0,0);
	if (r < 0) 
	{
		rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: ERROR: board_wallclock interval export failed with err=%i\n",r);
		r = -1;
		goto fail1;
	}

// encoders
	for ( i=0; i<N_ENCODERS; i++) 
	{// !! aggiustare la nomenclatura, si tratta di posizione
		// encoder_count
		r = hal_pin_float_newf(HAL_OUT, &(device_data-> enc_pos_fb[i]), comp_id, "Lcnc.%02d.encoder.%02d.pos-fb", 0, i);
		if (r < 0) 
		{
			rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: ERROR:  err encoder=%i\n", r);
			r = -1;
			goto fail1;
		}
		r = hal_pin_bit_newf(HAL_IN, &(device_data-> enc_res[i]), comp_id, "Lcnc.%02d.encoder.%02d.reset", 0, i);
		if (r < 0) 
		{
			rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: ERROR:  err encoder=%i\n", r);
			r = -1;
			goto fail1;
		}
		r = hal_pin_bit_newf(HAL_IN, &(device_data-> enc_en[i]), comp_id, "Lcnc.%02d.encoder.%02d.enable", 0, i);
		if (r < 0) 
		{
			rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: ERROR:  err encoder=%i\n", r);
			r = -1;
			goto fail1;
		}
		r = hal_pin_float_newf(HAL_OUT, &(device_data-> enc_vel_fb[i]), comp_id, "Lcnc.%02d.encoder.%02d.vel-fb", 0, i);
		if (r < 0) 
		{
			rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: ERROR:  err encoder=%i\n", r);
			r = -1;
			goto fail1;
		}
        // encoder scale
		r = hal_param_float_newf(HAL_RW, &(device_data-> enc_scale[i]), comp_id, "Lcnc.%02d.encoder.%02d.scale", 0, i);
		if (r < 0) 
		{
			rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: ERROR:  err enc_scale=%i\n", r);
			r = -1;
			goto fail1;
		}
		device_data-> enc_scale[i]=(hal_float_t)1.0;
        // encoder inversion
		r = hal_param_bit_newf(HAL_RW, &(device_data-> enc_inv[i]), comp_id, "Lcnc.%02d.encoder.%02d.inv", 0, i);
		if (r < 0) 
		{
			rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: ERROR:  err enc_scale=%i\n", r);
			r = -1;
			goto fail1;
		}
	}

// stepgens
	for ( i=0; i<N_STEPGENS;i++) 
	{
        r = hal_pin_float_newf(HAL_IN, &(device_data->stepgen_velocity_cmd[i]), comp_id, "Lcnc.%02d.stepgen.%02d.vel-cmd", 0, i);
        if(r != 0) return r;

        r = hal_pin_float_newf(HAL_IN, &(device_data->stepgen_acc_lim[i]), comp_id, "Lcnc.%02d.stepgen.%02d.acc_lim", 0, i);
        if(r != 0) return r;

        r = hal_pin_float_newf(HAL_OUT, &(device_data->stepgen_velocity_fb[i]), comp_id, "Lcnc.%02d.stepgen.%02d.vel-fb", 0, i);
        if(r != 0) return r;

        r = hal_pin_float_newf(HAL_OUT, &(device_data->stepgen_position_fb[i]), comp_id, "Lcnc.%02d.stepgen.%02d.pos-fb", 0, i);
        if(r != 0) return r;

        r = hal_pin_bit_newf(HAL_IN, &(device_data->stepgen_enable[i]), comp_id, "Lcnc.%02d.stepgen.%02d.enable", 0, i);
        if(r != 0) return r;

        r = hal_pin_bit_newf(HAL_IN, &(device_data->stepgen_reset[i]), comp_id, "Lcnc.%02d.stepgen.%02d.reset", 0, i);
        if(r != 0) return r;

        r = hal_param_float_newf(HAL_RW, &(device_data->stepgen_scale[i]), comp_id, "Lcnc.%02d.stepgen.%02d.scale", 0, i);
        if(r != 0) return r;
        device_data->stepgen_scale[i] = (hal_float_t)1.0;

        r = hal_param_bit_newf(HAL_RW, &(device_data->stepgen_step_inv[i]), comp_id, "Lcnc.%02d.stepgen.%02d.step_inv", 0, i);
        if(r != 0) return r;

        r = hal_param_bit_newf(HAL_RW, &(device_data->stepgen_dir_inv[i]), comp_id, "Lcnc.%02d.stepgen.%02d.dir_inv", 0, i);
        if(r != 0) return r;
    }

    r = hal_param_float_newf(HAL_RW, &(device_data->stepgen_step_width), comp_id, "Lcnc.%02d.stepgen-step_width", 0);
    if(r != 0) return r;
    device_data->stepgen_step_width = (hal_float_t)0.1;

    r = hal_param_float_newf(HAL_RW, &(device_data->stepgen_step_space), comp_id, "Lcnc.%02d.stepgen-step_space", 0);
    if(r != 0) return r;
    device_data->stepgen_step_space =(hal_float_t)0.1;

    r = hal_param_float_newf(HAL_RW, &(device_data->stepgen_dir_width), comp_id, "Lcnc.%02d.stepgen-dir_width", 0);
    if(r != 0) return r;
    device_data->stepgen_dir_width = (hal_float_t)0.1;

    r = hal_param_float_newf(HAL_RW, &(device_data->stepgen_setup), comp_id, "Lcnc.%02d.stepgen-setup_time", 0);
    if(r != 0) return r;
    device_data->stepgen_setup = (hal_float_t)1.0;

// PWM
	for ( i=0; i<N_PWM;i++) 
	{
		r = hal_pin_float_newf(HAL_IN, &(device_data-> pwm_freq[i]), comp_id, "Lcnc.%02d.pwm.%02d.freq", 0, i);
		if (r < 0) 
		{
			rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: ERROR:  err steptime=%i\n", r);
			r = -1;
			goto fail1;
		}
			r = hal_pin_float_newf(HAL_IN, &(device_data-> pwm_value[i]), comp_id, "Lcnc.%02d.pwm.%02d.value", 0, i);
		if (r < 0) 
		{
			rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: ERROR:  err steptime=%i\n", r);
			r = -1;
			goto fail1;
		}
			r = hal_pin_bit_newf(HAL_IN, &(device_data->pwm_enable[i]), comp_id, "Lcnc.%02d.pwm.%02d.enable", 0, i);
		if (r < 0) 
		{
			rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: ERROR:  err steptime=%i\n", r);
			r = -1;
			goto fail1;
		}
	        r = hal_param_float_newf(HAL_RW, &(device_data->pwm_scale[i]), comp_id, "Lcnc.%02d.pwm.%02d.scale", 0, i);
		if (r < 0) 
		{
			rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: ERROR:  err steptime=%i\n", r);
			r = -1;
			goto fail1;
		}
		device_data->pwm_scale[i]=(hal_float_t)100.0;
		
		r = hal_param_float_newf(HAL_RW, &(device_data-> pwm_offs[i]), comp_id, "Lcnc.%02d.pwm.%02d.offs", 0, i);
		if (r < 0) 
		{
			rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: ERROR:  err steptime=%i\n", r);
			r = -1;
			goto fail1;
		}
		
			r = hal_param_bit_newf(HAL_RW, &(device_data-> pwm_inv[i]), comp_id, "Lcnc.%02d.pwm.%02d.inv", 0, i);
		if (r < 0) 
		{
			rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: ERROR:  err steptime=%i\n", r);
			r = -1;
			goto fail1;
		}
    }

    /* STEP 4: export function */
    rtapi_snprintf(name, sizeof(name), "Lcnc.%02d.update", 0);
    r = hal_export_funct(name, update_port, device_data, 1, 0,comp_id);
    if (r < 0) 
    {
		rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: ERROR: port %d write funct export failed\n", 1);
		r = -1;
		goto fail1;
    }

    rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: installed driver for %d card(s)\n", num_ports);
    hal_ready(comp_id);
           
    return 0;


//####### ERROR ############

	fail1:

	eb_disconnect(&(device_data->eb));

	fail0:
    hal_exit(comp_id);
	return r;

}
//####### EXIT ############
void rtapi_app_exit(void)
{
	eb_disconnect(&(device_data->eb));
	hal_exit(comp_id);
}


/**************************************************************
###############################################################
* REALTIME PORT WRITE FUNCTION                                *
###############################################################
**************************************************************/



void update_port(void *arg, long period)
{
    data_hal *port;

    int i,res;
    static int_fast64_t internal_step_count[N_STEPGENS];
    static uint_fast32_t step_count[N_STEPGENS];
    static uint_fast32_t step_count_old[N_STEPGENS];
    static int_fast64_t internal_enc_count[N_ENCODERS];
    static uint_fast32_t enc_count[N_ENCODERS];
    static uint_fast32_t enc_count_old[N_ENCODERS];
    port = arg;
    uint_fast32_t tempvalue;
    union EbData 
	{
		   uint32_t value;
		   uint8_t bytes[4];
	} temp_reg_value; 
    float time_temp;
    uint32_t time_temp_u;
    uint_fast8_t read_header_ready=0,write_header_ready=0;

//----------------------------------
// Start of read section
//----------------------------------

	// !! prepare packet for read registers one by one from here
        uint8_t tx_read_packet_buffer[EB_HEADER_SIZE+RX_PAYLOAD_SIZE];
	// Header setup
	if(!read_header_ready)
	    {
	    eb_fill_header(tx_read_packet_buffer,1,RX_PAYLOAD_SIZE/4,CSR_MMIO_INST_BASE); // (buffer,is_read,number of read/writes,start address)
	    read_header_ready=1;
	}
        // Payload setup
        // read reset and status register
        temp_reg_value.value=htobe32(CSR_MMIO_INST_RES_ST_REG_ADDR); 
        memcpy((void*)(tx_read_packet_buffer+EB_HEADER_SIZE+RX_POS_RES_ST_REG),(void*)temp_reg_value.bytes,4);
        // read stepgen position
        for(i=0;i<N_STEPGENS;i++)
        {
			temp_reg_value.value=htobe32(CSR_MMIO_INST_SG_COUNT_0_ADDR+4*i);
			memcpy((void*)(tx_read_packet_buffer+EB_HEADER_SIZE+RX_POS_STEPCOUNT0+4*i),(void*)temp_reg_value.bytes,4);
		}
		// read wall clock
        temp_reg_value.value=htobe32(CSR_MMIO_INST_WALLCLOCK_ADDR); 
        memcpy((void*)(tx_read_packet_buffer+EB_HEADER_SIZE+RX_POS_WALLCLOCK),(void*)temp_reg_value.bytes,4);
        // read gpio in
        temp_reg_value.value=htobe32(CSR_MMIO_INST_GPIOS_IN_ADDR); 
        memcpy((void*)(tx_read_packet_buffer+EB_HEADER_SIZE+RX_POS_GPIOS_IN),(void*)temp_reg_value.bytes,4);
		// read encoders
        for(i=0;i<N_ENCODERS;i++)
        {
			temp_reg_value.value=htobe32(CSR_MMIO_INST_ENC_COUNT_0_ADDR+4*i); 
			memcpy((void*)(tx_read_packet_buffer+EB_HEADER_SIZE+RX_POS_ENCCOUNT0+4*i),(void*)temp_reg_value.bytes,4);
		}

    // #################  TRANSMIT DATA  ###############################################################
        res=eb_send(device_data->eb, tx_read_packet_buffer, EB_HEADER_SIZE+RX_PAYLOAD_SIZE);

    // #################  RECEIVE DATA  ###############################################################

	uint8_t rx_read_packet_buffer[EB_HEADER_SIZE+RX_PAYLOAD_SIZE];
        int count = eb_recv(device_data->eb, rx_read_packet_buffer, sizeof(rx_read_packet_buffer));
        if (count <0) {
                if(((port->num_errors_reported)%100)==0)
	            fprintf(stderr, "connection error - unexpected read length: %d\n errors %d max %d\n", count,(port->num_errors_reported), (port->tx_max_retries));
            (port->num_errors_reported)++;
//            return;
        }

        //end receive process
        else {
                if((port->num_errors_reported)>0)
		{
			fprintf(stderr, "connection restored\n", count);
			(port->num_errors_reported)=0;
			}
		}
	
	// Reset and status reg
	// read watchdog value
	memcpy((void*)temp_reg_value.bytes,(void*)(rx_read_packet_buffer+EB_HEADER_SIZE+RX_POS_RES_ST_REG),4);
	tempvalue=be32toh(temp_reg_value.value);
	*(port->watchdog_rd)=(hal_float_t)(tempvalue >> CSR_MMIO_INST_RES_ST_REG_WATCHDOG_OFFSET)*WDT_SCALE;
	
	// if watchdog bites or not enabled or tx errors
	if(*(port->watchdog_rd)==0 || *(port->enable)==0 || ((port->num_errors_reported)>(port->tx_max_retries))) 
	{
	    *(port->enabled)=0;
	}
	else
	{
	    if(*(port->enable_req) && !port->enable_req_old && *(port->enable))
	    {
		*(port->enabled)=1;
	    }
	}
	port->enable_req_old=*(port->enable_req);
	
	// gpio in
	memcpy((void*)temp_reg_value.bytes,(void*)(rx_read_packet_buffer+EB_HEADER_SIZE+RX_POS_GPIOS_IN),4);
	tempvalue=be32toh(temp_reg_value.value);
    for (i=0 ; i < N_INPUTS ; i++) 
	{
	    if (tempvalue & 1<<i) *(port->digital_in[i]) = 1 ;
	    else *(port->digital_in[i]) = 0;
	    *(port->digital_in_n[i])=!(*(port->digital_in[i]));
    }
	// wallclock
	memcpy((void*)temp_reg_value.bytes,(void*)(rx_read_packet_buffer+EB_HEADER_SIZE+RX_POS_WALLCLOCK),4);
	tempvalue=be32toh(temp_reg_value.value);
	port->wallclock_old=*(port->wallclock);
	*(port->wallclock)=tempvalue;
	if((*(port->wallclock))>(port->wallclock_old))
	    *(port->wallclock_intvl)=((float)(*(port->wallclock)-(port->wallclock_old)))/F_FPGA;
	else *(port->wallclock_intvl)=((float)((port->wallclock_old)-*(port->wallclock)))/F_FPGA;
	// steppers position and velocity feedback
	// introdurre calcolo di velocità basato sulla differenza tra le posizioni nel tempo (usando il wallclock)
    for ( i=0; i<N_STEPGENS;i++)
    {
		memcpy((void*)temp_reg_value.bytes,(void*)(rx_read_packet_buffer+EB_HEADER_SIZE+RX_POS_STEPCOUNT0+4*i),4);
		tempvalue=be32toh(temp_reg_value.value);
		step_count_old[i]=step_count[i]; // save last counter value
		step_count[i]=tempvalue;
		if(*(device_data->stepgen_reset[i])) internal_step_count[i]=0;
		internal_step_count[i]+=((int32_t)(step_count[i])-(int32_t)step_count_old[i]);
		*(port->stepgen_position_fb[i])=((hal_float_t)(internal_step_count[i]))/(port->stepgen_scale[i]);
	}

     ///////////// read encoders /////////////////////////////////////////////

    for ( i=0; i<N_ENCODERS;i++)
    {
		memcpy((void*)temp_reg_value.bytes,(void*)(rx_read_packet_buffer+EB_HEADER_SIZE+RX_POS_ENCCOUNT0+4*i),4);
		tempvalue=be32toh(temp_reg_value.value);
		enc_count_old[i]=enc_count[i]; // save last counter value
		enc_count[i]=tempvalue; // new counter value
		if(*(device_data->enc_res[i])) internal_enc_count[i]=0; // reset internal counter if needed
		if((port->enc_inv[i])) 
		    internal_enc_count[i]-=((int32_t)(enc_count[i])-(int32_t)enc_count_old[i]); //decrement internal counter if inverted
		else
		    internal_enc_count[i]+=((int32_t)(enc_count[i])-(int32_t)enc_count_old[i]); // increment internal counter
		*(port->enc_pos_fb[i])=((hal_float_t)(internal_enc_count[i]))*(port->enc_scale[i]); // apply scaling
		// calculate velocity
		port->enc_pos_fb_old[i]=*(port->enc_pos_fb[i]);				    
		if((*(port->wallclock_intvl))>0)
		    *(port->enc_vel_fb[i])=((*(port->enc_pos_fb[i]))-(port->enc_pos_fb_old[i]))/(*(port->wallclock_intvl));
    }
    
//----------------------------------
// End of read section
//----------------------------------

//----------------------------------
// Start of write section
//----------------------------------
    uint8_t tx_write_packet_buffer[EB_HEADER_SIZE+TX_PAYLOAD_SIZE];
	// Header setup
    if(!write_header_ready)
	{
	eb_fill_header(tx_write_packet_buffer,0,TX_PAYLOAD_SIZE/4,CSR_MMIO_INST_BASE); // (buffer,is_read,number of read/writes,start address)
	write_header_ready=1;
    }
    // Payload setup
	const float velfact=pow(2,VEL_SIZE_BITS)/F_FPGA; // velocity factor that is 2^32/F_FPGA
	float max_freq,freq_req,cmdtmp;
	int sign=1;
	uint32_t temp_sign=0;
	for (i=0;i<N_STEPGENS;i++)
	{
	    // maximum frequency: given by mximum step period, that is the iverse of the sum of pulse width and space
	    max_freq=1.0E6/(port->stepgen_step_width + port->stepgen_step_space); //[Hz]
	    // requested freqeuency of step generator
	    freq_req=(*(port->stepgen_velocity_cmd[i]))*(port->stepgen_scale[i]);
	    if(freq_req<0)
	    {
	        sign=-1;
	        freq_req=-freq_req;
	        temp_sign|=(1<<i);
	    }
	    else sign=1;
	    // limit frequency reuested to the maximum available
	    if(freq_req>max_freq) freq_req=max_freq;
	    // recalculate actual velocity for feedback
	    if(*(device_data->stepgen_enable[i]) && !(*(device_data->stepgen_reset[i]))) 
		*(device_data->stepgen_velocity_fb[i])=(double)sign*freq_req/(port->stepgen_scale[i]);
	    else *(device_data->stepgen_velocity_fb[i])=0;
	    // convert frequency request to step generator internal command word
	    cmdtmp=freq_req*velfact;
	    temp_reg_value.value=htobe32((uint32_t)cmdtmp); // VELOCITY1
        memcpy((void*)(tx_write_packet_buffer+EB_HEADER_SIZE+TX_POS_VELOCITY0+4*i),(void*)temp_reg_value.bytes,4);
    }
    temp_reg_value.value=htobe32(temp_sign); // STEPSIGN
    memcpy((void*)(tx_write_packet_buffer+EB_HEADER_SIZE+TX_POS_STEPSIGN),(void*)temp_reg_value.bytes,4);
    

    tempvalue=0;
    for (i=0;i<N_STEPGENS;i++)
    {
	    if (*(port->stepgen_enable[i]))
        {
            tempvalue|=(1<<i+CSR_MMIO_INST_STEP_RES_EN_SGENABLE_OFFSET);
        }
	    if (*(port->stepgen_reset[i]))
        {
            tempvalue|=(1<<i+CSR_MMIO_INST_STEP_RES_EN_SGRESET_OFFSET);
        }
    }
    temp_reg_value.value=htobe32(tempvalue); // STEP_RES_EN
    memcpy((void*)(tx_write_packet_buffer+EB_HEADER_SIZE+TX_POS_STEP_RES_EN),(void*)temp_reg_value.bytes,4);	
    
    tempvalue=0;
    for (i=0;i<N_STEPGENS;i++)
    {
	    if (port->stepgen_dir_inv[i])
        {
            tempvalue|=(1<<i+CSR_MMIO_INST_STEPDIRINV_DIR_INV_OFFSET);
        }
	    if (port->stepgen_step_inv[i])
        {
            tempvalue|=(1<<i+CSR_MMIO_INST_STEPDIRINV_STEP_INV_OFFSET);
        }
    }
    temp_reg_value.value=htobe32(tempvalue); // STEPDIRINV
    memcpy((void*)(tx_write_packet_buffer+EB_HEADER_SIZE+TX_POS_STEPDIRINV),(void*)temp_reg_value.bytes,4);
    tempvalue=0;
    time_temp=(port->stepgen_step_width)/F_FPGA_TIME_US;
    if(time_temp > ((1<<CSR_MMIO_INST_STEPTIMES_STEP_WIDTH_SIZE)-1)) time_temp_u=(1<<CSR_MMIO_INST_STEPTIMES_STEP_WIDTH_SIZE)-1;
    else time_temp_u=(uint32_t)time_temp;
    tempvalue|=time_temp_u<<CSR_MMIO_INST_STEPTIMES_STEP_WIDTH_OFFSET;
    time_temp=(port->stepgen_dir_width)/F_FPGA_TIME_US;
    if(time_temp > ((1<<CSR_MMIO_INST_STEPTIMES_DIR_WIDTH_SIZE)-1)) time_temp_u=(1<<CSR_MMIO_INST_STEPTIMES_DIR_WIDTH_SIZE)-1;
    else time_temp_u=(uint32_t)time_temp;
    tempvalue|=time_temp_u<<CSR_MMIO_INST_STEPTIMES_DIR_WIDTH_OFFSET;
    time_temp=(port->stepgen_setup)/F_FPGA_TIME_US;
    if(time_temp > ((1<<CSR_MMIO_INST_STEPTIMES_DIR_SETUP_SIZE)-1)) time_temp_u=(1<<CSR_MMIO_INST_STEPTIMES_DIR_SETUP_SIZE)-1;
    else time_temp_u=(uint32_t)time_temp;
    tempvalue|=time_temp_u<<CSR_MMIO_INST_STEPTIMES_DIR_SETUP_OFFSET;
    temp_reg_value.value=htobe32(tempvalue); 
    memcpy((void*)(tx_write_packet_buffer+EB_HEADER_SIZE+TX_POS_STEPTIMES),(void*)temp_reg_value.bytes,4);
	// GPIOS_OUT
    tempvalue=0;
    for (i=0;i<N_OUTPUTS;i++)
    {
	if ((*(port->digital_out[i])) ^ (port->digital_out_inv[i])) tempvalue|=(1<<i);
    }
    temp_reg_value.value=htobe32(tempvalue); 
    memcpy((void*)(tx_write_packet_buffer+EB_HEADER_SIZE+TX_POS_GPIOS_OUT),(void*)temp_reg_value.bytes,4);
    // PWM_0
    for(i=0;i<N_PWM;i++)
    {
	if(*(port->pwm_enable[i]) && *(port->pwm_freq[i])>0 && *(port->pwm_value[i])>=0)
	{
		float temp_period,temp_width;
		uint32_t temp_period_u=0,temp_width_u=0;
		tempvalue=0;
		temp_period=F_FPGA/(*(port->pwm_freq[i]));
		if(temp_period > 65535) temp_period=65535;
		if(temp_period < 2) temp_period=2;
		temp_period_u=(uint32_t)temp_period;
		tempvalue|=temp_period_u<<16;
		temp_width=(*(port->pwm_value[i])+(port->pwm_offs[i]))*(temp_period)/(port->pwm_scale[i]);
		if(temp_width > temp_period) temp_width=temp_period;
		if(temp_width < 0) temp_width=0;
		temp_width_u=(uint32_t)temp_width;
		if(port->pwm_inv[i]) tempvalue|=(temp_period_u-temp_width_u);
		else tempvalue|=temp_width_u;
		temp_reg_value.value=htobe32(tempvalue); 
	}
	else temp_reg_value.value=0;
	memcpy((void*)(tx_write_packet_buffer+EB_HEADER_SIZE+TX_POS_PWM0+4*i),(void*)temp_reg_value.bytes,4);
    }
    tempvalue=0;
    for (i=0;i<N_ENCODERS;i++)
    {
	    if (*(port->enc_res[i]))
        {
            tempvalue|=(1<<i+CSR_MMIO_INST_ENC_RES_EN_RESET_OFFSET);
        }
	    if (*(port->enc_en[i]))
        {
            tempvalue|=(1<<i+CSR_MMIO_INST_ENC_RES_EN_ENABLE_OFFSET);
        }
    }
    temp_reg_value.value=htobe32(tempvalue); // ENC_RES_EN
    memcpy((void*)(tx_write_packet_buffer+EB_HEADER_SIZE+TX_POS_ENC_RES_EN),(void*)temp_reg_value.bytes,4);	
    // check max value
    tempvalue=(uint32_t)((port->watchdog_wr)/WDT_SCALE);
    tempvalue=tempvalue<<CSR_MMIO_INST_RES_ST_REG_WATCHDOG_OFFSET;
    temp_reg_value.value=htobe32(tempvalue); // watchdog
    memcpy((void*)(tx_write_packet_buffer+EB_HEADER_SIZE+TX_POS_RES_STAT_REG),(void*)temp_reg_value.bytes,4);	

    // #################  TRANSMIT DATA  ###############################################################
        res=eb_send(device_data->eb, tx_write_packet_buffer, EB_HEADER_SIZE+TX_PAYLOAD_SIZE);
    // #################################################################################################

}

