
#include "rtapi_ctype.h"    /* isspace() */
#include "rtapi.h"          /* RTAPI realtime OS API */
#include "rtapi_app.h"      /* RTAPI realtime module decls */
#include "hal.h"            /* HAL public API decls */
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
#include <time.h>
#include <netinet/in.h>
#include "Lcnc.h"

/* module information */
MODULE_AUTHOR("");
MODULE_DESCRIPTION("");
MODULE_LICENSE("");


/***********************************************************************
*                STRUCTURES AND GLOBAL VARIABLES                       *
************************************************************************/

/* globals */
data_hal* device_data=0;
static int comp_id;     /* component ID */
static int num_ports;       /* number of ports configured */
static struct timespec spec_old;
uint8_t n_in,n_out,n_sg,n_en,n_pwm,intf_ver=0; /* number of each peripheral and interface version*/
uint32_t INIT_WRITE_addr,CONFIGURATION_addr,REGS_START_addr,REGS_START_value,VELOCITY0_addr,VELOCITYlast_addr,MAX_ACC0_addr,MAX_ACClast_addr;
uint32_t STEP_RES_EN_addr,STEPDIRINV_addr,STEPTIMES_addr,GPIOS_OUT_addr,PWM_0_addr,PWM_last_addr,ENC_RES_EN_addr;
uint32_t RES_ST_REG_addr,SG_COUNT_0_addr,SG_COUNT_last_addr,SG_VEL_0_addr,SG_VEL_last_addr,WALLCLOCK_addr,GPIOS_IN_addr;
uint32_t ENC_COUNT_0_addr,ENC_COUNT_last_addr,LAST_addr,TX_PAYLOAD_size,RX_PAYLOAD_size;
uint32_t TX_SECTION_START_addr,TX_SECTION_END_addr,RX_SECTION_START_addr,RX_SECTION_END_addr;
    
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

int eb_fill_header(uint8_t wb_buffer[20],int is_read, uint8_t num, uint32_t base_addr)
{
    union EbDataAddr 
    {
           uint32_t value;
           uint8_t bytes[4];
    } BaseAddr; 
    
    memset(wb_buffer, 0, 20);
    // etherbone packet header start
    wb_buffer[0] = 0x4e;    // Magic byte 0
    wb_buffer[1] = 0x6f;    // Magic byte 1
    wb_buffer[2] = 0x10;    // Version 1, all other flags 0
    wb_buffer[3] = 0x44;    // Address is 32-bits, port is 32-bits
    wb_buffer[4] = 0;       // Padding
    wb_buffer[5] = 0;       // Padding
    wb_buffer[6] = 0;       // Padding
    wb_buffer[7] = 0;       // Padding
    // etherbone packet header end
    
    // etherbone record start 
    wb_buffer[8] = 0;       // No Wishbone flags are set (cyc, wca, wff, etc.)
    wb_buffer[9] = 0x0f;    // Byte enable

    if (is_read) 
    {
        wb_buffer[10] = 0;  // Write count
        wb_buffer[11] = num;    // Read count
    }
    else 
    {
        wb_buffer[10] = num;    // Write count
        wb_buffer[11] = 0;  // Read count
    }
    BaseAddr.value=htobe32(base_addr);
    wb_buffer[12]= BaseAddr.bytes[0];
    wb_buffer[13]= BaseAddr.bytes[1];
    wb_buffer[14]= BaseAddr.bytes[2];
    wb_buffer[15]= BaseAddr.bytes[3];
    
    return 20;
} 

int eb_send(struct eb_connection *conn, const void *bytes, size_t len) 
{
    return sendto(conn->fd, bytes, len, MSG_DONTWAIT, conn->addr->ai_addr, conn->addr->ai_addrlen);
    //return send(conn->fd, bytes, len, 0);
}

int eb_recv(struct eb_connection *conn, void *bytes, size_t max_len) 
{
    //return recvfrom(conn->read_fd, bytes, max_len, 0, NULL, NULL);
    return recv(conn->read_fd, bytes, max_len, 0);
}

struct eb_connection *eb_connect(const char *addr, const char *port) 
{

    struct addrinfo hints;
    struct addrinfo* res = 0;
    int err;
    int sock;

    struct eb_connection *conn = malloc(sizeof(struct eb_connection));
    if (!conn) 
    {
        rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: couldn't allocate memory for eb_connection");
        return NULL;
    }

    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_protocol = IPPROTO_UDP;
    hints.ai_flags = AI_ADDRCONFIG;
    err = getaddrinfo(addr, port, &hints, &res);
    if (err != 0) 
    {
        rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: failed to resolve remote socket address (err=%d / %s)\n", err, gai_strerror(err));
        free(conn);
        return NULL;
    }

    // Rx half
    struct sockaddr_in si_me;

    memset((char *) &si_me, 0, sizeof(si_me));
    si_me.sin_family = res->ai_family;
    si_me.sin_port = ((struct sockaddr_in *)res->ai_addr)->sin_port;
    si_me.sin_addr.s_addr = htobe32(INADDR_ANY);

    int rx_socket;
    if ((rx_socket = socket(res->ai_family, res->ai_socktype, res->ai_protocol)) == -1)
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "Lcnc: Unable to create Rx socket: %s\n", strerror(errno));
        freeaddrinfo(res);
        free(conn);
        return NULL;
    }
    if (bind(rx_socket, (struct sockaddr*)&si_me, sizeof(si_me)) == -1) 
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "Lcnc: Unable to bind Rx socket to port: %s\n", strerror(errno));
        close(rx_socket);
        freeaddrinfo(res);
        free(conn);
        return NULL;
    }
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = RECV_TIMEOUT_US;
    err = setsockopt(rx_socket, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(timeout));
    if (err < 0) 
    {
        rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: could NOT to set setsockopt for tx\n");
        free(conn);
        return NULL;
    }
    // Tx half
    int tx_socket = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (tx_socket == -1) 
    {
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
    if (err < 0) 
    {
        rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: could NOT to set setsockopt for tx\n");
        free(conn);
        return NULL;
    }

    conn->read_fd = rx_socket;
    conn->fd = tx_socket;
    conn->addr = res;

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

double get_timestamp()
{
    struct timespec spec;

    if(spec_old.tv_sec==0)
    {
        clock_gettime(CLOCK_REALTIME, &spec_old);
    }
    clock_gettime(CLOCK_REALTIME, &spec);
    return ((double)(spec.tv_sec-spec_old.tv_sec)+(double)(spec.tv_nsec-spec_old.tv_nsec)/1000000000.0);
}


/***********************************************************************
*                       INIT AND EXIT CODE                             *
************************************************************************/

int rtapi_app_main(void)
{
    char name[HAL_NAME_LEN + 1];
    int i,r ;
    uint_fast32_t tempvalue;
    union EbData 
    {
           uint32_t value;
           uint8_t bytes[4];
    } temp_reg_value; 

    num_ports = 1;
    uint8_t tx_read_packet_buffer[EB_HEADER_SIZE+INIT_RX_PAYLOAD_SIZE];
    uint8_t tx_write_packet_buffer[EB_HEADER_SIZE+INIT_TX_PAYLOAD_SIZE];
    uint8_t rx_read_packet_buffer[EB_HEADER_SIZE+INIT_RX_PAYLOAD_SIZE];
    uint8_t init_ok=0;
    float timestamp,last_cycle_timestamp,first_timestamp;
    int count;

    // STEP 1: initialise the driver 
    comp_id = hal_init(DRIVER_NAME);
    if (comp_id < 0) 
    {
        rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: ERROR: hal_init() failed\n");
        goto fail0;
    }
    else rtapi_print_msg(RTAPI_MSG_INFO,"Lcnc:%f hal_init() ok\n",get_timestamp());

    // STEP 2: allocate shared memory for to_hal data
    device_data = hal_malloc(sizeof(data_hal));
    if (device_data == 0) 
    {
        rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: ERROR: hal_malloc() failed\n");
        r = -1;
        goto fail0;
    }
    else rtapi_print_msg(RTAPI_MSG_INFO ,"Lcnc:%f hal_malloc() ok\n",get_timestamp());

//###################################################
///// INIT ETH BOARD ; OPEN SOCKET
//###################################################
    device_data->eb = eb_connect(IP_ADDR, UDP_PORT);

    if (!device_data->eb)
    {
        rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: ERROR: failed to connect to board\n");
        goto fail1;
    }
    else rtapi_print_msg(RTAPI_MSG_INFO ,"Lcnc:%f connected to board \n",get_timestamp());
//###################################################
///// INIT ETH BOARD ; TRY TO CONNECT TO BOARD AND GATHER CONFIGURATON INFO
//###################################################

    
    // Step 1: Init register set to zero

    // Header setup
    eb_fill_header(tx_write_packet_buffer,0,1,INIT_REG_ADDR); // (buffer,is_read,number of read/writes,start address)
    // payload setup
    tempvalue=(uint32_t)(0x00000000);
    temp_reg_value.value=htobe32(tempvalue); // 
    memcpy((void*)(tx_write_packet_buffer+EB_HEADER_SIZE+INIT_REG_ADDR),(void*)temp_reg_value.bytes,4); 

    // #################  TRANSMIT DATA  ###############################################################
    eb_send(device_data->eb, tx_write_packet_buffer, EB_HEADER_SIZE+INIT_TX_PAYLOAD_SIZE);
    // #################################################################################################

    // Step 2: try to read signature from configuration register, in order to verify we are talking to 
    // a correctly configured board

    // Header setup
    eb_fill_header(tx_read_packet_buffer,1,2,0); // (buffer,is_read,number of read/writes,start address)
    // Payload setup
    // first step: try to read configuration register for signature and version
    temp_reg_value.value=htobe32(REG_START_REG_ADDR); 
    memcpy((void*)(tx_read_packet_buffer+EB_HEADER_SIZE+REG_START_REG_ADDR-REG_START_REG_ADDR),(void*)temp_reg_value.bytes,4);
    temp_reg_value.value=htobe32(CONFIGURATION_REG_ADDR); 
    memcpy((void*)(tx_read_packet_buffer+EB_HEADER_SIZE+CONFIGURATION_REG_ADDR-REG_START_REG_ADDR),(void*)temp_reg_value.bytes,4);
    
    first_timestamp=get_timestamp();
    while(init_ok<1 && (timestamp-first_timestamp)<10) // expire after 10 secs
    {

        // #################  TRANSMIT DATA  ###############################################################
        eb_send(device_data->eb, tx_read_packet_buffer, EB_HEADER_SIZE+INIT_RX_PAYLOAD_SIZE);
        // #################  RECEIVE DATA  ###############################################################

        int count = eb_recv(device_data->eb, rx_read_packet_buffer, sizeof(rx_read_packet_buffer));
        if (count == EB_HEADER_SIZE+INIT_RX_PAYLOAD_SIZE)
        {
            memcpy((void*)temp_reg_value.bytes,(void*)(rx_read_packet_buffer+EB_HEADER_SIZE+CONFIGURATION_REG_ADDR-REG_START_REG_ADDR),4);
            tempvalue=be32toh(temp_reg_value.value);
            init_ok++;
        }
        else 
        {
            last_cycle_timestamp=get_timestamp();
            fprintf(stderr, "Lcnc:%f init board, connection error- unexpected read length: %d, retry\n", last_cycle_timestamp, count);
            while(timestamp-last_cycle_timestamp<1) timestamp=get_timestamp(); // wait 1 sec
        }
    }


    // check if the value read is coherent
    if(init_ok==1)
    {
        intf_ver=(uint8_t)( tempvalue & 0x000000ff);
        if( intf_ver==1 && (uint8_t)((tempvalue & 0x0000ff00)>>8) =='n' && (uint8_t)((tempvalue & 0x00ff0000)>>16) == 'c'&&  (uint8_t)((tempvalue & 0xff000000)>>24 == 'L'))
        {
            init_ok++;
            fprintf(stderr, "Lcnc:%f detected board with interface version %d\n",get_timestamp(),intf_ver);
        }
    }

    // Step 3: set Init register to 0x55

    // Header setup
    eb_fill_header(tx_write_packet_buffer,0,1,INIT_REG_ADDR); // (buffer,is_read,number of read/writes,start address)
    // payload setup
    tempvalue=(uint32_t)(0x55L);
    temp_reg_value.value=htobe32(tempvalue); // watchdog
    memcpy((void*)(tx_write_packet_buffer+EB_HEADER_SIZE+INIT_REG_ADDR),(void*)temp_reg_value.bytes,4); 

    // #################  TRANSMIT DATA  ###############################################################
    eb_send(device_data->eb, tx_write_packet_buffer, EB_HEADER_SIZE+INIT_TX_PAYLOAD_SIZE);
    // #################################################################################################


    // Step 4: read back REGS_START register and also Configuration register

    // Header setup
    eb_fill_header(tx_read_packet_buffer,1,2,0); // (buffer,is_read,number of read/writes,start address)
    // Payload setup
    // read register start address register and configuration register for peripheral configuration
    temp_reg_value.value=htobe32(REG_START_REG_ADDR); 
    memcpy((void*)(tx_read_packet_buffer+EB_HEADER_SIZE+REG_START_REG_ADDR-REG_START_REG_ADDR),(void*)temp_reg_value.bytes,4);
    temp_reg_value.value=htobe32(CONFIGURATION_REG_ADDR); 
    memcpy((void*)(tx_read_packet_buffer+EB_HEADER_SIZE+CONFIGURATION_REG_ADDR-REG_START_REG_ADDR),(void*)temp_reg_value.bytes,4);

    first_timestamp=get_timestamp();
    while(init_ok==2 && (timestamp-first_timestamp)<10) // expire after 10 secs
    {

        // #################  TRANSMIT DATA  ###############################################################
        eb_send(device_data->eb, tx_read_packet_buffer, EB_HEADER_SIZE+INIT_RX_PAYLOAD_SIZE);
        // #################  RECEIVE DATA  ###############################################################

        int count = eb_recv(device_data->eb, rx_read_packet_buffer, sizeof(rx_read_packet_buffer));
        if (count == EB_HEADER_SIZE+INIT_RX_PAYLOAD_SIZE) 
        {
            memcpy((void*)temp_reg_value.bytes,(void*)(rx_read_packet_buffer+EB_HEADER_SIZE+CONFIGURATION_REG_ADDR-REG_START_REG_ADDR),4);
            tempvalue=be32toh(temp_reg_value.value);
            n_in  = (uint8_t)( tempvalue & 0x0000007f);
            n_out = (uint8_t)((tempvalue & 0x00003f80)>>7);
            n_sg  = (uint8_t)((tempvalue & 0x000fc000)>>14);
            n_en  = (uint8_t)((tempvalue & 0x03f00000)>>20);
            n_pwm = (uint8_t)((tempvalue & 0xfc000000)>>26);
            memcpy((void*)temp_reg_value.bytes,(void*)(rx_read_packet_buffer+EB_HEADER_SIZE+REG_START_REG_ADDR-REG_START_REG_ADDR),4);
            REGS_START_value=be32toh(temp_reg_value.value); 
            timestamp=get_timestamp();
            fprintf(stderr, "Lcnc:%f peripherals detected: \n%d inputs, \n%d outputs, \n%d stepgens, \n%d encoders, \n%d pwm \n",timestamp,n_in,n_out,n_sg,n_en,n_pwm);
            init_ok++;
        }
        else 
        {
            last_cycle_timestamp=get_timestamp();
            fprintf(stderr, "Lcnc:%f init board, connection error- unexpected read length: %d, retry\n", last_cycle_timestamp, count);
            while(timestamp-last_cycle_timestamp<1) timestamp=get_timestamp(); // wait 1 sec
        }
    }

    if(init_ok<3) //failed init
    {
    fprintf(stderr, "Lcnc:%f init failed, connection error\n", get_timestamp());
        r = -1;
        goto fail1;
    } 

/* calculate register map once at init
 * if a register address is equal to the previous in list, the register is not used
 * the address is incremented only when the functionality connected to the register is active,
 * the amount of increment depends on the size of the previous register set
 * */
    INIT_WRITE_addr     = 0x00;
    REGS_START_addr     = INIT_WRITE_addr+0x04;
    CONFIGURATION_addr  = REGS_START_addr+0x08;
    VELOCITY0_addr      = (n_sg>0?REGS_START_value:REGS_START_value-0x04);
    VELOCITYlast_addr   = VELOCITY0_addr+(n_sg>0?0x04*(n_sg-1):0);
    MAX_ACC0_addr       = VELOCITYlast_addr+(n_sg>0?0x04:0);
    MAX_ACClast_addr    = MAX_ACC0_addr+(n_sg>0?0x04*(n_sg-1):0);
    STEP_RES_EN_addr    = MAX_ACClast_addr+(n_sg>0?0x04:0);
    STEPDIRINV_addr     = STEP_RES_EN_addr+(n_sg>0?0x04:0);
    STEPTIMES_addr      = STEPDIRINV_addr+(n_sg>0?0x04:0);
    GPIOS_OUT_addr      = STEPTIMES_addr+(n_out>0?0x04:0);
    PWM_0_addr          = GPIOS_OUT_addr+(n_pwm>0?0x04:0);
    PWM_last_addr       = PWM_0_addr+(n_pwm>0?0x04*(n_pwm-1):0);
    ENC_RES_EN_addr     = PWM_last_addr+(n_en>0?0x04:0);
    RES_ST_REG_addr     = ENC_RES_EN_addr+0x04;
    SG_COUNT_0_addr     = RES_ST_REG_addr+(n_sg>0?0x04:0);
    SG_COUNT_last_addr  = SG_COUNT_0_addr+(n_sg>0?0x04*(n_sg-1):0);
    SG_VEL_0_addr       = SG_COUNT_last_addr+(n_sg>0?0x04:0);
    SG_VEL_last_addr    = SG_VEL_0_addr+(n_sg>0?0x04*(n_sg-1):0);
    WALLCLOCK_addr      = SG_VEL_last_addr+0x04;
    GPIOS_IN_addr       = WALLCLOCK_addr+(n_in>0?0x04:0);
    ENC_COUNT_0_addr    = GPIOS_IN_addr+(n_en>0?0x04:0);
    ENC_COUNT_last_addr = ENC_COUNT_0_addr+(n_en>0?0x04*(n_en-1):0);
    TX_SECTION_START_addr = VELOCITY0_addr;
    TX_SECTION_END_addr = RES_ST_REG_addr;
    TX_PAYLOAD_size     = TX_SECTION_END_addr+4-TX_SECTION_START_addr;
    RX_SECTION_START_addr = RES_ST_REG_addr;
    RX_SECTION_END_addr = ENC_COUNT_last_addr;
    RX_PAYLOAD_size     = RX_SECTION_END_addr+4-RX_SECTION_START_addr;
    
    //fprintf(stderr, "Lcnc register map:\nINIT_WRITE_addr %x\nCONFIGURATION_addr %x\nVELOCITY0_addr %x\nVELOCITYlast_addr %x\nMAX_ACC0_addr %x\nMAX_ACClast_addr %x\nSTEP_RES_EN_addr %x\nSTEPDIRINV_addr %x\nSTEPTIMES_addr %x\nGPIOS_OUT_addr %x\nPWM_0_addr %x\nPWM_last_addr %x\nENC_RES_EN_addr %x\nRES_ST_REG_addr %x\nSG_COUNT_0_addr %x\nSG_COUNT_last_addr %x\nSG_VEL_0_addr %x\nSG_VEL_last_addr %x\nWALLCLOCK_addr %x\nGPIOS_IN_addr %x\nENC_COUNT_0_addr %x\nENC_COUNT_last_addr %x\nTX_SECTION_START_addr %x\nTX_SECTION_END_addr %x\nRX_SECTION_START_addr %x\nRX_SECTION_END_addr %x\nLAST_addr %x\nTX_PAYLOAD_size %d bytes\nRX_PAYLOAD_size %d bytes\n",INIT_WRITE_addr,CONFIGURATION_addr,VELOCITY0_addr,VELOCITYlast_addr,MAX_ACC0_addr,MAX_ACClast_addr,STEP_RES_EN_addr,STEPDIRINV_addr,STEPTIMES_addr,GPIOS_OUT_addr,PWM_0_addr,PWM_last_addr,ENC_RES_EN_addr,RES_ST_REG_addr,SG_COUNT_0_addr,SG_COUNT_last_addr,SG_VEL_0_addr,SG_VEL_last_addr,WALLCLOCK_addr,GPIOS_IN_addr,ENC_COUNT_0_addr,ENC_COUNT_last_addr,TX_SECTION_START_addr,TX_SECTION_END_addr,RX_SECTION_START_addr,RX_SECTION_END_addr,LAST_addr,TX_PAYLOAD_size,RX_PAYLOAD_size);
    
//######################################################
//######### EXPORT SIGNALS, PIN, FUNCTION
//######################################################

// enable and reset
    r = hal_pin_bit_newf(HAL_IN, &(device_data->enable),comp_id, "Lcnc.%02d.enable", 0);
    if(r < 0) 
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "Lcnc: ERROR: enable pin export failed with err=%i\n",r);
        r = -1;
        goto fail1;
    }
    r = hal_pin_bit_newf(HAL_IN, &(device_data->enable_req),comp_id, "Lcnc.%02d.enable-request", 0);
    if(r < 0) 
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "Lcnc: ERROR: enable request pin export failed with err=%i\n",r);
        r = -1;
        goto fail1;
    }
    r = hal_pin_bit_newf(HAL_OUT, &(device_data->enabled),comp_id, "Lcnc.%02d.enabled", 0);
    if(r < 0) 
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "Lcnc: ERROR: enabled pin export failed with err=%i\n",r);
        r = -1;
        goto fail1;
    }
// tx retries timeout
    r = hal_param_u32_newf(HAL_RW, &(device_data->tx_max_retries),comp_id, "Lcnc.%02d.tx-max-retries", 0);
    if(r < 0) 
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "Lcnc: ERROR: max retries pin export failed with err=%i\n",r);
        r = -1;
        goto fail1;
    }
    device_data->tx_max_retries=(hal_u32_t)5;
// watchdog
    r = hal_pin_float_newf(HAL_OUT, &(device_data->watchdog_rd),comp_id, "Lcnc.%02d.watchdog-read", 0);
    if(r < 0) 
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "Lcnc: ERROR: watchdog read pin export failed with err=%i\n",r);
        r = -1;
        goto fail1;
    }
    r = hal_param_float_newf(HAL_RW, &(device_data->watchdog_wr),comp_id, "Lcnc.%02d.watchdog-write", 0);
    if(r < 0) 
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "Lcnc: ERROR: watchdog write pin export failed with err=%i\n",r);
        r = -1;
        goto fail1;
    }
    device_data->watchdog_wr=(hal_float_t)0.01;
// Inputs
    for(i=0;i<n_in;i++) 
    {
        r = hal_pin_bit_newf(HAL_OUT, &(device_data->digital_in[i]),comp_id, "Lcnc.%02d.input.%02d.in", 0, i);
        if(r < 0) 
        {
            rtapi_print_msg(RTAPI_MSG_ERR, "Lcnc: ERROR: input %d export failed with err=%i\n", i ,r);
            r = -1;
            goto fail1;
        }

        r = hal_pin_bit_newf(HAL_OUT, &(device_data->digital_in_n[i]),comp_id, "Lcnc.%02d.input.%02d.in-n", 0, i);
        if(r < 0) 
        {
            rtapi_print_msg(RTAPI_MSG_ERR, "Lcnc: ERROR: input-n %d export failed with err=%i\n", i,r);
            r = -1;
            goto fail1;
        }

    }

// outputs
    for(i=0;i<n_out;i++) 
    {
        r = hal_pin_bit_newf(HAL_IN, &(device_data->digital_out[i]),comp_id, "Lcnc.%02d.output.%02d.out", 0, i);
        if(r < 0) 
        {
            rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: ERROR: output %d export failed with err=%i\n", i,r);
            r = -1;
            goto fail1;
        }
        r = hal_param_bit_newf(HAL_RW, &(device_data->digital_out_inv[i]),comp_id, "Lcnc.%02d.output.%02d.inv", 0, i);
        if(r < 0) 
        {
            rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: ERROR: output inversion %d export failed with err=%i\n", i,r);
            r = -1;
            goto fail1;
        }
    }

// wallclock
    r = hal_pin_u32_newf(HAL_OUT, &(device_data->wallclock), comp_id, "Lcnc.%02d.wallclock.%02d.value", 0,0);
    if(r < 0) 
    {
        rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: ERROR: board_wallclock var export failed with err=%i\n",r);
        r = -1;
        goto fail1;
    }
    r = hal_pin_float_newf(HAL_OUT, &(device_data->wallclock_intvl), comp_id, "Lcnc.%02d.wallclock.%02d.interval", 0,0);
    if(r < 0) 
    {
        rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: ERROR: board_wallclock interval export failed with err=%i\n",r);
        r = -1;
        goto fail1;
    }

// encoders
    for(i=0;i<n_en;i++) 
    {// !! aggiustare la nomenclatura, si tratta di posizione
        // encoder_count
        r = hal_pin_float_newf(HAL_OUT, &(device_data-> enc_pos_fb[i]), comp_id, "Lcnc.%02d.encoder.%02d.pos-fb", 0, i);
        if(r < 0) 
        {
            rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: ERROR:  err encoder %i position feedback\n", r);
            r = -1;
            goto fail1;
        }
        r = hal_pin_bit_newf(HAL_IN, &(device_data-> enc_res[i]), comp_id, "Lcnc.%02d.encoder.%02d.reset", 0, i);
        if(r < 0) 
        {
            rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: ERROR:  err encoder %i reset\n", r);
            r = -1;
            goto fail1;
        }
        r = hal_pin_bit_newf(HAL_IN, &(device_data-> enc_en[i]), comp_id, "Lcnc.%02d.encoder.%02d.enable", 0, i);
        if(r < 0) 
        {
            rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: ERROR:  err encoder %i enable\n", r);
            r = -1;
            goto fail1;
        }
        r = hal_pin_float_newf(HAL_OUT, &(device_data-> enc_vel_fb[i]), comp_id, "Lcnc.%02d.encoder.%02d.vel-fb", 0, i);
        if(r < 0) 
        {
            rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: ERROR:  err encoder %i velocity\n", r);
            r = -1;
            goto fail1;
        }
        // encoder scale
        r = hal_param_float_newf(HAL_RW, &(device_data-> enc_scale[i]), comp_id, "Lcnc.%02d.encoder.%02d.scale", 0, i);
        if(r < 0) 
        {
            rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: ERROR:  err enc_scale %i scale\n", r);
            r = -1;
            goto fail1;
        }
        device_data-> enc_scale[i]=(hal_float_t)1.0;
        // encoder inversion
        r = hal_param_bit_newf(HAL_RW, &(device_data-> enc_inv[i]), comp_id, "Lcnc.%02d.encoder.%02d.inv", 0, i);
        if(r < 0) 
        {
            rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: ERROR:  err enc %i inversion\n", r);
            r = -1;
            goto fail1;
        }
    }

// stepgens
    for(i=0;i<n_sg;i++) 
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
    for(i=0;i<n_pwm;i++) 
    {
        r = hal_pin_float_newf(HAL_IN, &(device_data-> pwm_freq[i]), comp_id, "Lcnc.%02d.pwm.%02d.freq", 0, i);
        if(r < 0) 
        {
            rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: ERROR:  PWM %i frequency\n", r);
            r = -1;
            goto fail1;
        }
        r = hal_pin_float_newf(HAL_IN, &(device_data-> pwm_value[i]), comp_id, "Lcnc.%02d.pwm.%02d.value", 0, i);
        if(r < 0) 
        {
            rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: ERROR:  PWM %i value\n", r);
            r = -1;
            goto fail1;
        }
        r = hal_pin_bit_newf(HAL_IN, &(device_data->pwm_enable[i]), comp_id, "Lcnc.%02d.pwm.%02d.enable", 0, i);
        if(r < 0) 
        {
            rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: ERROR:  PWM %i enable\n", r);
            r = -1;
            goto fail1;
        }
        r = hal_param_float_newf(HAL_RW, &(device_data->pwm_scale[i]), comp_id, "Lcnc.%02d.pwm.%02d.scale", 0, i);
        if(r < 0) 
        {
            rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: ERROR:  PWM %i scale\n", r);
            r = -1;
            goto fail1;
        }
        device_data->pwm_scale[i]=(hal_float_t)100.0;
        
        r = hal_param_float_newf(HAL_RW, &(device_data-> pwm_offs[i]), comp_id, "Lcnc.%02d.pwm.%02d.offs", 0, i);
        if(r < 0) 
        {
            rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: ERROR:  PWM %i offset\n", r);
            r = -1;
            goto fail1;
        }
        
        r = hal_param_bit_newf(HAL_RW, &(device_data-> pwm_inv[i]), comp_id, "Lcnc.%02d.pwm.%02d.inv", 0, i);
        if(r < 0) 
        {
            rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: ERROR:  PWM %i inversion\n", r);
            r = -1;
            goto fail1;
        }
    }

    /* STEP 4: export function */
    rtapi_snprintf(name, sizeof(name), "Lcnc.%02d.update", 0);
    r = hal_export_funct(name, update_port, device_data, 1, 0,comp_id);
    if(r < 0) 
    {
        rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: ERROR: port %d write funct export failed\n", 1);
        r = -1;
        goto fail1;
    }

    rtapi_print_msg(RTAPI_MSG_INFO,"Lcnc: installed driver for %d card(s)\n", num_ports);
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
    static int_fast64_t internal_step_count[MAX_N_STEPGENS];
    static uint_fast32_t step_count[MAX_N_STEPGENS];
    static uint_fast32_t step_count_old[MAX_N_STEPGENS];
    static int_fast64_t internal_enc_count[MAX_N_ENCODERS];
    static uint_fast32_t enc_count[MAX_N_ENCODERS];
    static uint_fast32_t enc_count_old[MAX_N_ENCODERS];
    static uint8_t conn_err_notified=0;
    static uint8_t enable_err_notified=0;
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
    const float velfact=pow(2,VEL_SIZE_BITS)/F_FPGA; // velocity factor that is 2^32/F_FPGA
    uint8_t tx_read_packet_buffer[EB_HEADER_SIZE+RX_PAYLOAD_MAX_SIZE];
    uint8_t rx_read_packet_buffer[EB_HEADER_SIZE+RX_PAYLOAD_MAX_SIZE];
    uint8_t tx_write_packet_buffer[EB_HEADER_SIZE+TX_PAYLOAD_MAX_SIZE];
    uint32_t temp_period_u=0,temp_width_u=0;
    
//----------------------------------
// Start of read section
//----------------------------------

    // !! prepare packet for read registers one by one from here
    // Header setup
    if(!read_header_ready)
    {
        eb_fill_header(tx_read_packet_buffer,1,RX_PAYLOAD_size/4,0); // (buffer,is_read,number of read/writes,start address)
        read_header_ready=1;
    }
    // Payload setup
    // read reset and status register
    temp_reg_value.value=htobe32(RES_ST_REG_addr); 
    memcpy((void*)(tx_read_packet_buffer+EB_HEADER_SIZE+RES_ST_REG_addr-RX_SECTION_START_addr),(void*)temp_reg_value.bytes,4);
    // read stepgen position
    for(i=0;i<n_sg;i++)
    {
        temp_reg_value.value=htobe32(SG_COUNT_0_addr+4*i);
        memcpy((void*)(tx_read_packet_buffer+EB_HEADER_SIZE+SG_COUNT_0_addr-RX_SECTION_START_addr+4*i),(void*)temp_reg_value.bytes,4);
    }
    // read stepgen velocity
    for(i=0;i<n_sg;i++)
    {
        temp_reg_value.value=htobe32(SG_VEL_0_addr+4*i);
        memcpy((void*)(tx_read_packet_buffer+EB_HEADER_SIZE+SG_VEL_0_addr-RX_SECTION_START_addr+4*i),(void*)temp_reg_value.bytes,4);
    }
    // read wall clock
    temp_reg_value.value=htobe32(WALLCLOCK_addr); 
    memcpy((void*)(tx_read_packet_buffer+EB_HEADER_SIZE+WALLCLOCK_addr-RX_SECTION_START_addr),(void*)temp_reg_value.bytes,4);
    // read gpio in
    temp_reg_value.value=htobe32(GPIOS_IN_addr); 
    memcpy((void*)(tx_read_packet_buffer+EB_HEADER_SIZE+GPIOS_IN_addr-RX_SECTION_START_addr),(void*)temp_reg_value.bytes,4);
    // read encoders
    for(i=0;i<n_en;i++)
    {
        temp_reg_value.value=htobe32(ENC_COUNT_0_addr+4*i); 
        memcpy((void*)(tx_read_packet_buffer+EB_HEADER_SIZE+ENC_COUNT_0_addr-RX_SECTION_START_addr+4*i),(void*)temp_reg_value.bytes,4);
    }

    // #################  TRANSMIT DATA  ###############################################################
    res=eb_send(port->eb, tx_read_packet_buffer, EB_HEADER_SIZE+RX_PAYLOAD_size);

    // #################  RECEIVE DATA  ###############################################################

    int count = eb_recv(port->eb, rx_read_packet_buffer, sizeof(rx_read_packet_buffer));
    if (count != EB_HEADER_SIZE+RX_PAYLOAD_size) 
    {
        (port->num_errors_reported)++;
        if(conn_err_notified==0)
        {
            conn_err_notified=1;
            fprintf(stderr, "Lcnc:%f connection error- unexpected read length: %d\n errors %d max %d\n", get_timestamp(), count,(port->num_errors_reported), (port->tx_max_retries));
        }
    }

    else 
    {
        if((port->num_errors_reported)>0)
        {
            conn_err_notified=0;
            fprintf(stderr, "Lcnc:%f connection restored, lost %d packets, limit is set to %d\n",get_timestamp(), port->num_errors_reported, (port->tx_max_retries));
            (port->num_errors_reported)=0;
        }
    
        // Reset and status reg
        // read watchdog value
        memcpy((void*)temp_reg_value.bytes,(void*)(rx_read_packet_buffer+EB_HEADER_SIZE+RES_ST_REG_addr-RX_SECTION_START_addr),4);
        tempvalue=be32toh(temp_reg_value.value);
        port->watchdog_rd_old=*(port->watchdog_rd);
        *(port->watchdog_rd)=(hal_float_t)(tempvalue >> CSR_MMIO_INST_RES_ST_REG_WATCHDOG_OFFSET)*WDT_SCALE;
        // gpio in
        memcpy((void*)temp_reg_value.bytes,(void*)(rx_read_packet_buffer+EB_HEADER_SIZE+GPIOS_IN_addr-RX_SECTION_START_addr),4);
        tempvalue=be32toh(temp_reg_value.value);
        for (i=0;i<n_in;i++) 
        {
            if (tempvalue & 1<<i) *(port->digital_in[i]) = 1 ;
            else *(port->digital_in[i]) = 0;
            *(port->digital_in_n[i])=!(*(port->digital_in[i]));
        }
        // wallclock
        memcpy((void*)temp_reg_value.bytes,(void*)(rx_read_packet_buffer+EB_HEADER_SIZE+WALLCLOCK_addr-RX_SECTION_START_addr),4);
        tempvalue=be32toh(temp_reg_value.value);
        port->wallclock_old=*(port->wallclock);
        *(port->wallclock)=tempvalue;
        if((*(port->wallclock))>(port->wallclock_old))
            *(port->wallclock_intvl)=((float)(*(port->wallclock)-(port->wallclock_old)))/F_FPGA;
        else *(port->wallclock_intvl)=((float)((port->wallclock_old)-*(port->wallclock)))/F_FPGA;
        // steppers position and velocity feedback
        // introdurre calcolo di velocità basato sulla differenza tra le posizioni nel tempo (usando il wallclock)
        for (i=0;i<n_sg;i++)
        {
            memcpy((void*)temp_reg_value.bytes,(void*)(rx_read_packet_buffer+EB_HEADER_SIZE+SG_COUNT_0_addr-RX_SECTION_START_addr+4*i),4);
            tempvalue=be32toh(temp_reg_value.value);
            step_count_old[i]=step_count[i]; // save last counter value
            step_count[i]=tempvalue;
            if(*(device_data->stepgen_reset[i])) internal_step_count[i]=0;
            internal_step_count[i]+=((int32_t)(step_count[i])-(int32_t)step_count_old[i]);
            *(port->stepgen_position_fb[i])=((hal_float_t)(internal_step_count[i]))/(port->stepgen_scale[i]);
            memcpy((void*)temp_reg_value.bytes,(void*)(rx_read_packet_buffer+EB_HEADER_SIZE+SG_VEL_0_addr-RX_SECTION_START_addr+4*i),4);
            tempvalue=be32toh(temp_reg_value.value);
            *(port->stepgen_velocity_fb[i])=((hal_float_t)((int32_t)tempvalue))/((port->stepgen_scale[i])*velfact);
        }

        ///////////// read encoders /////////////////////////////////////////////

        for(i=0;i<n_en;i++)
        {
            memcpy((void*)temp_reg_value.bytes,(void*)(rx_read_packet_buffer+EB_HEADER_SIZE+ENC_COUNT_0_addr-RX_SECTION_START_addr+4*i),4);
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
    }
    // if watchdog bites or not enabled or tx errors
    if(*(port->watchdog_rd)==0 || *(port->enable)==0 || ((port->num_errors_reported)>(port->tx_max_retries))) 
    {
        *(port->enabled)=0;
        if(*(port->enable)!=0 && enable_err_notified==0)
        {
            enable_err_notified=1;
            if(*(port->watchdog_rd)==0) 
                rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc:%f driver disabled due to watchdog timer expiring, limit is set to %f\n",get_timestamp(),(port->watchdog_wr));
            else
                rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc:%f driver disabled due transmission maximum retries reached, limit is set to %d\n",get_timestamp(),(port->tx_max_retries));
        }
    }
    else
    {
        if(*(port->enable_req) && !port->enable_req_old && *(port->enable))
        {   
        enable_err_notified=0;
        *(port->enabled)=1;
        }
    }
    port->enable_req_old=*(port->enable_req);
    

//----------------------------------
// End of read section
//----------------------------------

//----------------------------------
// Start of write section
//----------------------------------
    
    // Header setup
    if(!write_header_ready)
    {
        eb_fill_header(tx_write_packet_buffer,0,TX_PAYLOAD_size/4,TX_SECTION_START_addr); // (buffer,is_read,number of read/writes,start address)
        write_header_ready=1;
    }
    // Payload setup
    float max_freq,freq_req;
    uint8_t accel_mult;
    double cmdtmp;
    for (i=0;i<n_sg;i++)
    {
        // maximum frequency: limited by minimum step period, that is the sum of pulse width and space width
        max_freq=1.0E6/(port->stepgen_step_width + port->stepgen_step_space); //[Hz]
        // requested frequency of step generator
        freq_req=(*(port->stepgen_velocity_cmd[i]))*(port->stepgen_scale[i]);
        // limit frequency requested to the maximum available
        if(freq_req>max_freq) freq_req=max_freq;
        else if(freq_req<-max_freq) freq_req=-max_freq;
        // convert velocity request to step generator internal command word
        cmdtmp=freq_req*velfact;
        temp_reg_value.value=htobe32((int32_t)cmdtmp); // VELOCITY
        memcpy((void*)(tx_write_packet_buffer+EB_HEADER_SIZE+VELOCITY0_addr-TX_SECTION_START_addr+4*i),(void*)temp_reg_value.bytes,4);
        // convert maximum acceleration to step generator internal command word
        cmdtmp=(double)(*(port->stepgen_acc_lim[i]))*(double)velfact*(double)velfact/(double)4*(double)(port->stepgen_scale[i]);// the factor is the same of velocity but squared
                                                                                            // the "/4" is due to the fact that acceleration 
                                                                                            // command is 30bits instead of 32
        if(cmdtmp<0) cmdtmp=-cmdtmp; // only positive acceleration
        // if command overflows number of bits allocated, is reduced 8-fold and acc_mult incremented by one until command does not overflows or acc_mult reaches the maximum allowed value 
        for(
        accel_mult=0;
        cmdtmp>((1<<(CSR_MMIO_INST_MAX_ACC_0_ACC_SIZE))-1) && accel_mult<((1<<(CSR_MMIO_INST_MAX_ACC_0_ACC_MULT_SIZE))-1);
        accel_mult++
        ) 
            cmdtmp=cmdtmp/(1<<(ACC_MULT_EXP));
            // if command still overflows, is saturated
        if(cmdtmp>((1<<(CSR_MMIO_INST_MAX_ACC_0_ACC_SIZE))-1)) cmdtmp=((1<<(CSR_MMIO_INST_MAX_ACC_0_ACC_SIZE))-1);
        tempvalue=(uint32_t)(cmdtmp);
        tempvalue|=((uint32_t)accel_mult)<<(CSR_MMIO_INST_MAX_ACC_0_ACC_MULT_OFFSET);
        temp_reg_value.value=htobe32(tempvalue); // Maximum Acceleration
        memcpy((void*)(tx_write_packet_buffer+EB_HEADER_SIZE+MAX_ACC0_addr-TX_SECTION_START_addr+4*i),(void*)temp_reg_value.bytes,4);
    }    

    tempvalue=0;
    for (i=0;i<n_sg;i++)
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
    memcpy((void*)(tx_write_packet_buffer+EB_HEADER_SIZE+STEP_RES_EN_addr-TX_SECTION_START_addr),(void*)temp_reg_value.bytes,4);    
    
    tempvalue=0;
    for (i=0;i<n_sg;i++)
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
    memcpy((void*)(tx_write_packet_buffer+EB_HEADER_SIZE+STEPDIRINV_addr-TX_SECTION_START_addr),(void*)temp_reg_value.bytes,4);
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
    memcpy((void*)(tx_write_packet_buffer+EB_HEADER_SIZE+STEPTIMES_addr-TX_SECTION_START_addr),(void*)temp_reg_value.bytes,4);
    // GPIOS_OUT
    tempvalue=0;
    for (i=0;i<n_out;i++)
    {
        if ((*(port->digital_out[i])) ^ (port->digital_out_inv[i])) tempvalue|=(1<<i);
    }
    temp_reg_value.value=htobe32(tempvalue); 
    memcpy((void*)(tx_write_packet_buffer+EB_HEADER_SIZE+GPIOS_OUT_addr-TX_SECTION_START_addr),(void*)temp_reg_value.bytes,4);
    // PWM_0
    for(i=0;i<n_pwm;i++)
    {
        if(*(port->pwm_enable[i]) && *(port->pwm_freq[i])>0 && *(port->pwm_value[i])>=0)
        {
            float temp_period,temp_width;
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
            memcpy((void*)(tx_write_packet_buffer+EB_HEADER_SIZE+PWM_0_addr-TX_SECTION_START_addr+4*i),(void*)temp_reg_value.bytes,4);
    }
    tempvalue=0;
    for (i=0;i<n_en;i++)
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
    memcpy((void*)(tx_write_packet_buffer+EB_HEADER_SIZE+ENC_RES_EN_addr-TX_SECTION_START_addr),(void*)temp_reg_value.bytes,4); 
    // check max value
    tempvalue=(uint32_t)((port->watchdog_wr)/WDT_SCALE);
    tempvalue=tempvalue<<CSR_MMIO_INST_RES_ST_REG_WATCHDOG_OFFSET;
    temp_reg_value.value=htobe32(tempvalue); // watchdog
    memcpy((void*)(tx_write_packet_buffer+EB_HEADER_SIZE+RES_ST_REG_addr-TX_SECTION_START_addr),(void*)temp_reg_value.bytes,4);   

    // #################  TRANSMIT DATA  ###############################################################
    res=eb_send(port->eb, tx_write_packet_buffer, EB_HEADER_SIZE+TX_PAYLOAD_size);
    // #################################################################################################
}

