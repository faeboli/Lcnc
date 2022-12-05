
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

static int num_boards;
#define MAX_CHAN 4
char *ipaddr[MAX_CHAN] ={0,};
char *udpport[MAX_CHAN] ={0,};
int debug=0;
RTAPI_MP_ARRAY_STRING(ipaddr, MAX_CHAN,"ip addresses");
RTAPI_MP_ARRAY_STRING(udpport, MAX_CHAN,"udp ports");
RTAPI_MP_INT(debug, "set to 1 to enable debug data");

/***********************************************************************
*                STRUCTURES AND GLOBAL VARIABLES                       *
************************************************************************/

/* globals */
data_hal* device_data_array[MAX_CHAN];
static int comp_id;     /* component ID */
static struct timespec spec_old,spec_old_1;

/***********************************************************************
*                       INIT AND EXIT CODE                             *
************************************************************************/

int rtapi_app_main(void)
{
    int i;

    /*
     * check the number of boards requested on the command line for example
     * loadrt Lcnc ipaddr="192.168.2.50","192.168.2.51" udpport="1234","1235"
     * will try to connect to two boards with the ip addresses and ports given as parameters 
     * */
    for (i = 0; i < MAX_CHAN; i++) 
    {
        if ( (ipaddr[i] == NULL) || (*ipaddr[i] == 0) || (udpport[i] == NULL) || (*udpport[i] == 0) ) break;
        num_boards = i + 1;
        rtapi_print_msg(RTAPI_MSG_ERR ,"Lcnc:%f requested board: %d with ip: %s port: %s\n",get_timestamp_f(),num_boards-1,ipaddr[i],udpport[i]);
    }
    
    // initialise the driver 
    comp_id = hal_init(DRIVER_NAME);
    if (comp_id < 0) 
    {
        rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc:%f ERROR: hal_init() failed\n",get_timestamp_f());
        return -1; 
    }
    else rtapi_print_msg(RTAPI_MSG_INFO,"Lcnc:%f hal_init() ok\n",get_timestamp_f());

    for(i=0;i<num_boards;i++)
    {
        //allocate shared memory for hal data
        device_data_array[i] = hal_malloc(sizeof(data_hal));
        if (device_data_array[i] == 0) 
        {
            rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc:%f ERROR: hal_malloc() failed for board %d\n",get_timestamp_f(),i);
            hal_exit(comp_id);
            return -1;    
        }
        else 
        {
            rtapi_print_msg(RTAPI_MSG_INFO ,"Lcnc:%f hal_malloc() ok for board %d\n",get_timestamp_f(),i);
            // initialize some data
            device_data_array[i]->board_id=i;
            sprintf(device_data_array[i]->ip_board_address,"%s",ipaddr[i]);
            sprintf(device_data_array[i]->udp_port,"%s",udpport[i]);
            device_data_array[i]->conn_err_notified=0;
            device_data_array[i]->enable_err_notified=0;
        }
        //configure boards pins and functions
        if (configure_board(device_data_array[i])!=0) 
        {
            rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc:%f ERROR: unable to configure board %d\n",get_timestamp_f(),i);
            hal_exit(comp_id);
            return -1;    
        }
        else 
        {
            rtapi_print_msg(RTAPI_MSG_INFO ,"Lcnc:%f board %d configured correctly\n",get_timestamp_f(),i);
        }
    }
    //everything ready
    hal_ready(comp_id);
}
//####### EXIT ############
void rtapi_app_exit(void)
{
//    eb_disconnect(&(device_data->eb));
    hal_exit(comp_id);
}

/*
 * configure each board
 * */
int configure_board(data_hal* device_data)
{
    char name[HAL_NAME_LEN + 1];
    uint8_t tx_read_packet_buffer[EB_HEADER_SIZE+INIT_RX_PAYLOAD_SIZE];
    uint8_t tx_write_packet_buffer[EB_HEADER_SIZE+INIT_TX_PAYLOAD_SIZE];
    uint8_t rx_read_packet_buffer[EB_HEADER_SIZE+INIT_RX_PAYLOAD_SIZE];
    uint_fast32_t tempvalue;
    union EbData 
    {
           uint32_t value;
           uint8_t bytes[4];
    } temp_reg_value; 
    conf_st_type conf_st=ConfInit;
    int conf_retry,conf_retry1;
    float timestamp;
    int count,r,i;

/*
 *  INIT ETH BOARD ; OPEN SOCKET
 * */
 
    device_data->eb = eb_connect(device_data->ip_board_address,device_data->udp_port);

    if (!device_data->eb)
    {
        rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc:%f ERROR: failed to connect to board %d\n",get_timestamp_f(),device_data->board_id);
        return -1;
    }
    else rtapi_print_msg(RTAPI_MSG_ERR ,"Lcnc:%f connected to board %d\n",get_timestamp_f(),device_data->board_id);


/*
 *  INIT ETH BOARD ; TRY TO CONNECT TO BOARD AND GATHER CONFIGURATON INFO
 * */

    while(conf_st!=ConfSuccess)
    {
        if(conf_st==ConfInit)
        {
            // Init register set to zero
            // Header setup
            eb_fill_header(tx_write_packet_buffer,0,1,INIT_REG_ADDR); // (buffer,is_read,number of read/writes,start address)
            // payload setup
            tempvalue=(uint32_t)(0x00000000);
            temp_reg_value.value=htobe32(tempvalue); // 
            memcpy((void*)(tx_write_packet_buffer+EB_HEADER_SIZE+INIT_REG_ADDR),(void*)temp_reg_value.bytes,4); 

            // #################  TRANSMIT DATA  ###############################################################
            eb_send(device_data->eb, tx_write_packet_buffer, EB_HEADER_SIZE+INIT_TX_PAYLOAD_SIZE);
            // #################################################################################################
            // try to read signature from configuration register, in order to verify we are talking to 
            // a correctly configured board
            // Header setup
            eb_fill_header(tx_read_packet_buffer,1,2,0); // (buffer,is_read,number of read/writes,start address)
            // Payload setup
            // first step: try to read configuration register for signature and version
            temp_reg_value.value=htobe32(REG_START_REG_ADDR); 
            memcpy((void*)(tx_read_packet_buffer+EB_HEADER_SIZE+REG_START_REG_ADDR-REG_START_REG_ADDR),(void*)temp_reg_value.bytes,4);
            temp_reg_value.value=htobe32(CONFIGURATION_REG_ADDR); 
            memcpy((void*)(tx_read_packet_buffer+EB_HEADER_SIZE+CONFIGURATION_REG_ADDR-REG_START_REG_ADDR),(void*)temp_reg_value.bytes,4);
            // #################  TRANSMIT DATA  ###############################################################
            eb_send(device_data->eb, tx_read_packet_buffer, EB_HEADER_SIZE+INIT_RX_PAYLOAD_SIZE);
        
            // move to next state
            conf_retry=0;
            conf_st=ConfReadSign;
        }
        else if(conf_st==ConfReadSign)
        {
            // #################  RECEIVE DATA  ###############################################################
            int count = eb_recv(device_data->eb, rx_read_packet_buffer, sizeof(rx_read_packet_buffer));
            if (count == EB_HEADER_SIZE+INIT_RX_PAYLOAD_SIZE)
            {
                memcpy((void*)temp_reg_value.bytes,(void*)(rx_read_packet_buffer+EB_HEADER_SIZE+CONFIGURATION_REG_ADDR-REG_START_REG_ADDR),4);
                tempvalue=be32toh(temp_reg_value.value);
                device_data->intf_ver=(uint8_t)( tempvalue & 0x000000ff);
                if(device_data->intf_ver==1 && (uint8_t)((tempvalue & 0x0000ff00)>>8) =='n' && (uint8_t)((tempvalue & 0x00ff0000)>>16) == 'c'&&  (uint8_t)((tempvalue & 0xff000000)>>24 == 'L'))
                {
                    rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc:%f board %d detected with interface version %d\n",get_timestamp_f(),device_data->board_id,device_data->intf_ver);
                    // move to next state
                    conf_retry1=0;
                    conf_st=ConfSetReg;
                }
                else 
                {
                    timestamp=get_timestamp_f();
                    rtapi_print_msg(RTAPI_MSG_ERR ,"Lcnc:%f value error, send retry %d\n",get_timestamp_f(),conf_retry1);
                    if(conf_retry1<5) 
                    {
                        conf_retry1++;
                        while(get_timestamp_f()-timestamp<0.1); // wait
                        conf_st=ConfInit;
                    }
                    else conf_st=ConfFail; // move to next state 
                }
            }
            else 
            {
                timestamp=get_timestamp_f();
                rtapi_print_msg(RTAPI_MSG_ERR ,"Lcnc:%f init board, connection error- unexpected read length: %d, retry %d\n",get_timestamp_f(),count,conf_retry);
                if(conf_retry<11) 
                {
                    conf_retry++;
                    while(get_timestamp_f()-timestamp<0.1); // wait
                }
                else 
                {
                    if(conf_retry1<5) 
                    {
                        conf_retry1++;
                        timestamp=get_timestamp_f();
                        rtapi_print_msg(RTAPI_MSG_ERR ,"Lcnc:%f send retry %d\n",get_timestamp_f(),conf_retry1+1);
                        while(get_timestamp_f()-timestamp<0.1); // wait
                        conf_st=ConfInit;
                    }
                    else conf_st=ConfFail; // move to next state 
                }
            }
        }
        else if(conf_st==ConfSetReg)
        {
            //set Init register to 0x55
            // Header setup
            eb_fill_header(tx_write_packet_buffer,0,1,INIT_REG_ADDR); // (buffer,is_read,number of read/writes,start address)
            // payload setup
            tempvalue=(uint32_t)(0x55L);
            temp_reg_value.value=htobe32(tempvalue); // watchdog
            memcpy((void*)(tx_write_packet_buffer+EB_HEADER_SIZE+INIT_REG_ADDR),(void*)temp_reg_value.bytes,4); 

            // #################  TRANSMIT DATA  ###############################################################
            eb_send(device_data->eb, tx_write_packet_buffer, EB_HEADER_SIZE+INIT_TX_PAYLOAD_SIZE);
            // #################################################################################################
            //Read back REGS_START register and also Configuration register
            // a correctly configured board
            // Header setup
            eb_fill_header(tx_read_packet_buffer,1,2,0); // (buffer,is_read,number of read/writes,start address)
            // Payload setup
            // first step: try to read configuration register for signature and version
            temp_reg_value.value=htobe32(REG_START_REG_ADDR); 
            memcpy((void*)(tx_read_packet_buffer+EB_HEADER_SIZE+REG_START_REG_ADDR-REG_START_REG_ADDR),(void*)temp_reg_value.bytes,4);
            temp_reg_value.value=htobe32(CONFIGURATION_REG_ADDR); 
            memcpy((void*)(tx_read_packet_buffer+EB_HEADER_SIZE+CONFIGURATION_REG_ADDR-REG_START_REG_ADDR),(void*)temp_reg_value.bytes,4);
            // #################  TRANSMIT DATA  ###############################################################
            eb_send(device_data->eb, tx_read_packet_buffer, EB_HEADER_SIZE+INIT_RX_PAYLOAD_SIZE);

            // move to next state
            conf_retry=0;
            conf_st=ConfReadConf;
        }
        else if(conf_st==ConfReadConf)
        {
            // #################  RECEIVE DATA  ###############################################################
            int count = eb_recv(device_data->eb, rx_read_packet_buffer, sizeof(rx_read_packet_buffer));
            if (count == EB_HEADER_SIZE+INIT_RX_PAYLOAD_SIZE)
            {
                memcpy((void*)temp_reg_value.bytes,(void*)(rx_read_packet_buffer+EB_HEADER_SIZE+CONFIGURATION_REG_ADDR-REG_START_REG_ADDR),4);
                tempvalue=be32toh(temp_reg_value.value);
                device_data->n_in  = (uint8_t)( tempvalue & 0x0000007f);
                device_data->n_out = (uint8_t)((tempvalue & 0x00003f80)>>7);
                device_data->n_sg  = (uint8_t)((tempvalue & 0x000fc000)>>14);
                device_data->n_en  = (uint8_t)((tempvalue & 0x03f00000)>>20);
                device_data->n_pwm = (uint8_t)((tempvalue & 0xfc000000)>>26);
                memcpy((void*)temp_reg_value.bytes,(void*)(rx_read_packet_buffer+EB_HEADER_SIZE+REG_START_REG_ADDR-REG_START_REG_ADDR),4);
                device_data->REGS_START_value=be32toh(temp_reg_value.value); 
                rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc:%f board %d peripherals detected: \n%d inputs, \n%d outputs, \n%d stepgens, \n%d encoders, \n%d pwm \n",get_timestamp_f(),device_data->board_id,device_data->n_in,device_data->n_out,device_data->n_sg,device_data->n_en,device_data->n_pwm);
                // move to next state
                conf_st=ConfSuccess;
            }
            else 
            {
                timestamp=get_timestamp_f();
                rtapi_print_msg(RTAPI_MSG_ERR ,"Lcnc:%f init board %d, connection error- unexpected read length: %d, retry %d\n",get_timestamp_f(),device_data->board_id,count,conf_retry+1);
                if(conf_retry<11) 
                {
                    conf_retry++;
                    while(get_timestamp_f()-timestamp<0.1); // wait
                }
                else
                {
                    timestamp=get_timestamp_f();
                    rtapi_print_msg(RTAPI_MSG_ERR ,"Lcnc:%f value error, retry %d\n",get_timestamp_f(),conf_retry1+1);
                    if(conf_retry1<5) 
                    {
                        conf_retry1++;
                        while(get_timestamp_f()-timestamp<0.1); // wait
                        conf_st=ConfSetReg;
                    }
                    else conf_st=ConfFail; // move to next state 
                }
            }
        }
        else if(conf_st==ConfFail)
        {
            fprintf(stderr, "Lcnc:%f board %d init failed, connection error\n",get_timestamp_f(),device_data->board_id);
            return -1;
        }
    }

/* calculate register map once at init
 * if a register address is equal to the previous in list, the register is not used
 * the address is incremented only when the functionality connected to the register is active,
 * the amount of increment depends on the size of the previous register set
 * */
    device_data->INIT_WRITE_addr     = 0x00;
    device_data->REGS_START_addr     = device_data->INIT_WRITE_addr+0x04;
    device_data->CONFIGURATION_addr  = device_data->REGS_START_addr+0x08;
    device_data->VELOCITY0_addr      = (device_data->n_sg>0?device_data->REGS_START_value:device_data->REGS_START_value-0x04);
    device_data->VELOCITYlast_addr   = device_data->VELOCITY0_addr+(device_data->n_sg>0?0x04*(device_data->n_sg-1):0);
    device_data->MAX_ACC0_addr       = device_data->VELOCITYlast_addr+(device_data->n_sg>0?0x04:0);
    device_data->MAX_ACClast_addr    = device_data->MAX_ACC0_addr+(device_data->n_sg>0?0x04*(device_data->n_sg-1):0);
    device_data->STEP_RES_EN_addr    = device_data->MAX_ACClast_addr+(device_data->n_sg>0?0x04:0);
    device_data->STEPDIRINV_addr     = device_data->STEP_RES_EN_addr+(device_data->n_sg>0?0x04:0);
    device_data->STEPTIMES_addr      = device_data->STEPDIRINV_addr+(device_data->n_sg>0?0x04:0);
    device_data->GPIOS_OUT_addr      = device_data->STEPTIMES_addr+(device_data->n_out>0?0x04:0);
    device_data->PWM_0_addr          = device_data->GPIOS_OUT_addr+(device_data->n_pwm>0?0x04:0);
    device_data->PWM_last_addr       = device_data->PWM_0_addr+(device_data->n_pwm>0?0x04*(device_data->n_pwm-1):0);
    device_data->ENC_RES_EN_addr     = device_data->PWM_last_addr+(device_data->n_en>0?0x04:0);
    device_data->RES_ST_REG_addr     = device_data->ENC_RES_EN_addr+0x04;
    device_data->SG_COUNT_0_addr     = device_data->RES_ST_REG_addr+(device_data->n_sg>0?0x04:0);
    device_data->SG_COUNT_last_addr  = device_data->SG_COUNT_0_addr+(device_data->n_sg>0?0x04*(device_data->n_sg-1):0);
    device_data->SG_VEL_0_addr       = device_data->SG_COUNT_last_addr+(device_data->n_sg>0?0x04:0);
    device_data->SG_VEL_last_addr    = device_data->SG_VEL_0_addr+(device_data->n_sg>0?0x04*(device_data->n_sg-1):0);
    device_data->WALLCLOCK_addr      = device_data->SG_VEL_last_addr+0x04;
    device_data->GPIOS_IN_addr       = device_data->WALLCLOCK_addr+(device_data->n_in>0?0x04:0);
    device_data->ENC_COUNT_0_addr    = device_data->GPIOS_IN_addr+(device_data->n_en>0?0x04:0);
    device_data->ENC_COUNT_last_addr = device_data->ENC_COUNT_0_addr+(device_data->n_en>0?0x04*(device_data->n_en-1):0);
    device_data->TX_SECTION_START_addr = device_data->VELOCITY0_addr;
    device_data->TX_SECTION_END_addr = device_data->RES_ST_REG_addr;
    device_data->TX_PAYLOAD_size     = device_data->TX_SECTION_END_addr+4-device_data->TX_SECTION_START_addr;
    device_data->RX_SECTION_START_addr = device_data->RES_ST_REG_addr;
    device_data->RX_SECTION_END_addr = device_data->ENC_COUNT_last_addr;
    device_data->RX_PAYLOAD_size     = device_data->RX_SECTION_END_addr+4-device_data->RX_SECTION_START_addr;
    if(debug)
    {
        fprintf(stderr, 
        "Lcnc register map:\nINIT_WRITE_addr %x\nCONFIGURATION_addr %x\nVELOCITY0_addr %x\nVELOCITYlast_addr %x\nMAX_ACC0_addr %x\nMAX_ACClast_addr %x\nSTEP_RES_EN_addr %x\nSTEPDIRINV_addr %x\nSTEPTIMES_addr %x\nGPIOS_OUT_addr %x\nPWM_0_addr %x\nPWM_last_addr %x\nENC_RES_EN_addr %x\nRES_ST_REG_addr %x\nSG_COUNT_0_addr %x\nSG_COUNT_last_addr %x\nSG_VEL_0_addr %x\nSG_VEL_last_addr %x\nWALLCLOCK_addr %x\nGPIOS_IN_addr %x\nENC_COUNT_0_addr %x\nENC_COUNT_last_addr %x\nTX_SECTION_START_addr %x\nTX_SECTION_END_addr %x\nRX_SECTION_START_addr %x\nRX_SECTION_END_addr %x\nLAST_addr %x\nTX_PAYLOAD_size %d bytes\nRX_PAYLOAD_size %d bytes\n",
        device_data->INIT_WRITE_addr,
        device_data->CONFIGURATION_addr,
        device_data->VELOCITY0_addr,
        device_data->VELOCITYlast_addr,
        device_data->MAX_ACC0_addr,
        device_data->MAX_ACClast_addr,
        device_data->STEP_RES_EN_addr,
        device_data->STEPDIRINV_addr,
        device_data->STEPTIMES_addr,
        device_data->GPIOS_OUT_addr,
        device_data->PWM_0_addr,
        device_data->PWM_last_addr,
        device_data->ENC_RES_EN_addr,
        device_data->RES_ST_REG_addr,
        device_data->SG_COUNT_0_addr,
        device_data->SG_COUNT_last_addr,
        device_data->SG_VEL_0_addr,
        device_data->SG_VEL_last_addr,
        device_data->WALLCLOCK_addr,
        device_data->GPIOS_IN_addr,
        device_data->ENC_COUNT_0_addr,
        device_data->ENC_COUNT_last_addr,
        device_data->TX_SECTION_START_addr,
        device_data->TX_SECTION_END_addr,
        device_data->RX_SECTION_START_addr,
        device_data->RX_SECTION_END_addr,
        device_data->LAST_addr,
        device_data->TX_PAYLOAD_size,
        device_data->RX_PAYLOAD_size);
    }
    
//######################################################
//######### EXPORT SIGNALS, PIN, FUNCTION
//######################################################

// enable and reset
    r = hal_pin_bit_newf(HAL_IN, &(device_data->enable),comp_id, "Lcnc.%02d.enable",device_data->board_id);
    if(r < 0) 
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "Lcnc: board %d ERROR: enable pin export failed with err=%i\n",device_data->board_id,r);
        return -1;
    }
    r = hal_pin_bit_newf(HAL_IN, &(device_data->enable_req),comp_id, "Lcnc.%02d.enable-request",device_data->board_id);
    if(r < 0) 
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "Lcnc: board %d ERROR: enable request pin export failed with err=%i\n",device_data->board_id,r);
        return -1;
    }
    r = hal_pin_bit_newf(HAL_OUT, &(device_data->enabled),comp_id, "Lcnc.%02d.enabled",device_data->board_id);
    if(r < 0) 
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "Lcnc: board %d ERROR: enabled pin export failed with err=%i\n",device_data->board_id,r);
        return -1;
    }
// tx retries timeout
    r = hal_param_u32_newf(HAL_RW, &(device_data->tx_max_retries),comp_id, "Lcnc.%02d.tx-max-retries",device_data->board_id);
    if(r < 0) 
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "Lcnc: board %d ERROR: max retries pin export failed with err=%i\n",r,device_data->board_id);
        return -1;
    }
    device_data->tx_max_retries=(hal_u32_t)10;
// watchdog
    r = hal_pin_float_newf(HAL_OUT, &(device_data->watchdog_rd),comp_id, "Lcnc.%02d.watchdog-read",device_data->board_id);
    if(r < 0) 
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "Lcnc: board %d ERROR: watchdog read pin export failed with err=%i\n",device_data->board_id,r);
        return -1;
    }
    r = hal_param_float_newf(HAL_RW, &(device_data->watchdog_wr),comp_id, "Lcnc.%02d.watchdog-write",device_data->board_id);
    if(r < 0) 
    {
        rtapi_print_msg(RTAPI_MSG_ERR, "Lcnc: board %d ERROR: watchdog write pin export failed with err=%i\n",device_data->board_id,r);
        return -1;
    }
    device_data->watchdog_wr=(hal_float_t)0.01;
// Inputs
    for(i=0;i<device_data->n_in;i++) 
    {
        r = hal_pin_bit_newf(HAL_OUT, &(device_data->digital_in[i]),comp_id, "Lcnc.%02d.input.%02d.in",device_data->board_id,i);
        if(r < 0) 
        {
            rtapi_print_msg(RTAPI_MSG_ERR, "Lcnc: board %d ERROR: input %d export failed with err=%i\n",device_data->board_id,i,r);
            return -1;
        }

        r = hal_pin_bit_newf(HAL_OUT, &(device_data->digital_in_n[i]),comp_id, "Lcnc.%02d.input.%02d.in-n",device_data->board_id,i);
        if(r < 0) 
        {
            rtapi_print_msg(RTAPI_MSG_ERR, "Lcnc: board %d ERROR: input-n %d export failed with err=%i\n",device_data->board_id,i,r);
            return -1;
        }

    }

// outputs
    for(i=0;i<device_data->n_out;i++) 
    {
        r = hal_pin_bit_newf(HAL_IN, &(device_data->digital_out[i]),comp_id, "Lcnc.%02d.output.%02d.out",device_data->board_id,i);
        if(r < 0) 
        {
            rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: board %d ERROR: output %d export failed with err=%i\n",device_data->board_id,i,r);
            return -1;
        }
        r = hal_param_bit_newf(HAL_RW, &(device_data->digital_out_inv[i]),comp_id, "Lcnc.%02d.output.%02d.inv",device_data->board_id,i);
        if(r < 0) 
        {
            rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: board %d ERROR: output inversion %d export failed with err=%i\n",device_data->board_id,i,r);
            return -1;
        }
    }

// wallclock
    r = hal_pin_u32_newf(HAL_OUT, &(device_data->wallclock), comp_id, "Lcnc.%02d.wallclock.value",device_data->board_id);
    if(r < 0) 
    {
        rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: board %d ERROR: board_wallclock var export failed with err=%i\n",device_data->board_id,r);
        return -1;
    }
    r = hal_pin_float_newf(HAL_OUT, &(device_data->wallclock_intvl), comp_id, "Lcnc.%02d.wallclock.interval",device_data->board_id);
    if(r < 0) 
    {
        rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: board %d ERROR: board_wallclock interval export failed with err=%i\n",device_data->board_id,r);
        return -1;
    }

// encoders
    for(i=0;i<device_data->n_en;i++) 
    {// !! aggiustare la nomenclatura, si tratta di posizione
        // encoder_count
        r = hal_pin_float_newf(HAL_OUT, &(device_data-> enc_pos_fb[i]), comp_id, "Lcnc.%02d.encoder.%02d.pos-fb",device_data->board_id,i);
        if(r < 0) 
        {
            rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: board %d ERROR:  err encoder %i position feedback\n",device_data->board_id,r);
            return -1;
        }
        r = hal_pin_bit_newf(HAL_IN, &(device_data-> enc_res[i]), comp_id, "Lcnc.%02d.encoder.%02d.reset",device_data->board_id,i);
        if(r < 0) 
        {
            rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: board %d ERROR:  err encoder %i reset\n",device_data->board_id,r);
            return -1;
        }
        r = hal_pin_bit_newf(HAL_IN, &(device_data-> enc_en[i]), comp_id, "Lcnc.%02d.encoder.%02d.enable",device_data->board_id,i);
        if(r < 0) 
        {
            rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: board %d ERROR:  err encoder %i enable\n",device_data->board_id,r);
            return -1;
        }
        r = hal_pin_float_newf(HAL_OUT, &(device_data-> enc_vel_fb[i]), comp_id,"Lcnc.%02d.encoder.%02d.vel-fb",device_data->board_id,i);
        if(r < 0) 
        {
            rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: board %d ERROR:  err encoder %i velocity\n",device_data->board_id,r);
            return -1;
        }
        // encoder scale
        r = hal_param_float_newf(HAL_RW, &(device_data-> enc_scale[i]), comp_id,"Lcnc.%02d.encoder.%02d.scale",device_data->board_id,i);
        if(r < 0) 
        {
            rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: board %d ERROR:  err enc_scale %i scale\n",device_data->board_id,r);
            return -1;
        }
        device_data-> enc_scale[i]=(hal_float_t)1.0;
        // encoder inversion
        r = hal_param_bit_newf(HAL_RW, &(device_data-> enc_inv[i]), comp_id,"Lcnc.%02d.encoder.%02d.inv",device_data->board_id,i);
        if(r < 0) 
        {
            rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: board %d ERROR:  err enc %i inversion\n",device_data->board_id,r);
            return -1;
        }
    }

// stepgens
    for(i=0;i<device_data->n_sg;i++) 
    {
        r = hal_pin_float_newf(HAL_IN, &(device_data->stepgen_velocity_cmd[i]), comp_id,"Lcnc.%02d.stepgen.%02d.vel-cmd",device_data->board_id,i);
        if(r != 0) return r;

        r = hal_pin_float_newf(HAL_IN, &(device_data->stepgen_acc_lim[i]), comp_id, "Lcnc.%02d.stepgen.%02d.acc_lim",device_data->board_id,i);
        if(r != 0) return r;

        r = hal_pin_float_newf(HAL_OUT, &(device_data->stepgen_velocity_fb[i]), comp_id, "Lcnc.%02d.stepgen.%02d.vel-fb",device_data->board_id,i);
        if(r != 0) return r;

        r = hal_pin_float_newf(HAL_OUT, &(device_data->stepgen_position_fb[i]), comp_id, "Lcnc.%02d.stepgen.%02d.pos-fb",device_data->board_id,i);
        if(r != 0) return r;

        r = hal_pin_bit_newf(HAL_IN, &(device_data->stepgen_enable[i]), comp_id, "Lcnc.%02d.stepgen.%02d.enable",device_data->board_id,i);
        if(r != 0) return r;

        r = hal_pin_bit_newf(HAL_IN, &(device_data->stepgen_reset[i]), comp_id, "Lcnc.%02d.stepgen.%02d.reset",device_data->board_id,i);
        if(r != 0) return r;

        r = hal_param_float_newf(HAL_RW, &(device_data->stepgen_scale[i]), comp_id, "Lcnc.%02d.stepgen.%02d.scale",device_data->board_id,i);
        if(r != 0) return r;
        device_data->stepgen_scale[i] = (hal_float_t)1.0;

        r = hal_param_bit_newf(HAL_RW, &(device_data->stepgen_step_inv[i]), comp_id, "Lcnc.%02d.stepgen.%02d.step_inv",device_data->board_id,i);
        if(r != 0) return r;

        r = hal_param_bit_newf(HAL_RW, &(device_data->stepgen_dir_inv[i]), comp_id, "Lcnc.%02d.stepgen.%02d.dir_inv",device_data->board_id,i);
        if(r != 0) return r;

    }

    r = hal_param_float_newf(HAL_RW, &(device_data->stepgen_step_width), comp_id, "Lcnc.%02d.stepgen-step_width",device_data->board_id);
    if(r != 0) return r;
    device_data->stepgen_step_width = (hal_float_t)100;

    r = hal_param_float_newf(HAL_RW, &(device_data->stepgen_step_space), comp_id, "Lcnc.%02d.stepgen-step_space",device_data->board_id);
    if(r != 0) return r;
    device_data->stepgen_step_space =(hal_float_t)100;

    r = hal_param_float_newf(HAL_RW, &(device_data->stepgen_dir_width), comp_id, "Lcnc.%02d.stepgen-dir_width",device_data->board_id);
    if(r != 0) return r;
    device_data->stepgen_dir_width = (hal_float_t)100;

    r = hal_param_float_newf(HAL_RW, &(device_data->stepgen_setup), comp_id, "Lcnc.%02d.stepgen-setup_time",device_data->board_id);
    if(r != 0) return r;
    device_data->stepgen_setup = (hal_float_t)1000;

// PWM
    for(i=0;i<device_data->n_pwm;i++) 
    {
        r = hal_pin_float_newf(HAL_IN, &(device_data-> pwm_freq[i]), comp_id, "Lcnc.%02d.pwm.%02d.freq",device_data->board_id,i);
        if(r < 0) 
        {
            rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: board %d ERROR:  PWM %i frequency\n",device_data->board_id,r);
            return -1;
        }
        r = hal_pin_float_newf(HAL_IN, &(device_data-> pwm_value[i]), comp_id, "Lcnc.%02d.pwm.%02d.value",device_data->board_id,i);
        if(r < 0) 
        {
            rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: board %d ERROR:  PWM %i value\n",device_data->board_id,r);
            return -1;
        }
        r = hal_pin_bit_newf(HAL_IN, &(device_data->pwm_enable[i]), comp_id, "Lcnc.%02d.pwm.%02d.enable",device_data->board_id,i);
        if(r < 0) 
        {
            rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: board %d ERROR:  PWM %i enable\n",device_data->board_id,r);
            return -1;
        }
        r = hal_param_float_newf(HAL_RW, &(device_data->pwm_scale[i]), comp_id, "Lcnc.%02d.pwm.%02d.scale",device_data->board_id,i);
        if(r < 0) 
        {
            rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: board %d ERROR:  PWM %i scale\n",device_data->board_id,r);
            return -1;
        }
        device_data->pwm_scale[i]=(hal_float_t)100.0;
        
        r = hal_param_float_newf(HAL_RW, &(device_data-> pwm_offs[i]), comp_id, "Lcnc.%02d.pwm.%02d.offs",device_data->board_id,i);
        if(r < 0) 
        {
            rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: board %d ERROR:  PWM %i offset\n",device_data->board_id,r);
            return -1;
        }
        
        r = hal_param_bit_newf(HAL_RW, &(device_data-> pwm_inv[i]), comp_id, "Lcnc.%02d.pwm.%02d.inv",device_data->board_id,i);
        if(r < 0) 
        {
            rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: board %d ERROR:  PWM %i inversion\n",device_data->board_id,r);
            return -1;
        }
    }

    /*
     * partial time outputs for debug
     * */
    for(i=0;i<8 && debug==1;i++) 
    {
        r = hal_pin_u32_newf(HAL_OUT, &(device_data->T[i]), comp_id, "Lcnc.%02d.T%02d",device_data->board_id,i);
        if(r < 0) 
        {
            rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: ERROR: board_wallclock var export failed with err=%i\n",r);
            return -1;
        }        
    }


    /* export function */
    rtapi_snprintf(name, sizeof(name), "Lcnc.%02d.update",device_data->board_id);
    r = hal_export_funct(name, update_port, device_data, 1, 0,comp_id);
    if(r < 0) 
    {
        rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: board %d ERROR: write funct export failed\n",device_data->board_id);
        return -1;
    }

    rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc: installed driver for board %d\n",device_data->board_id);

           
    return 0;


//####### ERROR ############
    fail1:

    eb_disconnect(&(device_data->eb));

    fail0:
    hal_exit(comp_id);
    return r;    
}

/**************************************************************
###############################################################
* REALTIME PORT WRITE FUNCTION                                *
###############################################################
**************************************************************/

void update_port(void *arg, long period)
{
    int i,res;
    uint32_t tempvalue;
    union EbData 
    {
           uint32_t value;
           uint8_t bytes[4];
    } temp_reg_value; 
    float time_temp;
    float max_freq,freq_req;
    uint8_t accel_mult;
    double cmdtmp;
    uint32_t time_temp_u,temp_period_u=0,temp_width_u=0;
    const float velfact=pow(2,VEL_SIZE_BITS)/F_FPGA; // velocity factor that is 2^32/F_FPGA
    uint8_t tx_read_packet_buffer[EB_HEADER_SIZE+RX_PAYLOAD_MAX_SIZE];
    uint8_t rx_read_packet_buffer[EB_HEADER_SIZE+RX_PAYLOAD_MAX_SIZE];
    uint8_t tx_write_packet_buffer[EB_HEADER_SIZE+TX_PAYLOAD_MAX_SIZE];
    data_hal *port;
    port = arg;
    unsigned long time_array[9];
    struct timespec spec_loc;
        
//----------------------------------
// Start of read section
//----------------------------------

    if(debug)
    {
        clock_gettime(CLOCK_REALTIME, &spec_loc);
        time_array[0]=(unsigned long)spec_loc.tv_nsec;
    }
    
    // !! prepare packet for read registers one by one from here
    // Header setup
    eb_fill_header(tx_read_packet_buffer,1,port->RX_PAYLOAD_size/4,0); // (buffer,is_read,number of read/writes,start address)
    
    // Payload setup
    // read reset and status register
    temp_reg_value.value=htobe32(port->RES_ST_REG_addr); 
    memcpy((void*)(tx_read_packet_buffer+EB_HEADER_SIZE+port->RES_ST_REG_addr-port->RX_SECTION_START_addr),(void*)temp_reg_value.bytes,4);
    // read stepgen position
    for(i=0;i<port->n_sg;i++)
    {
        temp_reg_value.value=htobe32(port->SG_COUNT_0_addr+4*i);
        memcpy((void*)(tx_read_packet_buffer+EB_HEADER_SIZE+port->SG_COUNT_0_addr-port->RX_SECTION_START_addr+4*i),(void*)temp_reg_value.bytes,4);
    }
    // read stepgen velocity
    for(i=0;i<port->n_sg;i++)
    {
        temp_reg_value.value=htobe32(port->SG_VEL_0_addr+4*i);
        memcpy((void*)(tx_read_packet_buffer+EB_HEADER_SIZE+port->SG_VEL_0_addr-port->RX_SECTION_START_addr+4*i),(void*)temp_reg_value.bytes,4);
    }
    // read wall clock
    temp_reg_value.value=htobe32(port->WALLCLOCK_addr); 
    memcpy((void*)(tx_read_packet_buffer+EB_HEADER_SIZE+port->WALLCLOCK_addr-port->RX_SECTION_START_addr),(void*)temp_reg_value.bytes,4);
    // read gpio in
    temp_reg_value.value=htobe32(port->GPIOS_IN_addr); 
    memcpy((void*)(tx_read_packet_buffer+EB_HEADER_SIZE+port->GPIOS_IN_addr-port->RX_SECTION_START_addr),(void*)temp_reg_value.bytes,4);
    // read encoders
    for(i=0;i<port->n_en;i++)
    {
        temp_reg_value.value=htobe32(port->ENC_COUNT_0_addr+4*i); 
        memcpy((void*)(tx_read_packet_buffer+EB_HEADER_SIZE+port->ENC_COUNT_0_addr-port->RX_SECTION_START_addr+4*i),(void*)temp_reg_value.bytes,4);
    }

    if(debug)
    {
        clock_gettime(CLOCK_REALTIME, &spec_loc);
        time_array[1]=(unsigned long)spec_loc.tv_nsec;
        *(port->T[0])=(uint32_t)(time_array[1]-time_array[0]);
    }

    // #################  TRANSMIT DATA  ###############################################################
    res=eb_send(port->eb, tx_read_packet_buffer, EB_HEADER_SIZE+port->RX_PAYLOAD_size);

    if(debug)
    {
        clock_gettime(CLOCK_REALTIME, &spec_loc);
        time_array[2]=(unsigned long)spec_loc.tv_nsec;
        *(port->T[1])=(uint32_t)(time_array[2]-time_array[1]);
    }

    int count = eb_recv(port->eb, rx_read_packet_buffer, sizeof(rx_read_packet_buffer));

    if(debug)
    {
        clock_gettime(CLOCK_REALTIME, &spec_loc);
        time_array[3]=(unsigned long)spec_loc.tv_nsec;
        *(port->T[2])=(uint32_t)(time_array[3]-time_array[2]);
    }

    if (count != EB_HEADER_SIZE+port->RX_PAYLOAD_size) 
    {
        (port->num_errors_reported)++;
        if(port->conn_err_notified==0)
        {
            port->conn_err_notified=1;
            if(debug) fprintf(stderr, "Lcnc:%f connection error- unexpected read length: %d\n errors %d max %d\n", get_timestamp_f(), count,(port->num_errors_reported), (port->tx_max_retries));
        }
    }

    else 
    {
        if((port->num_errors_reported)>0)
        {
            port->conn_err_notified=0;
            if(debug) fprintf(stderr, "Lcnc:%f connection restored, lost %d packets, limit is set to %d\n",get_timestamp_f(), port->num_errors_reported, (port->tx_max_retries));
            (port->num_errors_reported)=0;
        }
    
        // Reset and status reg
        // read watchdog value
        memcpy((void*)temp_reg_value.bytes,(void*)(rx_read_packet_buffer+EB_HEADER_SIZE+port->RES_ST_REG_addr-port->RX_SECTION_START_addr),4);
        tempvalue=be32toh(temp_reg_value.value);
        port->watchdog_rd_old=*(port->watchdog_rd);
        *(port->watchdog_rd)=(hal_float_t)(tempvalue >> CSR_MMIO_INST_RES_ST_REG_WATCHDOG_OFFSET)*WDT_SCALE;
        // gpio in
        memcpy((void*)temp_reg_value.bytes,(void*)(rx_read_packet_buffer+EB_HEADER_SIZE+port->GPIOS_IN_addr-port->RX_SECTION_START_addr),4);
        tempvalue=be32toh(temp_reg_value.value);
        for (i=0;i<port->n_in;i++) 
        {
            if (tempvalue & 1<<i) *(port->digital_in[i]) = 1 ;
            else *(port->digital_in[i]) = 0;
            *(port->digital_in_n[i])=!(*(port->digital_in[i]));
        }
        // wallclock
        memcpy((void*)temp_reg_value.bytes,(void*)(rx_read_packet_buffer+EB_HEADER_SIZE+port->WALLCLOCK_addr-port->RX_SECTION_START_addr),4);
        tempvalue=be32toh(temp_reg_value.value);
        port->wallclock_old=*(port->wallclock);
        *(port->wallclock)=tempvalue;
        if((*(port->wallclock))>(port->wallclock_old))
            *(port->wallclock_intvl)=((float)(*(port->wallclock)-(port->wallclock_old)))/F_FPGA;
        else *(port->wallclock_intvl)=((float)((port->wallclock_old)-*(port->wallclock)))/F_FPGA;
        // steppers position and velocity feedback
        // introdurre calcolo di velocità basato sulla differenza tra le posizioni nel tempo (usando il wallclock)
        for (i=0;i<port->n_sg;i++)
        {
            memcpy((void*)temp_reg_value.bytes,(void*)(rx_read_packet_buffer+EB_HEADER_SIZE+port->SG_COUNT_0_addr-port->RX_SECTION_START_addr+4*i),4);
            tempvalue=be32toh(temp_reg_value.value);
            port->step_count_old[i]=port->step_count[i]; // save last counter value
            port->step_count[i]=tempvalue;
            if(*(port->stepgen_reset[i])) port->internal_step_count[i]=0;
            port->internal_step_count[i]+=((int32_t)(port->step_count[i])-(int32_t)port->step_count_old[i]);
            *(port->stepgen_position_fb[i])=((hal_float_t)(port->internal_step_count[i]))/(port->stepgen_scale[i]);
            memcpy((void*)temp_reg_value.bytes,(void*)(rx_read_packet_buffer+EB_HEADER_SIZE+port->SG_VEL_0_addr-port->RX_SECTION_START_addr+4*i),4);
            tempvalue=be32toh(temp_reg_value.value);
            *(port->stepgen_velocity_fb[i])=((hal_float_t)((int32_t)tempvalue))/((port->stepgen_scale[i])*velfact);
        }

        ///////////// read encoders /////////////////////////////////////////////

        for(i=0;i<port->n_en;i++)
        {
            memcpy((void*)temp_reg_value.bytes,(void*)(rx_read_packet_buffer+EB_HEADER_SIZE+port->ENC_COUNT_0_addr-port->RX_SECTION_START_addr+4*i),4);
            tempvalue=be32toh(temp_reg_value.value);
            port->enc_count_old[i]=port->enc_count[i]; // save last counter value
            port->enc_count[i]=tempvalue; // new counter value
            if(*(port->enc_res[i])) port->internal_enc_count[i]=0; // reset internal counter if needed
            if((port->enc_inv[i])) 
                port->internal_enc_count[i]-=((int32_t)(port->enc_count[i])-(int32_t)port->enc_count_old[i]); //decrement internal counter if inverted
            else
                port->internal_enc_count[i]+=((int32_t)(port->enc_count[i])-(int32_t)port->enc_count_old[i]); // increment internal counter
            *(port->enc_pos_fb[i])=((hal_float_t)(port->internal_enc_count[i]))*(port->enc_scale[i]); // apply scaling
            // calculate velocity
            port->enc_pos_fb_old[i]=*(port->enc_pos_fb[i]);                 
            if((*(port->wallclock_intvl))>0)
                *(port->enc_vel_fb[i])=((*(port->enc_pos_fb[i]))-(port->enc_pos_fb_old[i]))/(*(port->wallclock_intvl));
        }
        
    }

    if(debug)
    {    
        clock_gettime(CLOCK_REALTIME, &spec_loc);
        time_array[4]=(unsigned long)spec_loc.tv_nsec;
        *(port->T[3])=(uint32_t)(time_array[4]-time_array[3]);
    }
    
    // if watchdog bites or not enabled or tx errors
    if(*(port->watchdog_rd)==0 || *(port->enable)==0 || ((port->num_errors_reported)>(port->tx_max_retries))) 
    {
        *(port->enabled)=0;
        if(*(port->enable)!=0 && port->enable_err_notified==0)
        {
            port->enable_err_notified=1;
            if(*(port->watchdog_rd)==0) 
                rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc:%f board %d driver disabled due to watchdog timer expiring, limit is set to %f\n",get_timestamp_f(),port->board_id,(port->watchdog_wr));
            else
                rtapi_print_msg(RTAPI_MSG_ERR,"Lcnc:%f board %d driver disabled due transmission maximum retries reached, limit is set to %d\n",get_timestamp_f(),port->board_id,(port->tx_max_retries));
        }
    }
    else
    {
        if(*(port->enable_req) && !port->enable_req_old && *(port->enable))
        {   
        port->enable_err_notified=0;
        *(port->enabled)=1;
        }
    }
    port->enable_req_old=*(port->enable_req);

    if(debug)
    {
        clock_gettime(CLOCK_REALTIME, &spec_loc);
        time_array[5]=(unsigned long)spec_loc.tv_nsec;
        *(port->T[4])=(uint32_t)(time_array[5]-time_array[4]);
    }
    
//----------------------------------
// End of read section
//----------------------------------

//----------------------------------
// Start of write section
//----------------------------------
    
    // Header setup
    eb_fill_header(tx_write_packet_buffer,0,port->TX_PAYLOAD_size/4,port->TX_SECTION_START_addr); // (buffer,is_read,number of read/writes,start address)

    // Payload setup
    for (i=0;i<port->n_sg;i++)
    {
        // maximum frequency: limited by minimum step period, that is the sum of pulse width and space width
        max_freq=1.0E9/(port->stepgen_step_width + port->stepgen_step_space); //[Hz]
        // requested frequency of step generator
        freq_req=(*(port->stepgen_velocity_cmd[i]))*(port->stepgen_scale[i]);
        // limit frequency requested to the maximum available
        if(freq_req>max_freq) freq_req=max_freq;
        else if(freq_req<-max_freq) freq_req=-max_freq;
        // convert velocity request to step generator internal command word
        cmdtmp=freq_req*velfact;
        temp_reg_value.value=htobe32((int32_t)cmdtmp); // VELOCITY
        memcpy((void*)(tx_write_packet_buffer+EB_HEADER_SIZE+port->VELOCITY0_addr-port->TX_SECTION_START_addr+4*i),(void*)temp_reg_value.bytes,4);
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
        memcpy((void*)(tx_write_packet_buffer+EB_HEADER_SIZE+port->MAX_ACC0_addr-port->TX_SECTION_START_addr+4*i),(void*)temp_reg_value.bytes,4);
    }    

    tempvalue=0;
    for (i=0;i<port->n_sg;i++)
    {
        if (*(port->stepgen_enable[i])) // set stepgen enable flags
        {
            tempvalue|=(1<<i+CSR_MMIO_INST_STEP_RES_EN_SGENABLE_OFFSET);
        }
        if (*(port->stepgen_reset[i])) // set stepgen reset flags
        {
            tempvalue|=(1<<i+CSR_MMIO_INST_STEP_RES_EN_SGRESET_OFFSET);
        }
    }
    temp_reg_value.value=htobe32(tempvalue); // STEP_RES_EN
    memcpy((void*)(tx_write_packet_buffer+EB_HEADER_SIZE+port->STEP_RES_EN_addr-port->TX_SECTION_START_addr),(void*)temp_reg_value.bytes,4);    
    
    tempvalue=0;
    for (i=0;i<port->n_sg;i++)
    {
        if (port->stepgen_dir_inv[i]) // set stepgen dir pin inversion flags
        {
            tempvalue|=(1<<i+CSR_MMIO_INST_STEPDIRINV_DIR_INV_OFFSET);
        }
        if (port->stepgen_step_inv[i])// set stepgen step pin inversion flags
        {
            tempvalue|=(1<<i+CSR_MMIO_INST_STEPDIRINV_STEP_INV_OFFSET);
        }
    }
    temp_reg_value.value=htobe32(tempvalue); // STEPDIRINV
    memcpy((void*)(tx_write_packet_buffer+EB_HEADER_SIZE+port->STEPDIRINV_addr-port->TX_SECTION_START_addr),(void*)temp_reg_value.bytes,4);
    // set times for stepgen signals
    tempvalue=0;
    time_temp=(port->stepgen_step_width)/F_FPGA_TIME_NS;
    if(time_temp > ((1<<CSR_MMIO_INST_STEPTIMES_STEP_WIDTH_SIZE)-1)) time_temp_u=(1<<CSR_MMIO_INST_STEPTIMES_STEP_WIDTH_SIZE)-1;
    else time_temp_u=(uint32_t)time_temp; //step width
    tempvalue|=time_temp_u<<CSR_MMIO_INST_STEPTIMES_STEP_WIDTH_OFFSET;
    time_temp=(port->stepgen_dir_width)/F_FPGA_TIME_NS;
    if(time_temp > ((1<<CSR_MMIO_INST_STEPTIMES_DIR_WIDTH_SIZE)-1)) time_temp_u=(1<<CSR_MMIO_INST_STEPTIMES_DIR_WIDTH_SIZE)-1;
    else time_temp_u=(uint32_t)time_temp; //dir width
    tempvalue|=time_temp_u<<CSR_MMIO_INST_STEPTIMES_DIR_WIDTH_OFFSET;
    time_temp=(port->stepgen_setup)/F_FPGA_TIME_NS;
    if(time_temp > ((1<<CSR_MMIO_INST_STEPTIMES_DIR_SETUP_SIZE)-1)) time_temp_u=(1<<CSR_MMIO_INST_STEPTIMES_DIR_SETUP_SIZE)-1;
    else time_temp_u=(uint32_t)time_temp; //dir setup time
    tempvalue|=time_temp_u<<CSR_MMIO_INST_STEPTIMES_DIR_SETUP_OFFSET;
    temp_reg_value.value=htobe32(tempvalue); 
    memcpy((void*)(tx_write_packet_buffer+EB_HEADER_SIZE+port->STEPTIMES_addr-port->TX_SECTION_START_addr),(void*)temp_reg_value.bytes,4);
    // GPIOS_OUT
    tempvalue=0;
    for (i=0;i<port->n_out;i++)
    {
        if ((*(port->digital_out[i])) ^ (port->digital_out_inv[i])) tempvalue|=(1<<i);
    }
    temp_reg_value.value=htobe32(tempvalue); 
    memcpy((void*)(tx_write_packet_buffer+EB_HEADER_SIZE+port->GPIOS_OUT_addr-port->TX_SECTION_START_addr),(void*)temp_reg_value.bytes,4);
    // PWM
    for(i=0;i<port->n_pwm;i++)
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
            memcpy((void*)(tx_write_packet_buffer+EB_HEADER_SIZE+port->PWM_0_addr-port->TX_SECTION_START_addr+4*i),(void*)temp_reg_value.bytes,4);
    }
    tempvalue=0;
    // encoders
    for (i=0;i<port->n_en;i++)
    {
        if (*(port->enc_res[i])) // set encoder reset flags
        {
            tempvalue|=(1<<i+CSR_MMIO_INST_ENC_RES_EN_RESET_OFFSET);
        }
        if (*(port->enc_en[i])) // set encoder enable flags
        {
            tempvalue|=(1<<i+CSR_MMIO_INST_ENC_RES_EN_ENABLE_OFFSET);
        }
    }
    temp_reg_value.value=htobe32(tempvalue); // ENC_RES_EN
    memcpy((void*)(tx_write_packet_buffer+EB_HEADER_SIZE+port->ENC_RES_EN_addr-port->TX_SECTION_START_addr),(void*)temp_reg_value.bytes,4); 
     // watchdog
    tempvalue=(uint32_t)((port->watchdog_wr)/WDT_SCALE);// check max value
    tempvalue=tempvalue<<CSR_MMIO_INST_RES_ST_REG_WATCHDOG_OFFSET;
    temp_reg_value.value=htobe32(tempvalue);
    memcpy((void*)(tx_write_packet_buffer+EB_HEADER_SIZE+port->RES_ST_REG_addr-port->TX_SECTION_START_addr),(void*)temp_reg_value.bytes,4);   

    if(debug)
    {    
        clock_gettime(CLOCK_REALTIME, &spec_loc);
        time_array[6]=(unsigned long)spec_loc.tv_nsec;
        *(port->T[5])=(uint32_t)(time_array[6]-time_array[5]);
    }
    
    // #################  TRANSMIT DATA  ###############################################################
    res=eb_send(port->eb, tx_write_packet_buffer, EB_HEADER_SIZE+port->TX_PAYLOAD_size);
    // #################################################################################################

    if(debug)
    {
        clock_gettime(CLOCK_REALTIME, &spec_loc);
        time_array[7]=(unsigned long)spec_loc.tv_nsec;
        *(port->T[6])=(uint32_t)(time_array[7]-time_array[6]);
    }    
}


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
    //return send(conn->fd, bytes, len, MSG_DONTWAIT);
}

int eb_recv(struct eb_connection *conn, void *bytes, size_t max_len) 
{
    //return recvfrom(conn->read_fd, bytes, max_len, MSG_DONTWAIT, NULL, NULL);
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
    struct timeval timeout,timeout1;
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
    
    timeout1.tv_sec = 0;
    timeout1.tv_usec = SEND_TIMEOUT_US;
    err = setsockopt(tx_socket, SOL_SOCKET, SO_SNDTIMEO, (char *)&timeout1, sizeof(timeout1));
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

float get_timestamp_f()
{
    struct timespec spec;

    if(spec_old.tv_sec==0 && spec_old.tv_nsec==0)
    {
        clock_gettime(CLOCK_REALTIME, &spec_old);
    }
    clock_gettime(CLOCK_REALTIME, &spec);
    return (float)((double)(spec.tv_sec-spec_old.tv_sec)+(double)(spec.tv_nsec-spec_old.tv_nsec)/1000000000.0);
}

unsigned long get_timestamp_ns()
{
    struct timespec spec;

    if(spec_old_1.tv_sec==0 && spec_old_1.tv_nsec==0)
    {
        clock_gettime(CLOCK_REALTIME, &spec_old_1);
    }
    clock_gettime(CLOCK_REALTIME, &spec);
    return ((unsigned long)(spec.tv_sec-spec_old_1.tv_sec))*1000000000UL+(unsigned long)(spec.tv_nsec-spec_old_1.tv_nsec);
}
