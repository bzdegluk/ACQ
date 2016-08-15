#ifndef ACQ_H
#define ACQ_H

#include "stm32l1xx_usart.h"

typedef unsigned short int uint;
#define endpoint_address (int)0x0004           //address of endpoint
#define daas_header     0xC6
#define daas_baud_rate  9600

#define type_enquire    0x00
#define type_data       0x01
#define type_request    0x02
#define type_answer     0x03

#define opcode_normal   0x01
#define opcode_extended 0x00

#define ext_read_int_memory     0x00

#define ACQ_Idle                (int)0x00
#define ACQ_sending_msg         (int)0x01
#define ACQ_waiting_answer      (int)0x02
#define ACQ_waiting_next_window (int)0x03
#define ACQ_waiting_next_loop   (int)0x04


#define loop_in_progress        (int)0x01
#define loop_waiting_end_curr   (int)0x02

#define App_connected               (int)0x01
#define App_disconnected            (int)0x02


#define RX_ERR          (int)0x01

#define transmitted     (int)0x01
#define not_transmitted (int)0x02
#define Spirit_waiting_window   (int)0x05

extern void daas_manage(void);
extern uint daas_send_read(uint header, uint length, uint type, uint opcode, long address, uint size); 
extern void message_to_send(void);
extern void send_ext_read(int address, char size);
extern void app_connect(void);
extern void configure(void);
extern void apply_command(void);
extern void say_hello(void);
extern char check_FW(void);
extern void send_to_Base_Station(void);

//extern char frame_send_read[8];      // frame buffor for send read command  //header/lentgh/ext_read/address/address/address/size/CRC//
extern char frame_send_read[8];      // frame buffor for send read command  //header/lentgh/ext_read/address/address/address/size/CRC//
extern int ACQ_state;
extern int comm_status;
extern int loop_state;
//extern int address_table[10];
extern int max_var_ind;
extern int address_table[10];
extern int var_index;
extern int size_table[10];
extern int size_index;
extern int frame_ind;
extern int frame_size;
extern char data_received[10];

//extern int ACQ_wait_answ_timer;
//extern int ACQ_in_silent_timer;
extern int ACQ_wait_answ_timer;
extern int ACQ_loop_timer;
extern int ACQ_session_timer;

extern char count_rec;
extern int data_ready_for_parsing;

extern int App;                        // status of connection with appliance 
extern int SysyTickCnt;                 // timer for the transmition synchronisation

//////// for SPIRIT connection
extern uint8_t buffer_for_transf[96], bft_cnt;
extern int Spirit_data;


//////////////////for command////

extern uint8_t command_buffer[96];
extern int command_size;
extern int command_index;
extern int command_pending;
#endif