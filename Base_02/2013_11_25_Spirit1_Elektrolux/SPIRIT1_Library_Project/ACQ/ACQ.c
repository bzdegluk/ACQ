#include "ACQ.h"

#include "SDK_EVAL_Config.h"
#include "SPIRIT_Config.h"
#include "SDK_Configuration_Common.h"
#include "SPIRIT_SDK_Application.h"
#include "SPIRIT_Commands.h"

#define LED_GREEN     LED2 
#define LED_YELLOW    LED1
extern FlagStatus xTxDoneFlag;

void daas_manage(void);
uint daas_send_read(uint header, uint length, uint type, uint opcode, long address, uint size); 
void message_to_send(void);
void send_ext_read(int address, char size);
void configure(char FW[]);

//extern char frame_send_read[8];      // frame buffor for send read command  //header/lentgh/ext_read/address/address/address/size/CRC//
char frame_send_read[8];      // frame buffor for send read command  //header/lentgh/ext_read/address/address/address/size/CRC//
int ACQ_state;
int comm_status;
int loop_state;
//extern int address_table[10];
int max_var_ind = 10;
int address_table[10] = {0x028D,0x028F,0x0291,0x0293,0x0295,0x0297,0x0299,0x029B,0x029D,0x029F};
int var_index;
int size_table[10] = {2,2,2,2,2,2,2,2,2,2};
int size_index;
int frame_ind = 1;
int frame_size;
char data_received[10];
char dupa[] ={'d','u','p','a','/n'};
int session_number = 5; // how many time to read all var before to sent them
int session_index;

char FW_ID[8];

//extern int ACQ_wait_answ_timer;
//extern int ACQ_in_silent_timer;
int ACQ_wait_answ_timer;
int ACQ_loop_timer;
int ACQ_session_timer;

  // for Uart it use
char count_rec = 0;
int data_ready_for_parsing;    // check the answered data


int App;                        // status of connection with appliance 

//////// for SPIRIT connection
uint8_t buffer_for_transf[211] = {0xC9, 0x00, 0x01, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//header(1), endpoint_nr(2), variables_number(1), time_stamp(6)
uint8_t bft_cnt = 10;
int Spirit_data;

/*
void daas_manage(void)
{
    switch (ACQ_state)
      
        case ACQ_Idle:          // communication state idle
        message_to send();      //check if message has to be send
          break;
        
        case ACQ_sending_msg:       //sending during progress
          break;
          
        case ACQ_waiting_answer    // waiting for the answer
                if(ACQ_wait_answ_timer == 0)    // check timeout for the answer               
                    comm_status = RX_ERR;
         
                      switch (comm_status)
                        
                             case RX_ERR:
                             Free_ACQ_buff();
                             ACQ_state = ACQ_Idle;
                             
                             
                             case RX_RDY:
                               Check_ACQ_answ();
         
           break;
         
        case  ACQ_in_silent_time        // waiting for the silent time end
                if (ACQ_silent_timer == 0)
                  ACQ_state = ACQ_Idle;
          break;
         
  
} */

void daas_manage(void)
{
  switch (ACQ_state)
  {
    //////////////////////////////////////////////////////////////////////////////////////
    case ACQ_Idle:              //ready for next ransmission
    ACQ_session_timer = 60;     // 60 ms for successful transmission
    message_to_send();          // start transmisionof the frame (first char - rest by interrupt)
    break;
    ///////////////////////////////////////////////////////////////////////////////////////
    case ACQ_sending_msg:       // sending in progress
    break;
    //////////////////////////////////////////////////////////////////////////////////////
    case ACQ_waiting_answer:    // waiting for the answer 
    if (ACQ_wait_answ_timer == 0)    // check timeout for the answer
    {
      comm_status = RX_ERR;
      // ACQ_state = ACQ_waiting_next_window;
      ACQ_state = ACQ_Idle;
      App = App_disconnected;
     }
     break;
    ///////////////////////////////////////////////////////////////////////////////////////
    case ACQ_waiting_next_window:       // waiting to finish 60ms to let start next session
     if (data_ready_for_parsing == 0)
      {
              /*  stara wersja odczytu odebranych danych wszystko co odebrane idzie do wysylki
              for (uint8_t rec_index = 0 ;rec_index < size_index ;rec_index++)
              {
                buffer_for_transf[bft_cnt] = data_received[rec_index];
                bft_cnt++;
              }  */
      //  docelowe przepisanie odebranych wartosci
      for (uint8_t rec_index = 3; rec_index < (3 + size_table[var_index-1]); rec_index++)
      {
        buffer_for_transf[bft_cnt] = data_received[rec_index];
        bft_cnt++;
         if (bft_cnt == 0x10)
        {
          bft_cnt = 0x10;
        }
        if (bft_cnt == 0x13)
        {
          bft_cnt = 0x13;
        }
         if (bft_cnt == 0x16)
        {
          bft_cnt = 0x16;
        }
         if (bft_cnt == 0xE0)
        {
          bft_cnt = 0xE0;
        }
      } 
      
      if (var_index > max_var_ind -1)                                     // check if address_table size reached
            {
              var_index = 0;
            }
        data_ready_for_parsing = 1;
      }    
      if (ACQ_session_timer == 0)
        {
          if (var_index != 0)
            {
              ACQ_state = ACQ_Idle;
            }
       else
        {
              ACQ_state = ACQ_waiting_next_loop;
              Spirit_data = not_transmitted;
         }
    }
    break;
    ///////////////////////////////////////////////////////////////////////////////////////////
    case ACQ_waiting_next_loop:    // waiting to finish 1000ms before start next loop
    if (Spirit_data == not_transmitted && session_index == (session_number-1))
    {
        SdkEvalLedToggle(LED_YELLOW);
        SpiritCmdStrobeFlushTxFifo();
        SpiritSpiWriteLinearFifo(96, buffer_for_transf);
        SpiritCmdStrobeTx();
        while(!xTxDoneFlag);  // Wait till Transmission is done
        Spirit_data = transmitted;
    }
    if (ACQ_loop_timer ==0)
    {
          ACQ_state = ACQ_Idle;
          /*
            if (bft_cnt == sum_data_session * 5)
          */
          session_index++;
          if (session_index == session_number)
          {
            session_index = 0;
            bft_cnt = 10;
          }
          
          ACQ_loop_timer = 1000;      // each variable readed one time per second
    }
    break;
    ////////////////////////////////////////////////////////////////////////////////////////////
  }
}    

void message_to_send(void)
{
  if (address_table[var_index] != 0)   // check if next address for read is not empty
        {             
             send_ext_read(address_table[var_index], size_table[var_index]);    // send ext read message
             ACQ_state = ACQ_sending_msg;                                       //change state of ACQ for sending message
             size_index = size_table[var_index]+4;                                // index of the size for the data to be received
   //         if (var_index > max_var_ind -2)                                     // check if address_table size reached
   //         {
   //           var_index = 0;
   //         }
   //         else
   //         {
             var_index++;                                                       // point next variable
   //         }
             
        }
  else
        {
             var_index = 0;             //if address empty then go to the first variable 
        }
}

void send_ext_read(int address, char size)
{
  frame_size = 7;
  frame_send_read[0] = 0xC6;
  frame_send_read[1] = 0x04;
  frame_send_read[2] = 0x80;
//  frame_send_read[3] = ((address & 0xFF0000) >>16);                            //header/lentgh/ext_read/address/address/address/size/CRC//
  frame_send_read[3] = ((address & 0xFF00) >>8);
  frame_send_read[4] = (address & 0xFF);
  frame_send_read[5] = size;
  frame_send_read[6] = frame_send_read[1] ^ frame_send_read[2] ^ frame_send_read[3] ^ frame_send_read[4] ^ frame_send_read[5];
  USART1->DR = (frame_send_read[0] & (uint16_t)0x01FF);                         // send first byte of the frame
  frame_ind = 1; 
  USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
  //indicate to the TC interupt start of the trasmission
   //USART_SendData(USART1, frame_send_read[0]);
   //while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}

void configure(char FW[])
{
  data_ready_for_parsing = 1;
  frame_size = 5;
  frame_send_read[0] = 0xC6;
  frame_send_read[1] = 0x02;
  frame_send_read[2] = 0xB0;
  frame_send_read[3] = 0x00;
  frame_send_read[4] = 0xB2;
  frame_ind = 1;  
  size_index = 8;
  count_rec = 0;
  USART1->DR = (frame_send_read[0] & (uint16_t)0x01FF);                         // send first byte of the frame
  USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
  

while (ACQ_state == !ACQ_waiting_answer);
  while (data_ready_for_parsing > 0)                                          // waiting for the answer
  {
    if (ACQ_wait_answ_timer == 0)
    {
      ACQ_state = ACQ_Idle;
      //break;
      return;
    }
  }
  FW[0] = data_received[3];
  FW[1] = data_received[4];
  FW[2] = data_received[5];
  FW[3] = data_received[6];
  
  
  data_ready_for_parsing = 1;
  frame_size = 5;
  frame_send_read[0] = 0xC6;
  frame_send_read[1] = 0x02;
  frame_send_read[2] = 0xB6;
  frame_send_read[3] = 0x00;
  frame_send_read[4] = 0xB4;
  frame_ind = 1; 
  size_index = 8;
  USART1->DR = (frame_send_read[0] & (uint16_t)0x01FF);                         // send first byte of the frame
  USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
  
  while (data_ready_for_parsing > 0)                                          // waiting for the answer
    {
    if (ACQ_wait_answ_timer == 0)
    {
      ACQ_state = ACQ_Idle;
      //break;
      return;
    }
  }
  FW[4] = data_received[3];
  FW[5] = data_received[4];
  FW[6] = data_received[5];
  FW[7] = data_received[6];
  
  data_ready_for_parsing = 1;
  frame_size = 7;
  frame_send_read[0] = 0xC6;
  frame_send_read[1] = 0x04;
  frame_send_read[2] = 0xB7;
  frame_send_read[3] = 0x06;
  frame_send_read[4] = 0x20;
  frame_send_read[5] = 0x00;
  frame_send_read[6] = 0x95;
  frame_ind = 1; 
  size_index = 5;
  USART1->DR = (frame_send_read[0] & (uint16_t)0x01FF);                         // send first byte of the frame
  USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
  SdkDelayMs(500);
  while (data_ready_for_parsing > 0)                                          // waiting for the answer
    {
    if (ACQ_wait_answ_timer == 0)
    {
      ACQ_state = ACQ_Idle;
      //break;
      return;
    }
  }
  
  if (data_received[2] == 0xF7 && data_received[3] == 0x00)
  {
        App = App_connected;                                                          //connection successful
        ACQ_state = ACQ_Idle;                                                           //ready for daas manage
  }
}

void app_connect()
{
  configure(FW_ID);
  
}






