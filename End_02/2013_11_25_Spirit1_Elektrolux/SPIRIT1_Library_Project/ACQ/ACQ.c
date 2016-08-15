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
void configure(void);
void apply_command(void);
void say_hello(void);
char check_FW(void);

//extern char frame_send_read[8];      // frame buffor for send read command  //header/lentgh/ext_read/address/address/address/size/CRC//
char frame_send_read[8];      // frame buffor for send read command  //header/lentgh/ext_read/address/address/address/size/CRC//
int ACQ_state;
int comm_status;
int loop_state;
//extern int address_table[10];
int max_var_ind = 10;
int address_table[10] = {0x028D,0x028F,0x0291,0x0293,0x0295,0x0297,0x0299,0x029B,0x029D,0x029F};
int var_index;
int size_table[10] = {2,2,2,2,2,2,2,1,1,1};
int size_index;
int frame_ind = 1;
int frame_size;
char data_received[10];
char prev_data_received[10];
char dupa[] ={'d','u','p','a','/n'};
uint8_t hello[96] = {0xDD, 'h','e','l','l','o',' ','I',' ','a','m',0x00, 0x01,52,0x00};
int session_number = 5; // how many time to read all var before to sent them
int session_index;
uint8_t crc;
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
uint8_t buffer_for_transf[96] = {0xC9, 0x00, 0x01, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//header(1), endpoint_nr(2), variables_number(1), time_stamp(6)

uint8_t bft_cnt = 10;
int Spirit_data;
uint8_t tempRegValue[4];

//////// for commands /////

uint8_t command_buffer[96];
int command_size = 60;
int command_index = 0;
int command_pending = 0;

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
    SpiritCmdStrobeRx();
    ACQ_session_timer = 60;     // 60 ms for successful transmission
    message_to_send();          // ***************start transmisionof the frame (first char - rest by interrupt)****************
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
        crc = data_received[1];
        for (uint8_t rec_index = 2; rec_index < (3 + size_table[var_index-1]); rec_index++)
        {
          crc = crc ^ data_received[rec_index];
        }
        if (crc == data_received[(3 + size_table[var_index-1])])
        {
                for (uint8_t rec_index = 3; rec_index < (3 + size_table[var_index-1]); rec_index++)
                  {
                    buffer_for_transf[bft_cnt] = data_received[rec_index];
                    //prev_data_received[rec_index] = data_received[rec_index];
                    bft_cnt++;
                  } 
        }
         else
         {
              for (uint8_t rec_index = 3; rec_index < (3 + size_table[var_index-1]); rec_index++)
                  {
                    // zostawiamy zawartosc bufora z poprzedniego loopa                    
                    bft_cnt++;
                  } 
         }

      
      
      
      if (var_index > max_var_ind -1)          //////// check if address_table size reached
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
      
        crc = buffer_for_transf[0];
        for(int i = 1; i < 95; i++)
        {
          crc = crc ^ buffer_for_transf[i];
        }
        buffer_for_transf[95] = crc;
        Spirit_data = Spirit_waiting_window;            // data ready to be sent to the Base Station
        
 //       SdkEvalLedToggle(LED_YELLOW);
        
                                                    // moved to the sent_to_the Base_Station function
    
        
        
        SpiritCmdStrobeRx();  //frame sent, start wait for the command
    }
    SpiritCmdStrobeRx();  //frame sent, start wait for the command
  
  //  if (Spirit_data == not_transmitted && session_index == (session_number-2))
  //  {
        
  //  }
    if (ACQ_loop_timer == 0)
    {
      if(Spirit_data != Spirit_waiting_window)
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

void configure(void)
{
  uint32_t eeprom_address = 0x08080001;
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
  FW_ID[0] = data_received[3];
  FW_ID[1] = data_received[4];
  FW_ID[2] = data_received[5];
  FW_ID[3] = data_received[6];
  
  
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
  FW_ID[4] = data_received[3];
  FW_ID[5] = data_received[4];
  FW_ID[6] = data_received[5];
  FW_ID[7] = data_received[6];
  
  
  
  data_ready_for_parsing = 1;
  frame_size = 7;
  frame_send_read[0] = 0xC6;
  frame_send_read[1] = 0x04;
  frame_send_read[2] = 0xB7;
  frame_send_read[3] = 0x06;
  frame_send_read[4] = check_FW();
  //frame_send_read[4] = 0x00;
  frame_send_read[5] = 0x00;
  if (frame_send_read[4] == 0x20)
    {
      frame_send_read[6] = 0x95;
    }else
    {
      frame_send_read[6] = 0xB5;
    }
  
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
        ACQ_state = ACQ_Idle;    //ready for daas manage
        // read the adresses storred in eeprom to be read                               //%%%%%%
        max_var_ind = (*(uint8_t*)(eeprom_address - 1));
        for (int i = 0; i < 10; i++)                                                    //%%%%%%
        {                                                                               //%%%%%%
          hello[23 +(3 * i)] = size_table[i] = ((*(uint8_t*)(eeprom_address + (3 * i))) & 0x03); //%%%%%%
          address_table[i] = (*(uint16_t*)(eeprom_address + (3 * i) +1));               //@@@@@@
          hello[24 + (3 * i)] = ((address_table[i] & 0xFF00) >> 8);                     //%%%%%%
          hello[25 + (3 * i)] = (address_table[i] & 0xFF);                              //%%%%%%
        } 
        say_hello();             //send report to the serwer about connection                                     
  }
}

void app_connect()
{
  hello[11] = buffer_for_transf[1] = ((endpoint_address & 0xFF00) >> 8);
  hello[12] = buffer_for_transf[2] = (endpoint_address & 0xFF);
  configure();
  
}

void apply_command(void)
{
  uint32_t eeprom_address = 0x08080001;
  if(command_buffer[0] == 0xC6)  // check if request to configure of variables
  {
    
      if (((command_buffer[1] * 256) + command_buffer[2]) == endpoint_address)
        {
          max_var_ind = command_buffer[4];
          for(int i = 0; i < max_var_ind; i++)
            {
              size_table[i] = command_buffer[5 + (i * 5)];
               address_table[i] = ((command_buffer[8 + (i * 5)] * 256) + command_buffer[9 + (i * 5)]);
            }
          // need to store into the eeprom memory
          DATA_EEPROM_Unlock();
          DATA_EEPROM_ProgramByte((eeprom_address - 1) ,max_var_ind);
          for (int i = 0; i < 10; i++)
            {
               DATA_EEPROM_ProgramByte((eeprom_address + (3 * i)) ,size_table[i]);
               DATA_EEPROM_ProgramHalfWord((eeprom_address + (3 * i) +1) ,address_table[i]);
               
            }
          DATA_EEPROM_Lock();
          // send back message with confirmation the data storing         
           command_buffer[0] = 0xCF;
           SdkEvalLedToggle(LED_YELLOW);
           SpiritCmdStrobeFlushTxFifo();
//           SpiritSpiWriteLinearFifo(command_buffer[3], command_buffer);
           SpiritSpiWriteLinearFifo(96, command_buffer);
           SpiritCmdStrobeTx();
           while(!xTxDoneFlag);  // Wait till Transmission is done
        }  
  }
  if(command_buffer[0] == 0xAF)  // check if request to start timer
  {
    if(command_buffer[3] == 0x0A)
    {
      SysyTickCnt = 0;
    }
  }
  command_pending = 0;
}

void say_hello(void)
{
      for(int i = 0; i <8; i++)  //  put the FW if readed
      {
        hello[i+15] = FW_ID[i];
      }
        hello[14] = App;         // put the status of connection
      crc = hello[0];
      for (int i = 1; i < 95; i++)
      {
        crc = crc ^ hello[i];
      }
      hello[95] = crc;
        SpiritCmdStrobeFlushTxFifo();
        SpiritSpiWriteLinearFifo(96, hello);
        SpiritCmdStrobeTx();
        while(!xTxDoneFlag);  // Wait till Transmission is done
}

char check_FW(void)
{
    if (FW_ID[0] == 'T' && FW_ID[1] == 'O' && FW_ID[2] == 'C')
    {
      return 0x20;
    }
        if (FW_ID[0] == 'U' && FW_ID[1] == 'F' && FW_ID[2] == 'B')
    {
      return 0x20;
    }
    
    return 0x00;
}

void send_to_Base_Station(void)
    {
        SpiritCmdStrobeFlushTxFifo();
        SpiritSpiWriteLinearFifo(96, buffer_for_transf);
        SpiritCmdStrobeTx();
        while(!xTxDoneFlag);  // Wait till Transmission is done
        //uint8_t tempRegValue[4];
        g_xStatus = SpiritSpiReadRegisters(IRQ_STATUS3_BASE, 4, tempRegValue);
        Spirit_data = transmitted;
    }