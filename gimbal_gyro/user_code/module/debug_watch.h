#ifndef DEBUG_WATCH_H
#define DEBUG_WATCH_H


#include "struct_typedef.h"
#include "usart.h"

typedef struct 
{
    fp32 data1;
    fp32 data2;
    fp32 data3;
    fp32 data4;
    fp32 data5;
    fp32 data6;
    fp32 data7;
    fp32 data8;
    
}debug_data_t;


typedef struct 
{
    fp32* yaw_rad;
    fp32* yaw_rad_s;
    fp32* yaw_set;
    fp32* pitch_rad;
    fp32* pitch_rad_s;
    fp32* pitch_set;
    

    UART_HandleTypeDef* gimbal_debug_uart;
} gimbal_debug_data_t;



class Debug
{
public:

fp32 tx_buffer[4][2]; 
// fp32 tx_buffer2[2]; 
// fp32 tx_buffer3[2]; 
// fp32 tx_buffer4[2]; 

debug_data_t debug_data;

gimbal_debug_data_t gimbal_debug_data;


void send_ending_int();

//无人机云台用调试
void gimbal_debug_init(float* yaw_rad , float* yaw_rad_s , float* yaw_set , float* pitch_rad, float* pitch_rad_s ,float* pitch_set, UART_HandleTypeDef* uart );
void gimbal_debug_send();

uint8_t UART_Send_Data(UART_HandleTypeDef *huart, uint8_t *Data, uint16_t Length);

};


extern Debug debug ;

#endif