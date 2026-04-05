#include "debug_watch.h"

#include "cmsis_os.h"

uint8_t tail[4] = {0x00,0x00,0x80,0x7f};

void Debug::send_ending_int()
{
    osDelay(1);
    UART_Send_Data(&huart1,tail,4);
}

uint8_t Debug::UART_Send_Data(UART_HandleTypeDef *huart, uint8_t *Data, uint16_t Length)
{
    return (HAL_UART_Transmit(huart, Data, Length,0xFFF));
}


void Debug::gimbal_debug_init(float* yaw_rad , float* yaw_rad_s , float* yaw_set , float* pitch_rad, float* pitch_rad_s ,float* pitch_set, UART_HandleTypeDef* uart )
{
    gimbal_debug_data.yaw_rad = yaw_rad;
    gimbal_debug_data.yaw_rad_s = yaw_rad_s;
    gimbal_debug_data.yaw_set = yaw_set;
    gimbal_debug_data.pitch_rad = pitch_rad;
    gimbal_debug_data.pitch_rad_s = pitch_rad_s;
    gimbal_debug_data.pitch_set = pitch_set;
    gimbal_debug_data.gimbal_debug_uart = uart;
}

void Debug::gimbal_debug_send()
{
    //fp32 send_data[6];
    tx_buffer[0][0] = *(gimbal_debug_data.yaw_rad);
    tx_buffer[0][1] = *(gimbal_debug_data.yaw_rad_s);

    tx_buffer[1][0] = *(gimbal_debug_data.yaw_set);
    tx_buffer[1][1] = *(gimbal_debug_data.pitch_rad);

    tx_buffer[2][0] = *(gimbal_debug_data.pitch_rad_s);
    tx_buffer[2][1] = *(gimbal_debug_data.pitch_set);

    for (uint8_t i = 0; i < 3; i++)
    {
        UART_Send_Data(gimbal_debug_data.gimbal_debug_uart,(uint8_t*)tx_buffer[i],8);
        while(HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY);
        /* code */
    }
    send_ending_int();
}

