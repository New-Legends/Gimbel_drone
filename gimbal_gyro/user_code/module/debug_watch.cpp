#include "debug_watch.h"

#include "cmsis_os.h"

uint8_t tail[4] = {0x00,0x00,0x80,0x7f};


void Debug::write_1_int()
{
    tx_buffer1[0]=debug_data.data1;
    tx_buffer1[1]=debug_data.data2;
}

void Debug::write_2_int()
{
    tx_buffer1[0] = debug_data.data1;
    tx_buffer1[1] = debug_data.data2;

    tx_buffer2[0] = debug_data.data3;
    tx_buffer2[1] = debug_data.data4;
}

void Debug::write_3_int()
{
    tx_buffer1[0] = debug_data.data1;
    tx_buffer1[1] = debug_data.data2;

    tx_buffer2[0] = debug_data.data3;
    tx_buffer2[1] = debug_data.data4;

    tx_buffer3[0] = debug_data.data5;
    tx_buffer3[1] = debug_data.data6;
}

void Debug::write_4_int()
{
    tx_buffer1[0] = debug_data.data1;
    tx_buffer1[1] = debug_data.data2;

    tx_buffer2[0] = debug_data.data3;
    tx_buffer2[1] = debug_data.data4;

    tx_buffer3[0] = debug_data.data5;
    tx_buffer3[1] = debug_data.data6;

    tx_buffer4[0] = debug_data.data7;
    tx_buffer4[1] = debug_data.data8;
}

void Debug::send_1_int()
{
    osDelay(1);
    UART_Send_Data(&huart6,(uint8_t*)tx_buffer1,8);
}

void Debug::send_2_int()
{
    osDelay(1);
    UART_Send_Data(&huart6,(uint8_t*)tx_buffer1,8);
    osDelay(1);
    UART_Send_Data(&huart6,(uint8_t*)tx_buffer2,8);
}

void Debug::send_3_int()
{
    osDelay(1);
    UART_Send_Data(&huart6,(uint8_t*)tx_buffer1,8);
    osDelay(1);
    UART_Send_Data(&huart6,(uint8_t*)tx_buffer2,8);
    osDelay(1);
    UART_Send_Data(&huart6,(uint8_t*)tx_buffer3,8);
}

void Debug::send_4_int()
{
    osDelay(1);
    UART_Send_Data(&huart6,(uint8_t*)tx_buffer1,8);
    osDelay(1);
    UART_Send_Data(&huart6,(uint8_t*)tx_buffer2,8);
    osDelay(1);
    UART_Send_Data(&huart6,(uint8_t*)tx_buffer3,8);
    osDelay(1);
    UART_Send_Data(&huart6,(uint8_t*)tx_buffer4,8);
}

void Debug::send_ending_int()
{
    osDelay(1);
    UART_Send_Data(&huart6,tail,4);
}

uint8_t Debug::UART_Send_Data(UART_HandleTypeDef *huart, uint8_t *Data, uint16_t Length)
{
    return (HAL_UART_Transmit(huart, Data, Length,0xFFF));
}


void Debug::gimbal_debug_init(float* yaw_rad , float* yaw_rad_s , float* pitch_rad, float* pitch_rad_s ,UART_HandleTypeDef* uart )
{
    gimbal_debug_data.yaw_rad = yaw_rad;
    gimbal_debug_data.yaw_rad_s = yaw_rad_s;
    gimbal_debug_data.yaw_set = NULL;
    gimbal_debug_data.pitch_rad = pitch_rad;
    gimbal_debug_data.pitch_rad_s = pitch_rad_s;
    gimbal_debug_data.pitch_set = NULL;
    gimbal_debug_data.gimbal_debug_uart = uart;
}

void Debug::gimbal_debug_send()
{
    //fp32 send_data[6];
    tx_buffer1[0] = *(gimbal_debug_data.yaw_rad);
    tx_buffer1[1] = *(gimbal_debug_data.yaw_rad_s);

    tx_buffer2[0] = *(gimbal_debug_data.yaw_set);
    tx_buffer2[1] = *(gimbal_debug_data.pitch_rad);

    tx_buffer3[0] = *(gimbal_debug_data.pitch_rad_s);
    tx_buffer3[1] = *(gimbal_debug_data.pitch_set);

    send_3_int();
    send_ending_int();
}

