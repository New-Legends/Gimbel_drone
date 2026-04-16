#include "communicate.h"

#include "main.h"
#include "string.h"

#include "bsp_usart.h"
#include "bsp_led.h"

#include "detect_task.h"
#include "Gimbal.h"

#include "Remote_control.h"
#include "Can_receive.h"
#include "vision.h"
#include "shoot.h"
#include "Referee.h"

#include "debug_watch.h"
#include "imudm.h"

Remote_control remote_control;

Can_receive can_receive;

Communicate communicate;

Referee referee;

Debug debug;
#define debug_watch 0

extern bool_t if_identify_target;
extern bool_t auto_switch;
extern Shoot shoot;

extern bool_t rune;
extern Gimbal gimbal;
extern imu_t imudm;

//在vision 定义
extern SendPacketTwist_t SendPacketTwist;
bool_t ch4;
extern bool auto_aim_flag;
void Communicate::init()
{
    remote_control.init();
    can_receive.init();
    referee.init();
    debug.gimbal_debug_init(&gimbal.gimbal_yaw_motor.gyro_angle,&gimbal.gimbal_yaw_motor.speed,&gimbal.gimbal_yaw_motor.gyro_angle_set,
                            &gimbal.gimbal_pitch_motor.gyro_angle,&gimbal.gimbal_pitch_motor.speed,&gimbal.gimbal_pitch_motor.gyro_angle_set,
                            &huart1);
    vision_init();
}

void Communicate::run()
{
    //发送数据给视觉
    //vision_send_data(gimbal.vision_cmdid);
    //#if GIMABL_VISION_MODE
    //vision_send_data_jun(1);
    //#else
    //vision_send_data_rv2(1);
    //#endif
    referee.unpack();
    debug.gimbal_debug_send();

}

#ifdef __cplusplus //告诉编译器，这部分代码按C语言的格式进行编译，而不是C++的
extern "C"
{

    //遥控器串口
    void USART3_IRQHandler(void)
    {
        if (huart3.Instance->SR & UART_FLAG_RXNE) //接收到数据
        {
            __HAL_UART_CLEAR_PEFLAG(&huart3);
        }
        else if (USART3->SR & UART_FLAG_IDLE)
        {
            static uint16_t this_time_rx_len = 0;

            __HAL_UART_CLEAR_PEFLAG(&huart3);

            if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
            {
                /* Current memory buffer used is Memory 0 */

                // disable DMA
                //失效DMA
                __HAL_DMA_DISABLE(&hdma_usart3_rx);

                // get receive data length, length = set_data_length - remain_length
                //获取接收数据长度,长度 = 设定长度 - 剩余长度
                this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

                // reset set_data_lenght
                //重新设定数据长度
                hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

                // set memory buffer 1
                //设定缓冲区1
                hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;

                // enable DMA
                //使能DMA
                __HAL_DMA_ENABLE(&hdma_usart3_rx);

                if (this_time_rx_len == RC_FRAME_LENGTH)
                {
                    remote_control.unpack(0);
                    //记录数据接收时间
                    detect_hook(DBUS_TOE);
                }
            }
            else
            {
                /* Current memory buffer used is Memory 1 */
                // disable DMA
                //失效DMA
                __HAL_DMA_DISABLE(&hdma_usart3_rx);

                // get receive data length, length = set_data_length - remain_length
                //获取接收数据长度,长度 = 设定长度 - 剩余长度
                this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

                // reset set_data_lenght
                //重新设定数据长度
                hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

                // set memory buffer 0
                //设定缓冲区0
                DMA1_Stream1->CR &= ~(DMA_SxCR_CT);

                // enable DMA
                //使能DMA
                __HAL_DMA_ENABLE(&hdma_usart3_rx);

                if (this_time_rx_len == RC_FRAME_LENGTH)
                {
                    //处理遥控器数据
                    remote_control.unpack(1);
                    //记录数据接收时间
                    detect_hook(DBUS_TOE);
                }
            }
        }
    }

    //视觉接收中断
    void USART1_IRQHandler(void)
    {
        static volatile uint8_t res;
        if (USART1->SR & UART_FLAG_IDLE)
        {
            __HAL_UART_CLEAR_PEFLAG(&huart1);

            static uint16_t this_time_rx_len = 0;

            if ((huart1.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
            {
                __HAL_DMA_DISABLE(huart1.hdmarx);
                this_time_rx_len = VISION_BUFFER_LEN - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
                __HAL_DMA_SET_COUNTER(huart1.hdmarx, VISION_BUFFER_LEN);
                huart1.hdmarx->Instance->CR |= DMA_SxCR_CT;
                __HAL_DMA_ENABLE(huart1.hdmarx);

                vision_read_data_rv2(Vision_Buffer[0]);
                // vision_read_data(Vision_Buffer[0]); //读取视觉数据
                //vision_read_SendPacketTwist(Vision_Buffer[0]);

                memset(Vision_Buffer[0], 0, 50);
                detect_hook(VISION_TOE);
            }
            else
            {
                __HAL_DMA_DISABLE(huart1.hdmarx);
                this_time_rx_len = VISION_BUFFER_LEN - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
                __HAL_DMA_SET_COUNTER(huart1.hdmarx, VISION_BUFFER_LEN);
                huart1.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
                __HAL_DMA_ENABLE(huart1.hdmarx);

                //vision_read_data(Vision_Buffer[1]); //读取视觉数据
                vision_read_data_rv2(Vision_Buffer[1]);

                //vision_read_SendPacketTwist(Vision_Buffer[1]);

                memset(Vision_Buffer[1], 0, 50);   //对象   内容  长度
                detect_hook(VISION_TOE);
            }
        }
    }

        // TODO 设备检查未更新
    //裁判串口数据
    void USART6_IRQHandler(void)
    {
        static volatile uint8_t res;
        if (USART6->SR & UART_FLAG_IDLE)
        {
            __HAL_UART_CLEAR_PEFLAG(&huart6);

            static uint16_t this_time_rx_len = 0;

            if ((huart6.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
            {
                __HAL_DMA_DISABLE(huart6.hdmarx);
                this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
                __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
                huart6.hdmarx->Instance->CR |= DMA_SxCR_CT;
                __HAL_DMA_ENABLE(huart6.hdmarx);
                fifo_s_puts(&referee.referee_fifo, (char *)(referee.usart6_buf[0]), this_time_rx_len);
                detect_hook(REFEREE_TOE);
            }
            else
            {
                __HAL_DMA_DISABLE(huart6.hdmarx);
                this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
                __HAL_DMA_SET_COUNTER(huart6.hdmarx, USART_RX_BUF_LENGHT);
                huart6.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
                __HAL_DMA_ENABLE(huart6.hdmarx);
                fifo_s_puts(&referee.referee_fifo, (char *)(referee.usart6_buf[1]), this_time_rx_len);
                detect_hook(REFEREE_TOE);
            }
        }
    }


    /**
     * @brief          hal库CAN回调函数,接收电机数据
     * @param[in]      hcan:CAN句柄指针
     * @retval         none
     */
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
    {

        if (hcan == &CAN_1) //接云台CAN 信息
        {
            HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
            switch (rx_header.StdId)
            {
            //云台机构电机
            case CAN_YAW_MOTOR_ID:
                can_receive.get_gimbal_motor_measure(0, rx_data);
                detect_hook(GIMBAL_YAW_MOTOR_TOE);
                break;
            case (CAN_YAW_GIMOTOR_ID+CAN_GIM_MOTOR_NUM): //这里实际上会被优化掉
                can_receive.get_gimbel_gim_measure((CAN_YAW_GIMOTOR_ID-1),rx_data);
                break;
            case (CAN_PITCH_GIMOTOR_ID+CAN_GIM_MOTOR_NUM):
                can_receive.get_gimbel_gim_measure((CAN_PITCH_GIMOTOR_ID-1),rx_data);
                break;
            default:
            {
                break;
            }
            }
        }
        else if (hcan == &CAN_2)
        {
            HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
            switch (rx_header.StdId)
            {
            /*dmimu数据*/
            case MASTER_ID:
                IMU_UpdateData(rx_data);
                break;
            /*发射机构电机*/
            case CAN_TRIGGER_MOTOR_ID:
                can_receive.get_shoot_motor_measure(2, rx_data);
                detect_hook(CAN_TRIGGER_MOTOR_ID);
                break;
            default:
            {
                break;
            }
            }
        }
    }
}

#endif
