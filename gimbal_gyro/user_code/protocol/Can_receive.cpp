#include "Can_receive.h"
#include "cmsis_os.h"
#include "user_lib.h"
#include "main.h"
#ifdef __cplusplus
extern "C"
{
#endif

#include "bsp_delay.h"


#ifdef __cplusplus
}
#endif
#include "bsp_can.h"
#include "can.h"

#include "struct_typedef.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;


void Can_receive::init()
{
    can_filter_init();
}

void Can_receive::get_gimbal_motor_measure(uint8_t num, uint8_t data[8])
{
    gimbal_motor[num].last_ecd = gimbal_motor[num].ecd;
    gimbal_motor[num].ecd = (uint16_t)(data[0] << 8 | data[1]);
    gimbal_motor[num].speed_rpm = (uint16_t)(data[2] << 8 | data[3]);
    gimbal_motor[num].given_current = (uint16_t)(data[4] << 8 | data[5]);
    gimbal_motor[num].temperate = data[6];
}

/**
 * @brief          发送Yaw电机电流
 * @param[in]      yaw yaw轴电流
 */
void Can_receive::can_cmd_yaw_motor(int16_t yaw)
{
    uint32_t send_mail_box;
    can_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    can_send_data[0] = yaw >> 8;
    can_send_data[1] = yaw;
    can_send_data[2] = 0;
    can_send_data[3] = 0;
    can_send_data[4] = 0;
    can_send_data[5] = 0;
    can_send_data[6] = 0;
    can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&GIMBAL_YAW_CAN, &can_tx_message, can_send_data, &send_mail_box);
}

/**
 * @brief          发送Pitch电机电流
 * @param[in]      pitch pitch轴电流
 */
void Can_receive::can_cmd_pitch_motor(int16_t pitch)
{
    uint32_t send_mail_box;
    can_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    can_send_data[0] = 0;
    can_send_data[1] = 0;
    can_send_data[2] = pitch >> 8;
    can_send_data[3] = pitch;
    can_send_data[4] = 0;
    can_send_data[5] = 0;
    can_send_data[6] = 0;
    can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&GIMBAL_PITCH_CAN, &can_tx_message, can_send_data, &send_mail_box);
}

/**
 * @brief          返回云台电机 6020电机数据指针
 * @param[in]      i: 电机编号,范围[0,1]
 * @retval         电机数据指针
 */
const motor_measure_t *Can_receive::get_gimbal_motor_measure_point(uint8_t i)
{
    return &gimbal_motor[i];
}

void Can_receive::get_shoot_motor_measure(uint8_t num, uint8_t data[8])
{
    shoot_motor[num].last_ecd = shoot_motor[num].ecd;
    shoot_motor[num].ecd = (uint16_t)(data[0] << 8 | data[1]);
    shoot_motor[num].speed_rpm = (uint16_t)(data[2] << 8 | data[3]);
    shoot_motor[num].given_current = (uint16_t)(data[4] << 8 | data[5]);
    shoot_motor[num].temperate = data[6];
}

/**
 * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
 * @param[in]      left_fric: (0x201) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      right_fric: (0x202) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      trigger: (0x203) 3508电机控制电流, 范围 [-16384,16384]
 * @param[in]      cover: (0x204) 3508电机控制电流, 范围 [-16384,16384]
 * @retval         none
 */
void Can_receive::can_cmd_shoot_motor(int16_t left_fric, int16_t right_fric, int16_t trigger, int16_t cover)
{
    uint32_t send_mail_box;
    can_tx_message.StdId = CAN_SHOOT_ALL_ID;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    can_send_data[0] = left_fric >> 8;
    can_send_data[1] = left_fric;
    can_send_data[2] = right_fric >> 8;
    can_send_data[3] = right_fric;
    can_send_data[4] = trigger >> 8;
    can_send_data[5] = trigger;
    can_send_data[6] = cover >> 8;
    can_send_data[7] = cover;

    //HAL_CAN_AddTxMessage(&SHOOT_CAN, &can_tx_message, can_send_data, &send_mail_box);
}



/**
 * @brief          返回云台电机 6020电机数据指针
 * @param[in]      i: 电机编号,范围[0,1]
 * @retval         电机数据指针
 */
const motor_measure_t *Can_receive::get_shoot_motor_measure_point(uint8_t i)
{
    return &shoot_motor[i];
}

    /*-------------------Gimbel电机GIM--------------------*/
/**
 * @brief 启动电机命令（协议3.2.5）
 * @param id 电机CAN ID
 */
void Can_receive::can_cmd_motor_start(uint8_t id)
{
    uint32_t send_mail_box;
    // 设置CAN消息头
    can_tx_message.StdId = id;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    // 填充启动指令（0x91=MOTCTRL_CMD_START_MOTOR）
    can_send_data[0] = 0x91;
    memset(&can_send_data[1], 0, 7); // 其余字节填0
    // 发送启动指令
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &can_tx_message, can_send_data, &send_mail_box);
}

/**
 * @brief 启动yaw和pitch电机
 */
void Can_receive::cmd_yaw_and_pitch(void)
{
    vTaskDelay(20);
    // for (uint8_t i = 0; i < 10; i++)
    // {
    //     can_cmd_motor_start(CAN_YAW_GIMOTOR_ID);
    //     vTaskDelay(3);
    // }
    for (uint8_t i = 0; i < 20; i++)
    {
        can_cmd_motor_start(CAN_PITCH_GIMOTOR_ID);
        vTaskDelay(3);
    }

}

/**
 * @brief 发送力矩控制指令到指定电机
 * @param id 电机CAN ID
 * @param torque 目标力矩值(N.m)
 * @param duration 执行时间
 */
void Can_receive::can_cmd_torque_control(uint8_t id, float torque, uint32_t duration)
{
    uint32_t send_mail_box;

    can_tx_message.StdId = id;          // 电机控制指令ID（如0x201）
    can_tx_message.IDE = CAN_ID_STD;    // 标准帧（确认电机协议要求）
    can_tx_message.RTR = CAN_RTR_DATA;  // 数据帧（非远程帧）
    can_tx_message.DLC = 0x08;          // 固定8字节（匹配例程协议）
    

    uint8_t* torquePtr = (uint8_t*)(&torque); 
    uint8_t* durationPtr = (uint8_t*)(&duration);
    
    can_send_data[0] = 0x93;                     // 命令字：力矩控制（0x93=MOTCTRL_CMD_TORQUE_CONTROL）
    can_send_data[1] = torquePtr[0];             
    can_send_data[2] = torquePtr[1];
    can_send_data[3] = torquePtr[2];
    can_send_data[4] = torquePtr[3];             
    can_send_data[5] = durationPtr[0];           
    can_send_data[6] = durationPtr[1];
    can_send_data[7] = durationPtr[2];

    //如果电机长时间没返回消息就认为电机下电了，并且是使能电机
    if((HAL_GetTick() - gimbel_gim[id-1].last_update_time) > GIM_LOSE_TIME)
    {
        can_cmd_motor_start(id);
    }
    else
    {
        HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(&GIMBAL_CAN, &can_tx_message, can_send_data, &send_mail_box);
        if (status != HAL_OK) {
            vTaskDelay(2);
            HAL_CAN_AddTxMessage(&GIMBAL_CAN, &can_tx_message, can_send_data, &send_mail_box);
            }	
    }
    

}

/**
 * @brief 解析GIM电机力矩控制响应数据
 * @param num 电机编号
 * @param data 接收到的8字节响应数据
 */
void Can_receive::get_gimbel_gim_measure(uint8_t num, uint8_t data[8])
{
    uint32_t time = 0;

    gimbel_gim[num].last_ecd = gimbel_gim[num].ecd;
    gimbel_gim[num].res_code = data[1]; //响应码
    gimbel_gim[num].temperate = (int8_t)data[2]; //电机温度
    uint16_t pos_int;
    pos_int = (uint16_t)(data[4]<<8 | data[3]);
    // 转换为rad（示例公式：-12.5~12.5）
    gimbel_gim[num].actual_position = (float)pos_int * 25.0f / 65535.0f - 12.5f;
    // 更新编码器值（保留原有逻辑，同步位置数据）
    gimbel_gim[num].ecd = pos_int;

    int16_t speed_int = (int16_t)(((uint16_t)data[5] << 4) | (data[6] >> 4));
    gimbel_gim[num].actual_speed = (float)speed_int * 130.0f / 4095.0f - 65.0f  ; //rad/s+0.01587677

    int16_t torque_int = (int16_t)(((uint16_t)(data[6] & 0x0F) << 8) | data[7]);
    gimbel_gim[num].actual_torque = (float)torque_int * 450.0f / 4095.0f - 225.0f;

    time = HAL_GetTick();
    gimbel_gim[num].update_interval = time - gimbel_gim[num].last_update_time;
    gimbel_gim[num].last_update_time = time;
    
}

    /*-------------------达妙IMU--------------------*/
/*
		发送指令
*/
void Can_receive::imu_send_cmd(uint8_t id, uint8_t reg_id,uint8_t ac,uint32_t data)
{
    uint32_t send_mail_box;
    can_tx_message.StdId = id;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;

    can_send_data[0] = 0xCC;
    can_send_data[1] = reg_id;
    can_send_data[2] = ac;
    can_send_data[3] = 0xDD;
    can_send_data[4] = 0;
    can_send_data[5] = 0;
    can_send_data[6] = 0;
    can_send_data[7] = 0;
    memcpy(can_send_data+4,&data,4);

    HAL_CAN_AddTxMessage(&DM_IMU_CAN, &can_tx_message, can_send_data, &send_mail_box);
}







