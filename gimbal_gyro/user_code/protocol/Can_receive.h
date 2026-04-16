#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "main.h"

#include "struct_typedef.h"
#include "vision.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

#define SHOOT_CAN hcan2
#define DM_IMU_CAN hcan2
#define GIMBAL_PITCH_CAN hcan2
#define GIMBAL_YAW_CAN hcan1
#define GIMBAL_CAN hcan1
#define TRIGGER_CAN hcan2
#define BOARD_COM_CAN hcan1
#define CAN_1 hcan1
#define CAN_2 hcan2

#define GIM_LOSE_TIME 200 //ms

//云台电机编号
enum gimbal_motor_id_e
{
    //底盘动力电机接收
    YAW_MOTOR = 0,
    PITCH_MOTOR,
};

//发射机构电机编号
enum shoot_motor_id_e
{
    //底盘动力电机接收
    LEFT_FRIC_MOTOR = 0,
    RIGHT_FRIC_MOTOR,
    TRIGGER_MOTOR,
    COVER_MOTOR,
};

typedef enum
{
    //发射机构电机接受ID can2
    CAN_LEFT_FRIC_MOTOR_ID = 0x201,
    CAN_RIGHT_FRIC_MOTOR_ID = 0x202,
    CAN_TRIGGER_MOTOR_ID = 0x203,
    CAN_COVER_MOTOR_ID = 0X204,
    CAN_SHOOT_ALL_ID = 0x200,

    //云台DJI电机接收ID   
    CAN_YAW_MOTOR_ID = 0x205,//can1
    CAN_PITCH_MOTOR_ID = 0x206,//can2
    CAN_GIMBAL_ALL_ID = 0x1FF,

    //云台GIM电机接收ID can1  在这里记录一下GIM电机的毛病，不可以和c板陀螺仪一起使用会报错0x02（指令错误），使能前要关闭pwm否则失败，
    CAN_PITCH_GIMOTOR_ID = 0x02,
    CAN_YAW_GIMOTOR_ID = 0x01,
    CAN_GIM_MOTOR_NUM = 0x02, //这个是电机的数量，masterid需要等于canid+电机数量

    //板间通信ID   can1
    CAN_RC_BOARM_COM_ID = 0x101,
    CAN_GIMBAL_BOARD_COM_ID = 0x102,
    CAN_SHOOT_SPEED_BOARD_COM_ID =0x304,

    //发送UI数据
    CAN_UI_COM_ID = 0x305,

    //传输 导航数据
    CAN_Sendpark= 0x306,
} can_msg_id_e;

// DJI motor data
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

// GIM电机数据结构体
typedef struct
{
    // 电机基本状态
    uint16_t last_ecd;          // 上次编码器值
    uint16_t ecd;               // 当前编码器值
    uint8_t temperate;          // 温度
    
    // GIM电机特有数据（根据协议3.2.7响应格式）
    uint8_t res_code;           // 响应码
    float actual_torque;        // 实际力矩 N.m
    float actual_speed;         // 实际速度 RAD/s
    float actual_position;      // 实际位置 RAD
    uint8_t fault_status;       // 故障状态
    
    // 控制状态
    bool is_running;            // 运行状态
    uint32_t last_update_time;  // 最后更新时间
    uint32_t update_interval;    // 每次运行的时间间隔
} GimMotor_measure_t;



class Can_receive
{

public:
    //云台电机反馈数据结构体
    motor_measure_t gimbal_motor[2];
    //发射机构电机反馈数据结构体
    motor_measure_t shoot_motor[4];

    GimMotor_measure_t gimbel_gim[2];

    //发送数据结构体
    CAN_TxHeaderTypeDef can_tx_message;
    uint8_t can_send_data[8];



    void init();

    /*--------------------云台DJI电机数据接收----------------------*/
    void get_gimbal_motor_measure(uint8_t num, uint8_t data[8]);
    void can_cmd_yaw_motor(int16_t yaw);
    void can_cmd_pitch_motor(int16_t pitch);
    const motor_measure_t *get_gimbal_motor_measure_point(uint8_t i);

    /*--------------------云台GIM电机数据接收----------------------*/
    void can_cmd_motor_start(uint8_t id);
    void cmd_yaw_and_pitch(void);
    void can_cmd_torque_control(uint8_t id, float torque, uint32_t duration);
    void get_gimbel_gim_measure(uint8_t num, uint8_t data[8]);

    /*-------------------发射机构电机数据接收--------------------*/
    void get_shoot_motor_measure(uint8_t num, uint8_t data[8]);
    void can_cmd_shoot_motor(int16_t left_fric, int16_t right_fric, int16_t trigger, int16_t cover); //动力电机数据
    const motor_measure_t *get_shoot_motor_measure_point(uint8_t i);

    /*-------------------达妙IMU发送--------------------*/
    void imu_send_cmd(uint8_t id, uint8_t reg_id,uint8_t ac,uint32_t data);

};

#endif
