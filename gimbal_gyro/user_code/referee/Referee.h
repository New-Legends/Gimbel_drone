#ifndef REFEREE_H
#define REFEREE_H

#include "main.h"

#include "struct_typedef.h"

#ifdef __cplusplus
extern "C"
{
#endif

#include "fifo.h"

#ifdef __cplusplus
}
#endif

#define USART_RX_BUF_LENGHT 512
#define REFEREE_FIFO_BUF_LENGTH 1024

#define HEADER_SOF 0xA5
#define REF_PROTOCOL_FRAME_MAX_SIZE 128

#define REF_PROTOCOL_HEADER_SIZE sizeof(frame_header_struct_t)
#define REF_PROTOCOL_CMD_SIZE 2
#define REF_PROTOCOL_CRC16_SIZE 2
#define REF_HEADER_CRC_LEN (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define REF_HEADER_CRC_CMDID_LEN (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
#define REF_HEADER_CMDID_LEN (REF_PROTOCOL_HEADER_SIZE + sizeof(uint16_t))

#pragma pack(push, 1)

typedef enum
{
    GAME_STATE_CMD_ID = 0x0001,
    GAME_RESULT_CMD_ID = 0x0002,
    GAME_ROBOT_HP_CMD_ID = 0x0003,
    FIELD_EVENTS_CMD_ID = 0x0101,
    REFEREE_WARNING_CMD_ID = 0x0104,
    DART_REMAINING_TIME_CMD_ID = 0x0105,
    ROBOT_STATE_CMD_ID = 0x0201,
    POWER_HEAT_DATA_CMD_ID = 0x0202,
    ROBOT_POS_CMD_ID = 0x0203,
    BUFF_MUSK_CMD_ID = 0x0204,
    ROBOT_HURT_CMD_ID = 0x0206,
    SHOOT_DATA_CMD_ID = 0x0207,
    BULLET_REMAINING_CMD_ID = 0x0208,
    RFID_STATUS_CMD_ID = 0x0209,

    DART_CLIENT_CMD_ID = 0x020A,
    GROUND_ROBOT_POSITION_CMD_ID = 0x020B,
    RADAR_MARK_DATA_CMD_ID = 0x020C,
    SENTRY_INFO_CMD_ID = 0x020D,
    RADAR_INFO_CMD_ID = 0x020E,

    STUDENT_INTERACTIVE_DATA_CMD_ID = 0x0301,
    ROBOT_INTERACTION_DATA_CMD_ID = 0x0302,
    MAP_COMMAND_CMD_ID = 0X0303,
    KEYBOARD_MOUSE_REMOTE_CONTROL_CMD_ID = 0X0304,
    MAP_ROBOT_DATA_CMD_ID = 0x0305,
    CUSTOM_CLIENT_DATA_CMD_ID = 0X306,
    MAP_DATA_CMD_ID = 0X0307,
    CUSTOM_INFO_CMD_ID = 0X0308,
    ROBOT_CUSTOM_DATA_CMD_ID = 0X0309,
    ROBOT_CUSTOM_DATA_2_CMD_ID = 0X0310,

    IDCustomData,
} referee_cmd_id_t;
typedef struct
{
    uint8_t SOF;
    uint16_t data_length;
    uint8_t seq;
    uint8_t CRC8;
} frame_header_struct_t;

typedef enum
{
    STEP_HEADER_SOF = 0,
    STEP_LENGTH_LOW = 1,
    STEP_LENGTH_HIGH = 2,
    STEP_FRAME_SEQ = 3,
    STEP_HEADER_CRC8 = 4,
    STEP_DATA_CRC16 = 5,
} unpack_step_e;

typedef struct
{
    frame_header_struct_t *p_header;
    uint16_t data_len;
    uint8_t protocol_packet[REF_PROTOCOL_FRAME_MAX_SIZE];
    unpack_step_e unpack_step;
    uint16_t index;
} unpack_data_t;

#pragma pack(pop)

//***********************数据包结构体*******************//

#define RED 0
#define BLUE 1

//机器人ID
typedef enum
{
    RED_HERO = 1,
    RED_ENGINEER = 2,
    RED_STANDARD_1 = 3,
    RED_STANDARD_2 = 4,
    RED_STANDARD_3 = 5,
    RED_AERIAL = 6,
    RED_SENTRY = 7,
    RED_RADAR = 9,
    BLUE_HERO = 101,
    BLUE_ENGINEER = 102,
    BLUE_STANDARD_1 = 103,
    BLUE_STANDARD_2 = 104,
    BLUE_STANDARD_3 = 105,
    BLUE_AERIAL = 106,
    BLUE_SENTRY = 107,
    BLUE_RADAR = 109,
} robot_id_t;


//比赛状态
typedef enum
{
    PROGRESS_UNSTART = 0,
    PROGRESS_PREPARE = 1,
    PROGRESS_SELFCHECK = 2,
    PROGRESS_5sCOUNTDOWN = 3,
    PROGRESS_BATTLE = 4,
    PROGRESS_CALCULATING = 5,
} game_progress_t;


typedef __packed struct // 0x0001
{
    /*game_type:
    0 1 RoboMaster 机甲大师赛
    0 1 RoboMaster 机甲大师单项赛
    0 1 ICRA RoboMaster 人工智能挑战赛
    0 1 RoboMaster 联盟赛 3V3
    0 1 RoboMaster 联盟赛 1V1
     */
    uint8_t game_type : 4;
    /*game_progress:
    0 1 未开始比赛
    0 1 准备阶段
    0 1 自检阶段
    0 1 5s 倒计时
    0 1 对战中
    0 1 比赛结算中
     */
    uint8_t game_progress : 4;
    /*stage_remain_time:
    1 2 当前阶段剩余时间*/
    uint16_t stage_remain_time;
    /*SyncTimeStamp:
    3 8 机器人接收到该指令的精确 Unix 时间，当机载端收到有效的 NTP 服务器授时后生效*/
    uint64_t SyncTimeStamp;

} ext_game_status_t;



typedef __packed struct // 0x0002
{
    /*winner:
    0 1 * 0 平局 1 红方胜利 2 蓝方胜利
     */
    uint8_t winner;

} ext_game_result_t;


typedef __packed struct // 0x0003
{
    /*red_base_HP:
    * 0 2   己方 1 号英雄机器人血量，若该机器人未上场或者被罚下，则血量为 0，下文同理 
    * 2 2   己方 2 号工程机器人血量 
    * 4 2   己方 3 号步兵机器人血量 
    * 6 2   己方 4 号步兵机器人血量 
    * 8 2   保留位 
    * 10 2  己方 7 号哨兵机器人血量 
    * 12 2  己方前哨站血量 
    * 14 2  己方基地血量 
    */
    uint16_t ally_1_robot_HP; 
    uint16_t ally_2_robot_HP; 
    uint16_t ally_3_robot_HP; 
    uint16_t ally_4_robot_HP; 
    uint16_t reserved; 
    uint16_t ally_7_robot_HP; 
    uint16_t ally_outpost_HP; 
    uint16_t ally_base_HP;

} ext_game_robot_HP_t;



typedef __packed struct // 0x0101
{
    /*event_type:
            0：未占领/未激活
            1：已占领/已激活
            bit 0-2：
            bit 0：己方补给区的占领状态，1 为已占领
            bit 1：保留位
            bit 2：己方补给区的占领状态，1 为已占领（仅 RMUL 适用）
            bit 3-6：己方能量机关状态
            bit 3-4：己方小能量机关的激活状态，0 为未激活，1 为已激活，2 为正在激活
            bit 5-6：己方大能量机关的激活状态，0 为未激活，1 为已激活，2 为正在激活
            bit 7-8：己方中央高地的占领状态，1 为被己方占领，2 为被对方占领 
    0   1   bit 9-10：己方梯形高地的占领状态，1 为已占领 
            bit 11-19：对方飞镖最后一次击中己方前哨站或基地的时间（0-420，开局默认为 0）
            bit 20-22：对方飞镖最后一次击中己方前哨站或基地的具体目标，开局默认为 0，1 为击中前哨站，2 为击中基地固定目标，
                        3 为击中基地随机固定目标，4 为击中基地随机移动目标，5 为击中基地末端移动目标。
            bit 23-24：中心增益点的占领状态，0 为未被占领，1 为被己方占领，2 为被对方占领，3 为被双方占领。（仅 RMUL 适用） 
            bit 25-26：己方堡垒增益点的占领状态，0 为未被占领，1 为被己方占领，2 为被对方占领，3 为被双方占领 
            bit 27-28：己方前哨站增益点的占领状态，0 为未被占领，1 为被己方占领，2 为被对方占领
            bit 29：己方基地增益点的占领状态，1 为已占领 
            bit 30-31：保留位
    */
    uint32_t event_data;

} ext_event_data_t;



typedef __packed struct //0x104
{
    /*己方最后一次受到判罚的等级：
            * 1：双方黄牌
    0   1   * 1：黄牌
            * 2：红牌
            * 3：判负
    */
    uint8_t level;
    /*offending_robot_id:
            * 己方最后一次受到判罚的违规机器人 ID。（如红 1 机器人 ID 为 1，蓝
    1   1   * 1 机器人 ID 为 101） 
            * 判负和双方黄牌时，该值为 0
     */
    uint8_t offending_robot_id;
    /*count
    2   1   *己方最后一次受到判罚的违规机器人对应判罚等级的违规次数。（开局默认*为 0。） 
    */
    uint8_t count;
} ext_referee_warning_t;



typedef __packed struct // 0x0105
{
    /*dart_remaining_time:
    0   1   己方飞镖发射剩余时间，单位：秒
    */
    uint8_t dart_remaining_time;
    /*bit 0-2：
            *最近一次己方飞镖击中的目标，开局默认为 0，1 为击中前哨站，2 为击中
            *基地固定目标，3 为击中基地随机目标，4 为击中基地随机移动目标，5 为击中基地末端移动目标
            *bit 3-5：
            *对方最近被击中的目标累计被击中计数，开局默认为 0，至多为 4
    1   2   *bit 6-8：
            *字节偏移量 大小 说明
            *飞镖此时选定的击打目标，开局默认或未选定/选定前哨站时为 0，选中基
            *地固定目标为 1，选中基地随机目标为 2，选中基地随机移动目标为 3，选中基地末端移动目标为 4
            *bit 9-15：保留
    */
    uint16_t dart_info;
    
} ext_dart_info_t;


 

typedef __packed struct // 0x0201
{
     /*robot_id:
     *本机器人 ID：
     *1：红方英雄机器人
     *2：红方工程机器人
     *3/4/5：红方步兵机器人
     *6：红方空中机器人
     *7：红方哨兵机器人
     *8：红方飞镖机器人
     *9：红方雷达站
     *101：蓝方英雄机器人
     *102：蓝方工程机器人
     *103/104/105：蓝方步兵机器人
     *106：蓝方空中机器人
     *107：蓝方哨兵机器人
     *108：蓝方飞镖机器人
     *109：蓝方雷达站。
     */
    uint8_t robot_id;
     /*robot_level:
     *机器人等级：
     *1：一级
     *2：二级
     *3：三级
     */
    uint8_t robot_level;
    /*current_HP:
     *机器人当前血量*/
    uint16_t current_HP; 
    /*maximum_HP:
     * 机器人上限血量*/
    uint16_t maximum_HP;
    /*shooter_barrel_cooling_value:
     * 机器人枪口热量每秒冷却值*/
    uint16_t shooter_barrel_cooling_value;
    /*shooter_barrel_heat_limit:
     * 机器人枪口热量上限*/
    uint16_t shooter_barrel_heat_limit;
     /*chassis_power_limit
     * 机器人底盘功率限制上限*/
    uint16_t chassis_power_limit; 
     /*主控电源输出情况：
        0 bit：gimbal 口输出： 1 为有 24V 输出，0 为无 24v 输出
        1 bit：chassis 口输出：1 为有 24V 输出，0 为无 24v 输出
        2 bit：shooter 口输出：1 为有 24V 输出，0 为无 24v 输出
     */
    uint8_t power_management_gimbal_output : 1;
    uint8_t power_management_chassis_output : 1; 
    uint8_t power_management_shooter_output : 1;

}ext_robot_status_t;



typedef __packed struct // 0x0202
{
    uint16_t reserved_1; 
    uint16_t reserved_2; 
    float reserved_3; 
    /*chassis_power_buffer:
     * 底盘缓冲能量 单位 J 焦耳*/
    uint16_t buffer_energy;
    /*shooter_17mm_barrel_heat
     * 17mm 发射机构的射击热量*/
    uint16_t shooter_17mm_barrel_heat;

    /*shooter_42mm_barrel_heat
     * 42mm 枪口热量*/
    uint16_t shooter_42mm_barrel_heat;

} ext_power_heat_data_t;



typedef __packed struct // 0x0203
{
    /*x:
     * 本机器人位置 x 坐标，单位 m*/
    float x;
    /*y:
     * 本机器人位置 y 坐标，单位 m*/
    float y;

    /*本机器人测速模块的朝向，单位：度。正北为 0 度*/
    float angle;

} ext_robot_pos_t;




typedef __packed struct // 0x0204
{
    /*recovery_buff
    机器人回血增益（百分比，值为 10 表示每秒恢复血量上限的 10%）*/
    uint8_t recovery_buff;
    /* cooling_buff
    机器人射击热量冷却增益具体值（直接值，值为 x 表示热量冷却增加 x/s） */
    uint8_t cooling_buff;
    /* defence_buff
    机器人防御增益（百分比，值为 50 表示 50%防御增益）*/
    uint8_t defence_buff;
    /* vulnerability_buff
    机器人负防御增益（百分比，值为 30 表示-30%防御增益）*/
    uint8_t vulnerability_buff;
    /* attack_buff
    机器人攻击增益（百分比，值为 50 表示 50%攻击增益）*/
    uint16_t attack_buff;

    /*机器人剩余能量值反馈，以 16 进制标识机器人剩余能量值比例，机器人初始能量视为 100% 
    * bit 0：在剩余能量≥125%时为 1，其余情况为 0 
    * bit 1：在剩余能量≥100%时为 1，其余情况为 0 
    * bit 2：在剩余能量≥50%时为 1，其余情况为 0 
    * bit 3：在剩余能量≥30%时为 1，其余情况为 0 
    * bit 4：在剩余能量≥15%时为 1，其余情况为 0 
    * bit 5：在剩余能量≥5%时为 1，其余情况为 0 
    * bit 6：在剩余能量≥1%时为 1，其余情况为 0 
    */
    uint8_t remaining_energy;

}ext_buff_t;




typedef __packed struct // 0x0206
{
    /*armor_type:
     * bit 0-3：当扣血原因为装甲模块被弹丸攻击、受撞击或离线时，该 4 bit 组成的数值为装甲模块或测速模块的 ID 编号；当其他原因导致扣血时，该数值为 0 
     * hurt_type:
     * bit 4-7：血量变化类型
        0：装甲模块被弹丸攻击导致扣血
        1：装甲模块或超级电容管理模块离线导致扣血
        5：装甲模块受到撞击导致扣血
     */
    uint8_t armor_id : 4; 
    uint8_t HP_deduction_reason : 4;

} ext_hurt_data_t;



typedef __packed struct // 0x0207
{
    /*bullet_type:
     *子弹类型:
     *bit1：17mm 弹丸 
     *bit2：42mm 弹丸
     */
    uint8_t bullet_type;
    /*shooter_id:
     *发射机构 ID：
     * 1：17mm 发射机构
     * 2：保留位
     * 3：42mm 发射机构
     */
    uint8_t shooter_number;
    /*bullet_freq:
     *子弹射速 单位 Hz*/
    uint8_t launching_frequency;
    /*bullet_speed:
     *子弹初速 单位 m/s*/
    float initial_speed;

} ext_shoot_data_t;




typedef __packed struct //0x0208
{
    /*bullet_remaining_num_17mm
     *机器人自身拥有的 17mm 弹丸允许发弹量*/
    uint16_t projectile_allowance_17mm;
    /*bullet_remaining_num_42mm
     *42mm 弹丸允许发弹量 */
    uint16_t projectile_allowance_42mm;
    /*coin_remaining_num
     *剩余金币数量*/
    uint16_t remaining_gold_coin;
    
    /*堡垒增益点提供的储备 17mm 弹丸允许发弹量； 该值与机器人是否实际占领堡垒无关 */
    uint16_t projectile_allowance_fortress; 

}ext_projectile_allowance_t;

typedef __packed struct //0x0209
{
    //bit位值为1/0的含义：是否已检测到该增益点RFID卡
    /*rfid_status:
     *机器人 RFID 状态:
     * bit 0：己方基地增益点 
     * bit 1：己方中央高地增益点 
     * bit 2：对方中央高地增益点 
     * bit 3：己方梯形高地增益点 
     * bit 4：对方梯形高地增益点
     * bit 5：己方地形跨越增益点（飞坡）（靠近己方一侧飞坡前） 
     * bit 6：己方地形跨越增益点（飞坡）（靠近己方一侧飞坡后） 
     * bit 7：对方地形跨越增益点（飞坡）（靠近对方一侧飞坡前） 
     * bit 8：对方地形跨越增益点（飞坡）（靠近对方一侧飞坡后） 
     * bit 9：己方地形跨越增益点（中央高地下方）
     * bit 10：己方地形跨越增益点（中央高地上方） 
     * bit 11：对方地形跨越增益点（中央高地下方） 
     * bit 12：对方地形跨越增益点（中央高地上方） 
     * bit 13：己方地形跨越增益点（公路下方） 
     * bit 14：己方地形跨越增益点（公路上方） 
     * bit 15：对方地形跨越增益点（公路下方） 
     * bit 16：对方地形跨越增益点（公路上方） 
     * bit 17：己方堡垒增益点 
     * bit 18：己方前哨站增益点 
     * bit 19：己方与资源区不重叠的补给区/RMUL 补给区 
     * bit 20：己方与资源区重叠的补给区 
     * bit 21：己方装配增益点 
     * bit 22：对方装配增益点 
     * bit 23：中心增益点（仅 RMUL 适用） 
     * bit 24：对方堡垒增益点 
     * bit 25：对方前哨站增益点 
     * bit 26：己方地形跨越增益点（隧道）（靠近己方一侧公路区下方） 
     * bit 27：己方地形跨越增益点（隧道）（靠近己方一侧公路区中间）
     * bit 28：己方地形跨越增益点（隧道）（靠近己方一侧公路区上方）
     * bit 29：己方地形跨越增益点（隧道）（靠近己方梯形高地较低处）
     * bit 30：对方地形跨越增益点（隧道）（靠近己方梯形高地较中间）
     * bit 31：对方地形跨越增益点（隧道）（靠近己方梯形高地较高处）
     * 
     * 注：所有 RFID 卡仅在赛内生效。在赛外，即使检测到对应的 RFID 卡，对应值也为 0。 
     * */
    uint32_t rfid_status;
    /*bit 0：对方地形跨越增益点（隧道）（靠近对方公路一侧下方） 
    * bit 1：对方地形跨越增益点（隧道）（靠近对方公路一侧中间）
    * bit 2：对方地形跨越增益点（隧道）（靠近对方公路一侧上方）
    * bit 3：对方地形跨越增益点（隧道）（靠近对方梯形高地较低处）
    * bit 4：对方地形跨越增益点（隧道）（靠近对方梯形高地较中间）
    * bit 5：对方地形跨越增益点（隧道）（靠近对方梯形高地较高处）
    */
    uint8_t rfid_status_2;

} ext_rfid_status_t;




typedef __packed struct // 0x020A
{
    /*dart_launch_opening_status
     * 当前飞镖发射口的状态:
     * 1：关闭
     * 2：正在开启或者关闭中
     * 0：已经开启
     * */
    uint8_t dart_launch_opening_status;

    //保留位
    uint8_t reserved;

    /*target_change_time:
     * 切换打击目标时的比赛剩余时间，单位秒，无/未切换动作，默认为0*/
    uint16_t target_change_time;

    /*operate_launch_cmd_time:
     * 最近一次操作手确定发射指令时的比赛剩余时间，单位秒, 初始值为 0*/
    uint16_t latest_launch_cmd_time;

}ext_dart_client_cmd_t;



typedef __packed struct //0x020B
{
    /*场地围挡在红方补给站附近的交点为坐标原点，沿场地长边向蓝方为 X 轴正方向，沿场地短边
    向红方停机坪为 Y 轴正方向,x,y为坐标*/

    /*机器人坐标
    * 己方英雄机器人位置 x 轴坐标，单位：m 
    * 己方英雄机器人位置 y 轴坐标，单位：m 
    * 己方工程机器人位置 x 轴坐标，单位：m 
    * 己方工程机器人位置 y 轴坐标，单位：m 
    * 己方 3 号步兵机器人位置 x 轴坐标，单位：m 
    * 己方 3 号步兵机器人位置 y 轴坐标，单位：m 
    * 己方 4 号步兵机器人位置 x 轴坐标，单位：m 
    * 己方 4 号步兵机器人位置 y 轴坐标，单位：m 
    * 保留位
    * 保留位
    */

    float hero_x; 
    float hero_y;
    float engineer_x;
    float engineer_y;
    float standard_3_x;
    float standard_3_y;
    float standard_4_x;
    float standard_4_y;
    float reserved_1; 
    float reserved_2; 
}ext_ground_robot_position_t;

/*对方的机器人被标记的进度*/

typedef __packed struct //0x020C
{
    /*
    * bit 0：对方 1 号英雄机器人易伤情况 
    * bit 1：对方 2 号工程机器人易伤情况 
    * bit 2：对方 3 号步兵机器人易伤情况 
    * bit 3：对方 4 号步兵机器人易伤情况 
    * bit 4：对方空中机器人特殊标识情况
    * bit 5：对方哨兵机器人易伤情况
    * bit 6：己方 1 号英雄机器人特殊标识情况 
    * bit 7：己方 2 号工程机器人特殊标识情况 
    * bit 8：己方 3 号步兵机器人特殊标识情况 
    * bit 9：己方 4 号步兵机器人特殊标识情况 
    * bit 10：己方空中机器人特殊标识情况
    * bit 11：己方哨兵机器人特殊标识情况
    * bit 12-15：保留位 
    * 
    * 备注：1、对方机器人：在对应机器人被标记进度≥100 时发送 1，被标记进度<100 时发送 0。 
            2、己方机器人：在对应机器人被标记进度≥50 时发送 1，被标记进度<50 时发送 0。 
    */
    uint16_t mark_progress;

}ext_radar_mark_data_t;

typedef __packed struct //0x020D
{
    /*sentry_status:
    * bit 0-10：除远程兑换外，哨兵机器人成功兑换的允许发弹量，开局为 
    *           0，在哨兵机器人成功兑换一定允许发弹量后，该值将变为哨兵机器人
    *           成功兑换的允许发弹量值 
    * bit 11-14：哨兵机器人成功远程兑换允许发弹量的次数，开局为 0，在
    *            哨兵机器人成功远程兑换允许发弹量后，该值将变为哨兵机器人成功远
    *            程兑换允许发弹量的次数 
    * bit 15-18：哨兵机器人成功远程兑换血量的次数，开局为 0，在哨兵机
    *            器人成功远程兑换血量后，该值将变为哨兵机器人成功远程兑换血量的
    *            次数 
    * bit 19：哨兵机器人当前是否可以确认免费复活，可以确认免费复活时
    *         值为 1，否则为 0 
    * bit 20：哨兵机器人当前是否可以兑换立即复活，可以兑换立即复活时
    *         值为 1，否则为 0 
    * bit 21-30：哨兵机器人当前若兑换立即复活需要花费的金币数。 
    * bit 31：保留 
    */
    uint32_t sentry_info;

    /*sentry_info_2:
    * bit 0：哨兵当前是否处于脱战状态，处于脱战状态时为 1，否则为 0 
    * bit 1-11：队伍 17mm 允许发弹量的剩余可兑换数
    * bit 12-13：哨兵当前姿态， 1为进攻姿态， 2为防御姿态， 3为移动姿态
    * bit 14：己方能量机关是否能够进入正在激活状态， 1为当前可激活
    * bit 15：保留位
    */
    uint16_t sentry_info_2;

} ext_sentry_info_t;



typedef __packed struct //0x020E
{
    /*rader_status:
    * bit 0-1：雷达是否拥有触发双倍易伤的机会，开局为 0，数值为雷达拥有触发
    * 双倍易伤的机会，至多为 2
    * bit 2：对方是否正在被触发双倍易伤
    *     0：对方未被触发双倍易伤
    *     1：对方正在被触发双倍易伤
    * bit 3-4：己方加密等级（即对方干扰波难度等级），开局为 1，最高为 3 
    * bit 5：当前是否可以修改密钥，1 为可修改 
    * bit 6-7：保留位 
     * */
    uint8_t radar_info;

} ext_radar_info_t;



//机器人交互数据通过常规链路发送，其数据段包含一个统一的数据段头结构。
typedef __packed struct // 0x0301
{
    //子内容id 需为开放的子内容id
    uint16_t data_cmd_id;

    //发送者id 需与自身id匹配，id编号详见附录
    uint16_t sender_id;

    //接收者id 仅限己方通信 需为规则允许的多机通讯接收者 若接收者为选手端，则仅可发送至发送者对应的选手端 ID编号详见附录
    uint16_t receiver_id;

    //uint16_t 内容数据段 x 最大为 112
    uint8_t user_data[112];

} ext_robot_interaction_data_t;




//---------------------------------------------------------//
    // 0x0F01

    /*设置出图信道并接收设置反馈，设置图传出图信道
    *设置值 1-6：设置信道为 1~6； 
    *反馈值 0：设置信道成功； 
    *反馈值 1：图传启动中，无法设置信道； 
    *反馈值 2：设置信道有误，无法设置。 
    */

//---------------------------------------------------------//


//---------------------------------------------------------//
    // 0x0F02

    /*查询出图信道
    *数据段无需信息，仅发送该命令码即可查询图传出图信道。 
    *反馈值 0：未设置信道； 
    *反馈值 1~6：当前运行的信道。
    */

//---------------------------------------------------------//



//操作手可使用自定义控制器通过图传链路向对应的机器人发送数据。
typedef __packed struct //0x0302
{
    /*自定义数据 30字节*/

    uint8_t data[30];

}ext_custom_robot_data_t;





typedef __packed struct //0x0303
{
    /*目标位置 x 轴坐标，单位 m 当发送目标机器人 ID 时，该值为 0 */
    float target_position_x;

    /*目标位置 y 轴坐标，单位 m当发送目标机器人 ID 时，该值为 0 */
    float target_position_y;

    /*云台手按下的键盘按键通用键值 无按键按下，则为 0 */
    uint8_t cmd_keyboard;

    /*对方机器人 ID 当发送坐标数据时，该值为 0 */
    uint8_t target_robot_id;

    /*信息来源 ID 信息来源的 ID，ID 对应关系详见附录*/
    uint16_t cmd_source;

}ext_map_command_t;



//键鼠控制
typedef __packed struct //0x0304
{
    //鼠标 x 轴移动速度，负值标识向左移动
    int16_t mouse_x;

    //鼠标 y 轴移动速度，负值标识向下移动
    int16_t mouse_y;

    //鼠标滚轮移动速度，负值标识向后滚动
    int16_t mouse_z;

    //鼠标左键是否按下：0 为未按下；1 为按下
    int8_t left_button_down;

    //鼠标右键是否按下：0 为未按下，1 为按下
    int8_t right_button_down;

    /*键盘按键信息，每个 bit 对应一个按键，0 为未按下，1 为按下： 
    bit 0：W 键  bit 1：S 键  bit 2：A 键  bit 3：D 键 
    bit 4：Shift 键  bit 5：Ctrl 键  bit 6：Q 键  bit 7：E 键 
    bit 8：R 键  bit 9：F 键  bit 10：G 键  bit 11：Z 键  
    bit 12：X 键  bit 13：C 键  bit 14：V 键  bit 15：B 键 */
    uint16_t keyboard_value;

    //保留位
    uint16_t reserved;

}ext_remote_control_t;


//选手端小地图可接收机器人数据。 
//雷达可通过常规链路向己方所有选手端发送对方机器人的坐标数据，该位置会在己方选手端小地图显示。 
typedef __packed struct //0x0305
{
    /*对方英雄机器人 x 位置坐标，单位：cm*/
    uint16_t opponent_hero_position_x;

    /*对方英雄机器人 y 位置坐标，单位：cm*/
    uint16_t opponent_hero_position_y;

    /*对方工程机器人 x 位置坐标，单位：cm*/
    uint16_t opponent_engineer_position_x;

    /*对方工程机器人 y 位置坐标，单位：cm*/
    uint16_t opponent_engineer_position_y;

    /*对方3号步兵机器人x位置坐标，单位：cm*/
    uint16_t opponent_infantry_3_position_x;

    /*对方3号步兵机器人y位置坐标，单位：cm*/
    uint16_t opponent_infantry_3_position_y;

    /*对方4号步兵机器人x位置坐标，单位：cm*/
    uint16_t opponent_infantry_4_position_x;

    /*对方4号步兵机器人y位置坐标，单位：cm*/
    uint16_t opponent_infantry_4_position_y;

    /*对方6号空中机器人x位置坐标，单位：cm*/
    uint16_t opponent_aerial_position_x;

    /*对方6号空中机器人y位置坐标，单位：cm*/
    uint16_t opponent_aerial_position_y;

    /*对方哨兵机器人x位置坐标，单位：cm*/
    uint16_t opponent_sentry_position_x;

    /*对方哨兵机器人y位置坐标，单位：cm*/
    uint16_t opponent_sentry_position_y;

    /*己方英雄机器人 x 位置坐标，单位：cm*/
    uint16_t ally_hero_position_x;

    /*己方英雄机器人 y 位置坐标，单位：cm*/
    uint16_t ally_hero_position_y;

    /*己方工程机器人 x 位置坐标，单位：cm*/
    uint16_t ally_engineer_position_x;

    /*己方工程机器人 y 位置坐标，单位：cm*/
    uint16_t ally_engineer_position_y;

    /*己方3号步兵机器人 x 位置坐标，单位：cm*/
    uint16_t ally_infantry_3_position_x;

    /*己方3号步兵机器人 y 位置坐标，单位：cm*/
    uint16_t ally_infantry_3_position_y;

    /*己方4号步兵机器人 x 位置坐标，单位：cm*/
    uint16_t ally_infantry_4_position_x;

    /*己方4号步兵机器人 y 位置坐标，单位：cm*/
    uint16_t ally_infantry_4_position_y;

    /*己方6号空中机器人 x 位置坐标，单位：cm*/
    uint16_t ally_aerial_position_x;

    /*己方6号空中机器人 y 位置坐标，单位：cm*/
    uint16_t ally_aerial_position_y;

    /*己方哨兵机器人 x 位置坐标，单位：cm*/
    uint16_t ally_sentry_position_x;

    /*己方哨兵机器人 y 位置坐标，单位：cm*/
    uint16_t ally_sentry_position_y;

    //备注：当 x、y 超出边界时显示在对应边缘处，当 x、y 均为 0 时，视为未发送此机器人坐标。 

}ext_map_robot_data_t;




//操作手可使用自定义控制器模拟键鼠操作选手端。
typedef __packed struct //0x0306
{
    /*键盘键值： bit 0-7：按键 1 键值   bit 8-15：按键 2 键值 
    *仅响应选手端开放的按键 使用通用键值，支持 2 键无冲，键值顺序变更不会改变按下状态，
    *若无新的按键信息，将保持上一帧数据的按下状态
    */
    uint16_t key_value;

    /*bit 0-11：鼠标 X 轴像素位置  bit 12-15：鼠标左键状态 */
    uint16_t x_position:12;
    uint16_t mouse_left:4;

    /*bit 0-11：鼠标 Y 轴像素位置  bit 12-15：鼠标右键状态*/
    uint16_t y_position:12;
    uint16_t mouse_right:4;

    /*位置信息使用绝对像素点值（赛事客户端使用的分辨率为 1920×1080，屏幕左上角为（0，0）） 
    鼠标按键状态 1 为按下，其他值为未按下，仅在出现鼠标图标后响应该信息，
    若无新的鼠标信息，选手端将保持上一帧数据的鼠标信息，当鼠标图标消失后该数据不再保持 */


    /*保留位*/
    uint16_t reserved;

}ext_custom_client_data_t;




/*哨兵机器人或选择了半自动控制方式的机器人可通过常规链路向对应的操作手选手端发送路径坐标数据，
该路径会在小地图上显示。*/
typedef __packed struct //0x0307
{
    /*1：到目标点攻击 2：到目标点防守 3：移动到目标点*/
    uint8_t intention;

    /*路径起点 x 轴坐标，单位：dm*/
    uint16_t start_position_x;

    /*路径起点 y 轴坐标，单位：dm*/
    uint16_t start_position_y;

    /*路径点 x 轴增量数组，单位：dm*/
    int8_t delta_x[49];

    /*路径点 y 轴增量数组，单位：dm*/
    int8_t delta_y[49];

    /*发送者ID 需与自身 ID 匹配，ID 编号详见附录*/
    uint16_t sender_id;

    /*备注:小地图左下角为坐标原点，水平向右为 X 轴正方向，竖直向上为 Y 轴正方向。
    显示位置将按照场地尺寸与小地图尺寸等比缩放，超出边界的位置将在边界处显示*/

    /*增量相较于上一个点位进行计算，共 49 个新点位，X 与 Y 轴增量对应组成点位*/

}ext_map_data_t;





//己方机器人可通过常规链路向己方任意选手端发送自定义的消息，该消息会在己方选手端特定位置显示。
typedef __packed struct //0x0308
{
    /*发送者的ID  需要校验发送者的 ID 正确性*/
    uint16_t sender_id;

    /*接收者的ID 需要校验接收者的 ID 正确性，仅支持发送己方选手端 */
    uint16_t receiver_id;

    /*以 utf-16 格式编码发送，支持显示中文。编码发送时请注意数据的大小端问题 */
    uint16_t user_data[30];

}ext_custom_info_t;




//机器人可通过图传链路向对应的操作手选手端连接的自定义控制器发送数据（RMUL 暂不适用）
typedef __packed struct //0x0309 
{ 
    uint8_t data[30];

}ext_robot_custom_data_t;




//机器人可通过图传链路向对应的操作手选手端连接的自定义控制器发送数据（RMUL 暂不适用）
typedef __packed struct //0x0310
{ 
    uint8_t data[300];

}ext_robot_custom_data_2_t;



typedef __packed struct
{
    fp32 data1;
    fp32 data2;
    fp32 data3;
    uint8_t data4;
} custom_data_t;

typedef __packed struct
{
    uint8_t data[64];
} ext_up_stream_data_t;

typedef __packed struct
{
    uint8_t data[32];
} ext_download_stream_data_t;

typedef __packed struct
{
    uint8_t point_x;
    uint8_t mode_1;
    float x_aim;
    float y_aim;
    uint8_t mode;
    uint8_t data[4];
} lidar_data_t;

typedef __packed struct
{
    __packed struct
    {
        uint8_t sof;              // 起始字节，固定值为0xA5
        uint16_t data_length;     // 数据帧中data的长度
        uint8_t seq;              // 包序号
        uint8_t crc8;             // 帧头CRC8校验
    } frame_header;               // 帧头
    __packed uint16_t cmd_id;     // 命令码
    __packed uint16_t cmd;    // 哨兵命令数据
    __packed uint16_t frame_tail; // 帧尾CRC16校验
} sentry_cmd_t;  

typedef __packed struct
{
    __packed struct
    {
        uint8_t sof;              // 起始字节，固定值为0xA5
        uint16_t data_length;     // 数据帧中data的长度
        uint8_t seq;              // 包序号
        uint8_t crc8;             // 帧头CRC8校验
    } frame_header;               // 帧头
    __packed uint16_t cmd_id;     // 命令码
    __packed struct
    {
        uint16_t data_cmd_id;
        uint16_t sender_id;
        uint16_t receiver_id;
        uint32_t sentry_cmd;
    }robot_interaction_data_t;
    __packed uint16_t frame_tail; // 帧尾CRC16校验
}sentry_interaction_t;




extern UART_HandleTypeDef huart6;

class Referee
{

public:
    uint8_t usart6_buf[2][USART_RX_BUF_LENGHT];
    fifo_s_t referee_fifo;
    uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];

    //裁判结构体数据
    frame_header_struct_t referee_receive_header;                //接受报文
    frame_header_struct_t referee_send_header;                   //发送报文
    /*0x10?*/
    ext_game_status_t game_state;                                //比赛状态
    ext_game_result_t game_result;                               //比赛结果
    ext_game_robot_HP_t game_robot_HP_t;                         //场上机器人的血量
    ext_event_data_t field_event;                                //场地事件数据
    ext_referee_warning_t referee_warning_t;                     //裁判警告信息  己方警告发生后发送
    ext_dart_info_t dart_remaining_time_t;                       //飞镖发射口倒计时
    ext_robot_status_t robot_status;                             //机器人状态
    ext_power_heat_data_t power_heat_data_t;                     //实时功率热量数据
    ext_robot_pos_t game_robot_pos_t;                            //机器人位置
    ext_buff_t buff_musk_t;                                      //机器人增益
    ext_hurt_data_t robot_hurt_t;                                //机器人受伤扣血类型
    ext_shoot_data_t shoot_data_t;                               //实时射击信息
    ext_projectile_allowance_t projectile_allowance_t;           //剩余允许发弹量
    ext_rfid_status_t rfid_status_t;                             //RFID状态
    /*0x20?*/
    ext_dart_client_cmd_t dart_client_cmd_t;                     //飞镖发射架状态
    ext_ground_robot_position_t ground_robot_position_t;         //机器人坐标位置
    ext_radar_mark_data_t radar_mark_data_t;                     //对方机器人被标记进度 
    ext_sentry_info_t sentry_info_t;                             //哨兵兑换状态
    ext_radar_info_t  radar_info_t ;                            //雷达易伤状态
    /*0x30?*/
    ext_robot_interaction_data_t student_interactive_data_t;     //机器人交互数据通过常规链路发送
    ext_custom_robot_data_t custom_robot_data;                   //自定义控制器通过常规链路发送数据
    ext_map_command_t map_command;                               //地图云台手地图标点
    ext_remote_control_t remote_control_t;                       //键鼠控制数据
    ext_map_robot_data_t map_robot_data;                         //选手端小地图对方机器人坐标
    ext_custom_client_data_t custom_client_data;                 //自定义控制器模拟键鼠
    ext_map_data_t map_data;                                     //操作手发送哨兵路径坐标数据
    ext_custom_info_t custom_info;                               //机器人发送数据给选手端
    ext_robot_custom_data_t robot_custom_data;                   //机器人向自定义控制器发送数据
    ext_robot_custom_data_2_t robot_custom_data_2;               //机器人向自定义控制器发送数据

    lidar_data_t leida_data;                                     //雷达通讯坐标

    sentry_cmd_t sentry_cmd;                                     //哨兵自主决策指令

    sentry_interaction_t sentry_interaction;                     //哨兵自主决策


    uint8_t Judge_Self_ID;                                       //当前机器人的ID
    uint16_t Judge_SelfClient_ID;                                //发送者机器人对应的客户端ID

    uint8_t Color;

    void init();

    void unpack(); //对数据进行解包

    void init_referee_struct_data();

    void referee_data_solve(uint8_t *frame);

    //*************************数据查询接口*********************************//
    //返回机器人ID
    void get_robot_id(uint8_t *color);
    //底盘输出功率,底盘功率缓存
    void get_chassis_power_and_buffer(fp32 *power, fp32 *buffer);

    void get_chassis_power_limit(fp32 *power_limit);

    // 17mm枪口热量上限, 17mm枪口实时热量 默认ID1
    void get_shooter_id1_17mm_cooling_limit_and_heat(uint16_t *id1_17mm_cooling_limit, uint16_t *id1_17mm_cooling_heat);
    void get_shooter_id2_17mm_cooling_limit_and_heat(uint16_t *id2_17mm_cooling_limit, uint16_t *id2_17mm_cooling_heat);
    // 17mm枪口枪口射速上限,17mm实时射速 默认ID1
    //void get_shooter_id1_17mm_speed_limit_and_bullet_speed(uint16_t *id1_17mm_speed_limit, fp32 *bullet_speed);
    // 17mm枪口热量冷却 默认ID1
    void get_shooter_id1_17mm_cooling_rate(uint16_t *id1_17mm_cooling_rate);
    // 42mm枪口热量上限, 42mm枪口实时热量
    void get_shooter_id1_42mm_cooling_limit_and_heat(uint16_t *id1_42mm_cooling_limit, uint16_t *id1_42mm_cooling_heat);
    // 42mm枪口枪口射速上限,42mm实时射速
    //void get_shooter_id1_42mm_speed_limit_and_bullet_speed(uint16_t *id1_42mm_speed_limit, fp32 *bullet_speed);
    // 42mm枪口热量冷却
    void get_shooter_id1_42mm_cooling_rate(uint16_t *id1_42mm_cooling_rate);
    //当前血量
    void get_remain_hp(uint16_t *remain_HP);
    //获取当前阵营颜色
    void get_color(uint8_t *color);
    //获取关节上电情况
    void get_power_mode(uint8_t *power_mode);
    //获取被击打数据
    void get_by_hurt(uint8_t *bt_hit);
    //获取剩余发射子弹数量
    void get_projectile_allowance_17mm(uint16_t *projectile_allowance);

    //是否被击打
    bool_t if_hit();

    //哨兵发送指令
    void send_sentry_cmd(uint8_t bit0);

    void leida_data_unpack(void);

    void determine_ID(void);
};

#endif

