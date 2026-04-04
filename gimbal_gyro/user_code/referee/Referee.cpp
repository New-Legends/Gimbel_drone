#include "Referee.h"


#include "main.h"
#include "bsp_usart.h"
#include "cmsis_os.h"
//#include "Ui.h"
//#include "CRC8_CRC16.H"

#ifdef __cplusplus
extern "C"
{
#endif

#include "CRC8_CRC16.h"
#include "fifo.h" 

#ifdef __cplusplus
}
#endif

extern UART_HandleTypeDef huart6;

unpack_data_t referee_unpack_obj;

static inline void copy_referee_payload(void *dst, size_t dst_size, const uint8_t *src, uint16_t payload_size)
{
    memset(dst, 0, dst_size);
    size_t copy_size = (payload_size < dst_size) ? payload_size : dst_size;
    memcpy(dst, src, copy_size);
}

void Referee::init(){

    init_referee_struct_data();
    fifo_s_init(&referee_fifo, referee_fifo_buf, REFEREE_FIFO_BUF_LENGTH);
    usart6_init(usart6_buf[0], usart6_buf[1], USART_RX_BUF_LENGHT);
}

void Referee::unpack(){

    uint8_t byte = 0;
    uint8_t sof = HEADER_SOF;
    unpack_data_t *p_obj = &referee_unpack_obj;

    while (fifo_s_used(&referee_fifo))
    {
        byte = fifo_s_get(&referee_fifo);
        switch (p_obj->unpack_step)
        {
        case STEP_HEADER_SOF:
        {
            if (byte == sof)
            {
                p_obj->unpack_step = STEP_LENGTH_LOW;
                p_obj->protocol_packet[p_obj->index++] = byte;
            }
            else
            {
                p_obj->index = 0;
            }
        }
        break;

        case STEP_LENGTH_LOW:
        {
            p_obj->data_len = byte;
            p_obj->protocol_packet[p_obj->index++] = byte;
            p_obj->unpack_step = STEP_LENGTH_HIGH;
        }
        break;

        case STEP_LENGTH_HIGH:
        {
            p_obj->data_len |= (byte << 8);
            p_obj->protocol_packet[p_obj->index++] = byte;

            if (p_obj->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN))
            {
                p_obj->unpack_step = STEP_FRAME_SEQ;
            }
            else
            {
                p_obj->unpack_step = STEP_HEADER_SOF;
                p_obj->index = 0;
            }
        }
        break;
        case STEP_FRAME_SEQ:
        {
            p_obj->protocol_packet[p_obj->index++] = byte;
            p_obj->unpack_step = STEP_HEADER_CRC8;
        }
        break;

        case STEP_HEADER_CRC8:
        {
            p_obj->protocol_packet[p_obj->index++] = byte;

            if (p_obj->index == REF_PROTOCOL_HEADER_SIZE)
            {
                if (verify_CRC8_check_sum(p_obj->protocol_packet, REF_PROTOCOL_HEADER_SIZE))
                {
                    p_obj->unpack_step = STEP_DATA_CRC16;
                }
                else
                {
                    p_obj->unpack_step = STEP_HEADER_SOF;
                    p_obj->index = 0;
                }
            }
        }
        break;

        case STEP_DATA_CRC16:
        {
            if (p_obj->index < (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
            {
                p_obj->protocol_packet[p_obj->index++] = byte;
            }
            if (p_obj->index >= (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
            {
                p_obj->unpack_step = STEP_HEADER_SOF;
                p_obj->index = 0;

                if (verify_CRC16_check_sum(p_obj->protocol_packet, REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
                {
                    referee_data_solve(p_obj->protocol_packet);
                }
            }
        }
        break;

        default:
        {
            p_obj->unpack_step = STEP_HEADER_SOF;
            p_obj->index = 0;
        }
        break;
        }
    }
}





void Referee::init_referee_struct_data(){
    memset(&referee_receive_header, 0, sizeof(frame_header_struct_t));
    memset(&referee_send_header, 0, sizeof(frame_header_struct_t));

    memset(&game_state, 0, sizeof(ext_game_status_t));
    memset(&game_result, 0, sizeof(ext_game_result_t));
    memset(&game_robot_HP_t, 0, sizeof(ext_game_robot_HP_t));
    memset(&field_event, 0, sizeof(ext_event_data_t));

    memset(&referee_warning_t, 0, sizeof(ext_referee_warning_t));
    memset(&dart_remaining_time_t, 0, sizeof(ext_dart_info_t));
    memset(&robot_status, 0, sizeof(ext_robot_status_t));
    memset(&power_heat_data_t, 0, sizeof(ext_power_heat_data_t));
    memset(&game_robot_pos_t, 0, sizeof(ext_robot_pos_t));
    memset(&buff_musk_t, 0, sizeof(ext_buff_t));
    memset(&robot_hurt_t, 0, sizeof(ext_hurt_data_t));
    memset(&shoot_data_t, 0, sizeof(ext_shoot_data_t));
    memset(&projectile_allowance_t, 0, sizeof(ext_projectile_allowance_t));
    memset(&rfid_status_t, 0, sizeof(ext_rfid_status_t));


    memset(&dart_client_cmd_t, 0, sizeof(ext_dart_client_cmd_t));
    memset(&ground_robot_position_t, 0, sizeof(ext_ground_robot_position_t));
    memset(&radar_mark_data_t, 0, sizeof(ext_radar_mark_data_t));
    memset(&sentry_info_t, 0, sizeof(ext_sentry_info_t));
    memset(&radar_info_t, 0, sizeof(ext_radar_info_t));

    memset(&student_interactive_data_t, 0, sizeof(ext_robot_interaction_data_t));
    memset(&custom_robot_data, 0,sizeof(ext_custom_robot_data_t));                   
    memset(&map_command, 0,sizeof(ext_map_command_t));
    memset(&remote_control_t, 0,sizeof(ext_remote_control_t));
    memset(&map_robot_data, 0,sizeof(ext_map_robot_data_t));
    memset(&custom_client_data, 0,sizeof(ext_custom_client_data_t));
    memset(&map_data, 0,sizeof(ext_map_data_t));
    memset(&custom_info, 0,sizeof(ext_custom_info_t));
    memset(&robot_custom_data, 0,sizeof(ext_robot_custom_data_t));
    memset(&robot_custom_data_2, 0,sizeof(ext_robot_custom_data_2_t));



}

void Referee::referee_data_solve(uint8_t *frame)
{
    uint16_t cmd_id = 0;

    uint8_t index = 0;

    memcpy(&referee_receive_header, frame, sizeof(frame_header_struct_t));

    index += sizeof(frame_header_struct_t);

    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);
    uint16_t payload_size = referee_receive_header.data_length;

    switch (cmd_id)
    {
        case GAME_STATE_CMD_ID:
        {
            copy_referee_payload(&game_state, sizeof(game_state), frame + index, payload_size);
        }
        break;
        case GAME_RESULT_CMD_ID:
        {
            copy_referee_payload(&game_result, sizeof(game_result), frame + index, payload_size);
        }
        break;
        case GAME_ROBOT_HP_CMD_ID:
        {
            copy_referee_payload(&game_robot_HP_t, sizeof(game_robot_HP_t), frame + index, payload_size);
        }
        break;

        case FIELD_EVENTS_CMD_ID:
        {
            copy_referee_payload(&field_event, sizeof(field_event), frame + index, payload_size);
        }
        break;

        case REFEREE_WARNING_CMD_ID:
        {
            copy_referee_payload(&referee_warning_t, sizeof(referee_warning_t), frame + index, payload_size);
        }
        break;
    
        case DART_REMAINING_TIME_CMD_ID:
        {
            copy_referee_payload(&dart_remaining_time_t, sizeof(dart_remaining_time_t), frame + index, payload_size);
        }
        break;

        case ROBOT_STATE_CMD_ID:
        {
            copy_referee_payload(&robot_status, sizeof(robot_status), frame + index, payload_size);
        }
        break;
        case POWER_HEAT_DATA_CMD_ID:
        {
            copy_referee_payload(&power_heat_data_t, sizeof(power_heat_data_t), frame + index, payload_size);
        }
        break;
        case ROBOT_POS_CMD_ID:
        {
            copy_referee_payload(&game_robot_pos_t, sizeof(game_robot_pos_t), frame + index, payload_size);
        }
        break;
        case BUFF_MUSK_CMD_ID:
        {
            copy_referee_payload(&buff_musk_t, sizeof(buff_musk_t), frame + index, payload_size);
        }
        break;

        case ROBOT_HURT_CMD_ID:
        {
            copy_referee_payload(&robot_hurt_t, sizeof(robot_hurt_t), frame + index, payload_size);
        }
        break;
        case SHOOT_DATA_CMD_ID:
        {
            copy_referee_payload(&shoot_data_t, sizeof(shoot_data_t), frame + index, payload_size);
        }
        break;
        case BULLET_REMAINING_CMD_ID:
        {
            copy_referee_payload(&projectile_allowance_t, sizeof(projectile_allowance_t), frame + index, payload_size);
        }
        break;
        case RFID_STATUS_CMD_ID:
        {
            copy_referee_payload(&rfid_status_t, sizeof(rfid_status_t), frame + index, payload_size);
        }
        break;


        case DART_CLIENT_CMD_ID:
        {
            copy_referee_payload(&dart_client_cmd_t, sizeof(dart_client_cmd_t), frame + index, payload_size);
        }
        break;
        case GROUND_ROBOT_POSITION_CMD_ID:
        {
            copy_referee_payload(&ground_robot_position_t, sizeof(ground_robot_position_t), frame + index, payload_size);
        }
        break;
        case RADAR_MARK_DATA_CMD_ID:
        {
            copy_referee_payload(&radar_mark_data_t, sizeof(radar_mark_data_t), frame + index, payload_size);
        }
        break;
    
        case SENTRY_INFO_CMD_ID:
        {
            copy_referee_payload(&sentry_info_t, sizeof(sentry_info_t), frame + index, payload_size);
        }
        break;
        case RADAR_INFO_CMD_ID:
        {
            copy_referee_payload(&radar_info_t, sizeof(radar_info_t), frame + index, payload_size);
        }
        break;

        case STUDENT_INTERACTIVE_DATA_CMD_ID:
        {
            copy_referee_payload(&student_interactive_data_t, sizeof(student_interactive_data_t), frame + index, payload_size);
        }
        break;


        case ROBOT_INTERACTION_DATA_CMD_ID:
        {
            copy_referee_payload(&custom_robot_data, sizeof(custom_robot_data), frame + index, payload_size);
        }
        break;

        case MAP_COMMAND_CMD_ID:
        {
            copy_referee_payload(&map_command, sizeof(map_command), frame + index, payload_size);
        }
        break;
        case KEYBOARD_MOUSE_REMOTE_CONTROL_CMD_ID:
        {
            copy_referee_payload(&remote_control_t, sizeof(remote_control_t), frame + index, payload_size);
        }
        break;

        case MAP_ROBOT_DATA_CMD_ID:
        {
            copy_referee_payload(&map_robot_data, sizeof(map_robot_data), frame + index, payload_size);
        }
        break;

        case CUSTOM_CLIENT_DATA_CMD_ID:
        {
            copy_referee_payload(&custom_client_data, sizeof(custom_client_data), frame + index, payload_size);
        }
        break;

        case MAP_DATA_CMD_ID:
        {
            copy_referee_payload(&map_data, sizeof(map_data), frame + index, payload_size);
        }
        break;

        case CUSTOM_INFO_CMD_ID:
        {
            copy_referee_payload(&custom_info, sizeof(custom_info), frame + index, payload_size);
        }
        break;

        case ROBOT_CUSTOM_DATA_CMD_ID:
        {
            copy_referee_payload(&robot_custom_data, sizeof(robot_custom_data), frame + index, payload_size);
        }
        break;

        case ROBOT_CUSTOM_DATA_2_CMD_ID:
        {
            copy_referee_payload(&robot_custom_data_2, sizeof(robot_custom_data_2), frame + index, payload_size);
        }
        break;


        default:
        {
            break;
        }
    }
}

//返回机器人ID
void Referee::get_robot_id(uint8_t *color)
{
    *color = robot_status.robot_id;
}

//************************功率控制***********************************
//底盘输出功率,底盘功率缓存
void Referee::get_chassis_power_and_buffer(fp32 *power, fp32 *buffer)
{
   *power = power_heat_data_t.reserved_3;
   *buffer = power_heat_data_t.buffer_energy;
}

//底盘输出功率,底盘功率缓存
void Referee::get_chassis_power_limit(fp32 *power_limit)
{
    *power_limit = robot_status.chassis_power_limit;
    
}

void Referee::send_sentry_cmd(uint8_t bit0)
{
    static uint8_t seq = 0;
    sentry_interaction.frame_header.sof = 0x5A;
    sentry_interaction.frame_header.data_length = sizeof(sentry_interaction.robot_interaction_data_t);
    sentry_interaction.frame_header.seq = seq++;
    sentry_interaction.frame_header.crc8 = get_CRC8_check_sum((uint8_t *)(&sentry_interaction.frame_header),4,0xff);
    sentry_interaction.cmd_id = 0x301;
    //sentry_interaction.cmd = bit0; //0是不复活1是复活
    sentry_interaction.robot_interaction_data_t.data_cmd_id = 0x120;
    sentry_interaction.robot_interaction_data_t.sender_id = Judge_Self_ID;
    sentry_interaction.robot_interaction_data_t.receiver_id = 0x8080;
    sentry_interaction.robot_interaction_data_t.sentry_cmd = bit0;
    sentry_interaction.frame_tail = get_CRC16_check_sum((uint8_t *)(&sentry_interaction),sizeof(sentry_interaction)-2,0xFFFF);
    HAL_UART_Transmit(&huart6,(uint8_t *)(&sentry_interaction),sizeof(sentry_interaction),0xFFF);
    while (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_TXE) == RESET);	
}

//0x201
//17mm枪口热量上限, 17mm枪口实时热量 默认ID1
void Referee::get_shooter_id1_17mm_cooling_limit_and_heat(uint16_t *id1_17mm_cooling_limit, uint16_t *id1_17mm_cooling_heat)
{
    *id1_17mm_cooling_limit = robot_status.shooter_barrel_heat_limit;
    //*id1_17mm_cooling_heat = power_heat_data_t.shooter_id1_17mm_cooling_heat;
}

void Referee::get_shooter_id2_17mm_cooling_limit_and_heat(uint16_t *id2_17mm_cooling_limit, uint16_t *id2_17mm_cooling_heat)
{
    *id2_17mm_cooling_limit = robot_status.shooter_barrel_heat_limit;
    *id2_17mm_cooling_heat = 0;
}

//17mm枪口热量冷却 默认ID1
void Referee::get_shooter_id1_17mm_cooling_rate(uint16_t *id1_17mm_cooling_rate)
{
    *id1_17mm_cooling_rate = robot_status.shooter_barrel_cooling_value;
}

//42mm枪口热量上限, 42mm枪口实时热量
void Referee::get_shooter_id1_42mm_cooling_limit_and_heat(uint16_t *id1_42mm_cooling_limit, uint16_t *id1_42mm_cooling_heat)
{
    *id1_42mm_cooling_limit = robot_status.shooter_barrel_heat_limit;
    //*id1_42mm_cooling_heat = power_heat_data_t.shooter_id1_42mm_cooling_heat;
}

//42mm枪口热量冷却
void Referee::get_shooter_id1_42mm_cooling_rate(uint16_t *id1_42mm_cooling_rate)
{
    *id1_42mm_cooling_rate = robot_status.shooter_barrel_cooling_value;
}

//当前血量
void Referee::get_remain_hp(uint16_t *current_HP)
{
    *current_HP = robot_status.current_HP;
}

//获取各部分是否上电
void Referee::get_power_mode(uint8_t *power_mode)
{
    *power_mode = robot_status.power_management_gimbal_output | robot_status.power_management_chassis_output<<1 | robot_status.power_management_shooter_output<<2;
}

//获取被击打数据
void Referee::get_by_hurt(uint8_t *by_hit)
{
    //*by_hit = robot_hurt_t.armor_type;
}

//获取剩余发射子弹数量
void Referee::get_projectile_allowance_17mm(uint16_t *projectile_allowance)
{
    *projectile_allowance = projectile_allowance_t.projectile_allowance_17mm;
}

//是否被击打
bool_t Referee::if_hit()
{
    //static uint16_t hp_detect_time = 0; //血量检测间隔
    static uint16_t last_hp = 0;

    //初始化血量记录
    if (last_hp == 0)
        last_hp = robot_status.current_HP;

    // if (hp_detect_time++ > 300)
    // {
    //     last_hp = robot_state.current_HP;
    //     hp_detect_time = 0;
    // }

    //受到高于5点的伤害,开始扭腰
    if (last_hp - robot_status.current_HP >= 5)
    {
		last_hp = robot_status.current_HP;
		return TRUE;
	}
    else
        return FALSE;
}

//

/**
  * @brief  判断自己红蓝方
  * @param  void
  * @retval RED   BLUE
  * @attention  数据打包,打包完成后通过串口发送到裁判系统
  */
void Referee::get_color(uint8_t *color)
{
    Judge_Self_ID = robot_status.robot_id; //读取当前机器人ID

    if (robot_status.robot_id > 10)
    {
        *color = BLUE;
    }
    else
    {
        *color = RED;
    }
}
/**
  * @brief  判断自身ID，选择客户端ID
  * @param  void
  * @retval RED   BLUE
  * @attention  数据打包,打包完成后通过串口发送到裁判系统
  */
void Referee::determine_ID(void)
{
    get_color(&Color);
    if (Color == BLUE)
    {
        Judge_SelfClient_ID = 0x0110 + (Judge_Self_ID - 0x10); //计算客户端ID
    }
    else if (Color == RED)
    {
        Judge_SelfClient_ID = 0x0100 + Judge_Self_ID; //计算客户端ID
    }
}

void Referee::leida_data_unpack(void)
{
    memcpy(&leida_data, student_interactive_data_t.user_data, sizeof(leida_data));
}



/***  用户函数  ****/

/**
  * @brief  返回机器人射击相关数据
  */

uint16_t* Referee::get_shoot_limit()
{
    return (uint16_t*)&(robot_status.shooter_barrel_heat_limit);
}

uint16_t* Referee::get_shoot_17mm_barrel_heat()
{
    return (uint16_t*)&(power_heat_data_t.shooter_17mm_barrel_heat);
}

uint16_t* Referee::get_shoot_barrel_cooling_value()
{
    return (uint16_t*)&(robot_status.shooter_barrel_cooling_value);
}


