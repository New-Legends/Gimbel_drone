#include "imudm.h"
#include "Can_receive.h"
#include "Communicate.h"
#include "user_lib.h"
#include <string.h>

imu_t imudm;
extern Can_receive can_receive;


void imu_init(uint8_t can_id,uint8_t mst_id,CAN_HandleTypeDef *hfdcan)
{
	imudm.can_id=can_id;
	imudm.mst_id=mst_id;
	imudm.can_handle=hfdcan;
}


void imu_write_reg(uint8_t reg_id,uint32_t data)
{
	can_receive.imu_send_cmd(imudm.can_id,reg_id,CMD_WRITE,data);
}

void imu_read_reg(uint8_t reg_id)
{
	can_receive.imu_send_cmd(imudm.can_id,reg_id,CMD_READ,0);
}

void imu_reboot()
{
	imu_write_reg(REBOOT_IMU,0);
}

void imu_accel_calibration()
{
	imu_write_reg(ACCEL_CALI,0);
}

void imu_gyro_calibration()
{
	imu_write_reg(GYRO_CALI,0);
}


void imu_change_com_port(imu_com_port_e port)
{
	imu_write_reg(CHANGE_COM,(uint8_t)port);
}

void imu_set_active_mode_delay(uint32_t delay)
{
	imu_write_reg(SET_DELAY,delay);
}

//设置成主动模式
void imu_change_to_active()
{
	imu_write_reg(CHANGE_ACTIVE,1);
}

void imu_set_active_output_type(uint8_t type)
{
    // 写寄存器0x0C，设置输出类型
    imu_write_reg(DATA_OUTPUT_SELECTION, type);
}

void imu_change_to_request()
{
	imu_write_reg(CHANGE_ACTIVE,0);
}

void imu_set_baud(imu_baudrate_e baud)
{
	imu_write_reg(SET_BAUD,(uint8_t)baud);
}

void imu_set_can_id(uint8_t can_id)
{
	imu_write_reg(SET_CAN_ID,can_id);
}

void imu_set_mst_id(uint8_t mst_id)
{
	imu_write_reg(SET_MST_ID,mst_id);
}

void imu_save_parameters()
{
	imu_write_reg(SAVE_PARAM,0);
}

void imu_restore_settings()
{
	imu_write_reg(RESTORE_SETTING,0);
}


void imu_request_accel()
{
	imu_read_reg(ACCEL_DATA);
}

void imu_request_gyro()
{
	imu_read_reg(GYRO_DATA);
}

void imu_request_euler()
{
	imu_read_reg(EULER_DATA);
}

void imu_request_quat()
{
	imu_read_reg(QUAT_DATA);
}



void IMU_UpdateAccel(uint8_t* pData)
{
	uint16_t accel[3];
	
	accel[0]=pData[3]<<8|pData[2];
	accel[1]=pData[5]<<8|pData[4];
	accel[2]=pData[7]<<8|pData[6];
	
	imudm.accel[0]=uint_to_float(accel[0],ACCEL_CAN_MIN,ACCEL_CAN_MAX,16);
	imudm.accel[1]=uint_to_float(accel[1],ACCEL_CAN_MIN,ACCEL_CAN_MAX,16);
	imudm.accel[2]=uint_to_float(accel[2],ACCEL_CAN_MIN,ACCEL_CAN_MAX,16);
	
}

void IMU_UpdateGyro(uint8_t* pData)
{
	uint16_t gyro[3];
	
	gyro[0]=pData[3]<<8|pData[2];
	gyro[1]=pData[5]<<8|pData[4];
	gyro[2]=pData[7]<<8|pData[6];
	
	imudm.gyro[0]=uint_to_float(gyro[0],GYRO_CAN_MIN,GYRO_CAN_MAX,16)/angle_to_rad;
	imudm.gyro[1]=uint_to_float(gyro[1],GYRO_CAN_MIN,GYRO_CAN_MAX,16)/angle_to_rad;
	imudm.gyro[2]=uint_to_float(gyro[2],GYRO_CAN_MIN,GYRO_CAN_MAX,16)/angle_to_rad;
}


void IMU_UpdateEuler(uint8_t* pData)
{
	int euler[3];
	
	euler[0]=pData[3]<<8|pData[2];
	euler[1]=pData[5]<<8|pData[4];
	euler[2]=pData[7]<<8|pData[6];
	
	imudm.angle[1]=(uint_to_float(euler[0],PITCH_CAN_MIN,PITCH_CAN_MAX,16))	/angle_to_rad;
	imudm.angle[2]=(uint_to_float(euler[1],YAW_CAN_MIN,YAW_CAN_MAX,16))		/angle_to_rad;
	imudm.angle[0]=(uint_to_float(euler[2],ROLL_CAN_MIN,ROLL_CAN_MAX,16))	/angle_to_rad;
}


void IMU_UpdateQuaternion(uint8_t* pData)
{
	int w = pData[1]<<6| ((pData[2]&0xF8)>>2);
	int x = (pData[2]&0x03)<<12|(pData[3]<<4)|((pData[4]&0xF0)>>4);
	int y = (pData[4]&0x0F)<<10|(pData[5]<<2)|(pData[6]&0xC0)>>6;
	int z = (pData[6]&0x3F)<<8|pData[7];
	
	imudm.q[0] = uint_to_float(w,Quaternion_MIN,Quaternion_MAX,14);
	imudm.q[1] = uint_to_float(x,Quaternion_MIN,Quaternion_MAX,14);
	imudm.q[2] = uint_to_float(y,Quaternion_MIN,Quaternion_MAX,14);
	imudm.q[3] = uint_to_float(z,Quaternion_MIN,Quaternion_MAX,14);
}

fp32* get_IMU_angle_point(void)
{
	return imudm.angle;
}

fp32* get_IMU_gyro_point(void)
{
	return imudm.gyro;
}

fp32* get_IMU_Quaternion_point(void)
{
	return imudm.q;
}

void IMU_UpdateData(uint8_t* pData)
{
	static uint32_t last_recive_time;
	uint32_t time = HAL_GetTick();
	imudm.imu_receive_daley_time = time - last_recive_time;
	last_recive_time = time;
	switch(pData[0])
	{
		case 1:
			IMU_UpdateAccel(pData);
			break;
		case 2:
			IMU_UpdateGyro(pData);
			break;
		case 3:
			IMU_UpdateEuler(pData);
			break;
		case 4:
			IMU_UpdateQuaternion(pData);
			break;
	}
}

void DM_IMU_INIT()
{
    imu_init(CAN_ID ,MASTER_ID ,NULL);
}

uint16_t tick_ms = 0;
void IMU_run()
{
#if IMU_active_mode

    // 2. ✅【核心缺失步骤】设置主动模式发送的数据类型！！
    // 0x01=加速度  0x02=角速度  0x03=欧拉角  0x04=四元数
    // imu_set_active_output_type(0x01);  // 这里选欧拉角，可自行修改
    // vTaskDelay(pdMS_TO_TICKS(30));

    // 3. 设置主动发送间隔 2ms (500Hz)
    imu_set_active_mode_delay(2);
    vTaskDelay(pdMS_TO_TICKS(30));

    

    // 5. 保存参数到Flash
    imu_save_parameters();
    vTaskDelay(pdMS_TO_TICKS(50));

	// 4. 开启主动模式
    imu_change_to_active();
    vTaskDelay(pdMS_TO_TICKS(30));

	imu_save_parameters();
    vTaskDelay(pdMS_TO_TICKS(50));

    // 6. 重启IMU（加载所有配置）
    imu_reboot();
    vTaskDelay(pdMS_TO_TICKS(200));  // 足够等待硬件重启

    // 配置完成！永久生效，删除任务
    vTaskDelete(NULL);


#else
    tick_ms++;
	if(tick_ms%3==0)
	{
		//imu_request_accel();
		imu_request_euler();
	}
	else if(tick_ms%2==0)
	{
		imu_request_gyro();
	}
	else if(tick_ms%1==0)
	{
		imu_request_quat();
	}
	
	if(tick_ms>1000)
		tick_ms=0;
#endif
}