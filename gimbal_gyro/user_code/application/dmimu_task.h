#ifndef DMIMU_TASK_H
#define DMIMU_TASK_H
#include "cmsis_os.h"
#include "imudm.h"
#include "gimbal.h"

//����ʼ����һ��ʱ��
#define DMIMU_TASK_INIT_TIME 201

#define DMIMU_CONTROL_TIME_MS 1


extern void dmimu_task(void *pvParameters);

extern bool_t gimbal_imu_open_flag ;

#endif 