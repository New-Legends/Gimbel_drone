//
// Created by WSJ on 2021/11/2.
//

#ifndef CLASSIS_BOARD_USER_LIB_H
#define CLASSIS_BOARD_USER_LIB_H

#include "struct_typedef.h"
#include "arm_math.h"

#define PI 3.14159265358979f
//循环限幅函数
extern fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue);
//弧度格式化为-PI~PI
#define rad_format(Ang) loop_fp32_constrain((Ang), -PI, PI)

//限幅函数
extern fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue);
//最大值限幅函数
extern void abs_limit(fp32 Value, fp32 MaxValue);

extern fp32 abs_fp32(fp32 Value);

extern int16_t abs_int16(int16_t Value);

//uint_to_float: 无符号整数转换为浮点数函数
extern uint16_t float_to_uint(float x_float, float x_min, float x_max, int bits);

extern float uint_to_float(int x_int, float x_min, float x_max, int bits);

//返回任务执行间隔时间
extern uint32_t get_running_time(uint32_t* last_time);

#endif //CLASSIS_BOARD_USER_LIB_H
