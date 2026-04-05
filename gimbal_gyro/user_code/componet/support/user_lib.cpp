//
// Created by WSJ on 2021/11/2.
//

#include "user_lib.h"
#include "arm_math.h"
#include "main.h"

//循环限幅函数
fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue) {
    if (maxValue < minValue) {
        return Input;
    }

    if (Input > maxValue) {
        fp32 len = maxValue - minValue;
        while (Input > maxValue) {
            Input -= len;
        }
    } else if (Input < minValue) {
        fp32 len = maxValue - minValue;
        while (Input < minValue) {
            Input += len;
        }
    }
    return Input;
}

//限幅函数
fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue) {
    if (Value < minValue)
        return
                minValue;
    else if (Value > maxValue)
        return
                maxValue;
    else
        return
                Value;
}

void abs_limit(fp32 Value, fp32 MaxValue){
    if (Value > MaxValue){
        Value = MaxValue;
    }
}

fp32 abs_fp32(fp32 Value)
{
    if (Value < 0)
    {
        Value = -Value;
    }

    return Value;
}

int16_t abs_int16(int16_t Value)
{
    if (Value < 0)
    {
        Value = -Value;
    }

    return Value;
}


uint16_t float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	/* Converts a float to an unsigned int, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return (uint16_t) ((x_float-offset)*((float)((1<<bits)-1))/span);
}
/**
************************************************************************
* @brief:      	uint_to_float: 无符号整数转换为浮点数函数
* @param[in]:   x_int: 待转换的无符号整数
* @param[in]:   x_min: 范围最小值
* @param[in]:   x_max: 范围最大值
* @param[in]:   bits:  无符号整数的位数
* @retval:     	浮点数结果
* @details:    	将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个浮点数
************************************************************************
**/
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}


uint32_t get_running_time(uint32_t* last_time)
{
    uint32_t time = HAL_GetTick();
    uint32_t running_time = time - *last_time;
    *last_time = time;
    return running_time;
}

/**
 * @brief 一阶低通滤波函数
 * @param now_data  本次原始采样数据
 * @param last_data 上次滤波后的数据
 * @param weight    新数据权重(0~1之间，如0.1表示新数据占10%，历史占90%)
 * @return float    本次滤波后的数据
 */
float first_order_low_pass_filter(float now_data, float last_data, float weight)
{
    // 一阶低通滤波核心公式：滤波值 = 新数据*权重 + 旧数据*(1-权重)
    float filter_data = now_data * weight + last_data * (1 - weight);
    
    return filter_data;
}