#ifndef CONFIG_H
#define CONFIG_H


/*------------------------云台---------------------------*/




//云台一阶高通滤波是否开启
#define GIMBAL_HIGH_PASS_FILTER 0



/*-----------------------发射机构------------------------------*/

//云台视觉调试模式 默认为0
#define GIMABL_VISION_DEBUG 1

//云台视觉是否打开
#define SHOOT_VISION_OPEN 0

//热量控制算法选择 1是我们的 0 是中科大的
#define SHOOT_HEAT_CTRL_SELECT 0

/*--------------------按键-------------------------------------*/
// turn 180°
//掉头180 按键 单击V 
#define KEY_PRESSED_GIMBAL_TURN_180 'Z' 
//云台自锁开关 长按C打开 单击C关闭
#define KEY_PRESSED_STOP_GIMBAL 'C'

//开启和关闭摩擦轮  单击G
#define KEY_PRESSED_SHOOT_FRIC 'G'

//视觉自瞄模式      单击E
#define KEY_PRESSED_VISION_COLOR_CHANGE 'E'
//视觉自瞄or打符模式改变 单机Q
#define KEY_PRESSED_VISION_SHOOT_CHANGE 'Q'

//打符
#define KEY_PRESSED_HIT_RUNE 'X'


#endif

