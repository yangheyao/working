//******************************************************************************              
//name:             DRV8834.h             
//introduce:        DRV8834驱动头文件       
//author:           甜甜的大香瓜                     
//email:            897503845@qq.com         
//QQ group          香瓜单片机之STM8/STM32(164311667)                      
//changetime:       2016.10.18       
//******************************************************************************     
#ifndef _DRV8834_H_  
#define _DRV8834_H_  
  
  
/*********************宏定义************************/     
//睡眠模式选择  
#define DRV8834_CONTROL_SLEEP                   0  
#define DRV8834_CONTROL_SLEEP_ON                0  
#define DRV8834_CONTROL_SLEEP_OFF               1  
  
  
//步数选择  
#define DRV8834_CONTROL_MICROSTEP               1  
#define DRV8834_CONTROL_MICROSTEP_FULL          0  
#define DRV8834_CONTROL_MICROSTEP_1_2           1    
#define DRV8834_CONTROL_MICROSTEP_1_4           2    
#define DRV8834_CONTROL_MICROSTEP_1_8           3    
#define DRV8834_CONTROL_MICROSTEP_1_16          4    
#define DRV8834_CONTROL_MICROSTEP_1_32          5    
  
  
//方向选择  
#define DRV8834_CONTROL_DIR                     2  
#define DRV8834_CONTROL_DIR_POSITIVE            0  
#define DRV8834_CONTROL_DIR_NEGATIVE            1  
  
  
//输出选择  
#define DRV8834_CONTROL_OUTPUTS                 3  
#define DRV8834_CONTROL_OUTPUTS_ENABLE          0  
#define DRV8834_CONTROL_OUTPUTS_DISABLE         1  
  
  
//脉冲脚电平选择  
#define DRV8834_CONTROL_STEP                    4  
#define DRV8834_CONTROL_STEP_INIT               0  
#define DRV8834_CONTROL_STEP_LOW                1  
#define DRV8834_CONTROL_STEP_HIGH               2  
  
  
//电机参数的宏  
#define MOTOR_CONFIG_DIRECTION_NEGATIVE         -1  
#define MOTOR_CONFIG_DIRECTION_POSITIVE         1  
  
#define MOTOR_CONFIG_SIZE_FULL                  0  
#define MOTOR_CONFIG_SIZE_1_2                   1  
  
#define MOTOR_CONFIG_STATUS_IDLE                0  
#define MOTOR_CONFIG_STATUS_ACTIVE              1

#define DRV8834_STEP_PER_CIRCLE                  (int)(16000)
  
/*********************外部变量************************/
#if 0
typedef struct     
{    
  signed long           DirectionSteps;                                           //含方向的步数    
  unsigned short        PulsePerSecond;                                           //每秒给的脉冲数   
  unsigned char         Size;                                                     //步长  
  unsigned long         StepsCount;                                               //电机走过的步数    
  unsigned char         Status;                                                   //电机转动状态   
  signed long           DirectionPosition;                                        //含方向的位置  
}MOTOR_CONFIG;   
  
extern MOTOR_CONFIG stMotor_Config;
#endif
  
  
/*********************函数声明************************/   
void DRV8834_Control(unsigned char nDRV8834_control, unsigned char nDRV8834_control_vaule);  
//unsigned long DRV8834_Step_Move(MOTOR_CONFIG *pMotor_Config);  
void DRV8834_Init(void);  
  
  
#endif  
