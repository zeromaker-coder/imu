/*********************************************************************************************************************
* MM32F327X-G8P Opensourec Library 即（MM32F327X-G8P 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
* 
* 本文件是 MM32F327X-G8P 开源库的一部分
* 
* MM32F327X-G8P 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
* 
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
* 
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
* 
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
* 
* 文件名称          main
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 8.32.4 or MDK 5.37
* 适用平台          MM32F327X_G8P
* 店铺链接          https://seekfree.taobao.com/
* 
* 修改记录
* 日期              作者                备注
* 2022-08-10        Teternal            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"

// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完


// *************************** 例程硬件连接说明 ***************************
// 核心板正常供电即可 无需额外连接


// *************************** 例程测试说明 ***************************


// **************************** 代码区域 ****************************
#define CHANNEL_NUMBER          (4)

#define PWM_CH1                 (TIM5_PWM_CH1_A0)
#define PWM_CH2                 (TIM5_PWM_CH2_A1)
#define PWM_CH3                 (TIM5_PWM_CH3_A2)
#define PWM_CH4                 (TIM5_PWM_CH4_A3)

#define Motor1_PWM              (PWM_CH4)           //左轮
#define Motor2_PWM              (PWM_CH2)           //右轮

#define Motor1_DIR              (A2)                //左轮
#define Motor2_DIR              (A0)                //右轮

#define PIT                             (TIM6_PIT )                              // 使用的周期中断编号 如果修改 需要同步对应修改周期中断编号与 isr.c 中的调用
#define PIT_PRIORITY                    (TIM6_IRQn)                              // 对应周期中断的中断编号 在 mm32f3277gx.h 头文件中查看 IRQn_Type 枚举体

#define ENCODER1_QUADDEC                 (TIM3_ENCODER)                          // 正交编码器对应使用的编码器接口 这里使用 TIM3 的编码器功能
#define ENCODER1_QUADDEC_A               (TIM3_ENCODER_CH1_B4)                   // A 相对应的引脚
#define ENCODER1_QUADDEC_B               (TIM3_ENCODER_CH2_B5)                   // B 相对应的引脚
 
#define ENCODER2_QUADDEC		                (TIM4_ENCODER)                   // 带方向编码器对应使用的编码器接口 这里使用 TIM4 的编码器功能
#define ENCODER2_QUADDEC_A               (TIM4_ENCODER_CH1_B6)                   // A 对应的引脚
#define ENCODER2_QUADDEC_B                 (TIM4_ENCODER_CH2_B7)                 // B 对应的引脚

#define PWM_PERIOD 30000  // PWM周期 30KHz 
#define MOTOR_DEAD_VAL 800  // 电机死区值
#define MOTOR_MAX 5000  // 电机最大速度

#define PIT_TIME 1  // 进入中断的时间间隔 单位 ms
int16 encoder1_data = 0;
int16 encoder2_data = 0;

uint16 time = 0;            // 在pit中断中的时间计数

typedef enum
{
    Left  = 0,
    Right = 1,
}motor_enum;

void Motor_Init(void)
{
    gpio_init(Motor1_DIR,GPO,GPIO_HIGH,GPO_PUSH_PULL);                          // 初始化电机1方向控制引脚
    gpio_init(Motor2_DIR,GPO,GPIO_HIGH,GPO_PUSH_PULL);                          // 初始化电机2方向控制引脚

    gpio_set_level(Motor1_DIR,GPIO_HIGH);                                       // 设置电机1方向为正转
    gpio_set_level(Motor2_DIR,GPIO_HIGH);                                       // 设置电机2方向为正转

    pwm_init(Motor1_PWM, 30000, 0);                                                // 初始化 PWM 通道 频率 30KHz 初始占空比 0%
    pwm_init(Motor2_PWM, 30000, 0);                                                // 初始化 PWM 通道 频率 30KHz 初始占空比 0%
}


/******************
 * 函数名：Motor_SetSpeed
 * 描述  ：设置电机速度
 * 输入  ：motor  电机选择
 *        speed  速度 范围 -10000~10000
 * 输出  ：无
 */
void Motor_SetSpeed(motor_enum motor, int16 speed)
{
    
    speed = (speed>0)?speed+MOTOR_DEAD_VAL:(speed<0)?speed-MOTOR_DEAD_VAL:0;              // 死区控制
    speed = (speed>MOTOR_MAX)?MOTOR_MAX:(speed<-MOTOR_MAX)?-MOTOR_MAX:speed;                            // 限制速度范围
    if(speed > 0)
    {
        if(motor == Left)
        {
            gpio_set_level(Motor1_DIR,GPIO_LOW);                                       // 设置电机1方向为正转
            pwm_set_duty(Motor1_PWM, speed);                                            // 更新对应通道占空比
        }
        else
        {
            gpio_set_level(Motor2_DIR,GPIO_HIGH);                                       // 设置电机2方向为正转
            pwm_set_duty(Motor2_PWM, speed);                                            // 更新对应通道占空比
        }
    }
    else
    {
        if(motor == Left)
        {
            gpio_set_level(Motor1_DIR,GPIO_HIGH);                                        // 设置电机1方向为反转
            pwm_set_duty(Motor1_PWM, -speed);                                           // 更新对应通道占空比
        }
        else
        {
            gpio_set_level(Motor2_DIR,GPIO_LOW);                                        // 设置电机2方向为反转
            pwm_set_duty(Motor2_PWM, -speed);                                           // 更新对应通道占空比
        }
    }
}



int main (void)
{
    clock_init(SYSTEM_CLOCK_120M);                                              // 初始化芯片时钟 工作频率为 120MHz
    debug_init();                                                               // 初始化默认 debug uart

    // 此处编写用户代码 例如外设初始化代码等

    Motor_Init();                             // 初始化电机
    encoder_quad_init(ENCODER1_QUADDEC, ENCODER1_QUADDEC_A, ENCODER1_QUADDEC_B);          // 初始化编码器模块与引脚 正交解码编码器模式
    encoder_quad_init(ENCODER2_QUADDEC, ENCODER2_QUADDEC_A, ENCODER2_QUADDEC_B);          // 初始化编码器模块与引脚 带方向增量编码器模式

    pit_ms_init(PIT, PIT_TIME);                                                           // 初始化 PIT 为周期中断

    interrupt_set_priority(PIT_PRIORITY, 0);                                              // 设置 PIT 对周期中断的中断优先级为 0
    // 此处编写用户代码 例如外设初始化代码等

    mpu6050_init();                                                                       // 初始化 MPU6050                                                   // 设置期望速度
    uint16 pwm=600;
    while(1)
    {
        printf("%f,%f,%f\n",imu_Angle_Filted.Roll,imu_Angle_Filted.Pitch,imu_Angle_Filted.Yaw);       // 打印欧拉角

    }


}

// **************************** 代码区域 ****************************





// **************************** 中断处理函数 ****************************
// 设置为

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PIT 的周期中断处理函数 这个函数将在 PIT 对应的定时器中断调用 详见 isr.c
// 参数说明     void
// 返回参数     void
// 使用示例     pit_handler();
//-------------------------------------------------------------------------------------------------------------------
void pit_handler (void)
{   
    time++;
    encoder1_data = encoder_get_count(ENCODER1_QUADDEC);                           // 获取编码器计数
    encoder2_data = -encoder_get_count(ENCODER2_QUADDEC);                          // 获取编码器计数
    encoder_clear_count(ENCODER1_QUADDEC);                                         // 清空编码器计数
    encoder_clear_count(ENCODER2_QUADDEC);                                         // 清空编码器计数

    // 每次25ms进入速度环闭环pid控制
    // 每次5ms之中 进行mpu6050姿态解算 进行直立环控制



    if (time % 5 == 0)
    {
        mpu6050_get_acc();                                                             // 获取加速度计数据
        mpu6050_get_gyro();                                                            // 获取陀螺仪数据
        IMU_getEuleranAngles();                                                        // 获取欧拉角
    }
    if (time % 25 == 0)
    {
        // 速度环控制
        time=0;
    }


    
    
}

// *************************** 例程常见问题说明 ***************************
