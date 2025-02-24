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

// 增量和累积限幅值
#define DELTA_LIMIT 1000
#define COMMAND_LIMIT 5000

#define PIT_TIME 2  // 进入中断的时间间隔 单位 ms
int16 encoder1_data = 0;
int16 encoder2_data = 0;

// 外环PID（平衡控制）参数
float desiredPitch = 20.0f;         // 期望倾斜角度（单位：度）
float Kp_angle = 10;
float Ki_angle = 0.1f;
float Kd_angle = 0.0f;

float angleIntegral = 0.0f;
float lastAngleError = 0.0f;

// 内环PID（速度控制）参数
float desiredSpeed = 0.0f;            // 期望基准速度（可为正、负，单位与编码器数据对应）


float Kp_speed = 20 ;
float Ki_speed = 0.5;
float Kd_speed = 0;         // 增量式D 没啥卵用

float speedIntegral_left = 0.0f, speedIntegral_right = 0.0f;        // 积分项
float lastSpeedError_left = 0.0f, lastSpeedError_right = 0.0f;      // 上一次误差
float preSpeedError_left = 0.0f, preSpeedError_right = 0.0f;        // 上上次误差

float P_NUM,I_NUM,D_NUM;    // 调试用 看看各个项的值
// 转向控制（外加控制），正负值表示左右转向
float desiredTurn = 0.0f;

int16 leftCommand=0;
int16 rightCommand=0;


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
    speed = (speed>5000)?5000:(speed<-5000)?-5000:speed;                            // 限制速度范围
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

void Set_Target_Pitch(float pitch)
{
    desiredPitch = pitch;
}

void Set_Target_Speed(int16 speed)
{
    desiredSpeed = speed;
}


int main (void)
{
    clock_init(SYSTEM_CLOCK_120M);                                              // 初始化芯片时钟 工作频率为 120MHz
    debug_init();                                                               // 初始化默认 debug uart

    // 此处编写用户代码 例如外设初始化代码等

    Motor_Init();                             // 初始化电机
    encoder_quad_init(ENCODER1_QUADDEC, ENCODER1_QUADDEC_A, ENCODER1_QUADDEC_B);   // 初始化编码器模块与引脚 正交解码编码器模式
    encoder_quad_init(ENCODER2_QUADDEC, ENCODER2_QUADDEC_A, ENCODER2_QUADDEC_B);          // 初始化编码器模块与引脚 带方向增量编码器模式

    pit_ms_init(PIT, PIT_TIME);                                                      // 初始化 PIT 为周期中断

    interrupt_set_priority(PIT_PRIORITY, 0);                                    // 设置 PIT 对周期中断的中断优先级为 0
    // 此处编写用户代码 例如外设初始化代码等

    mpu6050_init();                                                             // 初始化 MPU6050
    Set_Target_Speed(100);                                                     // 设置期望速度
    uint32 time = 0;
    while(1)
    {
        printf("%d,%d,%d,%d,%f,%f,%f\n",encoder1_data,encoder2_data,leftCommand,rightCommand,P_NUM,I_NUM,imu_Angle.Pitch);
        system_delay_ms(11);
    }
}

// **************************** 代码区域 ****************************



// 串级pid控制算法
// 目标 : 双路小车翘头平衡 实现小车的平衡控制 使得mpu6050的 Pitch 角度保持在20度左右 另外加入速度的控制 和 转向的控制
// 理想状态 : 小车在平衡状态下可以前进后退 旋转 
// 输入为编码器的期望值和方向的期望
// 现象为 小车稳定保证Pitch角度在20度左右 并且 小车速度为期望值 并且小车可以转向
// 串级PID控制算法的核心思想是将两个PID控制器串联起来，外环控制的是内环控制器的输入，内环控制器控制的是系统的输出
// 串级PID控制器的优点是可以有效的减小系统的超调量，提高系统的稳定性

// 串级PID控制函数：外环控制平衡，内环控制电机速度，综合获得左右电机控制命令
void cascade_pid_control(void)
{
    // 采样周期（单位：秒），本工程中PIT周期为PIT_TIME ms
    const float dt = PIT_TIME / 1000.0f;

    // -----------------------------
    // 外环PID：角度控制（平衡控制）
    // -----------------------------
    // 当前Pitch角度由IMU_getEuleranAngles()更新到 imu_Angle.Pitch
    float angleError = desiredPitch - imu_Angle.Pitch;
    angleIntegral += angleError * dt;
    float dAngle = (angleError - lastAngleError) / dt;
    float anglePID = Kp_angle * angleError + Ki_angle * angleIntegral + Kd_angle * dAngle;
    lastAngleError = angleError;
    
    // 调试先屏蔽外环PID

    anglePID = 0;

    // 外环输出作为内环的目标速度（例如：向前运动以产生纠正力）
    float targetSpeed = desiredSpeed + anglePID;

   


 // -----------------------------
    // 内环PID：速度控制（电机转速反馈）-- 增量式PID实现
    // -----------------------------
    // 左侧电机

    float currentLeftError = targetSpeed - (float)encoder1_data;
    P_NUM=Kp_speed * (currentLeftError - lastSpeedError_left);
    I_NUM=Ki_speed * currentLeftError;
    D_NUM=Kd_speed * (currentLeftError - 2.0f * lastSpeedError_left + preSpeedError_left);
    float delta_left = Kp_speed * (currentLeftError - lastSpeedError_left)
                         + Ki_speed * currentLeftError
                         + Kd_speed * (currentLeftError - 2.0f * lastSpeedError_left + preSpeedError_left);
    // 增量限幅
    if (delta_left > DELTA_LIMIT) delta_left = DELTA_LIMIT;
    if (delta_left < -DELTA_LIMIT) delta_left = -DELTA_LIMIT;
    // 累加增量调整控制命令
    leftCommand = leftCommand + (int16)delta_left;
    // 累积限幅
    if (leftCommand > COMMAND_LIMIT) leftCommand = COMMAND_LIMIT;
    if (leftCommand < -COMMAND_LIMIT) leftCommand = -COMMAND_LIMIT;
    // 更新误差存储
    preSpeedError_left = lastSpeedError_left;
    lastSpeedError_left = currentLeftError;

    
    // 右侧电机
    float currentRightError = targetSpeed - (float)encoder2_data;
    float delta_right = Kp_speed * (currentRightError - lastSpeedError_right)
                          + Ki_speed * currentRightError
                          + Kd_speed * (currentRightError - 2.0f * lastSpeedError_right + preSpeedError_right);
    // 增量限幅
    if (delta_right > DELTA_LIMIT) delta_right = DELTA_LIMIT;
    if (delta_right < -DELTA_LIMIT) delta_right = -DELTA_LIMIT;
    rightCommand = rightCommand + (int16)delta_right;
    // 累积限幅
    if (rightCommand > COMMAND_LIMIT) rightCommand = COMMAND_LIMIT;
    if (rightCommand < -COMMAND_LIMIT) rightCommand = -COMMAND_LIMIT;
    preSpeedError_right = lastSpeedError_right;
    lastSpeedError_right = currentRightError;
    
    // -----------------------------
    // 综合转向控制（直接加减desiredTurn）
    // -----------------------------
    leftCommand = leftCommand + (int16)desiredTurn;
    rightCommand = rightCommand - (int16)desiredTurn;

    Motor_SetSpeed(Left, leftCommand);
    Motor_SetSpeed(Right, rightCommand);
}




//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PIT 的周期中断处理函数 这个函数将在 PIT 对应的定时器中断调用 详见 isr.c
// 参数说明     void
// 返回参数     void
// 使用示例     pit_handler();
//-------------------------------------------------------------------------------------------------------------------
void pit_handler (void)
{
    encoder1_data = encoder_get_count(ENCODER1_QUADDEC);                           // 获取编码器计数
    encoder2_data = -encoder_get_count(ENCODER2_QUADDEC);                          // 获取编码器计数
    mpu6050_get_acc();                                                             // 获取加速度计数据
    mpu6050_get_gyro();                                                            // 获取陀螺仪数据
    IMU_getEuleranAngles();                                                        // 获取欧拉角

    cascade_pid_control();                                                         // 串级PID控制

    encoder_clear_count(ENCODER1_QUADDEC);                                         // 清空编码器计数
    encoder_clear_count(ENCODER2_QUADDEC);                                         // 清空编码器计数
}

// *************************** 例程常见问题说明 ***************************
