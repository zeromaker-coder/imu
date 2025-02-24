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

#define SPEED_CONTROL_PERIOD 25   // 分 25 份，即25ms内平滑输出
#define CAR_SPEED_SET 1500        // 目标车速（单位视具体系统而定）
#define CAR_POSITION_MAX 1000     // 积分上限
#define CAR_POSITION_MIN -1000    // 积分下限


#define MECH_MID 0.0f       // 定义机械中值（直立时目标角度），单位：度

int16 encoder1_data = 0;
int16 encoder2_data = 0;

uint16 time = 0;            // 在pit中断中的时间计数

// 外环速度 PID 控制相关全局变量
float carSpeed = 0.0f;              // 当前车速，由平均左右轮脉冲获得
float carSpeedPrev = 0.0f;          // 上一次车速，用于低通滤波
float carPosition = 0.0f;           // 积分项（车速积分，相当于车位置)
float speedControlOutOld = 0.0f;    // 上一次速度环 PID 输出值
float speedControlOutNew = 0.0f;    // 最新一次速度环 PID 输出值
float speedControlOutSmooth = 0.0f; // 平滑输出，用作内环角度环的输入   //真正的输出值 虽然25ms计算一次 相当于两个点之间的平滑过渡

// 外环平滑输出计时变量（单位：ms）
int speedControlPeriod = 0;

float speed_P = 0.2f;             // 比例系数
float speed_I = 0.05f;            // 积分系数

// 角度内环 PID 参数
float angle_kp = 1.0f;      // 比例系数
float angle_ki = 0.01f;     // 积分系数
float angle_kd = 0.0f;      // 微分系数（若不使用，可设为0）

// 角度内环 PID 内部变量
float angleIntegral = 0.0f;   // 积分累计
float angleLastError = 0.0f;  // 上一次误差
float angleControlOut = 0.0f; // 角度内环 PID 输出



// 编码器累加变量（假设在编码器中断中累加，每次控制后需清零）
volatile int16 leftMotorPulseSigma = 0;
volatile int16 rightMotorPulseSigma = 0;

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

// 速度外环 PID 控制函数，每 25ms调用一次
void Speed_PID_Control(void)
{
    float fP, fI;
    float fDelta;  // 速度误差

    // 计算当前车速：左右轮脉冲平均
    carSpeed = (leftMotorPulseSigma + rightMotorPulseSigma) * 0.5f;
    // 清零编码器累加值，确保下一周期重新采样
    leftMotorPulseSigma = rightMotorPulseSigma = 0;

    // 对车速进行低通滤波，使车速更平滑
    carSpeed = 0.7f * carSpeedPrev + 0.3f * carSpeed;
    carSpeedPrev = carSpeed;

    // 计算车速误差：目标速度与实际速度之差
    fDelta = CAR_SPEED_SET - carSpeed;

    // 计算比例项和积分项
    fP = fDelta * speed_P;
    fI = fDelta * speed_I;

    // 累加积分项（车速度积分，也可理解为车位置）
    carPosition += fI;

    // 积分限幅保护
    if ((int)carPosition > CAR_POSITION_MAX)
        carPosition = CAR_POSITION_MAX;
    if ((int)carPosition < CAR_POSITION_MIN)
        carPosition = CAR_POSITION_MIN;

    // 保存上一次的控制输出
    speedControlOutOld = speedControlOutNew;

    // 当前PI输出 = 比例项 + 积分项
    speedControlOutNew = fP + carPosition;

    // 重置平滑输出周期计数器，开始新的1ms平滑累计
    speedControlPeriod = 0;
}

// 速度外环平滑输出函数，每 1ms调用一次（在1ms系统滴答或定时中断中调用）
// 将 25ms 内计算得到的 PID 输出差值分步平滑累加到上一次的输出上
void Speed_Control_Output(void)
{
    float fValue;
    // 计算此次 PID 输出变动量
    fValue = speedControlOutNew - speedControlOutOld;
    // 按比例分份， (speedControlPeriod+1) / SPEED_CONTROL_PERIOD 为当前分配比例
    speedControlOutSmooth = fValue * (speedControlPeriod + 1) / SPEED_CONTROL_PERIOD + speedControlOutOld;
    
    // 更新平滑计数器，每1ms增加一次
    speedControlPeriod++;
    if (speedControlPeriod >= SPEED_CONTROL_PERIOD)
    {
        // 当累计满25份时，确保平滑输出与最新PID输出一致
        speedControlOutSmooth = speedControlOutNew;
        speedControlPeriod = SPEED_CONTROL_PERIOD;
    }
}


// 角度内环 PID 控制函数，每 5ms调用一次（与 IMU 解算周期一致）
// 根据 imu_Angle_Filted.Pitch 与目标机械中值 MECH_MID 的误差计算 PID 输出
void Angle_PID_Control(void)
{
    float error, P, I, D;
    
    // 误差：目标角度 - 当前角度
    error = MECH_MID  + speedControlOutSmooth - imu_Angle_Filted.Pitch;
    
    // 计算比例项
    P = error * angle_kp;
    
    // 积分项累加（可根据需要增加积分限幅保护）
    angleIntegral += error * angle_ki;
    I = angleIntegral;
    
    // 计算微分项
    D = (error - angleLastError) * angle_kd;
    angleLastError = error;
    
    // 角度内环控制输出
    angleControlOut = P + I + D;
    
    // 此处可根据实际需要，将 angleControlOut 与速度外环平滑输出叠加，
    // 或作为独立内环输出直接用于后续电机 PWM 控制调整直立角度。
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

    mpu6050_init();                                                                       // 初始化 MPU6050
    while(1)
    {
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

    leftMotorPulseSigma  += encoder1_data;                                         // 累加左轮脉冲
    rightMotorPulseSigma += encoder2_data;                                         // 累加右轮脉冲


    // 每次25ms进入速度环闭环pid控制
    // 每次5ms之中 进行mpu6050姿态解算 进行直立环控制



    if (time % 5 == 0)
    {
        mpu6050_get_acc();                                                             // 获取加速度计数据
        mpu6050_get_gyro();                                                            // 获取陀螺仪数据
        IMU_getEuleranAngles();                                                        // 获取欧拉角
        Angle_PID_Control();                                                           // 角度环控制

    }
    if (time % 25 == 0)
    {
        // 速度环控制
        Speed_PID_Control();

        time=0;
    }
    Speed_Control_Output();                                                          // 速度环平滑输出


    
    
}

// *************************** 例程常见问题说明 ***************************
