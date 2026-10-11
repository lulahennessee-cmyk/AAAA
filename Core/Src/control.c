#include "control.h"  
#include "motor.h"    


// 定义速度环 PID 控制器
PID_t speed_pid;

// 定义位置环 PID 控制器
PID_t angle_pid;


// 初始化电机控制系统中的两个 PID 控制器
void Motor_ControlInit(void)
{
    // 初始化速度环 PID
    PID_Init(
        &speed_pid, // 传入速度环 PID 结构体的地址
        1.0f,       // 比例
        0.0f,       // 积分
        0.0f,       // 微分
        5000.0f,    // PID 输出限幅
        3000.0f     // 积分项限幅
    );

    // 初始化位置环 PID
    PID_Init(
        &angle_pid, // 传入位置环 PID 结构体的地址
        5.0f,       // 比例
        0.0f,       // 积分
        0.0f,       // 微分
        2000.0f,    // PID 输出限幅
        1000.0f     // 积分项限幅
    );
}


// 速度控制函数：让电机实际转速接近目标转速
void Motor_SpeedControl(float target_speed)
{
    float output; // 保存速度 PID 的计算输出

    // 输入目标转速和实际转速，计算需要输出的控制量
    output = PID_Calc(
        &speed_pid,    // 使用速度环 PID
        target_speed,  // 目标转速
        motor1.speed_rpm // 电机当前实际转速，由电机反馈数据更新
    );

    // 将 PID 输出转换为 int16_t，并发送电流控制指令
    Motor_SendCurrent((int16_t)output);
}


// 位置控制函数：让电机实际角度接近目标角度
void Motor_PositionControl(float target_angle)
{
    float actual_angle; // 保存电机当前实际角度
    float target_speed; // 保存位置环计算出的目标转速

    // 获取电机当前累计旋转角度
    actual_angle = Motor_GetAngle();

    // 位置环 PID 根据目标角度与实际角度的差值计算目标转速
    target_speed = PID_Calc(
        &angle_pid,   // 使用位置环 PID
        target_angle, 
        actual_angle  
    );

    // 将位置环计算出的目标转速交给速度环控制
    // 速度环再计算电流指令并发送给电机

    Motor_SpeedControl(target_speed);
}
