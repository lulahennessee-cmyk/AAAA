#include "pid.h" 


// 初始化 PID 控制器
void PID_Init(PID_t *pid,
              float kp,
              float ki,
              float kd,
              float max_output,
              float max_integral)
{
    // 保存 PID 的三个控制参数
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    // 初始化目标值和实际测量值
    pid->target = 0;
    pid->measure = 0;

    // 初始化当前误差和上一次误差
    pid->error = 0;
    pid->last_error = 0;

    // 清空积分累计值和控制器输出
    pid->integral = 0;
    pid->output = 0;

    // 设置输出限幅和积分限幅
    pid->max_output = max_output;       // 限制 PID 最终输出的最大幅值
    pid->max_integral = max_integral;   // 限制积分累计值的最大幅值
}


// 计算 PID 控制器的输出
float PID_Calc(PID_t *pid,
               float target,
               float measure)
{
    float derivative;  // 保存误差变化量，供微分项使用

    // 保存本次输入的目标值和实际测量值
    pid->target = target;
    pid->measure = measure;

    // 计算当前误差：目标值减去实际值
    // 误差为正，说明实际值低于目标值；误差为负则相反
    pid->error = target - measure;

    // 累加误差，形成积分项的基础数据
    pid->integral += pid->error;

    // 积分上限保护：防止积分累计值过大
    if(pid->integral > pid->max_integral)
    {
        pid->integral = pid->max_integral;
    }

    // 积分下限保护
    if(pid->integral < -pid->max_integral)
    {
        pid->integral = -pid->max_integral;
    }

    // 计算误差变化量：本次误差减去上一次误差
    derivative = pid->error - pid->last_error;

    // 计算 PID 总输出
    pid->output =
        pid->kp * pid->error
        + pid->ki * pid->integral
        + pid->kd * derivative;

    if(pid->output > pid->max_output)
    {
        pid->output = pid->max_output;
    }//上限

    if(pid->output < -pid->max_output)
    {
        pid->output = -pid->max_output;
    }//下限

    // 保存本次误差，供下一次计算微分项时使用
    pid->last_error = pid->error;

    // 返回计算完成的控制输出
    return pid->output;
}
