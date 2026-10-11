
#include "motor.h"  
#include "can.h"   


//电机 1 的全局变量 motor1,保存电机反馈信息，例如编码器值、转速、电流、温度等
Motor_t motor1;


/*
 * 函数:Motor_Update
 * 函数作用：解析电机通过 CAN 返回的 8 字节反馈数据,更新电机转速、电流、温度和累计编码器计数。
 * 参数 data[8]：接收到的 8 字节电机反馈数据。
 *
 * 数据排列（按照当前代码的解析方式）：
 * data[0]、data[1]：编码器值
 * data[2]、data[3]：转速
 * data[4]、data[5]：电流
 * data[6]：温度
 */
void Motor_Update(uint8_t data[8])
{
    uint16_t new_ecd;  // 保存本次读取到的编码器值

    new_ecd = (uint16_t)((data[0] << 8) | data[1]);

    motor1.speed_rpm = (int16_t)((data[2] << 8) | data[3]);

    motor1.current = (int16_t)((data[4] << 8) | data[5]);

    motor1.temperature = data[6];


    // 判断是否为第一次收到有效的编码器反馈
    if(motor1.ecd_init == 0)
    {
        // 保存本次编码器值，作为初始编码器值
        motor1.ecd = new_ecd;

        // 保存本次编码器值，作为上一次编码器值
        motor1.last_ecd = new_ecd;

        // 初始化累计编码器变化量为 0
        // 此时还没有计算出有效的转动增量
        motor1.total_ecd = 0;

        // 标记编码器已经完成初始化
        motor1.ecd_init = 1;

        // 结束本次函数调用，避免把初始化过程当作实际转动
        return;
    }

    int16_t delta;// delta = 本次编码器值 - 上一次编码器值

    delta = (int16_t)(new_ecd - motor1.last_ecd);// 强制转换为 int16_t，目的是让差值可以表示正数和负数


    /*
     * 如果编码器从较大的值跳到较小的值，
     * 可能是计数从末尾绕回起点，而不是电机真的反向转动。
     * 因此，需要根据差值判断是否发生了计数边界跨越。
     */

    // 差值大于半圈计数范围：
    // 按照计数从末尾绕回起点的情况修正差值
    if(delta > 4096)
    {
        delta -= 8192;
    }

    else if(delta < -4096)
    {
        delta += 8192;
    }

    // 将本次修正后的变化量累加到总编码器计数
    // 这样可以记录多次采样之间累计的转动方向和距离
    motor1.total_ecd += delta;

    // 更新当前编码器值，供其他函数读取
    motor1.ecd = new_ecd;

    // 保存本次编码器值，作为下一次计算的上一次读数
    motor1.last_ecd = new_ecd;
}


/*
 * 函数：Motor_SendCurrent
 * 函数作用：通过 CAN 发送电机电流控制指令。
 * 参数 current：需要发送的有符号 16 位控制值。
 * 当前代码将 current 放在 CAN 数据区的前两个字节，
 * 其余 6 个字节设置为 0。
 */
void Motor_SendCurrent(int16_t current)
{
    CAN_TxHeaderTypeDef tx_header;// 定义 CAN 发送帧头，设置标识符、帧类型和数据长度等信息

    uint8_t tx_data[8];// 定义 8 字节 CAN 数据缓冲区

    uint32_t tx_mailbox;// 用于保存 HAL 返回的 CAN 发送邮箱编号


    // 设置 CAN 标准帧标识符
    // 当前代码使用 0x201
    tx_header.StdId = 0x200;

    // 标准帧不使用扩展标识符，此字段设为 0
    tx_header.ExtId = 0;

    // 指定使用标准 CAN 标识符（11 位）
    tx_header.IDE = CAN_ID_STD;

    // 指定发送普通数据帧，而不是远程请求帧
    tx_header.RTR = CAN_RTR_DATA;

    // 设置 CAN 数据长度为 8 字节
    tx_header.DLC = 8;

    // 不附加发送时间信息
    tx_header.TransmitGlobalTime = DISABLE;


    // 将 current 的高 8 位放入第 0 个数据字节
    tx_data[0] = (uint8_t)(current >> 8);
    // 将 current 的低 8 位放入第 1 个数据字节
    tx_data[1] = (uint8_t)(current & 0xFF);


    // 当前只使用第一个电机对应的数据位置
    tx_data[2] = 0;
    tx_data[3] = 0;
    tx_data[4] = 0;
    tx_data[5] = 0;
    tx_data[6] = 0;
    tx_data[7] = 0;


    // 请求 HAL 将数据加入 CAN 发送邮箱
    // 注意：调用成功不等于电调已经收到并执行指令
    // 原代码没有检查返回值，后续可以单独考虑增加发送错误处理
    HAL_CAN_AddTxMessage(
        &hcan1,       // 使用 CAN1 外设
        &tx_header,   // 指向发送帧头
        tx_data,      // 指向 8 字节数据
        &tx_mailbox   // 接收 HAL 返回的发送邮箱编号
    );
}


/*
 * 函数：Motor_GetAngle
 * 函数作用：将累计编码器计数换算成角度.
 */
float Motor_GetAngle(void)
{
    // 强制转换为浮点数，
    // 再乘以 360 度并除以 8192，得到角度
    return ((float)motor1.total_ecd * 360.0f) / 8192.0f;
}
