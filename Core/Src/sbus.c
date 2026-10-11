#include "sbus.h"   
#include "usart.h" 

volatile SBUS_Data_t sbus_data;

// 暂存
static uint8_t sbus_rx_byte;

// 存放一整帧 SBUS 数据
static uint8_t sbus_frame[SBUS_FRAME_LENGTH];

static uint8_t sbus_frame_index = 0;

// 初始化 SBUS 接收
void SBUS_Init(void)
{
    // 从一帧的开头开始接收
    sbus_frame_index = 0;

    // 初始化时先认为遥控器信号不可靠
    // 等接收到有效数据帧后，再更新这两个标志
    sbus_data.failsafe = 1;
    sbus_data.lost_frame = 1;

    // 启动 UART1 中断接收，每次接收 1 个字节
    // 接收完成后，HAL 库会调用接收完成回调函数
    if (HAL_UART_Receive_IT(&huart1, &sbus_rx_byte, 1) != HAL_OK)
    {
        // 如果启动接收失败，进入错误处理函数
        Error_Handler();
    }
}


// UART 接收完成中断回调
// 每成功接收到 1 个字节，就会进入这里
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
    {
        // 保存刚刚接收到的字节
        uint8_t byte = sbus_rx_byte;

        if (sbus_frame_index == 0)
        {
            if (byte == 0x0F)
            {
                sbus_frame[0] = byte;
                sbus_frame_index = 1;
            }
        }
        else
        {
            sbus_frame[sbus_frame_index] = byte;

            // 接收位置向后移动 1 个字节
            sbus_frame_index++;

            // 当接收字节数达到完整帧长度时，解析整帧
            if (sbus_frame_index >= SBUS_FRAME_LENGTH)
            {
                // 将完整的数据帧交给解析函数
                SBUS_Parse(sbus_frame);

                sbus_frame_index = 0;
            }
        }

        // 重新启动中断接收，继续接收下一个字节
        if (HAL_UART_Receive_IT(&huart1, &sbus_rx_byte, 1) != HAL_OK)
        {
            // 如果重新启动接收失败，重置接收位置
            sbus_frame_index = 0;
        }
    }
}


// UART 接收错误回调
// UART 发生接收错误时，尝试停止当前接收并重新启动
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
    {
        // 放弃当前未完成的数据帧
        sbus_frame_index = 0;

        // 尝试终止 UART1 当前的接收操作
        // (void) 表示这里不使用函数的返回值
        (void)HAL_UART_AbortReceive(&huart1);

        if (HAL_UART_Receive_IT(&huart1, &sbus_rx_byte, 1) != HAL_OK)
        {
            // 重新启动仍然失败，进入错误处理函数
            Error_Handler();
        }
    }
}


// 解析一帧 SBUS 数据
// buf 指向完整 SBUS 数据帧的首地址
void SBUS_Parse(uint8_t *buf)
{
    // 数据指针是否为空，以及帧头是否正确
    if (buf == NULL || buf[0] != 0x0F)
    {
        // 数据不符合基本要求，不继续解析
        return;
    }

    // 下面依次解析 16 个通道。

    
    sbus_data.ch[0] =
        ((uint32_t)buf[1] |
        ((uint32_t)buf[2] << 8)) & 0x07FF;

    sbus_data.ch[1] =
        (((uint32_t)buf[2] >> 3) |
        ((uint32_t)buf[3] << 5)) & 0x07FF;

    sbus_data.ch[2] =
        (((uint32_t)buf[3] >> 6) |
        ((uint32_t)buf[4] << 2) |
        ((uint32_t)buf[5] << 10)) & 0x07FF;

    sbus_data.ch[3] =
        (((uint32_t)buf[5] >> 1) |
        ((uint32_t)buf[6] << 7)) & 0x07FF;

    sbus_data.ch[4] =
        (((uint32_t)buf[6] >> 4) |
        ((uint32_t)buf[7] << 4)) & 0x07FF;

    sbus_data.ch[5] =
        (((uint32_t)buf[7] >> 7) |
        ((uint32_t)buf[8] << 1) |
        ((uint32_t)buf[9] << 9)) & 0x07FF;

    sbus_data.ch[6] =
        (((uint32_t)buf[9] >> 2) |
        ((uint32_t)buf[10] << 6)) & 0x07FF;

    sbus_data.ch[7] =
        (((uint32_t)buf[10] >> 5) |
        ((uint32_t)buf[11] << 3)) & 0x07FF;

    sbus_data.ch[8] =
        ((uint32_t)buf[12] |
        ((uint32_t)buf[13] << 8)) & 0x07FF;

    sbus_data.ch[9] =
        (((uint32_t)buf[13] >> 3) |
        ((uint32_t)buf[14] << 5)) & 0x07FF;

    sbus_data.ch[10] =
        (((uint32_t)buf[14] >> 6) |
        ((uint32_t)buf[15] << 2) |
        ((uint32_t)buf[16] << 10)) & 0x07FF;

    sbus_data.ch[11] =
        (((uint32_t)buf[16] >> 1) |
        ((uint32_t)buf[17] << 7)) & 0x07FF;

    sbus_data.ch[12] =
        (((uint32_t)buf[17] >> 4) |
        ((uint32_t)buf[18] << 4)) & 0x07FF;

    sbus_data.ch[13] =
        (((uint32_t)buf[18] >> 7) |
        ((uint32_t)buf[19] << 1) |
        ((uint32_t)buf[20] << 9)) & 0x07FF;

    sbus_data.ch[14] =
        (((uint32_t)buf[20] >> 2) |
        ((uint32_t)buf[21] << 6)) & 0x07FF;

    sbus_data.ch[15] =
        (((uint32_t)buf[21] >> 5) |
        ((uint32_t)buf[22] << 3)) & 0x07FF;

    // 从标志字节 buf[23] 中提取 lost_frame 标志
    // 取第 2 位，结果为 0 或 1
    sbus_data.lost_frame =
        (buf[23] >> 2) & 0x01;

    // 从标志字节 buf[23] 中提取 failsafe 标志
    // 取第 3 位，结果为 0 或 1
    sbus_data.failsafe =
        (buf[23] >> 3) & 0x01;
}