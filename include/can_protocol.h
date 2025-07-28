
//
// CAN Protocol Definitions
// CAN协议定义文件 - 从您的原始代码提取和完善
//

#pragma once

#include <cstdint>

// VCI设备类型定义
#define VCI_USBCAN1    3
#define VCI_USBCAN2    4

// CAN消息结构体
typedef struct _VCI_CAN_OBJ {
    uint32_t ID;           // CAN ID
    uint32_t TimeStamp;    // 时间戳
    uint8_t  TimeFlag;     // 时间标志
    uint8_t  SendType;     // 发送标志：0=正常发送，1=单次发送，2=自发自收，3=单次自发自收
    uint8_t  RemoteFlag;   // 远程标志：0=数据帧，1=远程帧
    uint8_t  ExternFlag;   // 扩展标志：0=标准帧，1=扩展帧
    uint8_t  DataLen;      // 数据长度DLC(<=8)
    uint8_t  Data[8];      // CAN数据
    uint8_t  Reserved[3];  // 系统保留
} VCI_CAN_OBJ;

// CAN初始化结构体
typedef struct _VCI_INIT_CONFIG {
    uint32_t AccCode;      // 验收码
    uint32_t AccMask;      // 屏蔽码
    uint32_t Reserved;     // 保留
    uint8_t  Filter;       // 滤波方式：0=双滤波，1=单滤波
    uint8_t  Timing0;      // 定时器0
    uint8_t  Timing1;      // 定时器1
    uint8_t  Mode;         // 模式：0=正常，1=只听
} VCI_INIT_CONFIG;

// 设备信息结构体
typedef struct _VCI_BOARD_INFO {
    uint16_t hw_Version;   // 硬件版本
    uint16_t fw_Version;   // 固件版本
    uint16_t dr_Version;   // 驱动版本
    uint16_t in_Version;   // 接口版本
    uint16_t irq_Num;      // 中断号
    uint8_t  can_Num;      // CAN通道数
    uint8_t  str_Serial_Num[20]; // 序列号
    uint8_t  str_hw_Type[40];    // 硬件类型
    uint8_t  Reserved[4];  // 系统保留
} VCI_BOARD_INFO;

// CAN库函数声明（需要根据您的实际CAN库调整）
extern "C" {
    // 设备管理
    int VCI_OpenDevice(uint32_t DeviceType, uint32_t DeviceInd, uint32_t Reserved);
    int VCI_CloseDevice(uint32_t DeviceType, uint32_t DeviceInd);
    int VCI_InitCAN(uint32_t DeviceType, uint32_t DeviceInd, uint32_t CANInd, VCI_INIT_CONFIG* pInitConfig);
    int VCI_StartCAN(uint32_t DeviceType, uint32_t DeviceInd, uint32_t CANInd);
    int VCI_ResetCAN(uint32_t DeviceType, uint32_t DeviceInd, uint32_t CANInd);
    
    // 数据收发
    uint32_t VCI_Transmit(uint32_t DeviceType, uint32_t DeviceInd, uint32_t CANInd, VCI_CAN_OBJ* pSend, uint32_t Len);
    uint32_t VCI_Receive(uint32_t DeviceType, uint32_t DeviceInd, uint32_t CANInd, VCI_CAN_OBJ* pReceive, uint32_t Len, int WaitTime);
    
    // 设备查找
    int VCI_FindUsbDevice2(VCI_BOARD_INFO* pInfo);
    
    // 缓冲区管理
    int VCI_ClearBuffer(uint32_t DeviceType, uint32_t DeviceInd, uint32_t CANInd);
}

// 电机控制协议常量
namespace CanProtocol {
    // 位置限制 (rad)
    constexpr double POS_MINX = -12.5;
    constexpr double POS_MAXX = 12.5;
    
    // 速度限制 (rad/s)  
    constexpr double SPD_MINX = -65.0;
    constexpr double SPD_MAXX = 65.0;
    
    // 增益限制
    constexpr double KP_MINX = 0.0;
    constexpr double KP_MAXX = 500.0;
    constexpr double KD_MINX = 0.0;
    constexpr double KD_MAXX = 5.0;
    
    // 电机类型对应的扭矩和电流限制
    struct MotorLimits {
        double torque_max;
        double torque_min;
        double current_max;
        double current_min;
    };
    
    // 电机类型限制表
    constexpr MotorLimits LSG_20_90_7090 = {90.0, -90.0, 20.0, -20.0};
    constexpr MotorLimits LSG_10_414 = {414.0, -414.0, 10.0, -10.0};
    constexpr MotorLimits LSG_17_80_6070_new = {80.0, -80.0, 17.0, -17.0};
    constexpr MotorLimits LSG_14_70_5060 = {70.0, -70.0, 14.0, -14.0};
    
    // 获取电机限制
    inline MotorLimits getMotorLimits(int motor_id) {
        if (motor_id == 1 || motor_id == 2 || motor_id == 7 || motor_id == 8) {
            return LSG_20_90_7090;
        } else if (motor_id == 3 || motor_id == 4 || motor_id == 9 || motor_id == 10) {
            return LSG_10_414;
        } else if (motor_id == 5 || motor_id == 11 || motor_id == 16 || motor_id == 23) {
            return LSG_17_80_6070_new;
        } else {
            return LSG_14_70_5060;
        }
    }
    
    // CAN通信超时设置 (ms)
    constexpr int TRANSMIT_TIMEOUT = 100;
    constexpr int RECEIVE_TIMEOUT = 50;
    constexpr int DEVICE_DETECTION_TIMEOUT = 200;
    
    // 重试次数
    constexpr int MAX_RETRY_COUNT = 3;
    constexpr int MAX_RECEIVE_RETRY = 5;
}
