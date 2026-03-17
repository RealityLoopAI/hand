#pragma once  // 防止头文件重复包含
#include <Arduino.h>  // Arduino核心库，提供基本数据类型和函数
#include <HLSCL.h>    // Feetech HLS3606M舵机控制库

/**
 * @brief 舵机数据结构体
 * @details 存储单个舵机的关键配置参数
 */
struct ServoData {
  uint16_t grasp_count;     // 抓取位置计数值 (0-4095)，12位分辨率
  uint16_t extend_count;   // 伸展位置计数值 (0-4095)，12位分辨率
  int8_t   servo_direction; // 舵机运动方向：1=正向，-1=反向
};

// 活跃（可变）工作副本，固件运行时使用:
extern ServoData sd[7];  // 7个舵机的全局数据数组，存储当前配置

// 不可变基线配置（编译时常量）:
extern const ServoData sd_base_left[7];   // 左手基线配置，7个舵机的默认参数
extern const ServoData sd_base_right[7];  // 右手基线配置，7个舵机的默认参数

// ----------------------- 归零模块API -----------------------
/**
 * @brief 检查归零过程是否正在进行
 * @return true-归零进行中，false-归零已完成
 * @details 用于主循环中判断是否可以处理其他命令
 */
bool HOMING_isBusy();

/**
 * @brief 启动归零过程
 * @details 执行所有7个舵机的自动归零校准
 *          阻塞式函数，归零完成前不会返回
 */
void HOMING_start();

// ----------------------- 工具函数 -----------------------
/**
 * @brief 重置舵机数据到基线配置
 * @details 根据编译时定义的左右手宏选择对应的基线配置
 *          将基线数据复制到全局舵机数据数组sd[]中
 */
void resetSdToBaseline();                 // 复制正确的基线配置 -> sd数组

// ----------------------- 外部依赖声明 -----------------------
// 在其他地方定义的对象，供归零模块使用:
extern HLSCL hlscl;                    // 舵机控制对象，用于与HLS3606M舵机通信
extern SemaphoreHandle_t gBusMux;      // 舵机总线访问信号量，防止多任务同时访问UART

// 使用主程序中定义的固定7元素舵机ID列表:
extern const uint8_t SERVO_IDS[7];     // 例如：{0,1,2,3,4,5,6}，7个舵机的总线地址