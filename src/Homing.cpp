#include "Homing.h"
#include "HandConfig.h"

// ----------------------- 不可变基线配置 -----------------------
// 总是定义两个符号，以便链接器可以解析它们
// 左手基线配置：7个舵机的抓取位置、伸展位置和方向
const ServoData sd_base_left[7] = {
  {3186,2048,1},   // 舵机0：拇指外展，抓取3186，伸展2048，正向
  {2048,865,-1},   // 舵机1：拇指屈曲，抓取2048，伸展865，反向
  {0,2980,1},      // 舵机2：拇指肌腱，抓取0，伸展2980，正向
  {4095,817,-1},   // 舵机3：食指，抓取4095，伸展817，反向
  {4095,817,-1},   // 舵机4：中指，抓取4095，伸展817，反向
  {4095,817,-1},   // 舵机5：无名指，抓取4095，伸展817，反向
  {4095,817,-1},   // 舵机6：小指，抓取4095，伸展817，反向
};

// 右手基线配置：7个舵机的抓取位置、伸展位置和方向
const ServoData sd_base_right[7] = {
  {910,2048,-1},   // 舵机0：拇指外展，抓取910，伸展2048，反向
  {2048,3231,1},   // 舵机1：拇指屈曲，抓取2048，伸展3231，正向
  {4095,1115,-1},  // 舵机2：拇指肌腱，抓取4095，伸展1115，反向
  {0,3278,1},      // 舵机3：食指，抓取0，伸展3278，正向
  {0,3278,1},      // 舵机4：中指，抓取0，伸展3278，正向
  {0,3278,1},      // 舵机5：无名指，抓取0，伸展3278，正向
  {0,3278,1},      // 舵机6：小指，抓取0，伸展3278，正向
};

// ----------------------- 工具函数 -----------------------
/**
 * @brief 重置舵机数据到基线配置
 * @details 根据编译时定义的左右手宏选择对应的基线配置
 *          将基线数据复制到全局舵机数据数组sd[]中
 */
void resetSdToBaseline() {
#if defined(RIGHT_HAND)
  const ServoData* src = sd_base_right;  // 使用右手基线配置
#elif defined(LEFT_HAND)
  const ServoData* src = sd_base_left;   // 使用左手基线配置
#else
  #warning "No hand macro defined; defaulting to RIGHT_HAND baseline"
  const ServoData* src = sd_base_right;  // 默认使用右手配置
#endif
  
  // 将基线配置复制到全局舵机数据数组
  for (int i = 0; i < 7; ++i) sd[i] = src[i];
}

// ----------------------- 归零状态标志 -----------------------
static volatile bool s_busy = false;  // 归零过程忙标志，volatile确保多线程可见性

/**
 * @brief 检查归零过程是否正在进行
 * @return true-归零进行中，false-归零已完成
 */
bool HOMING_isBusy() { 
  return s_busy; 
}

// ----------------------- 缓冲位置参数 -----------------------
static const float BUF_DEG = 10.0f;  // 缓冲角度：10度
static const uint16_t BUF_CNT =(uint16_t) ((BUF_DEG/360.0f) *4095.0f);  // 缓冲角度对应的计数值

/**
 * @brief 设置舵机的角度限位
 * @param servoID 舵机ID
 * @param minLim 最小角度限位 (0-4095)
 * @param maxLim 最大角度限位 (0-4095)
 * @details 写入舵机EPROM的角度限位寄存器，防止舵机超出安全范围
 */
static void set_servo_limits(uint8_t servoID, uint16_t minLim, uint16_t maxLim) {
  // 钳位到12位范围，防止越界
  if (minLim > 4095) minLim = 4095;
  if (maxLim > 4095) maxLim = 4095;

  // EPROM写入需要解锁（这会暂时禁用力矩输出）
  hlscl.unLockEprom(servoID);  // 解锁EPROM写入权限
  
  // 写入最小角度限位寄存器（地址9-10）
  hlscl.writeWord(servoID, HLSCL_MIN_ANGLE_LIMIT_L, minLim);
  
  // 写入最大角度限位寄存器（地址11-12）
  hlscl.writeWord(servoID, HLSCL_MAX_ANGLE_LIMIT_L, maxLim);
  
  hlscl.LockEprom(servoID);  // 锁定EPROM，防止意外修改

  // 恢复力矩输出，使运动命令能够正常工作
  hlscl.EnableTorque(servoID, 1);
}

/**
 * @brief 基于电流检测的单个舵机归零
 * @param index 舵机索引 (0-6)
 * @param direction 归零方向 (1=正向, -1=反向)
 * @param current_limit 电流限制阈值，用于检测接触
 * @details 通过检测电流突变来识别机械限位，然后执行校准
 */
static void zero_with_current(uint8_t index, int direction, int current_limit) {
  uint8_t servoID = SERVO_IDS[index];  // 获取舵机总线ID
  int current = 0;    // 当前电流值
  int position = 0;   // 当前位置值
  
  // 归零过程中，完全打开行程窗口，避免任何限位钳制
  set_servo_limits(servoID, 0, 0);  // 设置无限制的行程范围
  
  hlscl.ServoMode(servoID);   // 设置舵机为位置控制模式
  hlscl.FeedBack(servoID);    // 启用反馈读取
  
  uint32_t t0 = millis();  // 开始时间戳，用于超时检测
  
  // 电流检测循环：持续运动直到电流超过限制阈值
  while (abs(current) < current_limit) {
    // 向指定方向运动，速度2400，无加速度限制
    hlscl.WritePosEx(servoID, 50000 * direction, 2400, 0, current_limit);
    
    current  = hlscl.ReadCurrent(servoID);  // 读取当前电流
    position = hlscl.ReadPos(servoID);      // 读取当前位置
    
    if (millis() - t0 > 10000) break;  // 10秒超时保护
    vTaskDelay(pdMS_TO_TICKS(1));      // 短暂延迟，避免过度占用CPU
  }
  
  // 在接触点进行主校准
  hlscl.WritePosEx(servoID, position, 2400, 0, 1000);  // 移动到接触位置
  delay(30);  // 等待稳定
  hlscl.CalibrationOfs(servoID);  // 执行零点校准
  delay(30);  // 等待校准完成
  position = hlscl.ReadPos(servoID);  // 重新读取校准后的位置

  // 根据舵机ID执行不同的后处理逻辑
  if (servoID == 0) {
    // 拇指外展舵机：保持抓取姿势片刻
    hlscl.WritePosEx(servoID, sd[index].grasp_count, 2400, 0, 1000);
    delay(250);
  } else if (servoID == 1) {
    // 拇指屈曲舵机：移动到伸展位置
    hlscl.WritePosEx(servoID, sd[index].extend_count, 2400, 0, 1000);
    delay(250);
  } else if (servoID == 2) {
    // 拇指肌腱舵机：轻推并重新校准，然后伸展
    hlscl.WritePosEx(servoID, position + (direction * 2048), 2400, 0, 1000);
    delay(250);
    hlscl.CalibrationOfs(servoID);  // 再次校准
    delay(30);
    hlscl.WritePosEx(servoID, sd[index].extend_count - (direction * 625), 2400, 0, 1000);
    delay(30);
  } else {
    // 其他舵机：无特殊操作
    // 错误的舵机ID（理论上不会发生）
  }
}

/**
 * @brief 并行归零四个手指舵机（逻辑通道3-6）
 * @param firstIdx 起始索引 (3-6)
 * @param count 舵机数量 (1-4)
 * @param current_limit 电流限制阈值
 * @details 同时控制多个手指舵机进行归零，提高效率
 *          使用静态存储避免每次调用时的栈分配
 */
static void zero_fingers_parallel_with_current(uint8_t firstIdx, uint8_t count, int current_limit) {
  if (count == 0) return;  // 数量为0，直接返回
  if (firstIdx > 6) return;  // 起始索引越界
  if (firstIdx + count > 7) count = 7 - firstIdx;  // 调整数量，防止越界
  
  const uint32_t TIMEOUT_MS = 10000;  // 10秒超时时间
  
  // 使用静态存储避免每次调用时的栈分配（7个舵机的状态）
  static bool done[7];        // 完成标志数组
  static int  contactPos[7];  // 接触位置数组
  static int  cur[7];         // 电流值数组
  static int  pos[7];         // 位置值数组
  
  // 重置数组为每次调用的默认值（之前通过初始化器完成）
  for (uint8_t i = 0; i < 7; ++i) {
    done[i] = false;        // 标记所有舵机为未完成
    contactPos[i] = 0;      // 重置接触位置
    cur[i] = 0;             // 重置电流值
    pos[i] = 0;             // 重置位置值
  }
  
  // 初始化所有舵机：设置无限制行程和模式
  for (uint8_t i = 0; i < count; ++i) {
    uint8_t idx = firstIdx + i;
    uint8_t servoID = SERVO_IDS[idx];
    set_servo_limits(servoID, 0, 0);  // 设置无限制行程
    hlscl.ServoMode(servoID);         // 设置为位置控制模式
    hlscl.FeedBack(servoID);          // 启用反馈读取
  }
  
  uint32_t t0 = millis();  // 开始时间戳
  
  // 主归零循环
  while (true) {
    bool allDone = true;  // 所有舵机是否完成
    
    // 检查所有舵机是否都已完成归零
    for (uint8_t i = 0; i < count; ++i) {
      uint8_t idx = firstIdx + i;
      if (!done[idx]) { 
        allDone = false; 
        break; 
      }
    }
    
    if (allDone) break;  // 所有舵机完成，退出循环
    if (millis() - t0 > TIMEOUT_MS) break;  // 超时保护
    
    // 阶段1：发送运动命令给所有未完成的舵机
    for (uint8_t i = 0; i < count; ++i) {
      uint8_t idx = firstIdx + i;
      if (done[idx]) continue;  // 跳过已完成的舵机
      
      uint8_t servoID = SERVO_IDS[idx];
      int dir = sd[idx].servo_direction;  // 获取归零方向
      
      // 向指定方向运动
      hlscl.WritePosEx(servoID, 50000 * dir, 2400, 0, current_limit);
    }
    
    // 阶段2：读取所有未完成舵机的状态
    for (uint8_t i = 0; i < count; ++i) {
      uint8_t idx = firstIdx + i;
      if (done[idx]) continue;  // 跳过已完成的舵机
      
      uint8_t servoID = SERVO_IDS[idx];
      cur[idx] = hlscl.ReadCurrent(servoID);  // 读取电流
      pos[idx] = hlscl.ReadPos(servoID);      // 读取位置
      
      // 如果电流超过阈值，标记为完成并记录接触位置
      if (abs(cur[idx]) >= current_limit) {
        done[idx] = true;              // 标记为已完成
        contactPos[idx] = pos[idx];    // 记录接触位置
        
        // 缓慢移动到接触位置，避免冲击
        hlscl.WritePosEx(servoID, contactPos[idx], 60, 50, 1000);
      }
    }
    
    vTaskDelay(pdMS_TO_TICKS(1));  // 短暂延迟
  }
  
  // 后处理：对所有舵机执行校准和最终定位
  for (uint8_t i = 0; i < count; ++i) {
    uint8_t idx = firstIdx + i;
    uint8_t servoID = SERVO_IDS[idx];
    int dir = sd[idx].servo_direction;
    
    // 如果超时前未达到阈值，使用最后读取的位置作为回退
    int p = done[idx] ? contactPos[idx] : pos[idx];
    
    // 主校准：在接触点进行
    hlscl.WritePosEx(servoID, p, 2400, 0, 1000);
    delay(30);
    hlscl.CalibrationOfs(servoID);  // 执行零点校准
    delay(30);
    p = hlscl.ReadPos(servoID);     // 重新读取校准后的位置
    
    // 二次校准：轻推后重新校准
    hlscl.WritePosEx(servoID, p + (dir * 2048), 2400, 0, 1000);
    delay(300);
    hlscl.CalibrationOfs(servoID);  // 再次校准
    delay(30);
    
    // 最终定位：移动到伸展位置
    hlscl.WritePosEx(servoID, sd[idx].extend_count, 2400, 0, 1000);
  }
}

/**
 * @brief 归零所有舵机
 * @details 按顺序归零7个舵机：
 *          1. 拇指外展舵机 (索引0)
 *          2. 拇指屈曲舵机 (索引1)  
 *          3. 拇指肌腱舵机 (索引2)
 *          4. 四个手指舵机 (索引3-6，并行归零)
 *          最后执行归零后的稳定移动
 */
void zero_all_motors() {
  resetSdToBaseline();  // 重置到基线配置
  
  if (gBusMux) xSemaphoreTake(gBusMux, portMAX_DELAY);  // 获取总线访问权
  
  // 逐个归零拇指舵机（串行执行）
  zero_with_current(0,  sd[0].servo_direction, 950);   // 拇指外展舵机，电流阈值950
  zero_with_current(1,  sd[1].servo_direction, 950);   // 拇指屈曲舵机
  zero_with_current(2,  sd[2].servo_direction, 950);   // 拇指肌腱舵机
  
  // 并行归零四个手指舵机（索引3-6）
  zero_fingers_parallel_with_current(3, 4, 950);       // 四个手指一起归零
  
  // 归零后的稳定移动：将所有舵机移动到伸展位置
  hlscl.WritePosEx(SERVO_IDS[0], sd[0].extend_count, 2400, 0, 1023);   // 拇指外展到伸展
  hlscl.WritePosEx(SERVO_IDS[1], sd[1].extend_count, 2400, 0, 1023);   // 拇指屈曲到伸展
  hlscl.WritePosEx(SERVO_IDS[2], sd[2].extend_count, 2400, 0, 1023);   // 拇指肌腱到伸展
  hlscl.WritePosEx(SERVO_IDS[3], sd[3].extend_count, 2400, 0, 1023);   // 食指到伸展
  hlscl.WritePosEx(SERVO_IDS[4], sd[4].extend_count, 2400, 0, 1023);   // 中指到伸展
  hlscl.WritePosEx(SERVO_IDS[5], sd[5].extend_count, 2400, 0, 1023);   // 无名指到伸展
  hlscl.WritePosEx(SERVO_IDS[6], sd[6].extend_count, 2400, 0, 1023);   // 小指到伸展
  
  if (gBusMux) xSemaphoreGive(gBusMux);  // 释放总线访问权
}

/**
 * @brief 启动归零过程
 * @details 设置忙标志，执行所有舵机归零，然后清除忙标志
 *          这是归零模块的主要入口函数
 */
void HOMING_start() {
  s_busy = true;        // 设置归零忙标志
  zero_all_motors();    // 执行所有舵机归零（阻塞式）
  s_busy = false;       // 清除归零忙标志
}