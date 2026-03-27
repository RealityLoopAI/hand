// TetherIA - Open Source Hand
// Aero Hand Firmware Source Code
// 机器人手部控制固件 - 基于ESP32-S3 XIAO开发板
#include <Arduino.h>
#include <Wire.h>
#include <HLSCL.h>      // Feetech HLS3606M舵机控制库
#include <Preferences.h> // ESP32非易失性存储库
#include "HandConfig.h" // 左右手配置定义
#include "Homing.h"     // 舵机归零校准模块
#include <WiFi.h>       // WiFi库，用于远程控制和调试
#include <PubSubClient.h> // MQTT库，用于与MQTT服务器通信

const char* ssid = "mqtt_server";         // WiFI网络名称
const char* password = "12345678";     //WiFi密码

// MQTT配置参数
const char* mqtt_server = "192.168.137.1";  // MQTT服务器地址
const int mqtt_port = 1883;                  // MQTT服务器端口
const char* mqtt_client_id = "hand";   // MQTT客户端ID
const char* mqtt_username = "123";              // MQTT用户名（可选）
const char* mqtt_password = "456";              // MQTT密码（可选）
// MQTT主题定义
const char* topic_command = "hand";     // 测试接收主题
const char* topic_push = "hand"; // 测试状态主题
// MQTT客户端对象
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
// MQTT连接状态
bool mqtt_connected = false;
uint32_t last_mqtt_reconnect = 0;
uint32_t last_mqtt_publish = 0;


// 全局对象声明
HLSCL hlscl;            // 舵机控制对象，用于与HLS3606M舵机通信
Preferences prefs;      // 非易失性存储对象，用于保存配置参数

// ---- 舵机总线UART引脚定义 (ESP32-S3 XIAO开发板) ----
#define SERIAL2_TX_PIN 3  // UART发送引脚，连接舵机总线
#define SERIAL2_RX_PIN 2  // UART接收引脚，连接舵机总线

// ---- 舵机ID定义 (7个舵机对应7个手指) ----
const uint8_t SERVO_IDS[7] = { 0, 1, 2, 3, 4, 5, 6 };

ServoData sd[7];  // 舵机数据数组，存储每个舵机的抓取和伸展位置

// ---- 控制命令码定义 (16字节帧协议) ----
static const uint8_t HOMING    = 0x01;  // 归零初始化命令 - 执行自动归零校准
static const uint8_t SET_ID    = 0x02;  // 设置舵机ID命令 - 修改舵机总线地址
static const uint8_t TRIM      = 0x03;  // 微调命令 - 调整舵机伸展位置
static const uint8_t CAPTURE   = 0x04;  // 自定义抓取命令 - 控制舵机是抓取物体
static const uint8_t TEST      = 0x05;  // 自定义测试命令
static const uint8_t CTRL_POS  = 0x11;  // 位置控制模式 - 精确控制舵机角度
static const uint8_t CTRL_TOR  = 0x12;  // 力矩控制模式 - 控制舵机输出力矩
static const uint8_t GET_POS   = 0x22;  // 获取位置数据 - 读取所有舵机当前位置
static const uint8_t GET_VEL   = 0x23;  // 获取速度数据 - 读取所有舵机当前速度
static const uint8_t GET_CURR  = 0x24;  // 获取电流数据 - 读取所有舵机当前电流
static const uint8_t GET_TEMP  = 0x25;  // 获取温度数据 - 读取所有舵机当前温度
static const uint8_t SET_SPE   = 0x31;  // 设置速度参数 - 配置单个舵机运动速度
static const uint8_t SET_TOR   = 0x32;  // 设置力矩参数 - 配置单个舵机力矩限制

// ---- 同步位置写入默认参数 (SyncWritePosEx函数使用) ----
static uint16_t g_speed[7]  = {32766,32766,32766,32766,32766,32766,32766};  // 速度设定值 (0-32766)
static uint8_t  g_accel[7]  = {200,200,200,200,200,200,200};     // 加速度设定值 (0-255)，0表示无加速度限制
static uint16_t g_torque[7] = {900,900,900,700,700,700,700}; // 扭矩最大值 (0-1000)，防止过载
static const uint16_t HOLD_MAG = 5;       // 力矩控制模式下的最小保持力矩，小于此值的力矩将被忽略

// 上次命令的力矩值数组 (有符号，支持正负力矩)
static int16_t g_lastTorqueCmd[7] = {0};  // 记录每个舵机上次发送的力矩命令

// ---- 热保护参数 (全局安全设置) ----
static uint8_t  TEMP_CUTOFF_C    = 50;    // 过热保护温度阈值 (°C)，超过此温度将限制力矩
static uint16_t HOT_TORQUE_LIMIT = 200;   // 过热时的最大力矩限制，防止电机损坏

// ----- 舵机寄存器地址定义 (基于Feetech HLS3606M舵机规格) -----
#define REG_ID                 0x05       // ID寄存器地址，用于设置舵机总线地址
#define REG_CURRENT_LIMIT      28         // 电流限制寄存器地址(十进制)，保护舵机不过载
#define BROADCAST_ID           0xFE       // 广播ID，用于同时控制所有舵机
#define SCAN_MIN               0          // ID扫描最小范围
#define SCAN_MAX               253        // ID扫描最大范围
#define REG_BLOCK_LEN          15         // 同步读取数据块长度(字节)
#define REG_BLOCK_START        56         // 同步读取起始寄存器地址

// ----- 舵机状态数据结构 (用于实时监控和反馈) -----
struct ServoMetrics {
  uint16_t pos[7];  // 位置数据 (0-65535)，映射自原始12位位置(0-4095)
  uint16_t vel[7];  // 速度数据 (有符号15位表示)，正负表示方向
  uint16_t cur[7];  // 电流数据 (有符号15位表示)，反映负载情况
  uint16_t tmp[7];  // 温度数据 (0-255°C)，用于过热保护
};
static ServoMetrics gMetrics;  // 全局舵机状态实例，由同步读取任务更新

// -------- 全局控制模式状态枚举 --------
enum ControlMode{
  MODE_POS=0,   // 位置控制模式：精确控制舵机角度位置
  MODE_TORQUE=2 // 力矩控制模式：控制舵机输出力矩，用于柔顺控制
};
static ControlMode g_currentMode = MODE_POS;  // 当前控制模式，默认为位置控制

// ----- FreeRTOS信号量定义 (用于线程安全的多任务同步) -----
static SemaphoreHandle_t gMetricsMux;    // 舵机状态数据访问信号量，保护gMetrics
SemaphoreHandle_t gBusMux = nullptr;     // 舵机总线访问信号量，防止多任务同时访问UART

// ----- 归零模块API声明 (实现在Homing.h/cpp中) -----
bool HOMING_isBusy();    // 检查归零过程是否正在进行
void HOMING_start();      // 启动自动归零校准过程

// 函数提前声明
static inline void sendAckFrame(uint8_t header, const uint8_t* payload, size_t n);

// MQTT相关函数声明
void mqttCallback(char* topic, byte* payload, unsigned int length); // MQTT消息回调函数
bool mqttConnect();                                                 // MQTT连接函数，尝试连接到MQTT服务器
void mqttReconnect();                                               // MQTT重连函数，在连接丢失时调用      
void handleMqttCommand(const char* command);                        // 处理接收到的MQTT命令，根据命令内容执行相应操作

// ---- 温度监控辅助函数 ----
/**
 * @brief 线程安全地获取指定通道舵机的当前温度
 * @param ch 舵机通道号 (0-6)
 * @return 当前温度值 (°C)
 */
static inline uint8_t getTempC(uint8_t ch) {
  uint8_t t = 0;
  if (gMetricsMux) xSemaphoreTake(gMetricsMux, portMAX_DELAY);  // 获取信号量
  t = (uint8_t)gMetrics.tmp[ch];  // 读取温度数据
  if (gMetricsMux) xSemaphoreGive(gMetricsMux);  // 释放信号量
  return t;
}

/**
 * @brief 判断指定通道舵机是否过热
 * @param ch 舵机通道号 (0-6)
 * @return true-过热，false-正常
 */
static inline bool isHot(uint8_t ch) {
  return getTempC(ch) >= TEMP_CUTOFF_C;  // 比较当前温度与阈值
}

/**
 * @brief 返回两个16位无符号整数中的较小值
 * @param a 第一个数值
 * @param b 第二个数值
 * @return 较小的数值
 */
static inline uint16_t u16_min(uint16_t a, uint16_t b) { return (a < b) ? a : b; }

// ---- 舵机ID设置辅助函数 ----
extern void runReIdScanAndSet(uint8_t Id, uint16_t currentLimit);  // 外部ID设置函数
static volatile int g_lastFoundId;  // 最后找到的舵机ID，用于ID设置过程 

// ----- ID设置模式辅助函数 -----
/**
 * @brief 扫描舵机总线，确保只有一个舵机响应（ID设置前的安全检查）
 * @param outId 输出参数，存储找到的舵机ID
 * @param requestedNewId 请求设置的新ID
 * @return true-找到唯一舵机，false-未找到或多个舵机响应
 */
static bool scanRequireSingleServo(uint8_t* outId, uint8_t requestedNewId) {
  uint8_t first = 0xFF;  // 第一个响应的舵机ID
  int count = 0;         // 响应的舵机数量
  
  if (gBusMux) xSemaphoreTake(gBusMux, portMAX_DELAY);  // 获取总线访问权
  
  // 扫描所有可能的ID范围 (0-253)
  for (int id = SCAN_MIN; id <= SCAN_MAX; ++id) {
    if (id == BROADCAST_ID) continue;        // 跳过广播ID
    (void)hlscl.Ping((uint8_t)id);           // 发送Ping命令检测舵机
    if (!hlscl.getLastError()) {            // 如果无错误，说明舵机存在
      if (count == 0) first = (uint8_t)id;  // 记录第一个响应的舵机
      ++count;                               // 增加计数器
      if (count > 1) break;                  // 如果找到多个舵机，提前退出
    }
  }
  
  if (gBusMux) xSemaphoreGive(gBusMux);      // 释放总线访问权
  
  if (count == 1) {                          // 找到唯一舵机
    if (outId) *outId = first;               // 输出找到的ID
    return true;
  }
  
  if (count == 0) {                          // 未找到任何舵机
    uint8_t ack6[6] = { 0xFF, 0x00, requestedNewId, 0x00, 0x00, 0x00 };
    sendAckFrame(SET_ID, ack6, sizeof(ack6));  // 发送错误确认帧
    return false;
  }
  
  // 找到多个舵机（ID冲突）
  uint8_t ack14[14] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  sendAckFrame(SET_ID, ack14, sizeof(ack14));  // 发送冲突错误确认帧
  return false;
}
static void sendSetIdAck(uint8_t oldId, uint8_t newId, uint16_t curLimitWord) {
  uint16_t vals[7] = {0};
  vals[0] = oldId;
  vals[1] = newId;
  vals[2] = curLimitWord;
  uint8_t out[2 + 7*2];
  out[0] = SET_ID;  // 0x03
  out[1] = 0x00;    // filler
  for (int i = 0; i < 7; ++i) {
    out[2 + 2*i + 0] = (uint8_t)(vals[i] & 0xFF);
    out[2 + 2*i + 1] = (uint8_t)((vals[i] >> 8) & 0xFF);
  }
  Serial.write(out, sizeof(out));
}
// ---- NVS非易失性存储辅助函数 ----
/**
 * @brief 从NVS存储加载手动调整的伸展位置
 * @details 每个舵机对应一个"extX"键值，存储伸展位置的12位计数值(0-4095)
 */
static void loadManualExtendsFromNVS() {
  prefs.begin("hand", true);  // 以只读模式打开"hand"命名空间
  for (uint8_t i = 0; i < 7; ++i) {
    // 按逻辑通道索引存储，而不是舵机总线ID
    String key = "ext" + String(i);  // 生成键名：ext0, ext1, ..., ext6
    int v = prefs.getInt(key.c_str(), -1);  // 读取存储值，默认值-1表示未设置
    if (v >= 0 && v <= 4095) {               // 验证值在有效范围内
      sd[i].extend_count = v;                // 更新舵机伸展位置
    }
  }
}

/**
 * @brief 保存伸展位置到NVS存储
 * @details 将当前所有舵机的伸展位置持久化到非易失性存储
 */
static void saveExtendsToNVS() {
  prefs.begin("hand", false);  // 以读写模式打开"hand"命名空间
  for (uint8_t i = 0; i < 7; ++i) {
    // 按逻辑通道索引存储
    String kext = "ext" + String(i);  // 生成键名
    prefs.putInt(kext.c_str(), (int)sd[i].extend_count);  // 保存伸展位置
  }
  prefs.end();  // 关闭命名空间，确保数据写入Flash
}

// -------- 舵机软限位安全检查函数 --------
/**
 * @brief 检查并强制执行软限位保护
 * @details 在力矩控制模式下，防止舵机超出安全范围
 *          每20ms检查一次，避免频繁的舵机命令
 */
static void checkAndEnforceSoftLimits()
{
  static bool torqueLimited[7] = {false};  // 记录每个舵机是否已被限位
  static uint32_t lastCheckMs = 0;         // 上次检查时间戳
  uint32_t now = millis();
  
  if (now - lastCheckMs < 20) return;      // 20ms检查间隔，避免过度频繁
  lastCheckMs = now;

  // 只在力矩控制模式下应用软限位
  if (g_currentMode != MODE_TORQUE) {
    for (int i = 0; i < 7; ++i) torqueLimited[i] = false;  // 重置限位状态
    return;
  }

  if (gBusMux) xSemaphoreTake(gBusMux, portMAX_DELAY);  // 获取总线访问权
  
  for (uint8_t i = 0; i < 7; ++i) {
    uint8_t id = SERVO_IDS[i];
    int pos = hlscl.ReadPos(id);           // 读取当前舵机位置
    if (pos < 0) pos += 32768;             // 处理负位置值
    uint16_t raw = pos % 4096;             // 转换为12位原始位置(0-4095)
    
    uint16_t ext = sd[i].extend_count;     // 伸展位置
    uint16_t gra = sd[i].grasp_count;      // 抓取位置
    uint16_t rawMin = min(ext, gra);       // 安全范围最小值
    uint16_t rawMax = max(ext, gra);       // 安全范围最大值
    
    bool inRange = (raw >= rawMin && raw <= rawMax);  // 检查是否在安全范围内
    
    if (!inRange && !torqueLimited[i]) {
      // 超出安全范围且未限位：应用限位力矩
      int16_t limitedTorque = (g_lastTorqueCmd[i] >= 0) ? 200 : -200;  // 200单位限位力矩
      hlscl.WriteEle(id, limitedTorque);   // 写入限位力矩命令
      torqueLimited[i] = true;              // 标记为已限位
    } else if (inRange && torqueLimited[i]) {
      // 回到安全范围且已限位：恢复原始力矩
      hlscl.WriteEle(id, g_lastTorqueCmd[i]);  // 恢复上次命令的力矩
      torqueLimited[i] = false;             // 清除限位标记
    }
  }
  
  if (gBusMux) xSemaphoreGive(gBusMux);    // 释放总线访问权
}


// ---- 坐标映射辅助函数 (原始位置 ↔ 16位值) ----
/**
 * @brief 将原始舵机位置(0-4095)映射到16位值(0-65535)
 * @param ch 舵机通道号 (0-6)
 * @param raw 原始12位位置值 (0-4095)
 * @return 映射后的16位位置值 (0-65535)
 * @details 基于舵机的伸展和抓取位置进行线性映射
 */
static inline uint16_t mapRawToU16(uint8_t ch, uint16_t raw) {
  int32_t ext  = sd[ch].extend_count;  // 伸展位置
  int32_t gra  = sd[ch].grasp_count;   // 抓取位置
  int32_t span = gra - ext;            // 有效行程范围
  
  if (span == 0) return 0;  // 避免除零错误
  
  // 线性映射公式：将(raw-ext)映射到0-65535范围
  int32_t val = ((int32_t)(raw - ext) * 65535L) / span;
  
  // 钳位处理，确保值在有效范围内
  if (val < 0) val = 0;
  if (val > 65535) val = 65535;
  
  return (uint16_t)val;
}

/**
 * @brief 将16位值(0-65535)映射回原始舵机位置(0-4095)
 * @param ch 舵机通道号 (0-6)
 * @param u16 16位位置值 (0-65535)
 * @return 映射后的原始12位位置值 (0-4095)
 * @details 反向坐标转换，用于将控制命令转换为舵机实际位置
 */
static inline uint16_t mapU16ToRaw(uint8_t ch, uint16_t u16) {
  int32_t ext = sd[ch].extend_count;  // 伸展位置
  int32_t gra = sd[ch].grasp_count;   // 抓取位置
  int32_t raw32;
  
  if (ext == 0 && gra == 0) {
    // 如果未设置伸展/抓取位置，使用全范围映射
    raw32 = ((uint64_t)u16 * 4095u) / 65535u;
  } else {
    // 基于有效行程的线性映射
    raw32 = ext + ((int64_t)u16 * (gra - ext)) / 65535LL;
  }
  
  // 钳位处理，确保值在有效范围内
  if (raw32 < 0)    raw32 = 0;
  if (raw32 > 4095) raw32 = 4095;
  
  return (uint16_t)raw32;
}

// ---- 数据编码/解码辅助函数 ----
/**
 * @brief 小端序读取16位无符号整数
 * @param p 指向2字节数据的指针
 * @return 解码后的16位无符号整数
 * @details 从字节流中读取小端序格式的16位数据
 */
static inline uint16_t leu_u16(const uint8_t *p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);  // 低字节在前，高字节在后
}

/**
 * @brief 解码15位有符号数值（符号+幅度表示法）
 * @param lo 低8位数据
 * @param hi 高8位数据（包含符号位）
 * @return 解码后的有符号16位整数
 * @details HLS3606M舵机使用15位幅度+1位符号的格式表示有符号数据
 *          最高位(hi的第7位)为符号位，0=正数，1=负数
 */
static inline int16_t decode_signmag15(uint8_t lo, uint8_t hi) {
  uint16_t mag = ((uint16_t)(hi & 0x7F) << 8) | lo;  // 提取15位幅度值
  return (hi & 0x80) ? -(int16_t)mag : (int16_t)mag; // 根据符号位返回正负值
}

/**
 * @brief 复制7个16位数值的数组
 * @param dst 目标数组
 * @param src 源数组
 * @details 用于快速复制舵机状态数据
 */
static inline void copy7_u16(uint16_t dst[7], const uint16_t src[7]) {
  for (int i = 0; i < 7; ++i) dst[i] = src[i];  // 逐个复制7个元素
}

// ----- 数据发送辅助函数 ----
/**
 * @brief 发送包含7个16位数据的帧
 * @param header 帧头（控制命令码）
 * @param data 包含7个16位数据的数组
 * @details 帧格式：头部(1字节) + 填充(1字节) + 7个16位数据(14字节) = 16字节
 */
static inline void sendU16Frame(uint8_t header, const uint16_t data[7]) {
  uint8_t out[2 + 7*2];  // 2字节头部 + 14字节数据 = 16字节
  out[0] = header;       // 帧头：控制命令码
  out[1] = 0x00;         // 填充字节
  
  // 将7个16位数据转换为小端序字节流
  for (int i = 0; i < 7; ++i) {
    out[2 + 2*i + 0] = (uint8_t)(data[i] & 0xFF);   // 低字节
    out[2 + 2*i + 1] = (uint8_t)((data[i] >> 8) & 0xFF); // 高字节
  }
  
  Serial.write(out, sizeof(out));  // 通过USB串口发送完整帧
}

/**
 * @brief 发送确认帧（ACK帧）
 * @param header 帧头（控制命令码）
 * @param payload 负载数据指针
 * @param n 负载数据长度
 * @details 用于命令执行的确认响应，固定16字节长度
 */
static inline void sendAckFrame(uint8_t header, const uint8_t* payload, size_t n) {
  uint8_t out[16];       // 固定16字节帧
  out[0] = header;       // 帧头：原命令码
  out[1] = 0x00;         // 填充字节
  memset(out + 2, 0, 14); // 清空剩余14字节
  
  if (payload && n) {
    if (n > 14) n = 14;  // 限制负载长度不超过14字节
    memcpy(out + 2, payload, n);  // 复制负载数据
  }
  
  Serial.write(out, sizeof(out));  // 发送确认帧
}

// ----- 实时数据发送函数 (响应GET_*命令) -----
/**
 * @brief 发送所有舵机的位置数据
 * @details 线程安全地读取并发送7个舵机的当前位置
 */
void sendPositions() {
  uint16_t buf[7];
  xSemaphoreTake(gMetricsMux, portMAX_DELAY);  // 获取状态数据访问权
  copy7_u16(buf, gMetrics.pos);                // 复制位置数据
  xSemaphoreGive(gMetricsMux);                 // 释放访问权
  sendU16Frame(GET_POS, buf);                  // 发送位置帧
}

/**
 * @brief 发送所有舵机的速度数据
 * @details 线程安全地读取并发送7个舵机的当前速度
 */
void sendVelocities() {
  uint16_t buf[7];
  xSemaphoreTake(gMetricsMux, portMAX_DELAY);
  copy7_u16(buf, gMetrics.vel);
  xSemaphoreGive(gMetricsMux);
  sendU16Frame(GET_VEL, buf);
}

/**
 * @brief 发送所有舵机的电流数据
 * @details 线程安全地读取并发送7个舵机的当前电流
 */
void sendCurrents() {
  uint16_t buf[7];
  xSemaphoreTake(gMetricsMux, portMAX_DELAY);
  copy7_u16(buf, gMetrics.cur);
  xSemaphoreGive(gMetricsMux);
  sendU16Frame(GET_CURR, buf);
}

/**
 * @brief 发送所有舵机的温度数据
 * @details 线程安全地读取并发送7个舵机的当前温度
 */
void sendTemps() {
  uint16_t buf[7];
  xSemaphoreTake(gMetricsMux, portMAX_DELAY);
  copy7_u16(buf, gMetrics.tmp);
  xSemaphoreGive(gMetricsMux);
  sendU16Frame(GET_TEMP, buf);
}

// ----- 同步读取任务 (在核心1上运行) -----
/**
 * @brief 实时同步读取所有舵机数据的FreeRTOS任务
 * @param arg 任务参数（未使用）
 * @details 在核心1上独立运行，以100Hz频率同步读取7个舵机的状态数据
 *          使用同步读取协议提高效率，减少总线占用时间
 */
static void TaskSyncRead_Core1(void *arg) {
  uint8_t  rx[REG_BLOCK_LEN];          // 15字节接收缓冲区
  uint16_t pos[7], vel[7], cur[7], tmp[7];  // 临时存储7个舵机的4种数据
  
  const TickType_t period = pdMS_TO_TICKS(10);   // 任务周期：10ms = 100Hz采样率
  TickType_t nextWake = xTaskGetTickCount();     // 下次唤醒时间
  
  for (;;) {
    // 尝试获取总线访问权：如果控制任务正在使用总线，跳过本次循环
    if (gBusMux && xSemaphoreTake(gBusMux, 0) != pdTRUE) {
      vTaskDelayUntil(&nextWake, period);
      continue;
    }
    
    bool ok = true;  // 本次读取是否成功
    
    // 发送同步读取命令：一次TX读取整个舵机组（15字节数据块）
    hlscl.syncReadPacketTx((uint8_t*)SERVO_IDS, 7, REG_BLOCK_START, REG_BLOCK_LEN);
    
    // 逐个读取每个舵机的响应数据
    for (uint8_t i = 0; i < 7; ++i) {
      if (!hlscl.syncReadPacketRx(SERVO_IDS[i], rx)) { 
        ok = false;  // 读取失败
        break; 
      }
      
      // 解析接收到的数据
      uint16_t raw = leu_u16(&rx[0]);                    // 原始位置 (0-4095)
      pos[i] = mapRawToU16(i,raw);                       // 映射到16位位置 (0-65535)
      vel[i] = decode_signmag15(rx[2], rx[3]);           // 速度 (有符号15位)
      tmp[i] = rx[7];                                    // 温度 (8位无符号)
      cur[i] = decode_signmag15(rx[13], rx[14]);         // 电流 (有符号15位)
    }
    
    if (gBusMux) xSemaphoreGive(gBusMux);  // 释放总线访问权
    
    // 如果读取成功，更新全局状态数据
    if (ok) {
      xSemaphoreTake(gMetricsMux, portMAX_DELAY);  // 获取状态数据访问权
      for (int i = 0; i < 7; ++i) {
        gMetrics.pos[i] = pos[i];    // 更新位置数据
        gMetrics.vel[i] = vel[i];    // 更新速度数据
        gMetrics.tmp[i] = tmp[i];    // 更新温度数据
        gMetrics.cur[i] = cur[i];    // 更新电流数据
      }
      xSemaphoreGive(gMetricsMux);    // 释放状态数据访问权
    } 
    
    // 精确延时，确保100Hz采样率
    vTaskDelayUntil(&nextWake, period);
  }
}
//Set-ID and Trim Servo functions
static bool handleSetIdCmd(const uint8_t* payload) {
  // Parse request: two u16 words, little-endian
  uint16_t w0 = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8); // newId in low byte
  uint16_t w1 = (uint16_t)payload[2] | ((uint16_t)payload[3] << 8); // requested current limit
  uint8_t  newId    = (uint8_t)(w0 & 0xFF);
  uint16_t reqLimit = (w1 > 1023) ? 1023 : w1;
  // Invalid newId → ACK with oldId=0xFF, newId, cur=0
  if (newId > 253 || newId ==BROADCAST_ID) {
    uint8_t ack[6] = { 0xFF, 0x00, newId, 0x00, 0x00, 0x00 };
    sendAckFrame(SET_ID, ack, sizeof(ack));
    return true;
  }
  // Find any servo present
  uint8_t oldId = 0xFF;
  if (!scanRequireSingleServo(&oldId, newId)) return true; 
  
  if (newId != oldId) {
  if (gBusMux) xSemaphoreTake(gBusMux, portMAX_DELAY);
  (void)hlscl.Ping(newId);
  bool taken = !hlscl.getLastError();
  if (gBusMux) xSemaphoreGive(gBusMux);
  if (taken) {
    uint8_t ack14[14] = { oldId, 0x00, newId, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    sendAckFrame(SET_ID, ack14, sizeof(ack14));
    return true;
  }
  }
  if (gBusMux) xSemaphoreTake(gBusMux, portMAX_DELAY);
  (void)hlscl.unLockEprom(oldId);
  (void)hlscl.writeWord(oldId, REG_CURRENT_LIMIT, reqLimit);
  uint8_t targetId = oldId;
  if (newId != oldId) {
    (void)hlscl.writeByte(oldId, REG_ID, newId);   // REG_ID = 0x05
    targetId = newId;
  }
  (void)hlscl.LockEprom(targetId);
  
  // Read back limit for ACK
  uint16_t curLimitRead = 0;
  int rd = hlscl.readWord(targetId, REG_CURRENT_LIMIT);
  if (rd >= 0) curLimitRead = (uint16_t)rd;
  if (gBusMux) xSemaphoreGive(gBusMux);
  // Build 6-byte payload: oldId(LE16), newId(LE16), curLimit(LE16) and send
  uint8_t ack[6];
  ack[0] = oldId;                      // oldId lo
  ack[1] = 0x00;                       // oldId hi
  ack[2] = targetId;                   // newId lo
  ack[3] = 0x00;                       // newId hi
  ack[4] = (uint8_t)(curLimitRead & 0xFF);
  ack[5] = (uint8_t)((curLimitRead >> 8) & 0xFF);
  sendAckFrame(SET_ID, ack, sizeof(ack));
  return true;
}
static bool handleTrimCmd(const uint8_t* payload) {
  // Parse little-endian fields
  uint16_t rawCh  = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
  uint16_t rawDeg = (uint16_t)payload[2] | ((uint16_t)payload[3] << 8);
  int      ch      = (int)rawCh;         // 0..6
  int16_t  degrees = (int16_t)rawDeg;    // signed degrees
  // Validate channel (0..6)
  if (ch < 0 || ch >= 7) {
    // Optionally send a NACK or silent-accept to preserve framing
    return true;
  }
  // Degrees -> counts (≈11.375 counts/deg), clamp to 0..4095
  int delta_counts = (int)((float)degrees * 11.375f);
  int new_ext = (int)sd[ch].extend_count + delta_counts;
  if (new_ext < 0)    new_ext = 0;
  if (new_ext > 4095) new_ext = 4095;
  sd[ch].extend_count = (uint16_t)new_ext;
  // Persist to NVS
  prefs.begin("hand", false);
  prefs.putInt(String("ext" + String(ch)).c_str(), sd[ch].extend_count);
  prefs.end();
  // ACK payload: ch (u16, LE), extend_count (u16, LE)
  uint8_t ack[4];
  ack[0] = (uint8_t)(ch & 0xFF);
  ack[1] = (uint8_t)((ch >> 8) & 0xFF);
  ack[2] = (uint8_t)(sd[ch].extend_count & 0xFF);
  ack[3] = (uint8_t)((sd[ch].extend_count >> 8) & 0xFF);
  sendAckFrame(TRIM, ack, sizeof(ack));   // 16 bytes on the wire
  return true;
}
static bool handleSetSpeedCmd(const uint8_t* payload)
{
  uint16_t rawId = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
  uint16_t rawSpd = (uint16_t)payload[2] | ((uint16_t)payload[3] << 8);
  if (rawId >= 7) {
    //invalid index of servo actuator - So give error code (0–6 valid)
    uint8_t ack[4] = {0xFF, 0xFF, 0x00, 0x00};
    sendAckFrame(SET_SPE, ack, sizeof(ack));
    return true;
  }
  if (rawSpd > 32766) rawSpd = 32766;  // clamp
  g_speed[rawId] = rawSpd;
  // ACK back: servo id and speed
  uint8_t ack[4];
  ack[0] = (uint8_t)(rawId & 0xFF);
  ack[1] = (uint8_t)((rawId >> 8) & 0xFF);
  ack[2] = (uint8_t)(rawSpd & 0xFF);
  ack[3] = (uint8_t)((rawSpd >> 8) & 0xFF);
  sendAckFrame(SET_SPE, ack, sizeof(ack));
  return true;
}
static bool handleSetTorCmd(const uint8_t* payload)
{
  uint16_t rawId = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
  uint16_t rawTor = (uint16_t)payload[2] | ((uint16_t)payload[3] << 8);
  // Validate ID range (0..6)
  if (rawId >= 7) {
    uint8_t ack[4] = {0xFF, 0xFF, 0x00, 0x00};  // invalid ID ack
    sendAckFrame(SET_TOR, ack, sizeof(ack));
    return true;
  }
  if (rawTor > 1023) rawTor = 1023;
  // Update the per-servo torque value
  g_torque[rawId] = rawTor;
  uint8_t ack[4];
  ack[0] = (uint8_t)(rawId & 0xFF);
  ack[1] = (uint8_t)((rawId >> 8) & 0xFF);
  ack[2] = (uint8_t)(rawTor & 0xFF);
  ack[3] = (uint8_t)((rawTor >> 8) & 0xFF);
  sendAckFrame(SET_TOR, ack, sizeof(ack));
  return true;
}

// ----- 主机命令帧处理函数 -----
/**
 * @brief 处理来自主机的16字节命令帧
 * @param op 操作码（命令类型）
 * @return true-成功处理并消耗了帧，false-处理失败
 * @details 帧格式：操作码(1字节) + 填充(1字节) + 负载(14字节) = 16字节
 *          支持位置控制、力矩控制、参数设置、数据读取等多种命令
 */
uint8_t buf[15];
bool control_falg = true;
static bool handleHostFrame(uint8_t op) {
  // 等待完整帧到达：填充字节 + 14字节负载
  if(control_falg)
  {
    while (Serial.available() < 15) { /* 等待 */ }
    
    // 读取完整16字节帧（操作码已在参数中）
    for (int i = 0; i < 15; ++i) {
      int ch = Serial.read(); 
      if (ch < 0) return false;  // 读取失败
      buf[i] = (uint8_t)ch;
    }
  }
  const uint8_t* payload = &buf[1]; // buf[0] = 填充字节0x00（忽略）

  // 根据操作码分发到不同的处理函数
  switch (op) {
    case CTRL_POS: {
      // 位置控制模式：接收7个16位位置值，控制所有舵机
      int16_t pos[7];  // 目标位置数组（原始12位值）
      
      // 解析7个16位位置值
      for (int i = 0; i < 7; ++i) {
        uint16_t u16 = (uint16_t)payload[2*i] | ((uint16_t)payload[2*i+1] << 8);
        uint8_t  ch  = i;  // 舵机通道号
        pos[i] = mapU16ToRaw(ch, u16);  // 映射到原始舵机位置
      }

      // 力矩限制处理：如果电机过热，限制最大力矩
      uint16_t torque_eff[7];  // 有效力矩限制数组
      for (int i = 0; i < 7; ++i) {
        uint16_t base = g_torque[i]; // 基础力矩限制 (0-1023)
        if (isHot((uint8_t)i)) {     // 检查是否过热
          base = u16_min(base, HOT_TORQUE_LIMIT);  // 应用过热保护限制
        }
        torque_eff[i] = base;  // 设置有效力矩限制
      }

      // 执行位置控制命令
      if (gBusMux) xSemaphoreTake(gBusMux, portMAX_DELAY);  // 获取总线访问权
      
      // 如果当前不是位置模式，切换到位置模式
      if (g_currentMode != MODE_POS) {
        for (int i = 0; i < 7; ++i) {
          uint8_t id = SERVO_IDS[i];
          hlscl.ServoMode(id);  // 设置舵机为位置控制模式
        }
        g_currentMode = MODE_POS;  // 更新全局模式状态
      }
      
      // 同步写入所有舵机的位置、速度、加速度和力矩限制
      hlscl.SyncWritePosEx((uint8_t*)SERVO_IDS, 7, pos, g_speed, g_accel, torque_eff);
      
      if (gBusMux) xSemaphoreGive(gBusMux);  // 释放总线访问权
      return true;
    }

    case CTRL_TOR: 
    {   
      // 力矩控制模式：接收7个16位力矩值，控制所有舵机输出力矩
      int16_t torque_cmd[7];  // 力矩命令数组（有符号）
      
      // 解析7个16位力矩值并应用安全限制
      for (int i = 0; i < 7; ++i) 
      {
        uint16_t mag = (uint16_t)payload[2*i] | ((uint16_t)payload[2*i + 1] << 8); 
        if (mag > 1000) mag = 1000;   // 钳位最大力矩为1000
        if (mag < HOLD_MAG) mag = HOLD_MAG;  // 确保最小保持力矩
        
        if (isHot((uint8_t)i)) {
          mag = u16_min(mag, HOT_TORQUE_LIMIT);  // 过热保护力矩限制
        }
        
        // 根据伸展和抓取位置确定力矩方向
        // 伸展位置 > 抓取位置：正力矩使舵机向伸展方向运动
        int grasp_sign = (sd[i].extend_count > sd[i].grasp_count) ? +1 : -1;
        torque_cmd[i] = (int16_t)(grasp_sign * (int)mag);  // 计算有符号力矩
      } 
      
      // 保存力矩命令用于软限位检查
      for (int i = 0; i < 7; ++i) {
        g_lastTorqueCmd[i] = torque_cmd[i];
      }

      // 执行力矩控制命令
      if (gBusMux) xSemaphoreTake(gBusMux, portMAX_DELAY);  // 获取总线访问权

      // 如果当前不是力矩模式，切换到力矩模式
      if (g_currentMode != MODE_TORQUE) 
      { 
        for (int i = 0; i < 7; ++i) 
        { 
          uint8_t id = SERVO_IDS[i]; 
          hlscl.EleMode(id);  // 设置舵机为力矩控制模式
        } 
        g_currentMode = MODE_TORQUE;  // 更新全局模式状态
      } 
      
      // 逐个写入力矩命令到每个舵机
      for (int i = 0; i < 7; ++i) 
      { 
        uint8_t id = SERVO_IDS[i]; 
        hlscl.WriteEle(id, torque_cmd[i]);  // 写入力矩命令
      } 
      
      if (gBusMux) xSemaphoreGive(gBusMux);  // 释放总线访问权
      return true; 
    }

    case CAPTURE: {
      // 自定义抓取命令：控制舵机是否抓取物体
      // 将抓取拆分为三个动作，分别为：1. 手指移动到抓取位置 2. 施加力矩抓取 3. 松开手指
      // 抓取命令拆解，基于buf[0]字节控制，0x01=移动，0x02=抓取，0x03=松开
      switch (buf[0]) {
        case 0x00:  // 初始位置
        {
          int16_t pos[7];  // 目标位置数组（原始12位值）
          pos[0] = mapU16ToRaw(0, 0x0000);
          pos[1] = mapU16ToRaw(1, 0x0000);
          pos[2] = mapU16ToRaw(2, 0x0000);
          pos[3] = mapU16ToRaw(3, 0x0000);
          pos[4] = mapU16ToRaw(4, 0x0000);
          pos[5] = mapU16ToRaw(5, 0x0000);
          pos[6] = mapU16ToRaw(6, 0x0000);

          // 力矩限制处理：如果电机过热，限制最大力矩
          uint16_t torque_eff[7];  // 有效力矩限制数组
          for (int i = 0; i < 7; ++i) {
            uint16_t base = g_torque[i]; // 基础力矩限制 (0-1023)
            if (isHot((uint8_t)i)) {     // 检查是否过热
              base = u16_min(base, HOT_TORQUE_LIMIT);  // 应用过热保护限制
            }
            torque_eff[i] = base;  // 设置有效力矩限制
          }

          // 执行位置控制命令
          if (gBusMux) xSemaphoreTake(gBusMux, portMAX_DELAY);  // 获取总线访问权
          
          // 如果当前不是位置模式，切换到位置模式
          if (g_currentMode != MODE_POS) {
            for (int i = 0; i < 7; ++i) {
              uint8_t id = SERVO_IDS[i];
              hlscl.ServoMode(id);  // 设置舵机为位置控制模式
            }
            g_currentMode = MODE_POS;  // 更新全局模式状态
          }
          
          // 同步写入所有舵机的位置、速度、加速度和力矩限制
          hlscl.SyncWritePosEx((uint8_t*)SERVO_IDS, 7, pos, g_speed, g_accel, torque_eff);
          
          if (gBusMux) xSemaphoreGive(gBusMux);  // 释放总线访问权
          control_falg = true;
          return true;
        }

        case 0x01:  // 移动到抓取位置
        {
          int16_t pos[7];  // 目标位置数组（原始12位值）
          pos[0] = mapU16ToRaw(0, 0xffff);
          pos[1] = mapU16ToRaw(1, 0x0000);
          pos[2] = mapU16ToRaw(2, 0x0000);
          pos[3] = mapU16ToRaw(3, 0x0000);
          pos[4] = mapU16ToRaw(4, 0x0000);
          pos[5] = mapU16ToRaw(5, 0x0000);
          pos[6] = mapU16ToRaw(6, 0x0000);


          // 力矩限制处理：如果电机过热，限制最大力矩
          uint16_t torque_eff[7];  // 有效力矩限制数组
          for (int i = 0; i < 7; ++i) {
            uint16_t base = g_torque[i]; // 基础力矩限制 (0-1023)
            if (isHot((uint8_t)i)) {     // 检查是否过热
              base = u16_min(base, HOT_TORQUE_LIMIT);  // 应用过热保护限制
            }
            torque_eff[i] = base;  // 设置有效力矩限制
          }

          // 执行位置控制命令
          if (gBusMux) xSemaphoreTake(gBusMux, portMAX_DELAY);  // 获取总线访问权
          
          // 如果当前不是位置模式，切换到位置模式
          if (g_currentMode != MODE_POS) {
            for (int i = 0; i < 7; ++i) {
              uint8_t id = SERVO_IDS[i];
              hlscl.ServoMode(id);  // 设置舵机为位置控制模式
            }
            g_currentMode = MODE_POS;  // 更新全局模式状态
          }
          
          // 同步写入所有舵机的位置、速度、加速度和力矩限制
          hlscl.SyncWritePosEx((uint8_t*)SERVO_IDS, 7, pos, g_speed, g_accel, torque_eff);
          
          if (gBusMux) xSemaphoreGive(gBusMux);  // 释放总线访问权
          control_falg = true;
          return true;
        }
          
  
        case 0x02:  // 抓取物体
        {
          // 力矩控制模式：接收7个16位力矩值，控制所有舵机输出力矩
          int16_t torque_cmd[7];  // 力矩命令数组（有符号）
          uint8_t buffer[15]={0x00,0xff,0x00,0x00,0x00,0xff,0x00,0xff,0x00,0xff,0x00,0xff,0x00,0xff,0x00};
          
          // 解析7个16位力矩值并应用安全限制
          for (int i = 0; i < 7; ++i) 
          {
            uint16_t mag = (uint16_t)buffer[2*i + 1] | ((uint16_t)buffer[2*i + 2] << 8); 
            if (mag > 1000) mag = 1000;   // 钳位最大力矩为1000
            if (mag < HOLD_MAG) mag = HOLD_MAG;  // 确保最小保持力矩
            
            if (isHot((uint8_t)i)) {
              mag = u16_min(mag, HOT_TORQUE_LIMIT);  // 过热保护力矩限制
            }
            
            // 根据伸展和抓取位置确定力矩方向
            // 伸展位置 > 抓取位置：正力矩使舵机向伸展方向运动
            int grasp_sign = (sd[i].extend_count > sd[i].grasp_count) ? +1 : -1;
            torque_cmd[i] = (int16_t)(grasp_sign * (int)mag);  // 计算有符号力矩
          } 
          
          // 保存力矩命令用于软限位检查
          for (int i = 0; i < 7; ++i) {
            g_lastTorqueCmd[i] = torque_cmd[i];
          }

          // 执行力矩控制命令
          if (gBusMux) xSemaphoreTake(gBusMux, portMAX_DELAY);  // 获取总线访问权

          // 如果当前不是力矩模式，切换到力矩模式
          if (g_currentMode != MODE_TORQUE) 
          { 
            for (int i = 0; i < 7; ++i) 
            { 
              uint8_t id = SERVO_IDS[i]; 
              hlscl.EleMode(id);  // 设置舵机为力矩控制模式
            } 
            g_currentMode = MODE_TORQUE;  // 更新全局模式状态
          } 

          // 执行力矩控制命令

          // 逐个写入力矩命令到每个舵机
          for (int i = 0; i < 7; ++i) 
          { 
            uint8_t id = SERVO_IDS[i]; 
            hlscl.WriteEle(id, torque_cmd[i]);  // 写入力矩命令
          } 
          
          if (gBusMux) xSemaphoreGive(gBusMux);  // 释放总线访问权
          control_falg = true;
          return true; 
        }
          
        case 0x03:  // 松开手指
        {
          int16_t pos[7];  // 目标位置数组（原始12位值）
          pos[0] = mapU16ToRaw(0, 0xffff);
          pos[1] = mapU16ToRaw(1, 0x0000);
          pos[2] = mapU16ToRaw(2, 0x0000);
          pos[3] = mapU16ToRaw(3, 0x0000);
          pos[4] = mapU16ToRaw(4, 0x0000);
          pos[5] = mapU16ToRaw(5, 0x0000);
          pos[6] = mapU16ToRaw(6, 0x0000);


          // 力矩限制处理：如果电机过热，限制最大力矩
          uint16_t torque_eff[7];  // 有效力矩限制数组
          for (int i = 0; i < 7; ++i) {
            uint16_t base = g_torque[i]; // 基础力矩限制 (0-1023)
            if (isHot((uint8_t)i)) {     // 检查是否过热
              base = u16_min(base, HOT_TORQUE_LIMIT);  // 应用过热保护限制
            }
            torque_eff[i] = base;  // 设置有效力矩限制
          }

          // 执行位置控制命令
          if (gBusMux) xSemaphoreTake(gBusMux, portMAX_DELAY);  // 获取总线访问权
          
          // 如果当前不是位置模式，切换到位置模式
          if (g_currentMode != MODE_POS) {
            for (int i = 0; i < 7; ++i) {
              uint8_t id = SERVO_IDS[i];
              hlscl.ServoMode(id);  // 设置舵机为位置控制模式
            }
            g_currentMode = MODE_POS;  // 更新全局模式状态
          }
          
          // 同步写入所有舵机的位置、速度、加速度和力矩限制
          hlscl.SyncWritePosEx((uint8_t*)SERVO_IDS, 7, pos, g_speed, g_accel, torque_eff);
          
          if (gBusMux) xSemaphoreGive(gBusMux);  // 释放总线访问权
          control_falg = true;
          return true;
        }

        case 0x04:  //抓试管准备动作
        {
          int16_t pos[7];  // 目标位置数组（原始12位值）
          pos[0] = mapU16ToRaw(0, 0xffff);
          pos[1] = mapU16ToRaw(1, 0x6600);
          pos[2] = mapU16ToRaw(2, 0x3300);
          pos[3] = mapU16ToRaw(3, 0x5500);
          pos[4] = mapU16ToRaw(4, 0x6600);
          pos[5] = mapU16ToRaw(5, 0x7700);
          pos[6] = mapU16ToRaw(6, 0xffff);


          // 力矩限制处理：如果电机过热，限制最大力矩
          uint16_t torque_eff[7];  // 有效力矩限制数组
          for (int i = 0; i < 7; ++i) {
            uint16_t base = g_torque[i]; // 基础力矩限制 (0-1023)
            if (isHot((uint8_t)i)) {     // 检查是否过热
              base = u16_min(base, HOT_TORQUE_LIMIT);  // 应用过热保护限制
            }
            torque_eff[i] = base;  // 设置有效力矩限制
          }

          // 执行位置控制命令
          if (gBusMux) xSemaphoreTake(gBusMux, portMAX_DELAY);  // 获取总线访问权
          
          // 如果当前不是位置模式，切换到位置模式
          if (g_currentMode != MODE_POS) {
            for (int i = 0; i < 7; ++i) {
              uint8_t id = SERVO_IDS[i];
              hlscl.ServoMode(id);  // 设置舵机为位置控制模式
            }
            g_currentMode = MODE_POS;  // 更新全局模式状态
          }
          
          // 同步写入所有舵机的位置、速度、加速度和力矩限制
          hlscl.SyncWritePosEx((uint8_t*)SERVO_IDS, 7, pos, g_speed, g_accel, torque_eff);
          
          if (gBusMux) xSemaphoreGive(gBusMux);  // 释放总线访问权
          control_falg = true;
          return true;
        }

        case 0x05:  //抓试管动作
        {
          int16_t pos[7];  // 目标位置数组（原始12位值）
          pos[0] = mapU16ToRaw(0, 0xffff);
          pos[1] = mapU16ToRaw(1, 0x9900);
          pos[2] = mapU16ToRaw(2, 0x3300);
          pos[3] = mapU16ToRaw(3, 0x7000);
          pos[4] = mapU16ToRaw(4, 0x7700);
          pos[5] = mapU16ToRaw(5, 0x8000);
          pos[6] = mapU16ToRaw(6, 0xffff);


          // 力矩限制处理：如果电机过热，限制最大力矩
          uint16_t torque_eff[7];  // 有效力矩限制数组
          for (int i = 0; i < 7; ++i) {
            uint16_t base = g_torque[i]; // 基础力矩限制 (0-1023)
            if (isHot((uint8_t)i)) {     // 检查是否过热
              base = u16_min(base, HOT_TORQUE_LIMIT);  // 应用过热保护限制
            }
            torque_eff[i] = base;  // 设置有效力矩限制
          }

          // 执行位置控制命令
          if (gBusMux) xSemaphoreTake(gBusMux, portMAX_DELAY);  // 获取总线访问权
          
          // 如果当前不是位置模式，切换到位置模式
          if (g_currentMode != MODE_POS) {
            for (int i = 0; i < 7; ++i) {
              uint8_t id = SERVO_IDS[i];
              hlscl.ServoMode(id);  // 设置舵机为位置控制模式
            }
            g_currentMode = MODE_POS;  // 更新全局模式状态
          }
          
          // 同步写入所有舵机的位置、速度、加速度和力矩限制
          hlscl.SyncWritePosEx((uint8_t*)SERVO_IDS, 7, pos, g_speed, g_accel, torque_eff);
          
          if (gBusMux) xSemaphoreGive(gBusMux);  // 释放总线访问权
          control_falg = true;
          return true;
        }

        case 0x06:  //OK动作
        {
          int16_t pos[7];  // 目标位置数组（原始12位值）
          pos[0] = mapU16ToRaw(0, 0xdd00);
          pos[1] = mapU16ToRaw(1, 0x0000);
          pos[2] = mapU16ToRaw(2, 0x9900);
          pos[3] = mapU16ToRaw(3, 0xaa00);
          pos[4] = mapU16ToRaw(4, 0x0000);
          pos[5] = mapU16ToRaw(5, 0x0000);
          pos[6] = mapU16ToRaw(6, 0x0000);


          // 力矩限制处理：如果电机过热，限制最大力矩
          uint16_t torque_eff[7];  // 有效力矩限制数组
          for (int i = 0; i < 7; ++i) {
            uint16_t base = g_torque[i]; // 基础力矩限制 (0-1023)
            if (isHot((uint8_t)i)) {     // 检查是否过热
              base = u16_min(base, HOT_TORQUE_LIMIT);  // 应用过热保护限制
            }
            torque_eff[i] = base;  // 设置有效力矩限制
          }

          // 执行位置控制命令
          if (gBusMux) xSemaphoreTake(gBusMux, portMAX_DELAY);  // 获取总线访问权
          
          // 如果当前不是位置模式，切换到位置模式
          if (g_currentMode != MODE_POS) {
            for (int i = 0; i < 7; ++i) {
              uint8_t id = SERVO_IDS[i];
              hlscl.ServoMode(id);  // 设置舵机为位置控制模式
            }
            g_currentMode = MODE_POS;  // 更新全局模式状态
          }
          
          // 同步写入所有舵机的位置、速度、加速度和力矩限制
          hlscl.SyncWritePosEx((uint8_t*)SERVO_IDS, 7, pos, g_speed, g_accel, torque_eff);
          
          if (gBusMux) xSemaphoreGive(gBusMux);  // 释放总线访问权
          control_falg = true;
          return true;
        }

        default:
        {
          // 未知命令，忽略
          control_falg = true;
          return true;
        }
      }
    }


    case TEST: {
      // 测试命令：用于调试和验证响应
      Serial.println("hand test\n");
      return true;
    }
    case SET_TOR: {
      // 设置力矩参数命令：配置单个舵机的力矩限制
      return handleSetTorCmd(payload);
    }

    case SET_SPE: {
      // 设置速度参数命令：配置单个舵机的运动速度
      return handleSetSpeedCmd(payload);
    }

    case HOMING: {
      // 归零命令：执行自动归零校准
      HOMING_start();           // 启动归零过程（阻塞式）
      saveExtendsToNVS();       // 保存归零后的伸展位置
      sendAckFrame(HOMING, nullptr, 0);  // 发送归零完成确认
      return true;
    }

    case SET_ID: {
      // 设置舵机ID命令：修改舵机总线地址
      return handleSetIdCmd(payload);
    }

    case TRIM: {
      // 微调命令：调整舵机伸展位置
      return handleTrimCmd(payload);
    }

    case GET_POS: {
      // 获取位置数据命令：返回所有舵机当前位置
      sendPositions();
      return true;
    }

    case GET_VEL: {
      // 获取速度数据命令：返回所有舵机当前速度
      sendVelocities();
      return true;
    }

    case GET_TEMP: {
      // 获取温度数据命令：返回所有舵机当前温度
      sendTemps();
      return true;
    }

    case GET_CURR: {
      // 获取电流数据命令：返回所有舵机当前电流
      sendCurrents();
      return true;
    }

    default:
      // 未知控制码 — 消耗帧以保持协议对齐
      return true;
  }
}

// ========== MQTT功能实现 ==========
/**
 * @brief MQTT消息回调函数
 * @param topic 消息主题
 * @param payload 消息内容
 * @param length 消息长度
 */
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // 将payload转换为字符串
  char message[length + 1];
  memcpy(message, payload, length);
  message[length] = '\0';
  
  Serial.print("MQTT Message received: ");
  Serial.print(topic);
  Serial.print(" - ");
  Serial.println(message);
  
  // 处理控制命令
  if (strcmp(topic, topic_command) == 0) {
    handleMqttCommand(message);
  }
}
/**
 * @brief 连接MQTT服务器
 * @return true-连接成功，false-连接失败
 */
bool mqttConnect() {
  Serial.print("Connecting to MQTT server: : ");
  Serial.print(mqtt_server);
  Serial.print(":");
  Serial.println(mqtt_port);
  
  if (mqttClient.connect(mqtt_client_id, "123", "123")) {
    // 订阅控制命令主题
    if (mqttClient.subscribe(topic_command)) {
      Serial.print("Subscribed to: ");
      Serial.println(topic_command);
    }
    
    // 发布连接成功消息
    // mqttClient.publish(topic_push, "Hand Connected");
    // mqtt_connected = true;
    return true;
  } else {
    Serial.print("MQTT connection failed, rc=");
    Serial.println(mqttClient.state());
    return false;
  }
}
/**
 * @brief MQTT重连管理
 */
void mqttReconnect() {
  uint32_t current_time = millis();
  
  // 每5秒尝试重连一次
  if (current_time - last_mqtt_reconnect > 5000) {
    last_mqtt_reconnect = current_time;
    
    if (mqttConnect()) {
      Serial.println("MQTT Reconnected");
    } else {
      Serial.println("MQTT Reconnect Failed");
    }
  }
}
/**
 * @brief 处理MQTT控制命令
 * @param command 命令内容
 */
void handleMqttCommand(const char* command) {
  Serial.print("Processing MQTT command: ");
  Serial.println(command);
  
  // 解析命令
  if (strstr(command, "beaker_grab") != NULL) 
  {
    // 抓取命令
    buf[0] = 0x02; // 抓取动作标志
    control_falg = false;
    handleHostFrame(CAPTURE);
  } 
  else if (strstr(command, "beaker_setout") != NULL) 
  {
    // 准备命令
    buf[0] = 0x01; // 准备动作标志
    control_falg = false;
    handleHostFrame(CAPTURE);
  } 
  else if (strstr(command, "beaker_release") != NULL) 
  {
    // 释放命令
    buf[0] = 0x03; // 释放动作标志
    control_falg = false;
    handleHostFrame(CAPTURE);
  } 
  else if (strstr(command, "reset") != NULL) 
  {
    // 复位命令
    buf[0] = 0x00; // 复位动作标志
    control_falg = false;
    handleHostFrame(CAPTURE);
  } 
  else if (strstr(command, "tube_setout") != NULL) 
  {
    // 准备命令
    buf[0] = 0x04; 
    control_falg = false;
    handleHostFrame(CAPTURE);
  } 
  else if (strstr(command, "tube_grab") != NULL) 
  {
    // 抓取命令
    buf[0] = 0x05; 
    control_falg = false;
    handleHostFrame(CAPTURE);
  } 
  else if (strstr(command, "tube_release") != NULL) 
  {
    // 释放命令
    buf[0] = 0x04; 
    control_falg = false;
    handleHostFrame(CAPTURE);
  } 
  else if (strstr(command, "ok") != NULL) 
  {
    // 释放命令
    buf[0] = 0x06; 
    control_falg = false;
    handleHostFrame(CAPTURE);
  }
  else if (strstr(command, "feedback") != NULL) 
  {
    mqttClient.publish(topic_push, "{\"hand\": \"hand is OK\"}");
  } 

  else if (strstr(command, "test") != NULL) 
  {
    // 测试命令
    Serial.println("MQTT Test OK!");

  } 
  else
  {
    
  }
}

/**
 * @brief 系统初始化函数
 * @details Arduino框架的标准初始化函数，在程序启动时执行一次
 *          完成硬件初始化、参数加载、自动归零等启动流程
 */
void setup() {
  // USB调试串口初始化
  Serial.begin(921600);  // 高速921600波特率，用于调试输出
  delay(100);             // 等待串口稳定

  // 舵机总线UART初始化 @ 1 Mbps
  Serial2.begin(1000000, SERIAL_8N1, SERIAL2_RX_PIN, SERIAL2_TX_PIN);
  hlscl.pSerial = &Serial2;  // 设置舵机控制库的串口对象

  // 初始化舵机数据到基线配置
  resetSdToBaseline();        // 根据左右手配置重置舵机数据
  prefs.begin("hand", false); // 打开NVS存储命名空间
  loadManualExtendsFromNVS(); // 加载手动调整的伸展位置
  
  // 输出手部类型信息
  #if defined(LEFT_HAND)
    Serial.println("[BOOT] Hand Type: LEFT_HAND");
  #elif defined(RIGHT_HAND)
    Serial.println("[BOOT] Hand Type: RIGHT_HAND");
  #else
    Serial.println("[BOOT] Hand Type: UNKNOWN");
  #endif
  
  // 执行自动归零校准
  Serial.println("[BOOT] Auto Homing…");
  HOMING_start();             // 执行归零操作，校准所有舵机
  saveExtendsToNVS();         // 保存归零后的伸展位置到NVS
  Serial.println("[BOOT] Homing Complete");
  
  // ---- 启动时舵机连接检查 -----（逐个检查连接）
  Serial.println("\n[Init] Pinging servos...");
  for (uint8_t i = 0; i < 7; ++i) {
    uint8_t id = SERVO_IDS[i];
    int resp = hlscl.Ping(id);
    if (!hlscl.getLastError()) {
      Serial.print("  ID "); Serial.print(id); Serial.println(": OK");
    } else {
      Serial.print("  ID "); Serial.print(id); Serial.println(": NO REPLY");
    }
  }
  //Syncreadbegin to Start the syncread
  hlscl.syncReadBegin(sizeof(SERVO_IDS), REG_BLOCK_LEN, /*rx_fix*/ 8);

  for (int i = 0; i < 7; ++i){
  Serial.printf("Servo %d dir=%d\n", i, sd[i].servo_direction);}
  //Initialisation of Mutex and Task serial pinned to Core 1
  gBusMux =xSemaphoreCreateMutex();
  gMetricsMux = xSemaphoreCreateMutex();
  //创建同步读取任务，在1核上运行
  xTaskCreatePinnedToCore(TaskSyncRead_Core1, "SyncRead", 4096, NULL, 1, NULL, 1); // run on Core1

  WiFi.begin(ssid, password);
  Serial.println("\nConnecting to WiFi");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(50);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected");

  // MQTT客户端初始化
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);
  // 尝试连接MQTT服务器
  if (mqttConnect()) 
  {
    Serial.println("MQTT Connected");
  } 
  else 
  {
    Serial.println("MQTT Connection Failed");
  }
}

/**
 * @brief 主循环函数
 * @details Arduino框架的主循环，持续运行处理各种任务
 *          包括命令处理、安全检查、状态监控等
 */
void loop() {

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nWiFi Disconnected");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
      delay(500);
      Serial.print(".");
    }
    Serial.println("\nWiFi Reconnected");
  }

  // MQTT连接管理
  if (!mqttClient.connected()) {
    mqttReconnect();
  } else {
    mqttClient.loop(); // 处理MQTT消息
  }

  static uint32_t last_cmd_ms = 0; // 记录上次命令处理时间

  // 检查归零过程是否正在进行（如果正在归零，则清空串口缓冲区并等待）
  if (HOMING_isBusy()) {
    while (Serial.available()) { Serial.read(); } // 清空串口缓冲区，避免干扰归零过程
    vTaskDelay(pdMS_TO_TICKS(5)); // 短暂延迟5ms
    return; // 跳过本次循环的其他处理
  }

  // ------ 软限位检查 -------（在力矩控制模式下检查并强制软限位）
  checkAndEnforceSoftLimits();

  // 当有完整的16字节命令帧可用时进行处理（在可用时处理数据）
  if (Serial.available() >= 16) {
    int op = Serial.read(); // 读取操作码（命令类型）
    if (op >= 0) {
      if (handleHostFrame((uint8_t)op)) { // 处理命令帧
        last_cmd_ms = millis(); // 更新上次命令处理时间
        return; // 处理完成后返回
      }
    }
  }
}