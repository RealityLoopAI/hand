// HandConfig.h - 手部配置头文件
#pragma once  // 防止头文件重复包含

// 通过构建标志或取消注释下面的行来选择一种配置：
//   - PlatformIO:   在platformio.ini中添加 build_flags = -DRIGHT_HAND (或 -DLEFT_HAND)
//   - Arduino CLI:  使用 --build-property compiler.cpp.extra_flags="-DRIGHT_HAND"
//   - Arduino IDE:  直接取消注释下面的一行

#define LEFT_HAND        // 启用左手配置（当前激活）
// #define RIGHT_HAND     // 启用右手配置（注释状态）

// ----------------------- 配置验证 -----------------------
// 检查是否定义了至少一个手部配置
#if !defined(LEFT_HAND) && !defined(RIGHT_HAND)
  // 编译时错误：必须正确定义一个手部配置
  #error "必须正确定义一个手部配置: LEFT_HAND 或 RIGHT_HAND (使用构建标志或在HandConfig.h中取消注释)."
#endif

// 检查是否同时定义了左右手配置（冲突）
#if defined(LEFT_HAND) && defined(RIGHT_HAND)
  // 编译时错误：不能同时定义左右手配置
  #error "不能同时定义 LEFT_HAND 和 RIGHT_HAND 配置."
#endif