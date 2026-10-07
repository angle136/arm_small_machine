# 机械臂当前启用离线检测、电机通信和遥控器数据观测。
# SBUS 数组 [7] 只作为重力补偿安全门，不映射机械臂运动命令。

include(${CMAKE_CURRENT_LIST_DIR}/../../modules/module_config.cmake)

set(MODULES_SINGLE OFFLINE REMOTE MOTOR)
set(MODULES_GIMBAL  OFFLINE MOTOR)
set(MODULES_CHASSIS OFFLINE MOTOR)

set(OFFLINE_BEEP_ENABLE 1)
set(MOTOR_OFFLINE_ENABLE 1)
set(ARM_GRAVITY_REMOTE_GUARD_ENABLE 1) # 1=启用 SBUS[7] 安全门，0=跳过该安全门

# SBUS 接收机：USART3，25 字节标准 SBUS 数据；不启用图传遥控源。
set(REMOTE_UART           huart3)
set(REMOTE_SOURCE         1)
set(REMOTE_VT_SOURCE      0)
