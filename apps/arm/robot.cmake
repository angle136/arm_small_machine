# 机械臂当前启用离线检测、电机通信和遥控器数据观测。
# SBUS 数组 [7] 只作为重力补偿安全门，不映射机械臂运动命令。

include(${CMAKE_CURRENT_LIST_DIR}/../../modules/module_config.cmake)

set(MODULES_SINGLE OFFLINE REMOTE MOTOR LORA)
set(MODULES_GIMBAL  OFFLINE MOTOR)
set(MODULES_CHASSIS OFFLINE MOTOR)

set(OFFLINE_BEEP_ENABLE 1)
set(MOTOR_OFFLINE_ENABLE 1)
set(ARM_GRAVITY_REMOTE_GUARD_ENABLE 1) # 1=启用 SBUS[7] 安全门，0=跳过该安全门
set(ARM_SOFTWARE_LIMIT_ENABLE 0)       # 1=启用软件角度/raw限位及其失能，0=完全关闭软件限位

# SBUS 接收机：USART3，25 字节标准 SBUS 数据；不启用图传遥控源。
set(REMOTE_UART           huart3)
set(REMOTE_SOURCE         1)
set(REMOTE_VT_SOURCE      0)

# 塔石 L33 发送端：USART6=PG14/PG9，M0/M1 复用空闲 TIM1 引脚 PE13/PE9。
set(LORA_UART              huart6)
set(LORA_M0_GPIO_PORT      GPIOE)
set(LORA_M0_GPIO_PIN       GPIO_PIN_13)
set(LORA_M1_GPIO_PORT      GPIOE)
set(LORA_M1_GPIO_PIN       GPIO_PIN_9)
set(LORA_AUX_ENABLE        0)
set(LORA_TX_ENABLE         1)
set(LORA_RX_ENABLE         0)
set(LORA_TX_INTERVAL_MS    50)
set(LORA_OFFLINE_ENABLE    0)
