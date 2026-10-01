# 机械臂第一阶段：只启用离线检测和电机通信。
# IMU、INS、遥控器、运动学和动力学在完成逐台电机在线确认前不接入。

include(${CMAKE_CURRENT_LIST_DIR}/../../modules/module_config.cmake)

set(MODULES_SINGLE OFFLINE MOTOR)
set(MODULES_GIMBAL  OFFLINE MOTOR)
set(MODULES_CHASSIS OFFLINE MOTOR)

set(OFFLINE_BEEP_ENABLE 1)
set(MOTOR_OFFLINE_ENABLE 1)
