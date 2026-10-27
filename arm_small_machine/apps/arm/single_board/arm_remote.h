#ifndef _ARM_REMOTE_H_
#define _ARM_REMOTE_H_

#include <stdint.h>

#define ARM_REMOTE_CHANNEL_COUNT         16U
#define ARM_REMOTE_GRIPPER_CHANNEL_INDEX 0U
#define ARM_REMOTE_GRIPPER_CENTER_RAW    1024
#define ARM_REMOTE_GRIPPER_DEADBAND_RAW  100

/* 数组 [6] 三档开关，预留给后续功能。 */
#define ARM_REMOTE_AUX_CHANNEL_INDEX     6U
#define ARM_REMOTE_AUX_UP_RAW            240
#define ARM_REMOTE_AUX_MIDDLE_RAW        1024
#define ARM_REMOTE_AUX_DOWN_RAW          1807

#define ARM_REMOTE_GRAVITY_CHANNEL_INDEX 7U
#define ARM_REMOTE_GRAVITY_ENABLE_RAW    1807
#define ARM_REMOTE_GRAVITY_DISABLE_RAW   240

/** 遥控器在线状态和重力补偿安全门的 APP 快照。 */
typedef struct
{
    uint8_t online;         /**< 1 表示 50 ms 内收到过有效 SBUS 帧。 */
    uint8_t gravity_enable; /**< 安全门开启时数组[7]=1807且在线为1；安全门关闭时固定为1。 */
    int16_t ch7_raw;        /**< g_arm_remote_channels[7] 原始 11 位值，供 Ozone 直接观察。 */
} Arm_Remote_Debug_t;

/** Ozone 观察数组：下标 0~15 对应 SBUS CH1~CH16；业务代码使用 arm_remote_channel_get()。 */
extern volatile int16_t g_arm_remote_channels[ARM_REMOTE_CHANNEL_COUNT];

/** Ozone 观察状态：包含在线、CH7 和安全门状态；业务代码使用对应查询接口。 */
extern volatile Arm_Remote_Debug_t g_arm_remote_debug;

/**
 * @brief 初始化机械臂 APP 层遥控器快照。
 *
 * @param 无。
 *
 * @retval 无。
 *
 * @note 调用关系：由 robot_control_init() 在创建控制线程前调用；REMOTE 模块本身已由
 *       MODULE_Init() 初始化。本函数只清零 APP 调试快照，不配置串口、不创建线程。
 * @note 安全说明：安全门开关为 1 时初始化后 gravity_enable 保持 0；只有在线 SBUS 且
 *       g_arm_remote_channels[7]=1807 才会开启。开关为 0 时直接允许原有重力补偿流程。
 */
void arm_remote_init(void);

/**
 * @brief 刷新 SBUS 通道快照和数组 [7] 重力补偿安全门。
 *
 * @param 无。
 *
 * @retval 无。
 *
 * @note 调用关系：由 arm_control_task() 每个 ThreadX tick 调用；数据源由 REMOTE 模块的
 *       Remote Ctrl 线程异步解析。安全门开启时遥控器离线或数组[7]非法会立即关闭。
 */
void arm_remote_update(void);

/**
 * @brief 查询数组 [7] 安全门是否允许重力补偿输出。
 *
 * @param 无。
 *
 * @return uint8_t 安全门开启时在线且数组[7]等于1807返回1；安全门关闭时始终返回1。
 *
 * @note 调用关系：由 arm_gravity_update() 在最终输出前调用；本函数只读取 APP 快照。
 * @note 安全说明：安全门开关为 1 时，数组 [7]=240、遥控器离线以及任何非 1807 值均按关闭处理；
 *       开关为 0 时不执行这些判断。
 */
uint8_t arm_remote_gravity_enabled(void);

/**
 * @brief 读取一个已还原为 SBUS 原始值的遥控通道。
 *
 * @param channel_index 输入，数组下标 0~15，对应 SBUS CH1~CH16。
 *
 * @return int16_t 通道原始值；索引越界时返回 0。
 *
 * @note 调用关系：由 robot_control 的夹爪命令转换逻辑调用；不刷新遥控数据。
 */
int16_t arm_remote_channel_get(uint8_t channel_index);

/**
 * @brief 查询遥控器最近一次刷新时是否在线。
 *
 * @param 无。
 *
 * @return uint8_t 在线返回 1，离线返回 0。
 *
 * @note 调用关系：由 robot_control 的夹爪安全逻辑调用；只读取 APP 快照。
 */
uint8_t arm_remote_is_online(void);

#endif /* _ARM_REMOTE_H_ */
