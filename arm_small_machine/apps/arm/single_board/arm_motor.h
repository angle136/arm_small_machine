#ifndef _ARM_MOTOR_H_
#define _ARM_MOTOR_H_

#include "arm_def.h"
#include <stdint.h>

typedef struct
{
    uint8_t  valid;          /**< 已收到可用反馈。 */
    uint8_t  online;         /**< 离线检测判定在线。 */
    uint8_t  enabled;        /**< 电机软件输出使能。 */
    uint8_t  feedback_id;    /**< 达妙反馈 ID；GM6020 当前填 0。 */
    uint32_t tx_id;          /**< 本机发送 CAN ID。 */
    uint32_t rx_id;          /**< 电机反馈 CAN ID。 */
    uint16_t raw_position;   /**< 达妙 16 位位置或 GM6020 编码器值。 */
    int16_t  raw_speed;      /**< GM6020 原始转速；达妙当前填 0。 */
    int16_t  raw_current;    /**< GM6020 原始电流；达妙当前填 0。 */
    float    angle_raw_rad;  /**< 驱动层单圈角度，单位 rad。 */
    float    angle_rad;      /**< 驱动层累计角度，单位 rad。 */
    float    speed_rad_s;    /**< 驱动层角速度，单位 rad/s。 */
    float    torque_nm;      /**< 驱动层力矩反馈/估算，单位 N·m。 */
    float    joint_angle_rad;   /**< 软件零点和方向修正后的关节角，单位 rad。 */
    float    joint_speed_rad_s; /**< 机械关节坐标系角速度，单位 rad/s。 */
    float    model_angle_rad;   /**< 重力补偿模型角，单位 rad。 */
    float    model_speed_rad_s; /**< 重力补偿模型角速度，单位 rad/s。 */
    uint8_t  temperature_1;     /**< 达妙 MOS 温度或 GM6020 电机温度。 */
    uint8_t  temperature_2;     /**< 达妙转子温度；GM6020 当前填 0。 */
} Arm_Motor_Feedback_t;

typedef struct
{
    Arm_Motor_Feedback_t joint[ARM_MOTOR_COUNT];
} Arm_Feedback_Snapshot_t;

/** Ozone 观察用的 J1~J7 电机反馈快照。 */
extern volatile Arm_Feedback_Snapshot_t g_arm_feedback_snapshot;

/**
 * @brief 注册 J1~J7 电机并保持全部输出失能。
 *
 * @param 无。
 *
 * @retval 无。
 *
 * @note 调用关系：由 robot_control_init() 在 BSP_CAN_TaskInit() 之后调用；内部注册
 *       6 个达妙电机和 J4 GM6020，并初始化反馈快照。
 * @note 安全说明：注册完成后对所有电机执行 Motor_Stop()，不主动使能输出。
 */
void arm_motor_init(void);

/**
 * @brief 刷新 J1~J7 电机反馈快照。
 *
 * @param 无。
 *
 * @retval 无。
 *
 * @note 调用关系：由 arm_control_task() 每个 ThreadX tick 调用；只读取驱动缓存和离线状态，
 *       不发送 CAN 控制命令。
 */
void arm_motor_feedback_update(void);

/**
 * @brief 获取机械臂电机反馈快照只读指针。
 *
 * @param 无。
 *
 * @return const volatile Arm_Feedback_Snapshot_t* 全局反馈快照地址。
 *
 * @note 调用关系：由重力补偿和调试逻辑读取；函数不更新数据、不修改输出。
 */
const volatile Arm_Feedback_Snapshot_t *arm_motor_feedback_get(void);

/**
 * @brief 检查 J1~J7 是否全部在线且反馈有效。
 *
 * @param 无。
 *
 * @return uint8_t 全部有效返回 1，否则返回 0。
 *
 * @note 调用关系：由 arm_gravity_update() 在输出前调用；任一关节失效都会阻止整臂输出。
 */
uint8_t arm_motor_all_feedback_ready(void);

/**
 * @brief 应用 J1~J7 的输出使能和力矩命令。
 *
 * @param torque_nm 输入，长度为 ARM_MOTOR_COUNT 的关节力矩数组，单位 N·m。
 * @param enable    输入，长度为 ARM_MOTOR_COUNT 的关节使能数组，非 0 表示允许输出。
 *
 * @retval 无。
 *
 * @note 调用关系：只由 arm_gravity_update() 在遥控安全门和反馈检查通过后调用。
 * @note 安全说明：每个力矩仍经过 Motor_SetOutputTorque() 底层限幅；enable=0 的关节清零并失能。
 */
void arm_motor_apply_outputs(const float torque_nm[ARM_MOTOR_COUNT], const uint8_t enable[ARM_MOTOR_COUNT]);

/**
 * @brief 清零并失能 J1~J7 全部电机。
 *
 * @param 无。
 *
 * @retval 无。
 *
 * @note 调用关系：由初始化、遥控器关闭、遥控器离线或反馈异常路径调用，可每 tick 重复调用。
 * @note 安全说明：先清零待发送力矩，再清除软件使能，确保后续 Motor_ApplyAll() 只走安全输出路径。
 */
void arm_motor_stop_all(void);

/**
 * @brief 打印 J1~J7 在线状态和核心反馈。
 *
 * @param 无。
 *
 * @retval 无。
 *
 * @note 调用关系：由 arm_status_task() 每 1000 tick 调用；只读状态，不改变控制输出。
 */
void arm_motor_log_status(void);

#endif /* _ARM_MOTOR_H_ */
