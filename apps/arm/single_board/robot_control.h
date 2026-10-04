#ifndef _ARM_ROBOT_CONTROL_H_
#define _ARM_ROBOT_CONTROL_H_

#include "arm_def.h"
#include <stdint.h>

typedef struct
{
    uint8_t  valid;          /**< 反馈有效标志：1 表示已收到可用反馈或设备在线，0 表示无有效反馈。 */
    uint8_t  online;         /**< 离线检测状态：1 表示 Module_Offline 判定在线，0 表示超时离线。 */
    uint8_t  enabled;        /**< 输出使能状态：1 表示控制器允许输出，0 表示当前保持失能。 */
    uint8_t  feedback_id;    /**< 电机反馈帧中的 ID；达妙电机为电机 ID，GM6020 当前填 0。 */
    uint32_t tx_id;          /**< 本机发送到该电机的 CAN 标识符。 */
    uint32_t rx_id;          /**< 该电机反馈到本机的 CAN 标识符。 */
    uint16_t raw_position;   /**< 原始位置值；达妙为反馈报文 16 位位置，GM6020 为 0~8191 编码器值。 */
    int16_t  raw_speed;      /**< 原始速度值；GM6020 为转速反馈，达妙当前保留为 0。 */
    int16_t  raw_current;    /**< 原始电流/力矩电流值；GM6020 使用实际电流，达妙当前保留为 0。 */

    float angle_raw_rad;     /**< 单圈角度，单位 rad；达妙为电机单圈角度，GM6020 为编码器折算角度。 */
    float angle_rad;         /**< 累计角度，单位 rad；由电机驱动层维护。 */
    float speed_rad_s;       /**< 角速度，单位 rad/s。 */
    float torque_nm;         /**< 输出力矩估算值，单位 N·m。 */
    float joint_angle_rad;   /**< 软件零点标定后的机械关节角，单位 rad；竖直零点附近为 0。 */
    float joint_speed_rad_s; /**< 按关节方向修正后的机械关节角速度，单位 rad/s。 */
    float model_angle_rad;   /**< 对齐老工程重力补偿公式的模型角，单位 rad。 */
    float model_speed_rad_s; /**< 对齐老工程重力补偿公式的模型角速度，单位 rad/s。 */

    uint8_t temperature_1;   /**< 温度通道 1；达妙为 MOS 温度，GM6020 为电机温度。 */
    uint8_t temperature_2;   /**< 温度通道 2；达妙为转子温度，GM6020 当前填 0。 */
} Arm_Motor_Feedback_t;

typedef struct
{
    Arm_Motor_Feedback_t joint[ARM_MOTOR_COUNT];
} Arm_Feedback_Snapshot_t;

/**
 * @brief 机械臂电机反馈快照全局变量。
 *
 * @note 写入方：APP 层 arm_feedback_task() 周期调用 arm_feedback_snapshot_update() 更新。
 * @note 读取方：调试器 Watch 窗口或 APP 层业务逻辑可直接查看；跨线程读取时建议只读不写。
 */
extern volatile Arm_Feedback_Snapshot_t g_arm_feedback_snapshot;

/**
 * @brief 刷新机械臂全部电机反馈快照。
 *
 * @param 无。
 *
 * @retval 无。
 *
 * @note 调用关系：由 arm_feedback_task() 1tick 周期调用；也可由 APP 层调试代码手动调用。
 */
void arm_feedback_snapshot_update(void);

/**
 * @brief 获取机械臂电机反馈快照只读指针。
 *
 * @param 无。
 *
 * @return const volatile Arm_Feedback_Snapshot_t* 指向全局反馈快照的只读 volatile 指针。
 *
 * @note 调用关系：供 APP 层调试、限位标定或状态机读取；本函数不更新数据。
 */
const volatile Arm_Feedback_Snapshot_t *arm_feedback_snapshot_get(void);

/**
 * @brief 初始化机械臂 APP 控制入口。
 *
 * @param 无。
 *
 * @retval 无。
 *
 * @note 调用关系：由机器人启动流程调用；内部完成 CAN 任务初始化、电机注册、反馈线程和日志线程创建。
 */
void robot_control_init(void);

#endif /* _ARM_ROBOT_CONTROL_H_ */
