#ifndef _ARM_GRAVITY_H_
#define _ARM_GRAVITY_H_

#include <stdint.h>

typedef struct
{
    uint8_t remote_enable; /**< CH7 安全门状态。 */
    uint8_t feedback_ready; /**< J1~J7 反馈全部有效标志。 */
    uint8_t output_active; /**< 本周期已允许输出标志。 */
    float   tau2_cmd_nm;   /**< J2 最终命令力矩，单位 N·m。 */
    float   tau3_cmd_nm;   /**< J3 最终命令力矩，单位 N·m。 */
    float   tau4_cmd_nm;   /**< J4 最终命令力矩，单位 N·m。 */
    float   tau5_cmd_nm;   /**< J5 最终命令力矩，单位 N·m。 */
} Arm_Gravity_Debug_t;

/** Ozone 观察用的重力补偿安全门和最终命令快照。 */
extern volatile Arm_Gravity_Debug_t g_arm_gravity_debug;

/**
 * @brief 初始化重力补偿状态并确保全部电机失能。
 *
 * @param 无。
 *
 * @retval 无。
 *
 * @note 调用关系：由 robot_control_init() 在创建周期线程前调用。
 * @note 安全说明：清零调试力矩并调用 arm_motor_stop_all()，不会开启任何输出。
 */
void arm_gravity_init(void);

/**
 * @brief 执行一次重力补偿安全检查、计算和输出更新。
 *
 * @param 无。
 *
 * @retval 无。
 *
 * @note 调用关系：由 arm_control_task() 每个 ThreadX tick 调用；先检查 CH7 安全门，再检查
 *       J1~J7 反馈，全部通过后才计算并提交 J2~J5 重力补偿力矩。
 * @note 安全说明：遥控器离线、CH7 非 240 或任一反馈异常都会清零并失能 J1~J7。
 */
void arm_gravity_update(void);

#endif /* _ARM_GRAVITY_H_ */
