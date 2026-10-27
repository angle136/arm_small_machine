#ifndef _ARM_GRAVITY_H_
#define _ARM_GRAVITY_H_

#include "arm_def.h"
#include <stdint.h>

/** 重力补偿控制周期的可观测状态，仅供 Ozone 和上层只读查询。 */
typedef struct
{
    uint8_t remote_enable;                        /**< CH7 安全门状态。 */
    uint8_t feedback_ready;                       /**< J1~J7 反馈全部有效标志。 */
    uint8_t output_active;                        /**< 本周期已允许输出标志。 */
    float   tau2_cmd_nm;                          /**< J2 最终命令力矩，单位 N·m。 */
    float   tau3_cmd_nm;                          /**< J3 最终命令力矩，单位 N·m。 */
    float   tau4_cmd_nm;                          /**< J4 最终命令力矩，单位 N·m。 */
    float   tau5_cmd_nm;                          /**< J5 最终命令力矩，单位 N·m。 */
    float   damping_speed_rad_s[ARM_MOTOR_COUNT]; /**< J1~J7 滤波后的驱动层速度。 */
    float   damping_torque_nm[ARM_MOTOR_COUNT];   /**< J1~J7 阻尼命令力矩。 */
} Arm_Gravity_Debug_t;

/** Ozone 观察用的重力补偿、阻尼和最终命令快照；跨模块状态判断使用查询接口。 */
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
 * @brief 执行一次重力补偿与阻尼的安全检查、计算和输出更新。
 *
 * @param 无。
 *
 * @retval 无。
 *
 * @note 调用关系：由 arm_control_task() 每个 ThreadX tick 调用；先检查 CH7 安全门，再检查
 *       J1~J7 反馈，全部通过后才计算并提交 J2~J5 重力补偿及 J1~J7 阻尼力矩。
 * @note 安全说明：遥控器离线、CH7 非 1807 或任一反馈异常都会清零并失能 J1~J7。
 */
void arm_gravity_update(void);

/**
 * @brief 查询当前是否满足重力补偿及板间发送的全部安全条件。
 *
 * @param 无。
 *
 * @return uint8_t 最近一次 arm_gravity_update() 已提交电机输出返回 1，否则返回 0。
 *
 * @note 调用关系：由 robot_control 的板间发布逻辑调用；只读取控制状态，不计算力矩。
 * @note 安全说明：遥控关闭、遥控离线或任一关节反馈异常时返回 0。
 */
uint8_t arm_gravity_output_active(void);

#endif /* _ARM_GRAVITY_H_ */
