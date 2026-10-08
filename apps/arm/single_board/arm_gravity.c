#include "arm_gravity.h"

#include "arm_def.h"
#include "arm_motor.h"
#include "arm_remote.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

typedef struct
{
    float q2;
    float q3;
    float q4;
    float q5;
    float tau2_nm;
    float tau3_nm;
    float tau4_nm;
    float tau5_nm;
} Arm_Gravity_Result_t;

volatile Arm_Gravity_Debug_t g_arm_gravity_debug;

/**
 * @brief 使用同一反馈快照计算 J2~J5 重力补偿力矩。
 * @param result 输出，保存模型角和未应用比例/符号的模型力矩。
 * @retval 无。
 * @note 调用关系：仅由 arm_gravity_update() 在安全条件全部通过后调用，每 tick 最多一次。
 * @note 安全说明：本函数只计算，不访问电机驱动、不改变使能状态。
 */
static void arm_gravity_calculate(Arm_Gravity_Result_t *result)
{
    if (result == NULL) return;

    const volatile Arm_Feedback_Snapshot_t *feedback = arm_motor_feedback_get();
    result->q2 = feedback->joint[1].model_angle_rad;
    result->q3 = feedback->joint[2].model_angle_rad;
    result->q4 = feedback->joint[3].model_angle_rad;
    result->q5 = feedback->joint[4].model_angle_rad;

    result->tau2_nm = ARM_J2_BASE_MASS * ARM_J2_BASE_ARM_M * sinf(result->q2) +
                      ARM_J2_LINK_MASS * (ARM_J2_LINK_BASE_ARM_M * sinf(result->q2) -
                                          ARM_J2_LINK_J3_ARM_M * sinf(result->q2 + result->q3)) +
                      ARM_J2_DISTAL_MASS * (ARM_J2_DISTAL_BASE_ARM_M * sinf(result->q2) -
                                            ARM_J2_DISTAL_J3_ARM_M * sinf(result->q2 + result->q3) -
                                            ARM_J2_DISTAL_WRIST_ARM_M * sinf(result->q2 + result->q3 + result->q5));
    result->tau3_nm = ARM_J3_DISTAL_MASS *
                          (ARM_J3_LINK_ARM_M * sinf(result->q2 + result->q3) +
                           ARM_J3_WRIST_ARM_M * sinf(result->q2 + result->q3 + result->q5)) +
                      ARM_J3_GRAVITY_COM * sinf(result->q2 + result->q3);
    result->tau4_nm = ARM_J4_GRAVITY_COUPLE_SCALE * ARM_J5_GRAVITY_COM * sinf(result->q2 + result->q3) * sinf(result->q5) * sinf(result->q4);
    result->tau5_nm = ARM_J5_GRAVITY_COM * sinf(result->q5 + result->q2 + result->q3) * cosf(result->q4 * sinf(result->q2 + result->q3));
}

/**
 * @brief 清零重力补偿命令调试值。
 * @param 无。
 * @retval 无。
 * @note 调用关系：由初始化及所有安全拒绝路径调用；只清零 APP 快照，不直接发送命令。
 */
static void arm_gravity_clear_debug_torque(void)
{
    g_arm_gravity_debug.tau2_cmd_nm = 0.0f;
    g_arm_gravity_debug.tau3_cmd_nm = 0.0f;
    g_arm_gravity_debug.tau4_cmd_nm = 0.0f;
    g_arm_gravity_debug.tau5_cmd_nm = 0.0f;
}

/**
 * @brief 在保留全部关节在线反馈的条件下发送零力矩命令。
 * @param 无。
 * @retval 无。
 * @note 调用关系：仅由 arm_gravity_update() 在遥控器安全门和全部反馈检查通过后调用；
 *       零力矩测试开关打开期间每个控制周期调用一次。
 * @note 时序要求：沿用 arm_motor_apply_outputs() 和 Motor_ApplyAll() 的既有 CAN 调度。
 * @note 安全说明：J1~J7 保持硬件使能，但目标力矩全部为 0 N.m；底层反馈在线检查、
 *       角度限位和最终输出限幅继续生效。
 */
static void arm_gravity_apply_zero_torque_mode(void)
{
    static const float   torque_nm[ARM_MOTOR_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    static const uint8_t enable[ARM_MOTOR_COUNT]    = {1U, 1U, 1U, 1U, 1U, 1U, 1U};

    arm_gravity_clear_debug_torque();
    arm_motor_apply_outputs(torque_nm, enable);
}

void arm_gravity_init(void)
{
    memset((void *)&g_arm_gravity_debug, 0, sizeof(g_arm_gravity_debug));
    arm_motor_stop_all();
}

void arm_gravity_update(void)
{
    uint8_t remote_enable  = arm_remote_gravity_enabled();
    uint8_t feedback_ready = arm_motor_all_feedback_ready();

    g_arm_gravity_debug.remote_enable  = remote_enable;
    g_arm_gravity_debug.feedback_ready = feedback_ready;

    if (remote_enable == 0U || feedback_ready == 0U)
    {
        g_arm_gravity_debug.output_active = 0U;
        arm_gravity_clear_debug_torque();
        arm_motor_stop_all();
        return;
    }

    if (ARM_ZERO_TORQUE_MODE_ENABLE != 0U)
    {
        arm_gravity_apply_zero_torque_mode();
        g_arm_gravity_debug.output_active = 1U;
        return;
    }

    Arm_Gravity_Result_t gravity = {0};
    arm_gravity_calculate(&gravity);

    float torque_nm[ARM_MOTOR_COUNT] = {
        0.0f,
        (ARM_J2_GRAVITY_OUTPUT_ENABLE != 0U) ? ARM_J2_GRAVITY_OUTPUT_SIGN * ARM_J2_GRAVITY_TEST_SCALE * gravity.tau2_nm : 0.0f,
        (ARM_J3_GRAVITY_OUTPUT_ENABLE != 0U) ? ARM_J3_GRAVITY_OUTPUT_SIGN * ARM_J3_GRAVITY_TEST_SCALE * gravity.tau3_nm : 0.0f,
        (ARM_J4_GRAVITY_OUTPUT_ENABLE != 0U) ? ARM_J4_GRAVITY_OUTPUT_SIGN * ARM_J4_GRAVITY_TEST_SCALE * gravity.tau4_nm : 0.0f,
        (ARM_J5_GRAVITY_OUTPUT_ENABLE != 0U) ? ARM_J5_GRAVITY_OUTPUT_SIGN * ARM_J5_GRAVITY_TEST_SCALE * gravity.tau5_nm : 0.0f,
        0.0f,
        0.0f,
    };
    const uint8_t enable[ARM_MOTOR_COUNT] = {
        ARM_J1_ZERO_TORQUE_ENABLE,
        ARM_J2_GRAVITY_OUTPUT_ENABLE,
        ARM_J3_GRAVITY_OUTPUT_ENABLE,
        ARM_J4_GRAVITY_OUTPUT_ENABLE,
        ARM_J5_GRAVITY_OUTPUT_ENABLE,
        ARM_J6_ZERO_TORQUE_ENABLE,
        ARM_J7_ZERO_TORQUE_ENABLE,
    };

    g_arm_gravity_debug.tau2_cmd_nm = torque_nm[1];
    g_arm_gravity_debug.tau3_cmd_nm = torque_nm[2];
    g_arm_gravity_debug.tau4_cmd_nm = torque_nm[3];
    g_arm_gravity_debug.tau5_cmd_nm = torque_nm[4];

    arm_motor_apply_outputs(torque_nm, enable);
    g_arm_gravity_debug.output_active = 1U;
}
