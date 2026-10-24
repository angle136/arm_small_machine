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

typedef struct
{
    uint8_t enable;
    float   coefficient;
    float   deadband_rad_s;
    float   torque_limit_nm;
} Arm_Damping_Config_t;

static const Arm_Damping_Config_t g_arm_damping_config[ARM_MOTOR_COUNT] = {
    {ARM_J1_DAMPING_ENABLE, ARM_J1_DAMPING_COEFF, ARM_J1_DAMPING_DEADBAND_RAD_S, ARM_J1_DAMPING_LIMIT_NM},
    {ARM_J2_DAMPING_ENABLE, ARM_J2_DAMPING_COEFF, ARM_J2_DAMPING_DEADBAND_RAD_S, ARM_J2_DAMPING_LIMIT_NM},
    {ARM_J3_DAMPING_ENABLE, ARM_J3_DAMPING_COEFF, ARM_J3_DAMPING_DEADBAND_RAD_S, ARM_J3_DAMPING_LIMIT_NM},
    {ARM_J4_DAMPING_ENABLE, ARM_J4_DAMPING_COEFF, ARM_J4_DAMPING_DEADBAND_RAD_S, ARM_J4_DAMPING_LIMIT_NM},
    {ARM_J5_DAMPING_ENABLE, ARM_J5_DAMPING_COEFF, ARM_J5_DAMPING_DEADBAND_RAD_S, ARM_J5_DAMPING_LIMIT_NM},
    {ARM_J6_DAMPING_ENABLE, ARM_J6_DAMPING_COEFF, ARM_J6_DAMPING_DEADBAND_RAD_S, ARM_J6_DAMPING_LIMIT_NM},
    {ARM_J7_DAMPING_ENABLE, ARM_J7_DAMPING_COEFF, ARM_J7_DAMPING_DEADBAND_RAD_S, ARM_J7_DAMPING_LIMIT_NM},
};

static float   g_arm_damping_speed_rad_s[ARM_MOTOR_COUNT];
static uint8_t g_arm_damping_filter_initialized;

volatile Arm_Gravity_Debug_t g_arm_gravity_debug;

static float arm_damping_apply_deadband(float speed_rad_s, float deadband_rad_s)
{
    if (speed_rad_s > deadband_rad_s) return speed_rad_s - deadband_rad_s;
    if (speed_rad_s < -deadband_rad_s) return speed_rad_s + deadband_rad_s;
    return 0.0f;
}

static float arm_damping_limit_torque(float torque_nm, float limit_nm)
{
    if (torque_nm > limit_nm) return limit_nm;
    if (torque_nm < -limit_nm) return -limit_nm;
    return torque_nm;
}

static void arm_damping_reset(void)
{
    memset(g_arm_damping_speed_rad_s, 0, sizeof(g_arm_damping_speed_rad_s));
    memset((void *)g_arm_gravity_debug.damping_speed_rad_s, 0, sizeof(g_arm_gravity_debug.damping_speed_rad_s));
    memset((void *)g_arm_gravity_debug.damping_torque_nm, 0, sizeof(g_arm_gravity_debug.damping_torque_nm));
    g_arm_damping_filter_initialized = 0U;
}

/**
 * @brief 计算 J1~J7 驱动坐标系粘滞阻尼力矩。
 * @param torque_nm 输出，长度为 ARM_MOTOR_COUNT 的阻尼力矩数组。
 * @note 使用驱动层速度可直接保证阻尼力矩与电机转动方向相反，不依赖重力模型符号约定。
 */
static void arm_damping_calculate(float torque_nm[ARM_MOTOR_COUNT])
{
    if (torque_nm == NULL) return;

    const volatile Arm_Feedback_Snapshot_t *feedback = arm_motor_feedback_get();

    if (g_arm_damping_filter_initialized == 0U)
    {
        for (size_t i = 0U; i < ARM_MOTOR_COUNT; ++i) g_arm_damping_speed_rad_s[i] = feedback->joint[i].speed_rad_s;
        g_arm_damping_filter_initialized = 1U;
    }

    for (size_t i = 0U; i < ARM_MOTOR_COUNT; ++i)
    {
        const Arm_Damping_Config_t *config          = &g_arm_damping_config[i];
        float                       raw_speed_rad_s = feedback->joint[i].speed_rad_s;

        g_arm_damping_speed_rad_s[i] += ARM_DAMPING_FILTER_ALPHA * (raw_speed_rad_s - g_arm_damping_speed_rad_s[i]);
        g_arm_gravity_debug.damping_speed_rad_s[i] = g_arm_damping_speed_rad_s[i];

        if (ARM_DAMPING_ENABLE == 0U || config->enable == 0U)
        {
            torque_nm[i]                             = 0.0f;
            g_arm_gravity_debug.damping_torque_nm[i] = 0.0f;
            continue;
        }

        float effective_speed_rad_s = arm_damping_apply_deadband(g_arm_damping_speed_rad_s[i], config->deadband_rad_s);
        float damping_torque_nm     = -ARM_DAMPING_GLOBAL_SCALE * config->coefficient * effective_speed_rad_s;

        torque_nm[i]                             = arm_damping_limit_torque(damping_torque_nm, config->torque_limit_nm);
        g_arm_gravity_debug.damping_torque_nm[i] = torque_nm[i];
    }
}

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

    const float q23     = result->q2 + result->q3;
    const float sin_q23 = sinf(q23);
    const float cos_q23 = cosf(q23);
    const float sin_q5  = sinf(result->q5);
    const float cos_q5  = cosf(result->q5);
    const float cos_q4  = cosf(result->q4);
    /* J4 旋转腕部俯仰平面；q4=0 时两项都退化为旧模型的 sin(q2+q3+q5)。 */
    const float wrist_to_j23 = sin_q23 * cos_q5 + cos_q23 * cos_q4 * sin_q5;
    const float wrist_to_j5  = cos_q23 * sin_q5 + sin_q23 * cos_q4 * cos_q5;

    result->tau2_nm = ARM_J2_BASE_MASS * ARM_J2_BASE_ARM_M * sinf(result->q2) +
                      ARM_J2_LINK_MASS * (ARM_J2_LINK_BASE_ARM_M * sinf(result->q2) - ARM_J2_LINK_J3_ARM_M * sin_q23) +
                      ARM_J2_DISTAL_MASS *
                          (ARM_J2_DISTAL_BASE_ARM_M * sinf(result->q2) - ARM_J2_DISTAL_J3_ARM_M * sin_q23 - ARM_J2_DISTAL_WRIST_ARM_M * wrist_to_j23);
    result->tau3_nm = ARM_J3_DISTAL_MASS * (ARM_J3_LINK_ARM_M * sin_q23 + ARM_J3_WRIST_ARM_M * wrist_to_j23) + ARM_J3_GRAVITY_COM * sin_q23;
    result->tau4_nm = ARM_J4_GRAVITY_COUPLE_SCALE * ARM_J5_GRAVITY_COM * sin_q23 * sin_q5 * sinf(result->q4);
    result->tau5_nm = ARM_J5_GRAVITY_COM * wrist_to_j5;
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

void arm_gravity_init(void)
{
    memset((void *)&g_arm_gravity_debug, 0, sizeof(g_arm_gravity_debug));
    arm_damping_reset();
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
        arm_damping_reset();
        arm_motor_stop_all();
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
    float damping_torque_nm[ARM_MOTOR_COUNT] = {0};
    arm_damping_calculate(damping_torque_nm);
    for (size_t i = 0U; i < ARM_MOTOR_COUNT; ++i) torque_nm[i] += damping_torque_nm[i];

    const uint8_t enable[ARM_MOTOR_COUNT] = {
        (ARM_J1_ZERO_TORQUE_ENABLE != 0U) || (ARM_DAMPING_ENABLE != 0U && ARM_J1_DAMPING_ENABLE != 0U),
        (ARM_J2_GRAVITY_OUTPUT_ENABLE != 0U) || (ARM_DAMPING_ENABLE != 0U && ARM_J2_DAMPING_ENABLE != 0U),
        (ARM_J3_GRAVITY_OUTPUT_ENABLE != 0U) || (ARM_DAMPING_ENABLE != 0U && ARM_J3_DAMPING_ENABLE != 0U),
        (ARM_J4_GRAVITY_OUTPUT_ENABLE != 0U) || (ARM_DAMPING_ENABLE != 0U && ARM_J4_DAMPING_ENABLE != 0U),
        (ARM_J5_GRAVITY_OUTPUT_ENABLE != 0U) || (ARM_DAMPING_ENABLE != 0U && ARM_J5_DAMPING_ENABLE != 0U),
        (ARM_J6_ZERO_TORQUE_ENABLE != 0U) || (ARM_DAMPING_ENABLE != 0U && ARM_J6_DAMPING_ENABLE != 0U),
        (ARM_J7_ZERO_TORQUE_ENABLE != 0U) || (ARM_DAMPING_ENABLE != 0U && ARM_J7_DAMPING_ENABLE != 0U),
    };

    g_arm_gravity_debug.tau2_cmd_nm = torque_nm[1];
    g_arm_gravity_debug.tau3_cmd_nm = torque_nm[2];
    g_arm_gravity_debug.tau4_cmd_nm = torque_nm[3];
    g_arm_gravity_debug.tau5_cmd_nm = torque_nm[4];

    arm_motor_apply_outputs(torque_nm, enable);
    g_arm_gravity_debug.output_active = 1U;
}
