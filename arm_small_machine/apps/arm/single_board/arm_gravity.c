#include "arm_gravity.h"

#include "arm_def.h"
#include "arm_motor.h"
#include "arm_remote.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

/** 一次重力模型计算的输入角和原始模型力矩。 */
typedef struct
{
    float q2;      /**< J2 重力模型角，单位 rad。 */
    float q3;      /**< J3 重力模型角，单位 rad。 */
    float q4;      /**< J4 重力模型角，单位 rad。 */
    float q5;      /**< J5 重力模型角，单位 rad。 */
    float tau2_nm; /**< J2 未应用输出比例和方向的模型力矩，单位 N·m。 */
    float tau3_nm; /**< J3 未应用输出比例和方向的模型力矩，单位 N·m。 */
    float tau4_nm; /**< J4 未应用输出比例和方向的模型力矩，单位 N·m。 */
    float tau5_nm; /**< J5 未应用输出比例和方向的模型力矩，单位 N·m。 */
} Arm_Gravity_Result_t;

/** 单关节粘滞阻尼配置。 */
typedef struct
{
    uint8_t enable;          /**< 非 0 时计算该关节阻尼。 */
    float   coefficient;     /**< 粘滞阻尼系数，单位 N·m/(rad/s)。 */
    float   deadband_rad_s;  /**< 角速度死区，单位 rad/s。 */
    float   torque_limit_nm; /**< 阻尼分量绝对值上限，单位 N·m。 */
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

/**
 * @brief 对角速度应用连续死区，保留死区外的超出量。
 * @param speed_rad_s 输入角速度，单位 rad/s。
 * @param deadband_rad_s 输入对称死区半宽，单位 rad/s。
 * @return float 死区处理后的角速度，单位 rad/s。
 */
static float arm_damping_apply_deadband(float speed_rad_s, float deadband_rad_s)
{
    if (speed_rad_s > deadband_rad_s) return speed_rad_s - deadband_rad_s;
    if (speed_rad_s < -deadband_rad_s) return speed_rad_s + deadband_rad_s;
    return 0.0f;
}

/**
 * @brief 对阻尼力矩做对称限幅。
 * @param torque_nm 输入力矩，单位 N·m。
 * @param limit_nm 输入绝对值上限，单位 N·m。
 * @return float 限幅后的力矩，单位 N·m。
 */
static float arm_damping_limit_torque(float torque_nm, float limit_nm)
{
    if (torque_nm > limit_nm) return limit_nm;
    if (torque_nm < -limit_nm) return -limit_nm;
    return torque_nm;
}

/**
 * @brief 清空阻尼滤波器和阻尼调试快照。
 * @param 无。
 * @retval 无。
 * @note 调用关系：初始化或任一安全条件失效时调用；下一次计算从当前速度重新初始化。
 */
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
 * @retval 无。
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
    result->q2                                       = feedback->joint[ARM_JOINT_J2].model_angle_rad;
    result->q3                                       = feedback->joint[ARM_JOINT_J3].model_angle_rad;
    result->q4                                       = feedback->joint[ARM_JOINT_J4].model_angle_rad;
    result->q5                                       = feedback->joint[ARM_JOINT_J5].model_angle_rad;

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

/**
 * @brief 初始化重力补偿、阻尼及安全输出状态。
 * @param 无。
 * @retval 无。
 * @note 调用关系：由 robot_control_init() 在线程启动前调用。
 */
void arm_gravity_init(void)
{
    memset((void *)&g_arm_gravity_debug, 0, sizeof(g_arm_gravity_debug));
    arm_damping_reset();
    arm_motor_stop_all();
}

/**
 * @brief 完成一次安全门检查、重力/阻尼计算和电机输出提交。
 * @param 无。
 * @retval 无。
 * @note 安全说明：遥控安全门或任一反馈不满足时立即清零并失能全部关节。
 */
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

    g_arm_gravity_debug.tau2_cmd_nm = torque_nm[ARM_JOINT_J2];
    g_arm_gravity_debug.tau3_cmd_nm = torque_nm[ARM_JOINT_J3];
    g_arm_gravity_debug.tau4_cmd_nm = torque_nm[ARM_JOINT_J4];
    g_arm_gravity_debug.tau5_cmd_nm = torque_nm[ARM_JOINT_J5];

    arm_motor_apply_outputs(torque_nm, enable);
    g_arm_gravity_debug.output_active = 1U;
}

/**
 * @brief 返回最近一次控制周期是否已提交有效输出。
 * @return uint8_t 已提交返回 1，否则返回 0。
 */
uint8_t arm_gravity_output_active(void) { return g_arm_gravity_debug.output_active; }
