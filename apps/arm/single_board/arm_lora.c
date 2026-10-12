#include "arm_lora.h"

#include "arm_def.h"
#include "arm_motor.h"
#include "module_lora.h"
#include <math.h>
#include <stddef.h>

#define LOG_TAG "arm_lora"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

#define ARM_LORA_FIELD_COUNT 8U

static float   g_arm_lora_payload[ARM_LORA_FIELD_COUNT];
static uint8_t g_arm_lora_started;

static const char *const g_arm_lora_names[ARM_LORA_FIELD_COUNT] = {"J1", "J2", "J3", "J4", "J5", "J6", "J7", "tool"};
static const float       g_arm_lora_direction[ARM_MOTOR_COUNT]  = {
    ARM_LORA_J1_DIRECTION, ARM_LORA_J2_DIRECTION, ARM_LORA_J3_DIRECTION, ARM_LORA_J4_DIRECTION,
    ARM_LORA_J5_DIRECTION, ARM_LORA_J6_DIRECTION, ARM_LORA_J7_DIRECTION,
};
static const float g_arm_lora_min_rad[ARM_MOTOR_COUNT] = {
    ARM_LORA_J1_MIN_RAD, ARM_LORA_J2_MIN_RAD, ARM_LORA_J3_MIN_RAD, ARM_LORA_J4_MIN_RAD, ARM_LORA_J5_MIN_RAD, ARM_LORA_J6_MIN_RAD, ARM_LORA_J7_MIN_RAD,
};
static const float g_arm_lora_max_rad[ARM_MOTOR_COUNT] = {
    ARM_LORA_J1_MAX_RAD, ARM_LORA_J2_MAX_RAD, ARM_LORA_J3_MAX_RAD, ARM_LORA_J4_MAX_RAD, ARM_LORA_J5_MAX_RAD, ARM_LORA_J6_MAX_RAD, ARM_LORA_J7_MAX_RAD,
};

/**
 * @brief 将关节角限制到重装机器人接收端允许范围。
 *
 * @param value 输入，完成方向转换后的关节角，单位 rad。
 * @param min_value 输入，接收端下限，单位 rad。
 * @param max_value 输入，接收端上限，单位 rad。
 *
 * @return float 饱和后的关节角；非有限输入返回 0.0f。
 *
 * @note 调用关系：仅由 arm_lora_update() 调用。
 * @note 时序要求：纯计算，无阻塞调用。
 * @note 安全副作用：阻止越界值和 NaN/Inf 进入无线发送帧，不改变本机电机状态。
 */
static float arm_lora_clamp(float value, float min_value, float max_value)
{
    if (!isfinite(value)) return 0.0f;
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

void arm_lora_init(void)
{
    g_arm_lora_started = 0U;
    for (size_t i = 0; i < ARM_LORA_FIELD_COUNT; ++i) g_arm_lora_payload[i] = 0.0f;

    for (size_t i = 0; i < ARM_LORA_FIELD_COUNT; ++i)
    {
        if (Lora_Register(g_arm_lora_names[i], &g_arm_lora_payload[i], LORA_TYPE_FLOAT) < 0)
        {
            LOG_E("register %s failed", g_arm_lora_names[i]);
            return;
        }
    }

    if (Lora_Start() < 0)
    {
        LOG_E("sender start failed");
        return;
    }

    g_arm_lora_started = 1U;
    LOG_I("sender started: 8 floats, payload=32 bytes");
}

void arm_lora_update(void)
{
    if (g_arm_lora_started == 0U) return;

    const volatile Arm_Feedback_Snapshot_t *feedback = arm_motor_feedback_get();
    for (size_t i = 0; i < ARM_MOTOR_COUNT; ++i)
    {
        float directed_angle  = feedback->joint[i].joint_angle_rad * g_arm_lora_direction[i];
        g_arm_lora_payload[i] = arm_lora_clamp(directed_angle, g_arm_lora_min_rad[i], g_arm_lora_max_rad[i]);
    }
    g_arm_lora_payload[ARM_MOTOR_COUNT] = 0.0f;
}
