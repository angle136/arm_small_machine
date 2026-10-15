#include "arm_nrf24l01.h"

#include "arm_def.h"
#include "arm_motor.h"
#include "module_nrf24l01.h"
#include <math.h>
#include <stddef.h>

#define LOG_TAG "arm_nrf24l01"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

#define ARM_NRF24L01_FIELD_COUNT 8U

static float   g_arm_nrf24l01_payload[ARM_NRF24L01_FIELD_COUNT];
static uint8_t g_arm_nrf24l01_registered;

static const char *const g_arm_nrf24l01_names[ARM_NRF24L01_FIELD_COUNT] = {"J1", "J2", "J3", "J4", "J5", "J6", "J7", "tool"};
static const float       g_arm_nrf24l01_direction[ARM_MOTOR_COUNT]      = {
    ARM_NRF24L01_J1_DIRECTION, ARM_NRF24L01_J2_DIRECTION, ARM_NRF24L01_J3_DIRECTION, ARM_NRF24L01_J4_DIRECTION,
    ARM_NRF24L01_J5_DIRECTION, ARM_NRF24L01_J6_DIRECTION, ARM_NRF24L01_J7_DIRECTION,
};
static const float g_arm_nrf24l01_min_rad[ARM_MOTOR_COUNT] = {
    ARM_NRF24L01_J1_MIN_RAD, ARM_NRF24L01_J2_MIN_RAD, ARM_NRF24L01_J3_MIN_RAD, ARM_NRF24L01_J4_MIN_RAD,
    ARM_NRF24L01_J5_MIN_RAD, ARM_NRF24L01_J6_MIN_RAD, ARM_NRF24L01_J7_MIN_RAD,
};
static const float g_arm_nrf24l01_max_rad[ARM_MOTOR_COUNT] = {
    ARM_NRF24L01_J1_MAX_RAD, ARM_NRF24L01_J2_MAX_RAD, ARM_NRF24L01_J3_MAX_RAD, ARM_NRF24L01_J4_MAX_RAD,
    ARM_NRF24L01_J5_MAX_RAD, ARM_NRF24L01_J6_MAX_RAD, ARM_NRF24L01_J7_MAX_RAD,
};

static float arm_nrf24l01_clamp(float value, float min_value, float max_value)
{
    if (!isfinite(value)) return 0.0f;
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

void arm_nrf24l01_init(void)
{
    g_arm_nrf24l01_registered = 0U;
    for (size_t i = 0; i < ARM_NRF24L01_FIELD_COUNT; ++i) g_arm_nrf24l01_payload[i] = 0.0f;

    for (size_t i = 0; i < ARM_NRF24L01_FIELD_COUNT; ++i)
    {
        if (Module_NRF24L01_Register(g_arm_nrf24l01_names[i], &g_arm_nrf24l01_payload[i], NRF24L01_TYPE_FLOAT) < 0)
        {
            LOG_E("register %s failed", g_arm_nrf24l01_names[i]);
            return;
        }
    }

    g_arm_nrf24l01_registered = 1U;
    LOG_I("sender registered: 8 floats, payload=32 bytes");
}

void arm_nrf24l01_update(void)
{
    if (g_arm_nrf24l01_registered == 0U) return;

    const volatile Arm_Feedback_Snapshot_t *feedback = arm_motor_feedback_get();
    for (size_t i = 0; i < ARM_MOTOR_COUNT; ++i)
    {
        float directed_angle = feedback->joint[i].joint_angle_rad * g_arm_nrf24l01_direction[i];
        g_arm_nrf24l01_payload[i] =
            arm_nrf24l01_clamp(directed_angle, g_arm_nrf24l01_min_rad[i], g_arm_nrf24l01_max_rad[i]);
    }
    g_arm_nrf24l01_payload[ARM_MOTOR_COUNT] = 0.0f;
}
