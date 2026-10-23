#include "arm_remote.h"

#include "module_remote.h"
#include <stddef.h>
#include <string.h>

volatile int16_t            g_arm_remote_channels[ARM_REMOTE_CHANNEL_COUNT];
volatile Arm_Remote_Debug_t g_arm_remote_debug;

void arm_remote_init(void)
{
    memset((void *)g_arm_remote_channels, 0, sizeof(g_arm_remote_channels));
    memset((void *)&g_arm_remote_debug, 0, sizeof(g_arm_remote_debug));
}

void arm_remote_update(void)
{
    const Remote_Data_t *remote = Module_Remote_get_data();
    if (remote == NULL)
    {
        g_arm_remote_debug.online         = 0U;
        g_arm_remote_debug.gravity_enable = (ARM_GRAVITY_REMOTE_GUARD_ENABLE != 0) ? 0U : 1U;
        return;
    }

    for (size_t i = 0; i < ARM_REMOTE_CHANNEL_COUNT; ++i)
    {
        /* 模块内部仅对 CH1~CH4 做零偏；调试数组统一还原为 SBUS 原始 11 位值。 */
        g_arm_remote_channels[i] = (i < 4U) ? (int16_t)(remote->channels[i] + (int16_t)SBUS_CHX_BIAS) : remote->channels[i];
    }

    uint8_t online = ((Module_Remote_get_offline_status() & 0x01U) != 0U) ? 1U : 0U;
    int16_t ch7_raw = g_arm_remote_channels[ARM_REMOTE_GRAVITY_CHANNEL_INDEX];

    g_arm_remote_debug.online  = online;
    g_arm_remote_debug.ch7_raw = ch7_raw;

#if (ARM_GRAVITY_REMOTE_GUARD_ENABLE != 0)
    if (online == 0U || ch7_raw == ARM_REMOTE_GRAVITY_DISABLE_RAW)
    {
        g_arm_remote_debug.gravity_enable = 0U;
    }
    else if (ch7_raw == ARM_REMOTE_GRAVITY_ENABLE_RAW)
    {
        g_arm_remote_debug.gravity_enable = 1U;
    }
    else
    {
        /* 未定义位置、线路干扰或异常值全部按关闭处理。 */
        g_arm_remote_debug.gravity_enable = 0U;
    }
#else
    /* 关闭安全门时保持原有行为：不依赖遥控器，反馈有效即可进入重力补偿。 */
    g_arm_remote_debug.gravity_enable = 1U;
#endif
}

uint8_t arm_remote_gravity_enabled(void) { return g_arm_remote_debug.gravity_enable; }
