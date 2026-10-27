#include "arm_remote.h"

#include "module_remote.h"
#include <stddef.h>
#include <string.h>

volatile int16_t            g_arm_remote_channels[ARM_REMOTE_CHANNEL_COUNT];
volatile Arm_Remote_Debug_t g_arm_remote_debug;

/**
 * @brief 清零遥控器通道和安全门快照。
 * @param 无。
 * @retval 无。
 */
void arm_remote_init(void)
{
    memset((void *)g_arm_remote_channels, 0, sizeof(g_arm_remote_channels));
    memset((void *)&g_arm_remote_debug, 0, sizeof(g_arm_remote_debug));
}

/**
 * @brief 从 REMOTE 模块刷新原始 SBUS 通道及重力补偿安全门。
 * @param 无。
 * @retval 无。
 * @note 安全说明：数据源无效、遥控器离线或 CH7 非明确使能值时关闭安全门。
 */
void arm_remote_update(void)
{
    const Remote_Data_t *remote = Module_Remote_get_data();
    if (remote == NULL)
    {
        g_arm_remote_debug.online         = 0U;
        g_arm_remote_debug.gravity_enable = (ARM_GRAVITY_REMOTE_GUARD_ENABLE != 0) ? 0U : 1U;
        return;
    }

    for (size_t i = 0U; i < ARM_REMOTE_CHANNEL_COUNT; ++i)
    {
        /* 模块内部仅对 CH1~CH4 做零偏；调试数组统一还原为 SBUS 原始 11 位值。 */
        g_arm_remote_channels[i] = (i < 4U) ? (int16_t)(remote->channels[i] + (int16_t)SBUS_CHX_BIAS) : remote->channels[i];
    }

    uint8_t online  = ((Module_Remote_get_offline_status() & 0x01U) != 0U) ? 1U : 0U;
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

/**
 * @brief 返回最近一次刷新得到的重力补偿安全门状态。
 * @return uint8_t 允许输出返回 1，否则返回 0。
 */
uint8_t arm_remote_gravity_enabled(void) { return g_arm_remote_debug.gravity_enable; }

/**
 * @brief 读取一个已还原为 SBUS 原始值的遥控通道。
 * @param channel_index 输入，数组下标 0~15。
 * @return int16_t 通道原始值；索引越界返回 0。
 */
int16_t arm_remote_channel_get(uint8_t channel_index)
{
    if (channel_index >= ARM_REMOTE_CHANNEL_COUNT) return 0;
    return g_arm_remote_channels[channel_index];
}

/**
 * @brief 返回最近一次刷新得到的遥控器在线状态。
 * @return uint8_t 在线返回 1，离线返回 0。
 */
uint8_t arm_remote_is_online(void) { return g_arm_remote_debug.online; }
