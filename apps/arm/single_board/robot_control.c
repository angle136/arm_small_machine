#include "robot_control.h"

#include "arm_gravity.h"
#include "arm_motor.h"
#include "arm_remote.h"
#include "board_link.h"
#include "bsp_def.h"
#include "tx_api.h"

#include <string.h>

#define LOG_TAG "app_arm"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

static TX_THREAD                  g_arm_status_thread;
static TX_THREAD                  g_arm_control_thread;
static TX_THREAD                  g_arm_link_thread;
static TX_MUTEX                   g_arm_link_payload_mutex;
APPS_STACK_SECTION static uint8_t g_arm_status_stack[1536];
APPS_STACK_SECTION static uint8_t g_arm_control_stack[1024];
APPS_STACK_SECTION static uint8_t g_arm_link_stack[1024];
BUFFER_SECTION static uint8_t     g_arm_link_payload[BOARD_LINK_PAYLOAD_MAX];
BUFFER_SECTION static uint8_t     g_arm_link_tx_payload[BOARD_LINK_PAYLOAD_MAX];

/* 接收端坐标偏置，仅作用于发送值，不改变本机零点、限位或重力补偿。 */
static const float g_arm_link_angle_offset_rad[ARM_MOTOR_COUNT] = {
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.04309896f, 0.0f,
};

static void arm_publish_feedback(void)
{
    const volatile Arm_Feedback_Snapshot_t *feedback = arm_motor_feedback_get();

    if (tx_mutex_get(&g_arm_link_payload_mutex, TX_WAIT_FOREVER) != TX_SUCCESS)
    {
        return;
    }

    for (uint32_t i = 0U; i < ARM_MOTOR_COUNT; ++i)
    {
        float value = feedback->joint[i].joint_angle_rad + g_arm_link_angle_offset_rad[i];

        if (i == 1U) // 数组下标1对应J2
        {
            value = -value;
        }

        memcpy(&g_arm_link_payload[i * sizeof(float)], &value, sizeof(value));
    }

    const float tool_value = 0.0f;
    memcpy(&g_arm_link_payload[ARM_MOTOR_COUNT * sizeof(float)], &tool_value, sizeof(tool_value));

    tx_mutex_put(&g_arm_link_payload_mutex);
}

static void arm_link_tx_task(ULONG thread_input)
{
    (void)thread_input;

    const uint8_t payload_len = (uint8_t)((ARM_MOTOR_COUNT + 1U) * sizeof(float));

    while (1)
    {
        const ULONG cycle_start = tx_time_get();

        if (tx_mutex_get(&g_arm_link_payload_mutex, TX_WAIT_FOREVER) == TX_SUCCESS)
        {
            memcpy(g_arm_link_tx_payload, g_arm_link_payload, payload_len);
            tx_mutex_put(&g_arm_link_payload_mutex);

            /* UART 等待发生在锁外，控制线程仍可继续刷新安全状态和快照。 */
            board_link_tx(g_arm_link_tx_payload, payload_len);
        }

        const ULONG elapsed = tx_time_get() - cycle_start;
        if (elapsed < (ULONG)BOARD_LINK_TX_INTERVAL_MS)
        {
            tx_thread_sleep((ULONG)BOARD_LINK_TX_INTERVAL_MS - elapsed);
        }
    }
}

/**
 * @brief 机械臂状态日志线程入口。
 *
 * @param thread_input 输入，ThreadX 线程入口参数，当前未使用。
 *
 * @retval 无，线程创建后持续运行。
 *
 * @note 调用关系：由 robot_control_init() 创建；每 1000 tick 调用 arm_motor_log_status()。
 * @note 安全说明：线程只读反馈并输出日志，不改变遥控器安全门或电机命令。
 */
static void arm_status_task(ULONG thread_input)
{
    (void)thread_input;

    while (1)
    {
        arm_motor_log_status();
        tx_thread_sleep(1000);
    }
}

/**
 * @brief 机械臂周期控制线程入口。
 *
 * @param thread_input 输入，ThreadX 线程入口参数，当前未使用。
 *
 * @retval 无，线程创建后持续运行。
 *
 * @note 调用关系：由 robot_control_init() 创建；每 1 tick 依次刷新电机反馈、刷新 SBUS
 *       安全门，再执行重力补偿输出更新，最后发布 board_link 发送快照。
 * @note 安全说明：最终输出前由 arm_gravity_update() 同时检查 CH7 和全部关节反馈。
 */
static void arm_control_task(ULONG thread_input)
{
    (void)thread_input;

    while (1)
    {
        arm_motor_feedback_update();
        arm_remote_update();
        arm_gravity_update();
        arm_publish_feedback();
        tx_thread_sleep(1);
    }
}

void robot_control_init(void)
{
    arm_motor_init();
    board_link_init();
    arm_remote_init();
    arm_gravity_init();

    UINT payload_mutex_status = tx_mutex_create(&g_arm_link_payload_mutex, "arm_link_payload", TX_INHERIT);
    if (payload_mutex_status != TX_SUCCESS)
    {
        arm_motor_stop_all();
        LOG_E("arm link payload mutex create failed: %u", (unsigned)payload_mutex_status);
        return;
    }

    UINT link_status = tx_thread_create(&g_arm_link_thread, "arm_link_tx", arm_link_tx_task, 0, g_arm_link_stack, sizeof(g_arm_link_stack), 14, 14,
                                        TX_NO_TIME_SLICE, TX_DONT_START);
    if (link_status != TX_SUCCESS)
    {
        arm_motor_stop_all();
        LOG_E("arm link tx thread create failed: %u", (unsigned)link_status);
        return;
    }

    UINT status = tx_thread_create(&g_arm_status_thread, "arm_status", arm_status_task, 0, g_arm_status_stack, sizeof(g_arm_status_stack), 20, 20,
                                   TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
    {
        arm_motor_stop_all();
        LOG_E("arm_status thread create failed: %u", (unsigned)status);
        return;
    }

    UINT control_status = tx_thread_create(&g_arm_control_thread, "arm_control", arm_control_task, 0, g_arm_control_stack,
                                           sizeof(g_arm_control_stack), 13, 13, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (control_status != TX_SUCCESS)
    {
        arm_motor_stop_all();
        LOG_E("arm_control thread create failed: %u", (unsigned)control_status);
        return;
    }

    UINT link_resume_status = tx_thread_resume(&g_arm_link_thread);
    if (link_resume_status != TX_SUCCESS)
    {
        arm_motor_stop_all();
        LOG_E("arm link tx thread start failed: %u", (unsigned)link_resume_status);
        return;
    }

    LOG_I("arm initialized: %u motors, remote guard=%u SBUS[%u]=%d", ARM_MOTOR_COUNT, (unsigned)ARM_GRAVITY_REMOTE_GUARD_ENABLE,
          (unsigned)ARM_REMOTE_GRAVITY_CHANNEL_INDEX, ARM_REMOTE_GRAVITY_ENABLE_RAW);
}
