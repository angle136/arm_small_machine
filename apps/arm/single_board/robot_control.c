#include "robot_control.h"

#include "arm_gravity.h"
#include "arm_motor.h"
#include "arm_remote.h"
#include "bsp_can_task.h"
#include "bsp_def.h"
#include "tx_api.h"

#define LOG_TAG "app_arm"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

static TX_THREAD                  g_arm_status_thread;
static TX_THREAD                  g_arm_control_thread;
APPS_STACK_SECTION static uint8_t g_arm_status_stack[1536];
APPS_STACK_SECTION static uint8_t g_arm_control_stack[1024];

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
 *       安全门，再执行重力补偿输出更新，调用顺序不可交换。
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
        tx_thread_sleep(1);
    }
}

void robot_control_init(void)
{
    BSP_CAN_TaskInit();
    arm_motor_init();
    arm_remote_init();
    arm_gravity_init();

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

    LOG_I("arm initialized: %u motors, remote guard=%u SBUS[%u]=%d", ARM_MOTOR_COUNT,
          (unsigned)ARM_GRAVITY_REMOTE_GUARD_ENABLE, (unsigned)ARM_REMOTE_GRAVITY_CHANNEL_INDEX, ARM_REMOTE_GRAVITY_ENABLE_RAW);
}
