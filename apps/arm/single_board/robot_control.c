#include "robot_control.h"

#include "arm_gravity.h"
#include "arm_motor.h"
#include "arm_remote.h"
#include "bsp_can_task.h"
#include "bsp_def.h"
#include "tx_api.h"
#include <stdint.h>

#define LOG_TAG "app_arm"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

static TX_THREAD                  g_arm_status_thread;
static TX_THREAD                  g_arm_control_thread;
APPS_STACK_SECTION static uint8_t g_arm_status_stack[1536];
APPS_STACK_SECTION static uint8_t g_arm_control_stack[1024];

/**
 * @brief 输出 J2/J3 同一反馈快照的耦合诊断日志。
 *
 * @param 无。
 *
 * @retval 无。
 *
 * @note 调用关系：仅由 arm_status_task() 每 100 tick 调用；日志同时包含协议原始值、
 *       解码角、关节角和角速度之和，以及补偿命令力矩，便于离线拟合零点、比例和回差。
 * @note 时序要求：必须在 arm_motor_feedback_update() 周期刷新快照后调用；本函数只读取
 *       最近一次完整快照，不要求与 CAN 中断严格同步。
 * @note 安全说明：只读反馈和重力补偿调试量，不修改电机使能、控制命令或安全状态。
 */
static void arm_status_log_j23_diagnostic(void)
{
    const volatile Arm_Feedback_Snapshot_t *feedback = arm_motor_feedback_get();
    const volatile Arm_Motor_Feedback_t    *j2       = &feedback->joint[1];
    const volatile Arm_Motor_Feedback_t    *j3       = &feedback->joint[2];
    const volatile Arm_Motor_Feedback_t    *j5       = &feedback->joint[4];

    float q_sum  = j2->joint_angle_rad + j3->joint_angle_rad;
    float dq_sum = j2->joint_speed_rad_s + j3->joint_speed_rad_s;

    LOG_I("J23A p=%u,%u r=%.4f,%.4f q=%.4f,%.4f s=%.4f", (unsigned)j2->raw_position, (unsigned)j3->raw_position, j2->angle_raw_rad, j3->angle_raw_rad,
          j2->joint_angle_rad, j3->joint_angle_rad, q_sum);
    LOG_I("J23B dq=%.3f,%.3f ds=%.3f q5=%.4f tc=%.3f,%.3f ok=%u,%u en=%u,%u a=%u", j2->joint_speed_rad_s, j3->joint_speed_rad_s, dq_sum,
          j5->joint_angle_rad, g_arm_gravity_debug.tau2_cmd_nm, g_arm_gravity_debug.tau3_cmd_nm, (unsigned)(j2->valid != 0U && j2->online != 0U),
          (unsigned)(j3->valid != 0U && j3->online != 0U), (unsigned)j2->enabled, (unsigned)j3->enabled, (unsigned)g_arm_gravity_debug.output_active);
}

/**
 * @brief 机械臂状态日志线程入口。
 *
 * @param thread_input 输入，ThreadX 线程入口参数，当前未使用。
 *
 * @retval 无，线程创建后持续运行。
 *
 * @note 调用关系：由 robot_control_init() 创建；每 100 tick 输出一次 J2/J3 联合诊断日志，
 *       每 1000 tick 调用一次 arm_motor_log_status() 保留完整电机状态。
 * @note 安全说明：线程只读反馈并输出日志，不改变遥控器安全门或电机命令。
 */
static void arm_status_task(ULONG thread_input)
{
    (void)thread_input;
    uint8_t full_status_divider = 0U;

    while (1)
    {
        arm_status_log_j23_diagnostic();
        full_status_divider++;
        if (full_status_divider >= 10U)
        {
            arm_motor_log_status();
            full_status_divider = 0U;
        }
        tx_thread_sleep(100);
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

    LOG_I("arm initialized: %u motors, zero_tau=%u guard=%u SBUS[%u]=%d", ARM_MOTOR_COUNT, (unsigned)ARM_ZERO_TORQUE_MODE_ENABLE,
          (unsigned)ARM_GRAVITY_REMOTE_GUARD_ENABLE, (unsigned)ARM_REMOTE_GRAVITY_CHANNEL_INDEX, ARM_REMOTE_GRAVITY_ENABLE_RAW);
}
