#include "robot_control.h"

#include "arm_def.h"
#include "arm_gravity.h"
#include "arm_motor.h"
#include "arm_remote.h"
#include "board_link.h"
#include "bsp_def.h"
#include "tx_api.h"

#include <math.h>
#include <string.h>

#define LOG_TAG "app_arm"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

_Static_assert(sizeof(float) == 4U, "arm board_link requires 32-bit float");
_Static_assert(ARM_BOARD_LINK_STATUS_OFFSET == 32U, "arm board_link status offset changed");
_Static_assert(ARM_BOARD_LINK_PAYLOAD_LEN == 36U, "arm board_link payload length changed");
_Static_assert(ARM_BOARD_LINK_PAYLOAD_LEN <= BOARD_LINK_PAYLOAD_MAX, "arm board_link payload buffer is too small");

static TX_THREAD                  g_arm_status_thread;
static TX_THREAD                  g_arm_control_thread;
static TX_THREAD                  g_arm_link_thread;
static TX_MUTEX                   g_arm_link_payload_mutex;
APPS_STACK_SECTION static uint8_t g_arm_status_stack[1536];
APPS_STACK_SECTION static uint8_t g_arm_control_stack[1024];
APPS_STACK_SECTION static uint8_t g_arm_link_stack[1024];
BUFFER_SECTION static uint8_t     g_arm_link_payload[BOARD_LINK_PAYLOAD_MAX];
BUFFER_SECTION static uint8_t     g_arm_link_tx_payload[BOARD_LINK_PAYLOAD_MAX];
BUFFER_SECTION static uint8_t     g_arm_link_stop_payload[BOARD_LINK_PAYLOAD_MAX];
static volatile uint8_t           g_arm_link_tx_enabled;
static volatile uint8_t           g_arm_link_stop_pending;

/* 接收端坐标偏置，仅作用于发送值，不改变本机零点、限位或重力补偿。 */
static const float g_arm_link_angle_offset_rad[ARM_MOTOR_COUNT] = {
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.04309896f, 0.0f,
};

static const float g_arm_link_min_rad[ARM_MOTOR_COUNT] = {
    ARM_BOARD_LINK_J1_MIN_RAD, ARM_BOARD_LINK_J2_MIN_RAD, ARM_BOARD_LINK_J3_MIN_RAD, ARM_BOARD_LINK_J4_MIN_RAD,
    ARM_BOARD_LINK_J5_MIN_RAD, ARM_BOARD_LINK_J6_MIN_RAD, ARM_BOARD_LINK_J7_MIN_RAD,
};

static const float g_arm_link_max_rad[ARM_MOTOR_COUNT] = {
    ARM_BOARD_LINK_J1_MAX_RAD, ARM_BOARD_LINK_J2_MAX_RAD, ARM_BOARD_LINK_J3_MAX_RAD, ARM_BOARD_LINK_J4_MAX_RAD,
    ARM_BOARD_LINK_J5_MAX_RAD, ARM_BOARD_LINK_J6_MAX_RAD, ARM_BOARD_LINK_J7_MAX_RAD,
};

/**
 * @brief 将板间位置值限制在接收端允许范围内。
 * @param value 输入位置，单位 rad。
 * @param min_value 输入下限，单位 rad。
 * @param max_value 输入上限，单位 rad。
 * @return float 限幅后的值；NaN 或 Inf 返回 0。
 */
static float arm_link_clamp(float value, float min_value, float max_value)
{
    if (!isfinite(value)) return 0.0f;
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

/**
 * @brief 将遥控 CH1 原始值转换为夹爪速度命令。
 * @return float 左拨为闭合速度，右拨为释放速度，中位、死区或遥控离线为 0。
 * @note 只读取 arm_remote 快照，不直接访问 REMOTE 模块。
 */
static float arm_link_gripper_command(void)
{
    const int16_t gripper_raw = arm_remote_channel_get(ARM_REMOTE_GRIPPER_CHANNEL_INDEX);
    float         tool_value  = ARM_GRIPPER_STOP_COMMAND;

    if (arm_remote_is_online() != 0U)
    {
        if (gripper_raw < (ARM_REMOTE_GRIPPER_CENTER_RAW - ARM_REMOTE_GRIPPER_DEADBAND_RAW))
        {
            tool_value = ARM_GRIPPER_CLOSE_COMMAND;
        }
        else if (gripper_raw > (ARM_REMOTE_GRIPPER_CENTER_RAW + ARM_REMOTE_GRIPPER_DEADBAND_RAW))
        {
            tool_value = ARM_GRIPPER_RELEASE_COMMAND;
        }
    }

    return tool_value;
}

/**
 * @brief 将遥控数组 [6] 三档开关转换为板间控制状态位。
 * @return uint32_t 下档为完全跟随；上档、中档、离线或异常值均为保持当前姿态。
 * @note 返回值始终只置一个状态位，满足接收端两位互斥约定。
 */
static uint32_t arm_link_control_status(void)
{
    if (arm_remote_is_online() == 0U) return ARM_BOARD_LINK_STATUS_HOLD_CURRENT;

    const int16_t mode_raw = arm_remote_channel_get(ARM_REMOTE_AUX_CHANNEL_INDEX);
    if (mode_raw == ARM_REMOTE_AUX_DOWN_RAW) return ARM_BOARD_LINK_STATUS_FULL_FOLLOW;

    if (mode_raw == ARM_REMOTE_AUX_UP_RAW || mode_raw == ARM_REMOTE_AUX_MIDDLE_RAW)
    {
        return ARM_BOARD_LINK_STATUS_HOLD_CURRENT;
    }

    return ARM_BOARD_LINK_STATUS_HOLD_CURRENT;
}

/**
 * @brief 构造一帧活动状态下的 J1~J7 位置、夹爪速度及控制状态 payload。
 * @param payload 输出，容量至少为 ARM_BOARD_LINK_PAYLOAD_LEN 的字节缓冲区。
 * @retval 无。
 * @note 调用者持有 g_arm_link_payload_mutex；角度来源和发送坐标映射保持固定协议约定。
 */
static void arm_link_build_active_payload(uint8_t payload[BOARD_LINK_PAYLOAD_MAX])
{
    const volatile Arm_Feedback_Snapshot_t *feedback = arm_motor_feedback_get();

    for (uint32_t i = 0U; i < ARM_MOTOR_COUNT; ++i)
    {
        float value = feedback->joint[i].joint_angle_rad + g_arm_link_angle_offset_rad[i];

        if (i == ARM_JOINT_J2)
        {
            value = -value;
        }

        value = arm_link_clamp(value, g_arm_link_min_rad[i], g_arm_link_max_rad[i]);
        memcpy(&payload[i * sizeof(float)], &value, sizeof(value));
    }

    const float tool_value = arm_link_gripper_command();
    memcpy(&payload[ARM_BOARD_LINK_GRIPPER_FIELD_INDEX * sizeof(float)], &tool_value, sizeof(tool_value));

    const uint32_t control_status = arm_link_control_status();
    memcpy(&payload[ARM_BOARD_LINK_STATUS_OFFSET], &control_status, sizeof(control_status));
}

/**
 * @brief 在持锁状态下从最后活动帧构造一次夹爪零速停止帧。
 * @param 无。
 * @retval 无。
 * @note 仅在活动到停止的边沿置位待发送标志；J1~J7 保留最后位置。
 */
static void arm_link_deactivate_locked(void)
{
    if (g_arm_link_tx_enabled != 0U)
    {
        memcpy(g_arm_link_stop_payload, g_arm_link_payload, ARM_BOARD_LINK_PAYLOAD_LEN);
        const float tool_stop = ARM_GRIPPER_STOP_COMMAND;
        memcpy(&g_arm_link_stop_payload[ARM_BOARD_LINK_GRIPPER_FIELD_INDEX * sizeof(float)], &tool_stop, sizeof(tool_stop));
        g_arm_link_stop_pending = 1U;
    }
    g_arm_link_tx_enabled = 0U;
}

/**
 * @brief 发布控制周期对应的板间 payload 或切换到停止状态。
 * @param 无。
 * @retval 无。
 * @note 重力输出未激活时只排队一次夹爪零速帧，之后保持常规发送关闭。
 */
static void arm_link_publish_feedback(void)
{
    if (arm_gravity_output_active() == 0U)
    {
        if (tx_mutex_get(&g_arm_link_payload_mutex, TX_WAIT_FOREVER) == TX_SUCCESS)
        {
            arm_link_deactivate_locked();
            tx_mutex_put(&g_arm_link_payload_mutex);
        }
        else
        {
            g_arm_link_tx_enabled = 0U;
        }
        return;
    }

    if (tx_mutex_get(&g_arm_link_payload_mutex, TX_WAIT_FOREVER) != TX_SUCCESS)
    {
        g_arm_link_tx_enabled = 0U;
        return;
    }

    arm_link_build_active_payload(g_arm_link_payload);
    g_arm_link_tx_enabled = 1U;
    tx_mutex_put(&g_arm_link_payload_mutex);
}

/**
 * @brief 从共享状态取出本周期应发送的普通帧或停止帧。
 * @param stop_frame 输出，取到停止帧时写 1，普通帧或无帧时写 0。
 * @return uint8_t 已复制一帧到 g_arm_link_tx_payload 返回 1，无可发送帧返回 0。
 * @note 停止帧优先于重新启用后的普通帧，复制完成后立即释放互斥锁。
 */
static uint8_t arm_link_take_tx_payload(uint8_t *stop_frame)
{
    if (stop_frame == NULL) return 0U;
    *stop_frame = 0U;

    uint8_t send_ready = 0U;
    if (tx_mutex_get(&g_arm_link_payload_mutex, TX_WAIT_FOREVER) != TX_SUCCESS) return 0U;

    if (g_arm_link_stop_pending != 0U)
    {
        memcpy(g_arm_link_tx_payload, g_arm_link_stop_payload, ARM_BOARD_LINK_PAYLOAD_LEN);
        g_arm_link_stop_pending = 0U;
        send_ready              = 1U;
        *stop_frame             = 1U;
    }
    else if (g_arm_link_tx_enabled != 0U)
    {
        memcpy(g_arm_link_tx_payload, g_arm_link_payload, ARM_BOARD_LINK_PAYLOAD_LEN);
        send_ready = 1U;
    }

    tx_mutex_put(&g_arm_link_payload_mutex);
    return send_ready;
}

/**
 * @brief 在停止帧发送失败后重新置位待发送标志。
 * @param 无。
 * @retval 无。
 * @note 下一个发送周期仍优先尝试同一零速停止帧。
 */
static void arm_link_retry_stop_frame(void)
{
    if (tx_mutex_get(&g_arm_link_payload_mutex, TX_WAIT_FOREVER) != TX_SUCCESS) return;
    g_arm_link_stop_pending = 1U;
    tx_mutex_put(&g_arm_link_payload_mutex);
}

/**
 * @brief 板间 UART 周期发送线程入口。
 * @param thread_input ThreadX 线程入口参数，当前未使用。
 * @retval 无，线程创建后持续运行。
 * @note UART 阻塞等待始终发生在 payload 互斥锁之外。
 */
static void arm_link_tx_task(ULONG thread_input)
{
    (void)thread_input;

    while (1)
    {
        const ULONG cycle_start = tx_time_get();
        uint8_t     stop_frame  = 0U;
        uint8_t     send_ready  = arm_link_take_tx_payload(&stop_frame);

        if (send_ready != 0U && (stop_frame != 0U || g_arm_link_tx_enabled != 0U))
        {
            /* UART 等待发生在锁外，控制线程仍可继续刷新安全状态和快照。 */
            const uint8_t tx_success = board_link_tx(g_arm_link_tx_payload, (uint8_t)ARM_BOARD_LINK_PAYLOAD_LEN);
            if (stop_frame != 0U && tx_success == 0U) arm_link_retry_stop_frame();
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
        arm_link_publish_feedback();
        tx_thread_sleep(1);
    }
}

/**
 * @brief 初始化机械臂 APP 子模块并创建控制、发送和状态线程。
 * @param 无。
 * @retval 无。
 * @note 任一资源创建失败都会保持全部电机失能并停止后续初始化步骤。
 */
void robot_control_init(void)
{
    g_arm_link_tx_enabled   = 0U;
    g_arm_link_stop_pending = 0U;
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
