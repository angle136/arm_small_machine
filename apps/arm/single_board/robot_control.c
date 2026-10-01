#include "robot_control.h"

#include "arm_def.h"
#include "bsp_can.h"
#include "bsp_can_task.h"
#include "bsp_def.h"
#include "module_offline.h"
#include "motor_damiao.h"
#include "motor_dji.h"
#include "motor_def.h"
#include "tx_api.h"
#include <string.h>

#define LOG_TAG "app_arm"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

typedef struct
{
    const char *name;
    CAN_HandleTypeDef *hcan;
    uint32_t tx_id;
    uint32_t rx_id;
    Motor_Type_e type;
    Motor_Safety_Limit_s safety;
} Arm_Dm_Descriptor;

#define ARM_RAW_LIMIT_CONFIG(enable_, min_, max_) \
    {                                             \
        .enable           = (enable_),            \
        .use_raw_position = 1U,                   \
        .use_angle        = 0U,                   \
        .fatal_on_limit   = 1U,                   \
        .raw_min          = (min_),               \
        .raw_max          = (max_),               \
        .angle_min_rad    = 0.0f,                 \
        .angle_max_rad    = 0.0f,                 \
    }

static DM_Motor_t *g_dm_motors[6];
static DJI_Motor_t *g_j4_motor;
static const char *const g_dm_joint_names[6] = {"J1", "J2", "J3", "J5", "J6", "J7"};
static const uint8_t g_dm_snapshot_index[6] = {0, 1, 2, 4, 5, 6};
static TX_THREAD g_arm_status_thread;
static TX_THREAD g_arm_feedback_thread;
APPS_STACK_SECTION static uint8_t g_arm_status_stack[1536];
APPS_STACK_SECTION static uint8_t g_arm_feedback_stack[1024];

volatile Arm_Feedback_Snapshot_t g_arm_feedback_snapshot;

/**
 * @brief 根据机械臂达妙关节描述生成通用电机初始化配置。
 *
 * @param desc 输入，达妙关节静态描述指针，包含名称、CAN 句柄、发送/接收 ID 和电机型号。
 *
 * @return Motor_Init_Config_s 已填充的电机初始化配置结构体。
 *
 * @note 调用关系：由 arm_register_dm() 调用；本函数仅构造配置，不注册设备、不启动电机。
 */
static Motor_Init_Config_s arm_dm_config(const Arm_Dm_Descriptor *desc)
{
    Motor_Init_Config_s config = {0};

    config.offline_init_config.name = desc->name;
    config.offline_init_config.timeout_ms = 500;
    config.offline_init_config.beep_times = 0;
    config.offline_init_config.enable = MOTOR_OFFLINE_ENABLE;
    config.transport = MOTOR_TRANSPORT_CAN;
    config.transport_config.can.hcan = desc->hcan;
    config.transport_config.can.tx_id = desc->tx_id;
    config.transport_config.can.rx_id = desc->rx_id;
    config.setting_init_config.algorithm_type = CONTROL_LQR;
    config.setting_init_config.loop_type = OPEN_LOOP;
    config.setting_init_config.enableflag = 0;
    config.motor_init_info.motor_type = desc->type;
    config.motor_init_info.gear_ratio = 1.0f;
    config.motor_init_info.max_torque = 0.0f;
    config.safety_limit_config = desc->safety;

    return config;
}

/**
 * @brief 注册单个达妙关节电机并保持输出失能。
 *
 * @param index 输入，达妙电机数组索引，范围为 0~5，对应 J1、J2、J3、J5、J6、J7。
 * @param desc  输入，达妙关节静态描述指针。
 *
 * @retval 无。
 *
 * @note 调用关系：由 robot_control_init() 调用；内部调用 arm_dm_config()、Motor_DM_Init() 和 Motor_DM_Stop()。
 * @note 设计目的：当前阶段只做硬件连通和反馈验证，注册后立即停止，避免重力补偿前误输出。
 */
static void arm_register_dm(size_t index, const Arm_Dm_Descriptor *desc)
{
    Motor_Init_Config_s config = arm_dm_config(desc);

    g_dm_motors[index] = Motor_DM_Init(&config, DM_MIT_MODE);
    if (g_dm_motors[index] == NULL)
    {
        LOG_E("%s registration failed", desc->name);
        return;
    }

    /* Keep the first bring-up passive: the DM driver emits only zero MIT frames. */
    Motor_Stop(&g_dm_motors[index]->base);
    LOG_I("registered %s CAN tx=0x%03lX rx=0x%03lX", desc->name,
          (unsigned long)desc->tx_id, (unsigned long)desc->rx_id);
}

/**
 * @brief 注册 J4 关节 GM6020 电机并保持输出失能。
 *
 * @param 无。
 *
 * @retval 无。
 *
 * @note 调用关系：由 robot_control_init() 调用；内部调用 Motor_DJI_Init() 和 Motor_DJI_Stop()。
 * @note CAN 配置：J4 使用 CAN1，发送 ID 为 ARM_J4_DJI_TX_ID，反馈 ID 由 DJI 驱动按电调 ID 解码。
 */
static void arm_register_j4(void)
{
    Motor_Init_Config_s config = {0};

    config.offline_init_config.name = "J4_GM6020";
    config.offline_init_config.timeout_ms = 500;
    config.offline_init_config.beep_times = 0;
    config.offline_init_config.enable = MOTOR_OFFLINE_ENABLE;
    config.transport = MOTOR_TRANSPORT_CAN;
    config.transport_config.can.hcan = BSP_CAN_HANDLE1;
    config.transport_config.can.tx_id = ARM_J4_DJI_TX_ID;
    config.setting_init_config.algorithm_type = CONTROL_LQR;
    config.setting_init_config.loop_type = OPEN_LOOP;
    config.setting_init_config.enableflag = 0;
    config.motor_init_info.motor_type = GM6020_CURRENT;
    config.motor_init_info.gear_ratio = 1.0f;
    config.motor_init_info.torque_constant = 0.741f;
    config.motor_init_info.max_torque = 0.0f;
    config.safety_limit_config =
        (Motor_Safety_Limit_s)ARM_RAW_LIMIT_CONFIG(ARM_J4_LIMIT_ENABLE, ARM_J4_RAW_MIN, ARM_J4_RAW_MAX);

    g_j4_motor = Motor_DJI_Init(&config);
    if (g_j4_motor == NULL)
    {
        LOG_E("J4_GM6020 registration failed");
        return;
    }

    Motor_Stop(&g_j4_motor->base);
    LOG_I("registered J4_GM6020 CAN tx=0x%03X rx=0x%03X", ARM_J4_DJI_TX_ID, ARM_J4_DJI_RX_ID);
}

/**
 * @brief 刷新机械臂全部关节的调试反馈快照。
 *
 * @param 无。
 *
 * @retval 无。
 *
 * @note 调用关系：由 arm_feedback_task() 周期调用；内部读取达妙/DJI 驱动缓存和 Module_Offline 在线状态。
 * @note 数据用途：供调试器 Watch、APP 层限位标定和后续安全逻辑读取；本函数不发送控制指令。
 */
void arm_feedback_snapshot_update(void)
{
    for (size_t i = 0; i < 6; ++i)
    {
        DM_Motor_t *motor = g_dm_motors[i];
        if (motor == NULL) continue;

        volatile Arm_Motor_Feedback_t *feedback =
            &g_arm_feedback_snapshot.joint[g_dm_snapshot_index[i]];
        Can_Device *can_dev = (Can_Device *)motor->base.transport_dev;

        feedback->valid          = (motor->measure.id != 0U);
        feedback->online         = (Module_Offline_get_device_status(motor->base.offline_dev) == STATE_ONLINE);
        feedback->enabled        = motor->base.setting.enableflag;
        feedback->feedback_id    = motor->measure.id;
        feedback->tx_id          = (can_dev != NULL) ? can_dev->tx_id : 0U;
        feedback->rx_id          = (can_dev != NULL) ? can_dev->rx_id : 0U;
        feedback->raw_position   = motor->measure.raw_position;
        feedback->raw_speed      = 0;
        feedback->raw_current    = 0;
        feedback->angle_raw_rad  = motor->base.measure.single_round_angle;
        feedback->angle_rad      = motor->base.measure.total_angle;
        feedback->speed_rad_s    = motor->base.measure.speed_rad;
        feedback->torque_nm      = motor->base.measure.torque_nm;
        feedback->temperature_1  = (uint8_t)motor->measure.T_Mos;
        feedback->temperature_2  = (uint8_t)motor->measure.T_Rotor;
    }

    if (g_j4_motor != NULL)
    {
        volatile Arm_Motor_Feedback_t *feedback = &g_arm_feedback_snapshot.joint[3];
        Can_Device *can_dev = (Can_Device *)g_j4_motor->base.transport_dev;

        feedback->online         = (Module_Offline_get_device_status(g_j4_motor->base.offline_dev) == STATE_ONLINE);
        feedback->valid          = (g_j4_motor->measure.ecd != 0U) || feedback->online;
        feedback->enabled        = g_j4_motor->base.setting.enableflag;
        feedback->feedback_id    = 0U;
        feedback->tx_id          = (can_dev != NULL) ? can_dev->tx_id : 0U;
        feedback->rx_id          = (can_dev != NULL) ? can_dev->rx_id : 0U;
        feedback->raw_position   = g_j4_motor->measure.ecd;
        feedback->raw_speed      = (int16_t)g_j4_motor->measure.speed_rpm;
        feedback->raw_current    = g_j4_motor->measure.real_current;
        feedback->angle_raw_rad  = g_j4_motor->base.measure.single_round_angle;
        feedback->angle_rad      = g_j4_motor->base.measure.total_angle;
        feedback->speed_rad_s    = g_j4_motor->base.measure.speed_rad;
        feedback->torque_nm      = g_j4_motor->base.measure.torque_nm;
        feedback->temperature_1  = g_j4_motor->measure.temperature;
        feedback->temperature_2  = 0U;
    }
}

/**
 * @brief 获取机械臂电机反馈快照只读指针。
 *
 * @param 无。
 *
 * @return const volatile Arm_Feedback_Snapshot_t* 指向 g_arm_feedback_snapshot 的只读 volatile 指针。
 *
 * @note 调用关系：供 APP 层读取；函数本身不加锁、不复制数据，适合调试观察和轻量状态读取。
 */
const volatile Arm_Feedback_Snapshot_t *arm_feedback_snapshot_get(void)
{
    return &g_arm_feedback_snapshot;
}

/**
 * @brief 打印机械臂各电机当前在线状态和核心反馈值。
 *
 * @param 无。
 *
 * @retval 无。
 *
 * @note 调用关系：由 arm_status_task() 每秒调用；内部只读取驱动缓存并通过 ulog 输出日志。
 */
static void arm_log_status(void)
{
    for (size_t i = 0; i < 6; ++i)
    {
        DM_Motor_t *motor = g_dm_motors[i];
        if (motor == NULL) continue;

        LOG_I("%s DM online=%u id=%u q=%.3f dq=%.3f tau=%.3f temp=%u/%u",
              g_dm_joint_names[i],
              (unsigned)(Module_Offline_get_device_status(motor->base.offline_dev) == STATE_ONLINE),
              (unsigned)motor->measure.id,
              motor->base.measure.total_angle,
              motor->base.measure.speed_rad,
              motor->base.measure.torque_nm,
              (unsigned)motor->measure.T_Mos,
              (unsigned)motor->measure.T_Rotor);
    }

    if (g_j4_motor != NULL)
    {
        LOG_I("J4 GM6020 online=%u ecd=%u speed=%.1f current=%d temp=%u",
              (unsigned)(Module_Offline_get_device_status(g_j4_motor->base.offline_dev) == STATE_ONLINE),
              (unsigned)g_j4_motor->measure.ecd,
              g_j4_motor->measure.speed_rpm,
              (int)g_j4_motor->measure.real_current,
              (unsigned)g_j4_motor->measure.temperature);
    }
}

/**
 * @brief 机械臂状态日志线程入口函数。
 *
 * @param thread_input 输入，ThreadX 线程入口参数，当前未使用。
 *
 * @retval 无，线程函数创建后不主动返回。
 *
 * @note 调用关系：由 robot_control_init() 通过 tx_thread_create() 创建；内部周期调用 arm_log_status()。
 */
static void arm_status_task(ULONG thread_input)
{
    (void)thread_input;

    while (1)
    {
        arm_log_status();
        tx_thread_sleep(1000);
    }
}

/**
 * @brief 机械臂反馈快照刷新线程入口函数。
 *
 * @param thread_input 输入，ThreadX 线程入口参数，当前未使用。
 *
 * @retval 无，线程函数创建后不主动返回。
 *
 * @note 调用关系：由 robot_control_init() 通过 tx_thread_create() 创建；内部周期调用 arm_feedback_snapshot_update()。
 * @note 周期说明：当前每 1 个 ThreadX tick 刷新一次，便于在线观察各关节原始值和角度变化。
 */
static void arm_feedback_task(ULONG thread_input)
{
    (void)thread_input;

    while (1)
    {
        arm_feedback_snapshot_update();
        tx_thread_sleep(1);
    }
}

/**
 * @brief 初始化机械臂 APP 控制模块。
 *
 * @param 无。
 *
 * @retval 无。
 *
 * @note 调用关系：由机器人类型入口调用；内部依次初始化 CAN 任务、注册 6 个达妙电机、注册 J4 GM6020、
 *       清零反馈快照，并创建反馈刷新线程和状态日志线程。
 * @note 安全策略：当前 bring-up 阶段只注册电机和读取反馈，所有电机注册后保持输出失能。
 */
void robot_control_init(void)
{
    static const Arm_Dm_Descriptor dm_descriptors[] = {
        {"J1_DM6220", BSP_CAN_HANDLE2, ARM_J1_DM_TX_ID, ARM_J1_DM_RX_ID, DM6220,
         ARM_RAW_LIMIT_CONFIG(ARM_J1_LIMIT_ENABLE, ARM_J1_RAW_MIN, ARM_J1_RAW_MAX)},
        {"J2_DM4340", BSP_CAN_HANDLE2, ARM_J2_DM_TX_ID, ARM_J2_DM_RX_ID, DM4340,
         ARM_RAW_LIMIT_CONFIG(ARM_J2_LIMIT_ENABLE, ARM_J2_RAW_MIN, ARM_J2_RAW_MAX)},
        {"J3_DM4310", BSP_CAN_HANDLE2, ARM_J3_DM_TX_ID, ARM_J3_DM_RX_ID, DM4310,
         ARM_RAW_LIMIT_CONFIG(ARM_J3_LIMIT_ENABLE, ARM_J3_RAW_MIN, ARM_J3_RAW_MAX)},
        {"J5_DM4310", BSP_CAN_HANDLE1, ARM_J5_DM_TX_ID, ARM_J5_DM_RX_ID, DM4310,
         ARM_RAW_LIMIT_CONFIG(ARM_J5_LIMIT_ENABLE, ARM_J5_RAW_MIN, ARM_J5_RAW_MAX)},
        {"J6_DM3507", BSP_CAN_HANDLE1, ARM_J6_DM_TX_ID, ARM_J6_DM_RX_ID, DM3507,
         ARM_RAW_LIMIT_CONFIG(ARM_J6_LIMIT_ENABLE, ARM_J6_RAW_MIN, ARM_J6_RAW_MAX)},
        {"J7_DM3507", BSP_CAN_HANDLE1, ARM_J7_DM_TX_ID, ARM_J7_DM_RX_ID, DM3507,
         ARM_RAW_LIMIT_CONFIG(ARM_J7_LIMIT_ENABLE, ARM_J7_RAW_MIN, ARM_J7_RAW_MAX)},
    };

    /* BSP_Init starts clocks and GPIO; CAN tasking must precede device registration. */
    BSP_CAN_TaskInit();

    for (size_t i = 0; i < 6; ++i)
    {
        arm_register_dm(i, &dm_descriptors[i]);
    }
    arm_register_j4();
    memset((void *)&g_arm_feedback_snapshot, 0, sizeof(g_arm_feedback_snapshot));

    UINT feedback_status =
        tx_thread_create(&g_arm_feedback_thread, "arm_feedback", arm_feedback_task, 0,
                         g_arm_feedback_stack, sizeof(g_arm_feedback_stack),
                         13, 13, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (feedback_status != TX_SUCCESS)
    {
        LOG_E("arm_feedback thread create failed: %u", (unsigned)feedback_status);
        return;
    }

    UINT status = tx_thread_create(&g_arm_status_thread, "arm_status", arm_status_task, 0,
                                   g_arm_status_stack, sizeof(g_arm_status_stack),
                                   20, 20, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
    {
        LOG_E("arm_status thread create failed: %u", (unsigned)status);
        return;
    }

    LOG_I("arm bring-up initialized: %u motors registered, outputs disabled", ARM_MOTOR_COUNT);
}
