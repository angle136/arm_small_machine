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
#include <math.h>
#include <string.h>

#define LOG_TAG "app_arm"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

#define ARM_PI_RAD     3.14159265354f
#define ARM_TWO_PI_RAD 6.28318530708f

typedef struct
{
    const char *name;
    CAN_HandleTypeDef *hcan;
    uint32_t tx_id;
    uint32_t rx_id;
    Motor_Type_e type;
    Motor_Safety_Limit_s safety;
} Arm_Dm_Descriptor;

#define ARM_ANGLE_LIMIT_CONFIG(enable_, source_, min_, max_) \
    {                                                       \
        .enable           = (enable_),                      \
        .use_raw_position = 0U,                             \
        .fatal_on_limit   = 1U,                             \
        .raw_min          = 0,                               \
        .raw_max          = 0,                               \
        .angle_source     = (source_),                      \
        .angle_min_rad    = (min_),                          \
        .angle_max_rad    = (max_),                          \
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
 * @brief 判断电机是否已经收到有效反馈且未被离线检测判定为超时。
 *
 * @param base 输入，电机基类指针。
 *
 * @return uint8_t 返回 1 表示当前反馈链路有效，返回 0 表示尚未收到有效反馈
 *         或已经离线。
 *
 * @note 调用关系：由反馈快照刷新函数和状态日志函数调用；本函数只读取状态，
 *       不访问 CAN、不改变电机使能状态。
 */
static uint8_t arm_feedback_online(const Motor_Base *base)
{
    if (base == NULL || base->feedback_valid == 0U || base->offline_dev == NULL) return 0U;
    return (Module_Offline_get_device_status(base->offline_dev) == STATE_ONLINE) ? 1U : 0U;
}

/**
 * @brief 将角度限制到 [-pi, pi]，用于单圈编码器零点差值换算。
 *
 * @param angle_rad 输入，待归一化角度，单位 rad。
 *
 * @return float 归一化后的角度，单位 rad。
 *
 * @note 调用关系：由 arm_joint_angle_from_raw() 调用；函数不读取硬件、不修改全局状态。
 */
static float arm_wrap_to_pi(float angle_rad)
{
    while (angle_rad > ARM_PI_RAD)
    {
        angle_rad -= ARM_TWO_PI_RAD;
    }

    while (angle_rad < -ARM_PI_RAD)
    {
        angle_rad += ARM_TWO_PI_RAD;
    }

    return angle_rad;
}

/**
 * @brief 将电机原始反馈角换算为竖直零点下的机械关节角。
 *
 * @param joint_index 输入，关节索引，0~6 分别对应 J1~J7。
 * @param raw_rad     输入，驱动层单圈/协议角度，单位 rad。
 *
 * @return float 标定后的机械关节角，单位 rad；竖直零点附近返回 0。
 *
 * @note 调用关系：由 arm_feedback_snapshot_update() 调用，用于调试和后续重力补偿。
 * @note 安全说明：本函数只做坐标换算，不参与电机使能、不下发力矩。
 */
static float arm_joint_angle_from_raw(uint8_t joint_index, float raw_rad)
{
    switch (joint_index)
    {
    case 0:
        return (ARM_J1_ZERO_VALID != 0U) ? ARM_J1_JOINT_DIR * (raw_rad - ARM_J1_ZERO_RAW_RAD) : 0.0f;
    case 1:
        return ARM_J2_JOINT_DIR * (raw_rad - ARM_J2_ZERO_RAW_RAD);
    case 2:
        return ARM_J3_JOINT_DIR * (raw_rad - ARM_J3_ZERO_RAW_RAD);
    case 3:
        return ARM_J4_JOINT_DIR * arm_wrap_to_pi(raw_rad - ARM_J4_ZERO_RAW_RAD);
    case 4:
        return ARM_J5_JOINT_DIR * (raw_rad - ARM_J5_ZERO_RAW_RAD);
    case 5:
        return ARM_J6_JOINT_DIR * (raw_rad - ARM_J6_ZERO_RAW_RAD);
    case 6:
        return ARM_J7_JOINT_DIR * (raw_rad - ARM_J7_ZERO_RAW_RAD);
    default:
        return 0.0f;
    }
}

/**
 * @brief 按机械关节方向修正角速度符号。
 *
 * @param joint_index 输入，关节索引，0~6 分别对应 J1~J7。
 * @param speed_rad_s 输入，驱动层角速度，单位 rad/s。
 *
 * @return float 机械关节坐标系下的角速度，单位 rad/s。
 *
 * @note 调用关系：由 arm_feedback_snapshot_update() 调用；方向宏完成验证前均保持 +1。
 */
static float arm_joint_speed_from_raw(uint8_t joint_index, float speed_rad_s)
{
    switch (joint_index)
    {
    case 0:
        return (ARM_J1_ZERO_VALID != 0U) ? ARM_J1_JOINT_DIR * speed_rad_s : 0.0f;
    case 1:
        return ARM_J2_JOINT_DIR * speed_rad_s;
    case 2:
        return ARM_J3_JOINT_DIR * speed_rad_s;
    case 3:
        return ARM_J4_JOINT_DIR * speed_rad_s;
    case 4:
        return ARM_J5_JOINT_DIR * speed_rad_s;
    case 5:
        return ARM_J6_JOINT_DIR * speed_rad_s;
    case 6:
        return ARM_J7_JOINT_DIR * speed_rad_s;
    default:
        return 0.0f;
    }
}

/**
 * @brief 将竖直零点关节角换算为老工程重力补偿公式使用的模型角。
 *
 * @param joint_index     输入，关节索引，0~6 分别对应 J1~J7。
 * @param joint_angle_rad 输入，竖直零点坐标系下的机械关节角，单位 rad。
 *
 * @return float 老工程模型坐标系下的关节角，单位 rad。
 *
 * @note 调用关系：由 arm_feedback_snapshot_update() 调用；当前仅 J3 在竖直姿态下补偿 pi，
 *       使 sin(J2 + J3) 等旧公式项与老工程坐标习惯保持一致。
 */
static float arm_model_angle_from_joint(uint8_t joint_index, float joint_angle_rad)
{
    switch (joint_index)
    {
    case 2:
        return joint_angle_rad + ARM_PI_RAD;
    default:
        return joint_angle_rad;
    }
}

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
    config.motor_init_info.max_torque = ARM_TEMP_OUTPUT_TORQUE_LIMIT_NM;
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
    config.motor_init_info.max_torque = ARM_TEMP_OUTPUT_TORQUE_LIMIT_NM;
    config.safety_limit_config = (Motor_Safety_Limit_s)ARM_ANGLE_LIMIT_CONFIG(
        ARM_J4_LIMIT_ENABLE, MOTOR_SAFETY_ANGLE_TOTAL, ARM_J4_ANGLE_MIN, ARM_J4_ANGLE_MAX);

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
        float joint_angle_rad =
            arm_joint_angle_from_raw(g_dm_snapshot_index[i], motor->base.measure.single_round_angle);
        float joint_speed_rad_s =
            arm_joint_speed_from_raw(g_dm_snapshot_index[i], motor->base.measure.speed_rad);

        feedback->valid          = motor->base.feedback_valid;
        feedback->online         = arm_feedback_online(&motor->base);
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
        feedback->joint_angle_rad = joint_angle_rad;
        feedback->joint_speed_rad_s = joint_speed_rad_s;
        feedback->model_angle_rad = arm_model_angle_from_joint(g_dm_snapshot_index[i], joint_angle_rad);
        feedback->model_speed_rad_s = joint_speed_rad_s;
        feedback->temperature_1  = (uint8_t)motor->measure.T_Mos;
        feedback->temperature_2  = (uint8_t)motor->measure.T_Rotor;
    }

    if (g_j4_motor != NULL)
    {
        volatile Arm_Motor_Feedback_t *feedback = &g_arm_feedback_snapshot.joint[3];
        Can_Device *can_dev = (Can_Device *)g_j4_motor->base.transport_dev;
        float joint_angle_rad =
            arm_joint_angle_from_raw(3U, g_j4_motor->base.measure.single_round_angle);
        float joint_speed_rad_s =
            arm_joint_speed_from_raw(3U, g_j4_motor->base.measure.speed_rad);

        feedback->online         = arm_feedback_online(&g_j4_motor->base);
        feedback->valid          = g_j4_motor->base.feedback_valid;
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
        feedback->joint_angle_rad = joint_angle_rad;
        feedback->joint_speed_rad_s = joint_speed_rad_s;
        feedback->model_angle_rad = arm_model_angle_from_joint(3U, joint_angle_rad);
        feedback->model_speed_rad_s = joint_speed_rad_s;
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

        float raw_angle = motor->base.measure.single_round_angle;
        float joint_angle = arm_joint_angle_from_raw(g_dm_snapshot_index[i], raw_angle);
        float joint_speed = arm_joint_speed_from_raw(g_dm_snapshot_index[i], motor->base.measure.speed_rad);

        LOG_I("%s DM on=%u v=%u en=%u id=%u raw=%.3f q=%.3f dq=%.3f tau=%.3f T=%u/%u",
              g_dm_joint_names[i],
              (unsigned)arm_feedback_online(&motor->base),
              (unsigned)motor->base.feedback_valid,
              (unsigned)motor->base.setting.enableflag,
              (unsigned)motor->measure.id,
              raw_angle,
              joint_angle,
              joint_speed,
              motor->base.measure.torque_nm,
              (unsigned)motor->measure.T_Mos,
              (unsigned)motor->measure.T_Rotor);
    }

    if (g_j4_motor != NULL)
    {
        float raw_angle = g_j4_motor->base.measure.single_round_angle;
        float joint_angle = arm_joint_angle_from_raw(3U, raw_angle);
        float joint_speed = arm_joint_speed_from_raw(3U, g_j4_motor->base.measure.speed_rad);

        LOG_I("J4 GM6020 on=%u v=%u en=%u ecd=%u raw=%.3f q=%.3f dq=%.3f rpm=%.1f cur=%d T=%u",
              (unsigned)arm_feedback_online(&g_j4_motor->base),
              (unsigned)g_j4_motor->base.feedback_valid,
              (unsigned)g_j4_motor->base.setting.enableflag,
              (unsigned)g_j4_motor->measure.ecd,
              raw_angle,
              joint_angle,
              joint_speed,
              g_j4_motor->measure.speed_rpm,
              (int)g_j4_motor->measure.real_current,
              (unsigned)g_j4_motor->measure.temperature);
    }
}

/**
 * @brief 判断 J2/J3/J4/J5 是否具备 J5 重力补偿计算所需的有效反馈。
 *
 * @param 无。
 *
 * @return uint8_t 返回 1 表示反馈有效且在线，返回 0 表示至少一个相关关节不可用。
 *
 * @note 调用关系：由 J5 重力补偿 dry-run 打印和输出更新函数调用；本函数只读取快照状态。
 */
static uint8_t arm_j5_gravity_feedback_ready(void)
{
    const volatile Arm_Feedback_Snapshot_t *fb = arm_feedback_snapshot_get();

    const volatile Arm_Motor_Feedback_t *j2 = &fb->joint[1];
    const volatile Arm_Motor_Feedback_t *j3 = &fb->joint[2];
    const volatile Arm_Motor_Feedback_t *j4 = &fb->joint[3];
    const volatile Arm_Motor_Feedback_t *j5 = &fb->joint[4];

    return (j2->online != 0U && j2->valid != 0U &&
            j3->online != 0U && j3->valid != 0U &&
            j4->online != 0U && j4->valid != 0U &&
            j5->online != 0U && j5->valid != 0U) ? 1U : 0U;
}

/**
 * @brief 计算 J5 单关节旧工程重力补偿力矩。
 *
 * @param q2_out  输出，可为 NULL；返回 J2 模型角，单位 rad。
 * @param q3_out  输出，可为 NULL；返回 J3 模型角，单位 rad。
 * @param q4_out  输出，可为 NULL；返回 J4 模型角，单位 rad。
 * @param q5_out  输出，可为 NULL；返回 J5 模型角，单位 rad。
 *
 * @return float J5 重力补偿力矩估计值，单位 N·m。
 *
 * @note 调用关系：由 dry-run 日志和 J5 输出更新函数调用；只读反馈快照，不改变电机状态。
 */
static float arm_calc_j5_gravity_torque(float *q2_out, float *q3_out, float *q4_out, float *q5_out)
{
    const volatile Arm_Feedback_Snapshot_t *fb = arm_feedback_snapshot_get();
    const volatile Arm_Motor_Feedback_t *j2 = &fb->joint[1];
    const volatile Arm_Motor_Feedback_t *j3 = &fb->joint[2];
    const volatile Arm_Motor_Feedback_t *j4 = &fb->joint[3];
    const volatile Arm_Motor_Feedback_t *j5 = &fb->joint[4];

    float q2 = j2->model_angle_rad;
    float q3 = j3->model_angle_rad;
    float q4 = j4->model_angle_rad;
    float q5 = j5->model_angle_rad;

    if (q2_out != NULL) *q2_out = q2;
    if (q3_out != NULL) *q3_out = q3;
    if (q4_out != NULL) *q4_out = q4;
    if (q5_out != NULL) *q5_out = q5;

    return ARM_J5_GRAVITY_COM * sinf(q5 + q2 + q3) * cosf(q4);
}

#if (ARM_J5_GRAVITY_LOG_ENABLE != 0U)
/**
 * @brief 打印 J5 单关节重力补偿 dry-run 计算结果。
 *
 * @param 无。
 *
 * @retval 无。
 *
 * @note 调用关系：由 arm_status_task() 每秒调用；只读取反馈快照并计算旧工程 J5 重力项。
 * @note 安全说明：本函数不调用 Motor_Start()，不调用 Motor_SetOutputTorque()，不改变任何电机输出。
 */
static void arm_log_j5_gravity_test(void)
{
    if (arm_j5_gravity_feedback_ready() == 0U)
    {
        LOG_W("gc J5 skip: feedback not ready");
        return;
    }

    float q2 = 0.0f;
    float q3 = 0.0f;
    float q4 = 0.0f;
    float q5 = 0.0f;
    float tau_g = arm_calc_j5_gravity_torque(&q2, &q3, &q4, &q5);
    float tau_test = ARM_J5_GRAVITY_TEST_SCALE * tau_g;

    LOG_I("gc J5 q2=%.3f q3=%.3f q4=%.3f q5=%.3f tau=%.3f cmd%.0f%%=%.3f",
          q2,
          q3,
          q4,
          q5,
          tau_g,
          ARM_J5_GRAVITY_TEST_SCALE * 100.0f,
          tau_test);
}
#endif /* ARM_J5_GRAVITY_LOG_ENABLE */

/**
 * @brief 按配置更新 J5 单关节重力补偿输出。
 *
 * @param 无。
 *
 * @retval 无。
 *
 * @note 调用关系：由 arm_feedback_task() 在刷新反馈快照后周期调用。
 * @note 安全说明：默认 ARM_J5_GRAVITY_OUTPUT_ENABLE 为 0，本函数不输出；打开后只使能 J5，
 *       按 ARM_J5_GRAVITY_TEST_SCALE 小比例输出，并继续受底层 max_torque 硬限幅保护。
 */
static void arm_j5_gravity_output_update(void)
{
#if (ARM_J5_GRAVITY_OUTPUT_ENABLE != 0U)
    DM_Motor_t *j5_motor = g_dm_motors[3];
    if (j5_motor == NULL) return;

    if (arm_j5_gravity_feedback_ready() == 0U)
    {
        Motor_Stop(&j5_motor->base);
        Motor_SetOutputTorque(&j5_motor->base, 0.0f);
        return;
    }

    float tau_g = arm_calc_j5_gravity_torque(NULL, NULL, NULL, NULL);
    float tau_cmd = ARM_J5_GRAVITY_OUTPUT_SIGN * ARM_J5_GRAVITY_TEST_SCALE * tau_g;

    Motor_Start(&j5_motor->base);
    Motor_SetOutputTorque(&j5_motor->base, tau_cmd);
#endif /* ARM_J5_GRAVITY_OUTPUT_ENABLE */
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
#if (ARM_J5_GRAVITY_LOG_ENABLE != 0U)
        arm_log_j5_gravity_test();
#endif /* ARM_J5_GRAVITY_LOG_ENABLE */
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
        arm_j5_gravity_output_update();
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
        {"J1_DM4310", BSP_CAN_HANDLE2, ARM_J1_DM_TX_ID, ARM_J1_DM_RX_ID, DM4310,
         ARM_ANGLE_LIMIT_CONFIG(ARM_J1_LIMIT_ENABLE, MOTOR_SAFETY_ANGLE_DISABLED, ARM_J1_ANGLE_MIN, ARM_J1_ANGLE_MAX)},
        {"J2_DM4340", BSP_CAN_HANDLE2, ARM_J2_DM_TX_ID, ARM_J2_DM_RX_ID, DM4340,
         ARM_ANGLE_LIMIT_CONFIG(ARM_J2_LIMIT_ENABLE, MOTOR_SAFETY_ANGLE_SINGLE, ARM_J2_ANGLE_MIN, ARM_J2_ANGLE_MAX)},
        {"J3_DM4310", BSP_CAN_HANDLE2, ARM_J3_DM_TX_ID, ARM_J3_DM_RX_ID, DM4310,
         ARM_ANGLE_LIMIT_CONFIG(ARM_J3_LIMIT_ENABLE, MOTOR_SAFETY_ANGLE_SINGLE, ARM_J3_ANGLE_MIN, ARM_J3_ANGLE_MAX)},
        {"J5_DM4310", BSP_CAN_HANDLE1, ARM_J5_DM_TX_ID, ARM_J5_DM_RX_ID, DM4310,
         ARM_ANGLE_LIMIT_CONFIG(ARM_J5_LIMIT_ENABLE, MOTOR_SAFETY_ANGLE_SINGLE, ARM_J5_ANGLE_MIN, ARM_J5_ANGLE_MAX)},
        {"J6_DM3507", BSP_CAN_HANDLE1, ARM_J6_DM_TX_ID, ARM_J6_DM_RX_ID, DM3507,
         ARM_ANGLE_LIMIT_CONFIG(ARM_J6_LIMIT_ENABLE, MOTOR_SAFETY_ANGLE_TOTAL, ARM_J6_ANGLE_MIN, ARM_J6_ANGLE_MAX)},
        {"J7_DM3507", BSP_CAN_HANDLE1, ARM_J7_DM_TX_ID, ARM_J7_DM_RX_ID, DM3507,
         ARM_ANGLE_LIMIT_CONFIG(ARM_J7_LIMIT_ENABLE, MOTOR_SAFETY_ANGLE_SINGLE, ARM_J7_ANGLE_MIN, ARM_J7_ANGLE_MAX)},
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
