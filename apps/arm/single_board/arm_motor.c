#include "arm_motor.h"

#include "bsp_can.h"
#include "module_offline.h"
#include "motor_damiao.h"
#include "motor_def.h"
#include "motor_dji.h"
#include <stddef.h>
#include <string.h>

#define LOG_TAG "arm_motor"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

#define ARM_PI_RAD     3.14159265354f
#define ARM_TWO_PI_RAD 6.28318530708f

typedef struct
{
    const char          *name;
    CAN_HandleTypeDef   *hcan;
    uint32_t             tx_id;
    uint32_t             rx_id;
    Motor_Type_e         type;
    Motor_Safety_Limit_s safety;
} Arm_Dm_Descriptor_t;

#define ARM_ANGLE_LIMIT_CONFIG(enable_, source_, min_, max_)                                                                                         \
    {                                                                                                                                                \
        .enable           = (enable_),                                                                                                               \
        .use_raw_position = 0U,                                                                                                                      \
        .fatal_on_limit   = 1U,                                                                                                                      \
        .raw_min          = 0,                                                                                                                       \
        .raw_max          = 0,                                                                                                                       \
        .angle_source     = (source_),                                                                                                               \
        .angle_min_rad    = (min_),                                                                                                                  \
        .angle_max_rad    = (max_),                                                                                                                  \
    }

static DM_Motor_t       *g_dm_motors[6];
static DJI_Motor_t      *g_j4_motor;
static const char *const g_dm_joint_names[6]    = {"J1", "J2", "J3", "J5", "J6", "J7"};
static const uint8_t     g_dm_snapshot_index[6] = {0U, 1U, 2U, 4U, 5U, 6U};

volatile Arm_Feedback_Snapshot_t g_arm_feedback_snapshot;

/**
 * @brief 判断单个电机反馈是否有效且在线。
 * @param base 输入，电机基类指针。
 * @return uint8_t 有效且在线返回 1，否则返回 0。
 * @note 调用关系：由反馈刷新和日志函数调用；只读状态，无安全副作用。
 */
static uint8_t arm_motor_feedback_online(const Motor_Base *base)
{
    if (base == NULL || base->feedback_valid == 0U || base->offline_dev == NULL) return 0U;
    return (Module_Offline_get_device_status(base->offline_dev) == STATE_ONLINE) ? 1U : 0U;
}

/**
 * @brief 将角度限制到 [-pi, pi]。
 * @param angle_rad 输入，待归一化角度，单位 rad。
 * @return float 归一化后的角度，单位 rad。
 * @note 调用关系：由关节角换算调用；纯计算，无安全副作用。
 */
static float arm_motor_wrap_to_pi(float angle_rad)
{
    while (angle_rad > ARM_PI_RAD) angle_rad -= ARM_TWO_PI_RAD;
    while (angle_rad < -ARM_PI_RAD) angle_rad += ARM_TWO_PI_RAD;
    return angle_rad;
}

/**
 * @brief 将驱动层角度换算为机械关节角。
 * @param joint_index 输入，0~6 对应 J1~J7。
 * @param raw_rad 输入，驱动层角度，单位 rad。
 * @return float 软件零点和方向修正后的角度，单位 rad。
 * @note 调用关系：由反馈刷新和日志函数调用；纯计算，不下发控制命令。
 */
static float arm_motor_joint_angle(uint8_t joint_index, float raw_rad)
{
    switch (joint_index)
    {
    case 0U:
        return (ARM_J1_ZERO_VALID != 0U) ? ARM_J1_JOINT_DIR * (raw_rad - ARM_J1_ZERO_RAW_RAD) : 0.0f;
    case 1U:
        return ARM_J2_JOINT_DIR * (raw_rad - ARM_J2_ZERO_RAW_RAD);
    case 2U:
        return ARM_J3_JOINT_DIR * (raw_rad - ARM_J3_ZERO_RAW_RAD);
    case 3U:
        return ARM_J4_JOINT_DIR * arm_motor_wrap_to_pi(raw_rad - ARM_J4_ZERO_RAW_RAD);
    case 4U:
        return ARM_J5_JOINT_DIR * (raw_rad - ARM_J5_ZERO_RAW_RAD);
    case 5U:
        return ARM_J6_JOINT_DIR * (raw_rad - ARM_J6_ZERO_RAW_RAD);
    case 6U:
        return ARM_J7_JOINT_DIR * (raw_rad - ARM_J7_ZERO_RAW_RAD);
    default:
        return 0.0f;
    }
}

/**
 * @brief 按机械关节方向修正角速度。
 * @param joint_index 输入，0~6 对应 J1~J7。
 * @param speed_rad_s 输入，驱动层角速度，单位 rad/s。
 * @return float 机械关节坐标系角速度，单位 rad/s。
 * @note 调用关系：由反馈刷新和日志函数调用；纯计算，不下发控制命令。
 */
static float arm_motor_joint_speed(uint8_t joint_index, float speed_rad_s)
{
    switch (joint_index)
    {
    case 0U:
        return (ARM_J1_ZERO_VALID != 0U) ? ARM_J1_JOINT_DIR * speed_rad_s : 0.0f;
    case 1U:
        return ARM_J2_JOINT_DIR * speed_rad_s;
    case 2U:
        return ARM_J3_JOINT_DIR * speed_rad_s;
    case 3U:
        return ARM_J4_JOINT_DIR * speed_rad_s;
    case 4U:
        return ARM_J5_JOINT_DIR * speed_rad_s;
    case 5U:
        return ARM_J6_JOINT_DIR * speed_rad_s;
    case 6U:
        return ARM_J7_JOINT_DIR * speed_rad_s;
    default:
        return 0.0f;
    }
}

/**
 * @brief 将机械关节角换算为重力模型角。
 * @param joint_index 输入，0~6 对应 J1~J7。
 * @param joint_angle_rad 输入，机械关节角，单位 rad。
 * @return float 重力补偿模型角，单位 rad。
 * @note 调用关系：由反馈刷新调用；当前仅 J3 增加 pi，不产生输出副作用。
 */
static float arm_motor_model_angle(uint8_t joint_index, float joint_angle_rad)
{
    return (joint_index == 2U) ? (joint_angle_rad + ARM_PI_RAD) : joint_angle_rad;
}

/**
 * @brief 构造达妙电机注册配置。
 * @param desc 输入，关节静态描述。
 * @return Motor_Init_Config_s 完整注册配置。
 * @note 调用关系：由达妙注册函数调用；只构造配置，不注册或使能电机。
 */
static Motor_Init_Config_s arm_motor_dm_config(const Arm_Dm_Descriptor_t *desc)
{
    Motor_Init_Config_s config = {0};

    config.offline_init_config.name           = desc->name;
    config.offline_init_config.timeout_ms     = 500;
    config.offline_init_config.beep_times     = 0;
    config.offline_init_config.enable         = MOTOR_OFFLINE_ENABLE;
    config.transport                          = MOTOR_TRANSPORT_CAN;
    config.transport_config.can.hcan          = desc->hcan;
    config.transport_config.can.tx_id         = desc->tx_id;
    config.transport_config.can.rx_id         = desc->rx_id;
    config.setting_init_config.algorithm_type = CONTROL_LQR;
    config.setting_init_config.loop_type      = OPEN_LOOP;
    config.setting_init_config.enableflag     = 0;
    config.motor_init_info.motor_type         = desc->type;
    config.motor_init_info.gear_ratio         = 1.0f;
    config.motor_init_info.max_torque         = ARM_TEMP_OUTPUT_TORQUE_LIMIT_NM;
    config.safety_limit_config                = desc->safety;
    return config;
}

/**
 * @brief 注册单个达妙关节并保持失能。
 * @param index 输入，达妙数组索引 0~5。
 * @param desc 输入，关节静态描述。
 * @retval 无。
 * @note 调用关系：由 arm_motor_init() 调用；注册失败仅记录日志，成功后立即 Motor_Stop()。
 */
static void arm_motor_register_dm(size_t index, const Arm_Dm_Descriptor_t *desc)
{
    Motor_Init_Config_s config = arm_motor_dm_config(desc);
    g_dm_motors[index]         = Motor_DM_Init(&config, DM_MIT_MODE);
    if (g_dm_motors[index] == NULL)
    {
        LOG_E("%s registration failed", desc->name);
        return;
    }

    Motor_Stop(&g_dm_motors[index]->base);
    LOG_I("registered %s CAN tx=0x%03lX rx=0x%03lX", desc->name, (unsigned long)desc->tx_id, (unsigned long)desc->rx_id);
}

/**
 * @brief 注册 J4 GM6020 并保持失能。
 * @param 无。
 * @retval 无。
 * @note 调用关系：由 arm_motor_init() 调用；注册失败仅记录日志，成功后立即 Motor_Stop()。
 */
static void arm_motor_register_j4(void)
{
    Motor_Init_Config_s config = {0};

    config.offline_init_config.name           = "J4_GM6020";
    config.offline_init_config.timeout_ms     = 500;
    config.offline_init_config.beep_times     = 0;
    config.offline_init_config.enable         = MOTOR_OFFLINE_ENABLE;
    config.transport                          = MOTOR_TRANSPORT_CAN;
    config.transport_config.can.hcan          = BSP_CAN_HANDLE1;
    config.transport_config.can.tx_id         = ARM_J4_DJI_TX_ID;
    config.setting_init_config.algorithm_type = CONTROL_LQR;
    config.setting_init_config.loop_type      = OPEN_LOOP;
    config.setting_init_config.enableflag     = 0;
    config.motor_init_info.motor_type         = GM6020_CURRENT;
    config.motor_init_info.gear_ratio         = 1.0f;
    config.motor_init_info.torque_constant    = 0.741f;
    config.motor_init_info.max_torque         = ARM_TEMP_OUTPUT_TORQUE_LIMIT_NM;
    config.safety_limit_config =
        (Motor_Safety_Limit_s)ARM_ANGLE_LIMIT_CONFIG(ARM_J4_LIMIT_ENABLE, MOTOR_SAFETY_ANGLE_TOTAL, ARM_J4_ANGLE_MIN, ARM_J4_ANGLE_MAX);

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
 * @brief 按 J1~J7 索引获取电机基类指针。
 * @param joint_index 输入，0~6 对应 J1~J7。
 * @return Motor_Base* 已注册电机基类；索引非法或未注册返回 NULL。
 * @note 调用关系：由输出应用和全停函数调用；只做映射，不改变电机状态。
 */
static Motor_Base *arm_motor_base(uint8_t joint_index)
{
    if (joint_index == 3U) return (g_j4_motor != NULL) ? &g_j4_motor->base : NULL;
    if (joint_index < 3U) return (g_dm_motors[joint_index] != NULL) ? &g_dm_motors[joint_index]->base : NULL;
    if (joint_index < ARM_MOTOR_COUNT) return (g_dm_motors[joint_index - 1U] != NULL) ? &g_dm_motors[joint_index - 1U]->base : NULL;
    return NULL;
}

void arm_motor_init(void)
{
    static const Arm_Dm_Descriptor_t dm_descriptors[] = {
        {"J1_DM4310", BSP_CAN_HANDLE2, ARM_J1_DM_TX_ID, ARM_J1_DM_RX_ID, DM4310,
         ARM_ANGLE_LIMIT_CONFIG(ARM_J1_LIMIT_ENABLE, MOTOR_SAFETY_ANGLE_SINGLE, ARM_J1_ANGLE_MIN, ARM_J1_ANGLE_MAX)},
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

    memset(g_dm_motors, 0, sizeof(g_dm_motors));
    g_j4_motor = NULL;
    memset((void *)&g_arm_feedback_snapshot, 0, sizeof(g_arm_feedback_snapshot));

    for (size_t i = 0; i < 6U; ++i) arm_motor_register_dm(i, &dm_descriptors[i]);
    arm_motor_register_j4();
    arm_motor_stop_all();
}

void arm_motor_feedback_update(void)
{
    for (size_t i = 0; i < 6U; ++i)
    {
        DM_Motor_t *motor = g_dm_motors[i];
        if (motor == NULL) continue;

        uint8_t                        joint_index      = g_dm_snapshot_index[i];
        if (joint_index >= ARM_MOTOR_COUNT) continue;
        volatile Arm_Motor_Feedback_t *feedback         = &g_arm_feedback_snapshot.joint[joint_index];
        Can_Device                    *can_dev          = (Can_Device *)motor->base.transport_dev;
        float                          joint_angle_rad  = arm_motor_joint_angle(joint_index, motor->base.measure.single_round_angle);
        float                          joint_speed_rad_s = arm_motor_joint_speed(joint_index, motor->base.measure.speed_rad);

        feedback->valid             = motor->base.feedback_valid;
        feedback->online            = arm_motor_feedback_online(&motor->base);
        feedback->enabled           = motor->base.setting.enableflag;
        feedback->feedback_id       = motor->measure.id;
        feedback->tx_id             = (can_dev != NULL) ? can_dev->tx_id : 0U;
        feedback->rx_id             = (can_dev != NULL) ? can_dev->rx_id : 0U;
        feedback->raw_position      = motor->measure.raw_position;
        feedback->raw_speed         = 0;
        feedback->raw_current       = 0;
        feedback->angle_raw_rad     = motor->base.measure.single_round_angle;
        feedback->angle_rad         = motor->base.measure.total_angle;
        feedback->speed_rad_s       = motor->base.measure.speed_rad;
        feedback->torque_nm         = motor->base.measure.torque_nm;
        feedback->joint_angle_rad   = joint_angle_rad;
        feedback->joint_speed_rad_s = joint_speed_rad_s;
        feedback->model_angle_rad   = arm_motor_model_angle(joint_index, joint_angle_rad);
        feedback->model_speed_rad_s = joint_speed_rad_s;
        feedback->temperature_1     = (uint8_t)motor->measure.T_Mos;
        feedback->temperature_2     = (uint8_t)motor->measure.T_Rotor;
    }

    if (g_j4_motor != NULL)
    {
        volatile Arm_Motor_Feedback_t *feedback          = &g_arm_feedback_snapshot.joint[3];
        Can_Device                    *can_dev           = (Can_Device *)g_j4_motor->base.transport_dev;
        float                          joint_angle_rad   = arm_motor_joint_angle(3U, g_j4_motor->base.measure.single_round_angle);
        float                          joint_speed_rad_s = arm_motor_joint_speed(3U, g_j4_motor->base.measure.speed_rad);

        feedback->online            = arm_motor_feedback_online(&g_j4_motor->base);
        feedback->valid             = g_j4_motor->base.feedback_valid;
        feedback->enabled           = g_j4_motor->base.setting.enableflag;
        feedback->feedback_id       = 0U;
        feedback->tx_id             = (can_dev != NULL) ? can_dev->tx_id : 0U;
        feedback->rx_id             = (can_dev != NULL) ? can_dev->rx_id : 0U;
        feedback->raw_position      = g_j4_motor->measure.ecd;
        feedback->raw_speed         = (int16_t)g_j4_motor->measure.speed_rpm;
        feedback->raw_current       = g_j4_motor->measure.real_current;
        feedback->angle_raw_rad     = g_j4_motor->base.measure.single_round_angle;
        feedback->angle_rad         = g_j4_motor->base.measure.total_angle;
        feedback->speed_rad_s       = g_j4_motor->base.measure.speed_rad;
        feedback->torque_nm         = g_j4_motor->base.measure.torque_nm;
        feedback->joint_angle_rad   = joint_angle_rad;
        feedback->joint_speed_rad_s = joint_speed_rad_s;
        feedback->model_angle_rad   = arm_motor_model_angle(3U, joint_angle_rad);
        feedback->model_speed_rad_s = joint_speed_rad_s;
        feedback->temperature_1     = g_j4_motor->measure.temperature;
        feedback->temperature_2     = 0U;
    }
}

const volatile Arm_Feedback_Snapshot_t *arm_motor_feedback_get(void) { return &g_arm_feedback_snapshot; }

uint8_t arm_motor_all_feedback_ready(void)
{
    for (size_t i = 0; i < ARM_MOTOR_COUNT; ++i)
    {
        if (g_arm_feedback_snapshot.joint[i].online == 0U || g_arm_feedback_snapshot.joint[i].valid == 0U) return 0U;
    }
    return 1U;
}

void arm_motor_apply_outputs(const float torque_nm[ARM_MOTOR_COUNT], const uint8_t enable[ARM_MOTOR_COUNT])
{
    if (torque_nm == NULL || enable == NULL)
    {
        arm_motor_stop_all();
        return;
    }

    for (uint8_t i = 0U; i < ARM_MOTOR_COUNT; ++i)
    {
        Motor_Base *motor = arm_motor_base(i);
        if (motor == NULL) continue;

        if (enable[i] != 0U)
        {
            Motor_SetOutputTorque(motor, torque_nm[i]);
            Motor_Start(motor);
        }
        else
        {
            Motor_SetOutputTorque(motor, 0.0f);
            Motor_Stop(motor);
        }
    }
}

void arm_motor_stop_all(void)
{
    for (uint8_t i = 0U; i < ARM_MOTOR_COUNT; ++i)
    {
        Motor_Base *motor = arm_motor_base(i);
        if (motor == NULL) continue;
        Motor_SetOutputTorque(motor, 0.0f);
        Motor_Stop(motor);
    }
}

void arm_motor_log_status(void)
{
    for (size_t i = 0; i < 6U; ++i)
    {
        DM_Motor_t *motor = g_dm_motors[i];
        if (motor == NULL) continue;

        float raw_angle   = motor->base.measure.single_round_angle;
        float joint_angle = arm_motor_joint_angle(g_dm_snapshot_index[i], raw_angle);
        float joint_speed = arm_motor_joint_speed(g_dm_snapshot_index[i], motor->base.measure.speed_rad);

        LOG_I("%s DM on=%u v=%u en=%u hw_cmd=%u id=%u raw=%.3f q=%.3f dq=%.3f tau=%.3f T=%u/%u", g_dm_joint_names[i],
              (unsigned)arm_motor_feedback_online(&motor->base), (unsigned)motor->base.feedback_valid,
              (unsigned)motor->base.setting.enableflag, (unsigned)motor->hardware_start_sent, (unsigned)motor->measure.id, raw_angle, joint_angle,
              joint_speed, motor->base.measure.torque_nm, (unsigned)motor->measure.T_Mos, (unsigned)motor->measure.T_Rotor);
    }

    if (g_j4_motor != NULL)
    {
        float raw_angle   = g_j4_motor->base.measure.single_round_angle;
        float joint_angle = arm_motor_joint_angle(3U, raw_angle);
        float joint_speed = arm_motor_joint_speed(3U, g_j4_motor->base.measure.speed_rad);

        LOG_I("J4 GM6020 on=%u v=%u en=%u ecd=%u raw=%.3f q=%.3f dq=%.3f rpm=%.1f cur=%d T=%u",
              (unsigned)arm_motor_feedback_online(&g_j4_motor->base), (unsigned)g_j4_motor->base.feedback_valid,
              (unsigned)g_j4_motor->base.setting.enableflag, (unsigned)g_j4_motor->measure.ecd, raw_angle, joint_angle, joint_speed,
              g_j4_motor->measure.speed_rpm, (int)g_j4_motor->measure.real_current, (unsigned)g_j4_motor->measure.temperature);
    }
}
