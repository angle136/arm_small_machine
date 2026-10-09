/*
 * 机械臂 APP 层静态配置。
 *
 * 本文件集中保存关节 CAN ID、软件零点、方向约定和安全限位参数。
 * 所有角度参数默认使用 rad，原始编码器值会在注释中显式标明。
 */
#ifndef _ARM_DEF_H_
#define _ARM_DEF_H_

#define ARM_MOTOR_COUNT                 7

/*
 * 临时输出力矩硬限幅，单位 N·m。
 *
 * 用于重力补偿探索阶段，防止试参时力矩过大导致机械臂快速运动或损伤结构。
 * 该值会写入 Motor_Base.info.max_torque，并由模块层在最终 Apply 前统一限幅。
 */
#define ARM_TEMP_OUTPUT_TORQUE_LIMIT_NM 5.0f

/* J2/J3/J4/J5 重力补偿输出配置；所有输出继续受硬限幅和角度安全限位保护。 */

#define ARM_J5_GRAVITY_OUTPUT_ENABLE    1U
#define ARM_J5_GRAVITY_COM              0.38f
#define ARM_J5_GRAVITY_TEST_SCALE       1.0f
#define ARM_J5_GRAVITY_OUTPUT_SIGN      (-1.0f)

#define ARM_J4_GRAVITY_OUTPUT_ENABLE    1U
#define ARM_J4_GRAVITY_COUPLE_SCALE     1.1f
#define ARM_J4_GRAVITY_TEST_SCALE       1.0f
#define ARM_J4_GRAVITY_OUTPUT_SIGN      (-1.0f)

#define ARM_J2_GRAVITY_OUTPUT_ENABLE    1U
#define ARM_J2_BASE_MASS                5.0f
#define ARM_J2_BASE_ARM_M              0.20f
#define ARM_J2_LINK_MASS               8.75f
#define ARM_J2_LINK_BASE_ARM_M         0.20f
#define ARM_J2_LINK_J3_ARM_M           0.08f
#define ARM_J2_DISTAL_MASS             4.5f
#define ARM_J2_DISTAL_BASE_ARM_M       0.20f
#define ARM_J2_DISTAL_J3_ARM_M         0.12f
#define ARM_J2_DISTAL_WRIST_ARM_M      0.08f
#define ARM_J2_GRAVITY_TEST_SCALE      1.0f
#define ARM_J2_GRAVITY_OUTPUT_SIGN      1.0f

#define ARM_J3_GRAVITY_OUTPUT_ENABLE    1U
#define ARM_J3_GRAVITY_COM              0.85f
#define ARM_J3_DISTAL_MASS              4.5f
#define ARM_J3_LINK_ARM_M               0.12f
#define ARM_J3_WRIST_ARM_M              0.08f
#define ARM_J3_GRAVITY_TEST_SCALE       1.0f
#define ARM_J3_GRAVITY_OUTPUT_SIGN      1.0f

/* J1/J6/J7 仅发送零力矩使能帧，用于解除失能状态下的电磁阻尼，不参与重力补偿。 */
#define ARM_J1_ZERO_TORQUE_ENABLE       1U
#define ARM_J6_ZERO_TORQUE_ENABLE       1U
#define ARM_J7_ZERO_TORQUE_ENABLE       1U

/* 达妙命令 ID / 反馈 ID，沿用旧机械臂的 CAN 分配。 */
#define ARM_J1_DM_TX_ID                 0x01U
#define ARM_J1_DM_RX_ID                 0x11U
#define ARM_J2_DM_TX_ID                 0x08U
#define ARM_J2_DM_RX_ID                 0x18U
#define ARM_J3_DM_TX_ID                 0x03U
#define ARM_J3_DM_RX_ID                 0x13U
#define ARM_J4_DJI_TX_ID                0x04U
#define ARM_J4_DJI_RX_ID                0x208U
#define ARM_J5_DM_TX_ID                 0x05U
#define ARM_J5_DM_RX_ID                 0x15U
#define ARM_J6_DM_TX_ID                 0x06U
#define ARM_J6_DM_RX_ID                 0x16U
#define ARM_J7_DM_TX_ID                 0x07U
#define ARM_J7_DM_RX_ID                 0x17U

/*
 * 机械臂竖直零点软件标定值。
 *
 * 标定姿态：整臂大致竖直，并将该姿态作为后续重力补偿的机械关节零点。
 * 使用范围：仅用于 APP 层将电机原始反馈换算为机械臂关节坐标，不写入电机内部零点。
 */
#define ARM_J1_ZERO_VALID               1U
#define ARM_J1_ZERO_RAW_RAD             (-1.836f)
#define ARM_J1_JOINT_DIR                1.0f

#define ARM_J2_ZERO_VALID               1U
#define ARM_J2_ZERO_RAW_RAD             (-0.995f)
#define ARM_J2_JOINT_DIR                (-1.0f)

#define ARM_J3_ZERO_VALID               1U
#define ARM_J3_ZERO_RAW_RAD             3.183f
#define ARM_J3_JOINT_DIR                1.0f

#define ARM_J4_ZERO_VALID               1U
#define ARM_J4_ZERO_ECD                 1320U
#define ARM_J4_ZERO_RAW_RAD             1.01242732f
#define ARM_J4_JOINT_DIR                1.0f

#define ARM_J5_ZERO_VALID               1U
#define ARM_J5_ZERO_RAW_RAD             0.040f
#define ARM_J5_JOINT_DIR                (-1.0f)

#define ARM_J6_ZERO_VALID               1U
#define ARM_J6_ZERO_RAW_RAD             1.086f
#define ARM_J6_JOINT_DIR                1.0f

#define ARM_J7_ZERO_VALID               1U
#define ARM_J7_ZERO_RAW_RAD             0.066f
/*
 * J7 方向说明：
 * 老工程曾使用 -dm35077.pos；当前工程按实机观察统一末端同类转轴方向，
 * 因此 J7 改为 raw 增大时 q 增大。后续移植老工程中涉及 J7 的逻辑时，
 * 需要显式确认是否仍要套用旧符号。
 */
#define ARM_J7_JOINT_DIR                1.0f

/*
 * Raw motor-angle limits in rad, before joint-zero conversion.
 * J4/J6 use accumulated angles; the other joints use single/protocol angles.
 *
 * Limits below are measured with collision clearance after zero calibration.
 */
#define ARM_J1_LIMIT_ENABLE             1U
#define ARM_J1_ANGLE_MIN                (-5.80f)
#define ARM_J1_ANGLE_MAX                (2.19f)

#define ARM_J2_LIMIT_ENABLE             1U
#define ARM_J2_ANGLE_MIN                (-2.90f)
#define ARM_J2_ANGLE_MAX                (0.92f)

#define ARM_J3_LIMIT_ENABLE             1U
#define ARM_J3_ANGLE_MIN                (0.14f)
#define ARM_J3_ANGLE_MAX                (5.40f)

#define ARM_J4_LIMIT_ENABLE             1U
#define ARM_J4_ANGLE_MIN                (-5.865f)
#define ARM_J4_ANGLE_MAX                (5.86f)

#define ARM_J5_LIMIT_ENABLE             1U
#define ARM_J5_ANGLE_MIN                (-1.57f)
#define ARM_J5_ANGLE_MAX                (1.55f)

#define ARM_J6_LIMIT_ENABLE             1U
#define ARM_J6_ANGLE_MIN                (-5.23f)
#define ARM_J6_ANGLE_MAX                (7.408f)

#define ARM_J7_LIMIT_ENABLE             1U
#define ARM_J7_ANGLE_MIN                (-2.068f)
#define ARM_J7_ANGLE_MAX                (2.09f)

#endif /* _ARM_DEF_H_ */
