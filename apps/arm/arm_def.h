/*
 * 机械臂 APP 层静态配置。
 *
 * 本文件集中保存关节 CAN ID、软件零点、方向约定和安全限位参数。
 * 所有角度参数默认使用 rad，原始编码器值会在注释中显式标明。
 */
#ifndef _ARM_DEF_H_
#define _ARM_DEF_H_

#define ARM_MOTOR_COUNT 7

/*
 * 临时输出力矩硬限幅，单位 N·m。
 *
 * 用于重力补偿探索阶段，防止试参时力矩过大导致机械臂快速运动或损伤结构。
 * 该值会写入 Motor_Base.info.max_torque，并由模块层在最终 Apply 前统一限幅。
 */
#define ARM_TEMP_OUTPUT_TORQUE_LIMIT_NM 1.5f

/* 达妙命令 ID / 反馈 ID，沿用旧机械臂的 CAN 分配。 */
#define ARM_J1_DM_TX_ID 0x02U
#define ARM_J1_DM_RX_ID 0x12U
#define ARM_J2_DM_TX_ID 0x08U
#define ARM_J2_DM_RX_ID 0x18U
#define ARM_J3_DM_TX_ID 0x03U
#define ARM_J3_DM_RX_ID 0x13U
#define ARM_J4_DJI_TX_ID 0x04U
#define ARM_J4_DJI_RX_ID 0x208U
#define ARM_J5_DM_TX_ID 0x05U
#define ARM_J5_DM_RX_ID 0x15U
#define ARM_J6_DM_TX_ID 0x06U
#define ARM_J6_DM_RX_ID 0x16U
#define ARM_J7_DM_TX_ID 0x07U
#define ARM_J7_DM_RX_ID 0x17U

/*
 * 机械臂竖直零点软件标定值。
 *
 * 标定姿态：整臂大致竖直，并将该姿态作为后续重力补偿的机械关节零点。
 * 使用范围：仅用于 APP 层将电机原始反馈换算为机械臂关节坐标，不写入电机内部零点。
 */
#define ARM_J1_ZERO_VALID    0U
#define ARM_J1_ZERO_RAW_RAD  0.0f
#define ARM_J1_JOINT_DIR     1.0f

#define ARM_J2_ZERO_VALID    1U
#define ARM_J2_ZERO_RAW_RAD  (-0.039292f)
#define ARM_J2_JOINT_DIR     (-1.0f)

#define ARM_J3_ZERO_VALID    1U
#define ARM_J3_ZERO_RAW_RAD  3.104830f
#define ARM_J3_JOINT_DIR     1.0f

#define ARM_J4_ZERO_VALID    1U
#define ARM_J4_ZERO_ECD      1428U
#define ARM_J4_ZERO_RAW_RAD  1.094969f
#define ARM_J4_JOINT_DIR     1.0f

#define ARM_J5_ZERO_VALID    1U
#define ARM_J5_ZERO_RAW_RAD  0.022773f
#define ARM_J5_JOINT_DIR     (-1.0f)

#define ARM_J6_ZERO_VALID    1U
#define ARM_J6_ZERO_RAW_RAD  1.022191f
#define ARM_J6_JOINT_DIR     1.0f

#define ARM_J7_ZERO_VALID    1U
#define ARM_J7_ZERO_RAW_RAD  0.037774f
/*
 * J7 方向说明：
 * 老工程曾使用 -dm35077.pos；当前工程按实机观察统一末端同类转轴方向，
 * 因此 J7 改为 raw 增大时 q 增大。后续移植老工程中涉及 J7 的逻辑时，
 * 需要显式确认是否仍要套用旧符号。
 */
#define ARM_J7_JOINT_DIR     1.0f

/*
 * 机械臂关节角度限位配置，单位 rad。
 *
 * J4/J6 会跨圈，使用软件累计角度；其余关节不跨圈，使用当前反馈的单圈/协议角度。
 * J1 尚未提供实测限位，因此保持关闭；J2~J7 使用当前已提供的限位值。
 */
#define ARM_J1_LIMIT_ENABLE 0U
#define ARM_J1_ANGLE_MIN    0.0f
#define ARM_J1_ANGLE_MAX    0.0f

#define ARM_J2_LIMIT_ENABLE 1U
#define ARM_J2_ANGLE_MIN    (-1.9f)
#define ARM_J2_ANGLE_MAX    (1.8f)

#define ARM_J3_LIMIT_ENABLE 1U
#define ARM_J3_ANGLE_MIN    (0.08f)
#define ARM_J3_ANGLE_MAX    (6.0f)

#define ARM_J4_LIMIT_ENABLE 1U
#define ARM_J4_ANGLE_MIN    (-4.5f)
#define ARM_J4_ANGLE_MAX    (7.0f)

#define ARM_J5_LIMIT_ENABLE 1U
#define ARM_J5_ANGLE_MIN    (-1.6f)
#define ARM_J5_ANGLE_MAX    (1.6f)

#define ARM_J6_LIMIT_ENABLE 1U
#define ARM_J6_ANGLE_MIN    (-5.2f)
#define ARM_J6_ANGLE_MAX    (7.0f)

#define ARM_J7_LIMIT_ENABLE 1U
#define ARM_J7_ANGLE_MIN    (-2.0f)
#define ARM_J7_ANGLE_MAX    (2.0f)

#endif /* _ARM_DEF_H_ */
