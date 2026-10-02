#ifndef _ARM_DEF_H_
#define _ARM_DEF_H_

#define ARM_MOTOR_COUNT 7

/* 达妙命令 ID / 反馈 ID，沿用旧机械臂的 CAN 分配。 */
#define ARM_J1_DM_TX_ID 0x01U
#define ARM_J1_DM_RX_ID 0x11U
#define ARM_J2_DM_TX_ID 0x02U
#define ARM_J2_DM_RX_ID 0x12U
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
#define ARM_J3_ANGLE_MIN    (0.2f)
#define ARM_J3_ANGLE_MAX    (5.53f)

#define ARM_J4_LIMIT_ENABLE 1U
#define ARM_J4_ANGLE_MIN    (-4.5f)
#define ARM_J4_ANGLE_MAX    (9.0f)

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
