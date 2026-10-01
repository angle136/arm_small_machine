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
 * 机械臂关节底层 raw 限位配置。
 *
 * 当前还没有写入实测限位值，默认关闭；后续填入实测 raw_min/raw_max 并将对应 ENABLE 改为 1 后，
 * Motor_ApplyAll() 会在每次下发前做底层检查，任一关节超限会失能全部电机并触发蜂鸣报警。
 */
#define ARM_J1_LIMIT_ENABLE 0U
#define ARM_J1_RAW_MIN      0
#define ARM_J1_RAW_MAX      0

#define ARM_J2_LIMIT_ENABLE 0U
#define ARM_J2_RAW_MIN      0
#define ARM_J2_RAW_MAX      0

#define ARM_J3_LIMIT_ENABLE 0U
#define ARM_J3_RAW_MIN      0
#define ARM_J3_RAW_MAX      0

#define ARM_J4_LIMIT_ENABLE 0U
#define ARM_J4_RAW_MIN      0
#define ARM_J4_RAW_MAX      0

#define ARM_J5_LIMIT_ENABLE 0U
#define ARM_J5_RAW_MIN      0
#define ARM_J5_RAW_MAX      0

#define ARM_J6_LIMIT_ENABLE 0U
#define ARM_J6_RAW_MIN      0
#define ARM_J6_RAW_MAX      0

#define ARM_J7_LIMIT_ENABLE 0U
#define ARM_J7_RAW_MIN      0
#define ARM_J7_RAW_MAX      0

#endif /* _ARM_DEF_H_ */
