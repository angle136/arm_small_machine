#ifndef _ARM_ROBOT_CONTROL_H_
#define _ARM_ROBOT_CONTROL_H_

/**
 * @brief 初始化机械臂 APP 控制入口。
 *
 * @param 无。
 *
 * @retval 无。
 *
 * @note 调用关系：由 APP_Init() 调用；依次初始化 CAN 调度、关节电机、SBUS 调试快照、
 *       重力补偿安全状态，最后创建控制线程和状态日志线程。
 * @note 安全说明：初始化不会直接开启输出；只有遥控器在线、CH7=240 且 J1~J7 反馈全部
 *       有效时，周期线程才允许重力补偿输出。
 */
void robot_control_init(void);

#endif /* _ARM_ROBOT_CONTROL_H_ */
