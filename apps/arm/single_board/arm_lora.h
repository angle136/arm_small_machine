#ifndef _ARM_LORA_H_
#define _ARM_LORA_H_

/**
 * @brief 注册 J1~J7 与 tool 的 LoRa 发送变量并启动发送。
 *
 * @param 无。
 *
 * @retval 无；注册或启动失败时记录错误且保持发送未启动。
 *
 * @note 调用关系：由 robot_control_init() 在 LoRa 模块初始化完成后调用一次。
 * @note 时序要求：注册顺序固定为 J1、J2、J3、J4、J5、J6、J7、tool，启动后不得再注册。
 * @note 安全副作用：只启动无线数据发送，不改变电机使能、力矩或 CAN 调度。
 */
void arm_lora_init(void);

/**
 * @brief 刷新 LoRa 持久化发送数组。
 *
 * @param 无；内部读取最近一次 J1~J7 反馈快照的 joint_angle_rad。
 *
 * @retval 无。
 *
 * @note 调用关系：由 arm_control_task() 在 arm_motor_feedback_update() 后每 tick 调用。
 * @note 时序要求：只更新内存，实际串口发送由 LoRa 模块按 50 ms 周期执行。
 * @note 安全副作用：逐轴完成方向转换和接收端范围饱和，tool 固定写入 0.0f；不发送 CAN 命令。
 */
void arm_lora_update(void);

#endif /* _ARM_LORA_H_ */
