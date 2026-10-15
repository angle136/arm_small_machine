#ifndef _ARM_NRF24L01_H_
#define _ARM_NRF24L01_H_

/**
 * @brief 注册 J1~J7 与 tool 的 NRF24L01 发送变量。
 *
 * @note 注册顺序固定为 J1、J2、J3、J4、J5、J6、J7、tool，共 32 字节。
 * @note 只注册无线发送数据，不改变电机使能、力矩或 CAN 调度。
 */
void arm_nrf24l01_init(void);

/**
 * @brief 刷新 NRF24L01 持久化发送数组。
 *
 * @note 由机械臂控制线程在反馈刷新后调用，实际发送由模块线程按配置周期执行。
 */
void arm_nrf24l01_update(void);

#endif /* _ARM_NRF24L01_H_ */
