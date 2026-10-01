#include "motor_base.h"
#include "bsp_can.h"
#include "bsp_dwt.h"
#include "motor_algorithm.h"

#define MOTOR_CAN_SLOT_GAP_S 0.0002f

static Motor_Base *g_motor_list = NULL;

/**
 * @brief 判断电机是否属于指定 CAN 总线分组。
 *
 * @param motor     输入，电机基类指针。
 * @param bus_index 输入，CAN 总线序号，当前约定 0=CAN1，1=CAN2，2=CAN3。
 *
 * @return uint8_t 返回 1 表示该电机使用指定 CAN 总线；返回 0 表示不属于该组或不是 CAN 电机。
 *
 * @note 调用关系：由 motor_find_next_can() 调用；本函数只读取 transport_dev，不发送报文。
 */
static uint8_t motor_is_can_bus(const Motor_Base *motor, uint8_t bus_index)
{
    if (motor == NULL || motor->transport != MOTOR_TRANSPORT_CAN || motor->transport_dev == NULL) return 0U;

    const Can_Device      *can_dev = (const Can_Device *)motor->transport_dev;
    const CAN_Bus_Manager *bus     = (const CAN_Bus_Manager *)can_dev->_bus;
    if (bus == NULL) return 0U;

    switch (bus_index)
    {
    case 0:
        return BSP_CAN_IS_HANDLE1(bus->hcan) ? 1U : 0U;
    case 1:
        return BSP_CAN_IS_HANDLE2(bus->hcan) ? 1U : 0U;
    case 2:
        return BSP_CAN_IS_HANDLE3(bus->hcan) ? 1U : 0U;
    default:
        return 0U;
    }
}

/**
 * @brief 从链表指定位置开始查找下一台属于指定 CAN 总线的电机。
 *
 * @param start     输入，查找起点指针。
 * @param bus_index 输入，CAN 总线序号，当前约定 0=CAN1，1=CAN2，2=CAN3。
 *
 * @return Motor_Base* 找到则返回电机基类指针；未找到则返回 NULL。
 *
 * @note 调用关系：由 Motor_ApplyAll() 的 CAN 槽位调度调用；本函数不发送报文。
 */
static Motor_Base *motor_find_next_can(Motor_Base *start, uint8_t bus_index)
{
    for (Motor_Base *motor = start; motor; motor = motor->next)
    {
        if (motor_is_can_bus(motor, bus_index)) return motor;
    }
    return NULL;
}

/**
 * @brief 注册电机对象到全局链表。
 *
 * @param motor 输入，需要注册的电机基类指针。
 *
 * @retval 无。
 *
 * @note 调用关系：由各电机驱动 Init 函数调用；注册后由 Motor_ControlAll() 和 Motor_ApplyAll() 统一调度。
 */
void Motor_Register(Motor_Base *motor)
{
    if (motor == NULL) return;
    motor->next  = g_motor_list;
    g_motor_list = motor;
}

/**
 * @brief 执行所有已注册电机的控制计算。
 *
 * @param 无。
 *
 * @retval 无。
 *
 * @note 调用关系：由 module_motor.c 中的 motor_task_entry() 周期调用；本阶段只计算输出，不发送报文。
 */
void Motor_ControlAll(void)
{
    for (Motor_Base *motor = g_motor_list; motor; motor = motor->next)
    {
        uint8_t offline = (motor->offline_dev != NULL && Module_Offline_get_device_status(motor->offline_dev) == STATE_OFFLINE);

        if (offline || motor->setting.enableflag == 0)
        {
            motor->controller.output           = 0;
            motor->controller.output_torque    = 0;
            motor->controller.speed_PID.Output = 0;
            motor->controller.speed_PID.Iout   = 0;
            motor->controller.angle_PID.Output = 0;
            motor->controller.angle_PID.Iout   = 0;
            continue;
        }

        /* 开环电机: 无控制计算 (Apply 阶段直通 ref) */
        if (motor->setting.loop_type == OPEN_LOOP) continue;

        motor->controller.output_torque = Motor_CalculateTorque(motor);
    }
}

/**
 * @brief 应用所有已注册电机的输出并触发传输层发送。
 *
 * @param 无。
 *
 * @retval 无。
 *
 * @note 调用关系：由 module_motor.c 中的 motor_task_entry() 在功率限制后调用。
 * @note 调度策略：按 CAN 槽位轮询发送；每个槽位内每条 CAN 总线最多应用一台电机，
 *       槽位之间等待 MOTOR_CAN_SLOT_GAP_S，复刻旧工程“CAN1/CAN2 可同片发送，同一 CAN 内部要隔开”的节奏。
 */
void Motor_ApplyAll(void)
{
    Motor_Base *can_cursor[BSP_CAN_BUS_NUM];
    for (uint8_t bus_index = 0; bus_index < BSP_CAN_BUS_NUM; ++bus_index)
    {
        can_cursor[bus_index] = g_motor_list;
    }

    while (1)
    {
        uint8_t sent_in_slot = 0U;

        for (uint8_t bus_index = 0; bus_index < BSP_CAN_BUS_NUM; ++bus_index)
        {
            Motor_Base *motor = motor_find_next_can(can_cursor[bus_index], bus_index);
            if (motor != NULL)
            {
                if (motor->Apply != NULL) motor->Apply(motor);
                can_cursor[bus_index] = motor->next;
                sent_in_slot          = 1U;
            }
        }

        if (sent_in_slot == 0U) break;
        BSP_DWT_Delay(MOTOR_CAN_SLOT_GAP_S);
    }

    for (Motor_Base *motor = g_motor_list; motor; motor = motor->next)
    {
        if (motor->transport != MOTOR_TRANSPORT_CAN && motor->Apply != NULL)
        {
            motor->Apply(motor);
        }
    }
}

void Motor_Start(Motor_Base *m)
{
    if (m == NULL) return;
    m->setting.enableflag = 1;
}

void Motor_Stop(Motor_Base *m)
{
    if (m == NULL) return;
    m->setting.enableflag = 0;
}

void Motor_SetRef(Motor_Base *m, float ref)
{
    if (m == NULL) return;
    m->controller.ref = ref;
}

void Motor_ChangeFeed(Motor_Base *m, Closeloop_Type_e loop, uint8_t feedback_source)
{
    if (m == NULL) return;
    if (loop == ANGLE_LOOP)
        m->setting.angle_feedback_source = feedback_source;
    else if (loop == SPEED_LOOP)
        m->setting.speed_feedback_source = feedback_source;
}

void Motor_OuterLoop(Motor_Base *m, Closeloop_Type_e outer_loop)
{
    if (m == NULL) return;
    m->setting.loop_type = outer_loop;
}

void Motor_SetForwardTorque(Motor_Base *m, float torque)
{
    if (m == NULL) return;
    m->controller.feedforward_torque = torque;
}

void Motor_SetOutputTorque(Motor_Base *m, float torque)
{
    if (m == NULL) return;
    m->controller.output_torque = torque;
}
