#include "tx_api.h"
#include "tx_port.h"
#include "bsp_can.h"
#include "bsp_can_task.h"
#include "bsp_def.h"

#define LOG_TAG "bsp_can_task"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

#define BSP_CAN_RX_TASK_PRIORITY   3
#define BSP_CAN_TX_TASK_PRIORITY   4
#define BSP_CAN_TASK_STACK_SIZE    1024U

/* 临时总线监听：默认恢复 CAN1 的逐帧日志，调试结束后改为 0 即可关闭。 */
#define BSP_CAN_CAN1_RX_LOG_ENABLE 0U

TX_SEMAPHORE g_can_rx_sem;
TX_SEMAPHORE g_can_tx_sem;

static TX_THREAD                  g_can_rx_thread;
static TX_THREAD                  g_can_tx_thread;
APPS_STACK_SECTION static uint8_t g_can_rx_stack[BSP_CAN_TASK_STACK_SIZE];
APPS_STACK_SECTION static uint8_t g_can_tx_stack[BSP_CAN_TASK_STACK_SIZE];

#if BSP_CAN_CAN1_RX_LOG_ENABLE
static void can_log_can1_rx(const BSP_CanMsg_t *msg)
{
    if (msg == NULL) return;

    switch (msg->len)
    {
    case 0:
        LOG_I("CAN1 ID=0x%03lX IDE=Standard DLC=0 DATA=", (unsigned long)msg->id);
        break;
    case 1:
        LOG_I("CAN1 ID=0x%03lX IDE=Standard DLC=1 DATA=%02X", (unsigned long)msg->id, msg->data[0]);
        break;
    case 2:
        LOG_I("CAN1 ID=0x%03lX IDE=Standard DLC=2 DATA=%02X %02X", (unsigned long)msg->id, msg->data[0], msg->data[1]);
        break;
    case 3:
        LOG_I("CAN1 ID=0x%03lX IDE=Standard DLC=3 DATA=%02X %02X %02X", (unsigned long)msg->id, msg->data[0], msg->data[1], msg->data[2]);
        break;
    case 4:
        LOG_I("CAN1 ID=0x%03lX IDE=Standard DLC=4 DATA=%02X %02X %02X %02X", (unsigned long)msg->id, msg->data[0], msg->data[1], msg->data[2],
              msg->data[3]);
        break;
    case 5:
        LOG_I("CAN1 ID=0x%03lX IDE=Standard DLC=5 DATA=%02X %02X %02X %02X %02X", (unsigned long)msg->id, msg->data[0], msg->data[1], msg->data[2],
              msg->data[3], msg->data[4]);
        break;
    case 6:
        LOG_I("CAN1 ID=0x%03lX IDE=Standard DLC=6 DATA=%02X %02X %02X %02X %02X %02X", (unsigned long)msg->id, msg->data[0], msg->data[1],
              msg->data[2], msg->data[3], msg->data[4], msg->data[5]);
        break;
    case 7:
        LOG_I("CAN1 ID=0x%03lX IDE=Standard DLC=7 DATA=%02X %02X %02X %02X %02X %02X %02X", (unsigned long)msg->id, msg->data[0], msg->data[1],
              msg->data[2], msg->data[3], msg->data[4], msg->data[5], msg->data[6]);
        break;
    default:
        LOG_I("CAN1 ID=0x%03lX IDE=Standard DLC=8 DATA=%02X %02X %02X %02X %02X %02X %02X %02X %02X", (unsigned long)msg->id, msg->data[0],
              msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5], msg->data[6], msg->data[7]);
        break;
    }
}
#endif

/* RX 任务 */
static void can_rx_task_entry(ULONG arg)
{
    (void)arg;

    while (1)
    {
        tx_semaphore_get(&g_can_rx_sem, TX_WAIT_FOREVER);

        for (int i = 0; i < BSP_CAN_BUS_NUM; i++)
        {
            if (!g_can_bus[i].initialized) continue;

            BSP_CanMsg_t msg;
            while (kfifo_get(&g_can_bus[i].rx_fifo, &msg))
            {
#if BSP_CAN_CAN1_RX_LOG_ENABLE
                if (BSP_CAN_IS_HANDLE1(g_can_bus[i].hcan)) can_log_can1_rx(&msg);
#endif
                Can_Device *dev = can_dev_find(&g_can_bus[i], msg.id);
                if (dev != NULL && dev->rx_callback != NULL) dev->rx_callback(dev, msg.data, msg.len);
            }
        }
    }
}

/* TX 任务 */
static void can_tx_task_entry(ULONG arg)
{
    (void)arg;

    while (1)
    {
        tx_semaphore_get(&g_can_tx_sem, TX_WAIT_FOREVER);

        for (int i = 0; i < BSP_CAN_BUS_NUM; i++)
        {
            if (!g_can_bus[i].initialized) continue;

            BSP_CanMsg_t msg;
            while (kfifo_get(&g_can_bus[i].tx_fifo, &msg))
            {
#if defined(STM32H723xx)
                FDCAN_TxHeaderTypeDef TxHeader = {
                    .Identifier          = msg.id,
                    .IdType              = FDCAN_STANDARD_ID,
                    .TxFrameType         = FDCAN_DATA_FRAME,
                    .DataLength          = (msg.len > 8) ? FDCAN_DLC_BYTES_8 : msg.len,
                    .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
                    .BitRateSwitch       = FDCAN_BRS_OFF,
                    .FDFormat            = FDCAN_CLASSIC_CAN,
                };
                while (HAL_FDCAN_AddMessageToTxFifoQ(msg.hcan, &TxHeader, msg.data) != HAL_OK) tx_thread_sleep(1);
#elif defined(STM32F407xx)
                CAN_TxHeaderTypeDef TxHeader = {.StdId = msg.id, .DLC = msg.len, .RTR = CAN_RTR_DATA, .IDE = CAN_ID_STD};
                uint32_t            mailbox;
                while (HAL_CAN_AddTxMessage(msg.hcan, &TxHeader, msg.data, &mailbox) != HAL_OK) tx_thread_sleep(1);
#elif defined(STM32F105xC) || defined(STM32F103xB)
                CAN_TxHeaderTypeDef TxHeader = {.StdId = msg.id, .DLC = msg.len, .RTR = CAN_RTR_DATA, .IDE = CAN_ID_STD};
                uint32_t            mailbox;
                while (HAL_CAN_AddTxMessage(msg.hcan, &TxHeader, msg.data, &mailbox) != HAL_OK) tx_thread_sleep(1);
#endif
            }
        }
    }
}

/* 对外函数 */

void BSP_CAN_TaskInit(void)
{
    if (tx_semaphore_create(&g_can_rx_sem, "can_rx_sem", 0) != TX_SUCCESS)
    {
        LOG_E("rx sem create failed");
        return;
    }
    if (tx_semaphore_create(&g_can_tx_sem, "can_tx_sem", 0) != TX_SUCCESS)
    {
        LOG_E("tx sem create failed");
        return;
    }

    if (tx_thread_create(&g_can_rx_thread, "CAN RX Task", can_rx_task_entry, 0, g_can_rx_stack, BSP_CAN_TASK_STACK_SIZE, BSP_CAN_RX_TASK_PRIORITY,
                         BSP_CAN_RX_TASK_PRIORITY, TX_NO_TIME_SLICE, TX_AUTO_START) != TX_SUCCESS)
    {
        LOG_E("rx task create failed");
        return;
    }

    if (tx_thread_create(&g_can_tx_thread, "CAN TX Task", can_tx_task_entry, 0, g_can_tx_stack, BSP_CAN_TASK_STACK_SIZE, BSP_CAN_TX_TASK_PRIORITY,
                         BSP_CAN_TX_TASK_PRIORITY, TX_NO_TIME_SLICE, TX_AUTO_START) != TX_SUCCESS)
    {
        LOG_E("tx task create failed");
        return;
    }

    LOG_I("CAN tasks ready");
}
