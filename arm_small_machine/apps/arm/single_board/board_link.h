/*
 * @Author: angle136 angieleevcd@gmail.com
 * @Date: 2026-10-16 22:33:57
 * @LastEditors: angle136 angieleevcd@gmail.com
 * @LastEditTime: 2026-10-17 12:04:42
 * @FilePath: \arm_small_machine\apps\arm\single_board\board_link.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef _ARM_BOARD_LINK_H_
#define _ARM_BOARD_LINK_H_

#include <stdint.h>

/* 参考接收端协议：0xAA | LEN | SEQ | payload | 0xFF。 */
#ifndef BOARD_LINK_UART
#define BOARD_LINK_UART huart6
#endif
#ifndef BOARD_LINK_BAUD
#define BOARD_LINK_BAUD 921600U
#endif
#ifndef BOARD_LINK_PAYLOAD_MAX
#define BOARD_LINK_PAYLOAD_MAX 32U
#endif
#ifndef BOARD_LINK_TX_ENABLE
#define BOARD_LINK_TX_ENABLE 1
#endif
#ifndef BOARD_LINK_RX_ENABLE
#define BOARD_LINK_RX_ENABLE 0
#endif
#ifndef BOARD_LINK_TX_INTERVAL_MS
#define BOARD_LINK_TX_INTERVAL_MS 10U
#endif
#ifndef BOARD_LINK_TX_TIMEOUT_MS
#define BOARD_LINK_TX_TIMEOUT_MS 10U
#endif
#ifndef BOARD_LINK_TX_TRACE_COUNT
#define BOARD_LINK_TX_TRACE_COUNT 256U
#endif

#if BOARD_LINK_TX_ENABLE
typedef struct
{
    uint32_t call_no;
    uint32_t attempt_no;
    uint32_t start_us;
    uint32_t interval_us;
    uint32_t duration_us;
    int32_t  result;
    uint32_t uart_error;
    uint32_t dma_error;
    uint16_t dma_ndtr_before;
    uint16_t dma_ndtr_after;
    uint8_t  seq;
    uint8_t  length;
    uint8_t  uart_state_before;
    uint8_t  uart_state_after;
    uint8_t  dma_state_before;
    uint8_t  dma_state_after;
} Board_Link_Tx_Trace_t;

typedef struct
{
    uint32_t init_state; /* 0=not called, 1=UART init failed, 2=BSP init failed, 3=ready */
    uint32_t call_count;
    uint32_t invalid_call_count;
    uint32_t rate_limited_count;
    uint32_t attempt_count;
    uint32_t success_count;
    uint32_t failure_count;
    uint32_t write_index;
    uint32_t record_count;
    uint32_t last_attempt_us;
    uint32_t last_interval_us;
    uint32_t min_interval_us;
    uint32_t max_interval_us;
    uint32_t last_duration_us;
    uint32_t min_duration_us;
    uint32_t max_duration_us;
    int32_t  last_result;
    uint32_t last_uart_error;
    uint32_t last_dma_error;
    uint8_t  last_seq;
    uint8_t  last_length;
    uint8_t  dma_tx_configured;
    uint8_t  reserved;
} Board_Link_Tx_Debug_t;

extern volatile Board_Link_Tx_Debug_t g_board_link_tx_debug;
extern volatile Board_Link_Tx_Trace_t g_board_link_tx_trace[BOARD_LINK_TX_TRACE_COUNT];
#endif

void board_link_init(void);

#if BOARD_LINK_TX_ENABLE
void board_link_tx(const void *data, uint8_t len);
#endif

#endif /* _ARM_BOARD_LINK_H_ */
