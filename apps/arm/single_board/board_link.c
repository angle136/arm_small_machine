#include "board_link.h"

#include "bsp_def.h"
#include "bsp_dwt.h"
#include "bsp_uart.h"
#include "usart.h"

#include <string.h>

#define LOG_TAG "board_link"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

#define BOARD_LINK_HDR  0xAAU
#define BOARD_LINK_TAIL 0xFFU

static UART_Device *g_board_link_uart;

#if BOARD_LINK_TX_ENABLE
BUFFER_SECTION static uint8_t g_board_link_txbuf[3U + BOARD_LINK_PAYLOAD_MAX + 1U];
static uint8_t                g_board_link_tx_seq;
static uint32_t               g_board_link_last_tx_ms;
static uint32_t               g_board_link_last_log_ms;

volatile Board_Link_Tx_Debug_t g_board_link_tx_debug;
volatile Board_Link_Tx_Trace_t g_board_link_tx_trace[BOARD_LINK_TX_TRACE_COUNT];
#endif

void board_link_init(void)
{
#if BOARD_LINK_TX_ENABLE
    memset((void *)&g_board_link_tx_debug, 0, sizeof(g_board_link_tx_debug));
    memset((void *)g_board_link_tx_trace, 0, sizeof(g_board_link_tx_trace));
#endif

    BOARD_LINK_UART.Init.BaudRate = BOARD_LINK_BAUD;
    if (HAL_UART_Init(&BOARD_LINK_UART) != HAL_OK)
    {
#if BOARD_LINK_TX_ENABLE
        g_board_link_tx_debug.init_state = 1U;
#endif
        LOG_E("uart init failed: baud=%lu", (unsigned long)BOARD_LINK_BAUD);
        return;
    }

    UART_Device_init_config config = {
        .huart           = &BOARD_LINK_UART,
        .expected_rx_len = 0U,
        .rx_buf          = NULL,
        .rx_buf_size     = 0U,
        .rx_mode         = UART_MODE_BLOCKING,
        .tx_mode         = BOARD_LINK_UART.hdmatx ? UART_MODE_DMA : UART_MODE_BLOCKING,
    };
    g_board_link_uart = BSP_UART_Device_Init(&config);
    if (g_board_link_uart == NULL)
    {
#if BOARD_LINK_TX_ENABLE
        g_board_link_tx_debug.init_state = 2U;
#endif
        LOG_E("bsp uart device init failed");
        return;
    }

#if BOARD_LINK_TX_ENABLE
    g_board_link_tx_debug.dma_tx_configured = BOARD_LINK_UART.hdmatx != NULL;
    g_board_link_tx_debug.init_state        = 3U;
#endif

    LOG_I("board link ready: instance=%p baud=%lu tx=%u rx=%u interval=%lu ms dma_tx=%u dma_rx=%u", (void *)BOARD_LINK_UART.Instance,
          (unsigned long)BOARD_LINK_BAUD, (unsigned)BOARD_LINK_TX_ENABLE, (unsigned)BOARD_LINK_RX_ENABLE, (unsigned long)BOARD_LINK_TX_INTERVAL_MS,
          BOARD_LINK_UART.hdmatx != NULL, BOARD_LINK_UART.hdmarx != NULL);
}

#if BOARD_LINK_TX_ENABLE
void board_link_tx(const void *data, uint8_t len)
{
    const uint8_t *payload = (const uint8_t *)data;
    const uint32_t now_ms  = (uint32_t)BSP_DWT_GetTimeline_ms();

    g_board_link_tx_debug.call_count++;
    if (g_board_link_uart == NULL || payload == NULL || len == 0U || len > BOARD_LINK_PAYLOAD_MAX)
    {
        g_board_link_tx_debug.invalid_call_count++;
        return;
    }
    if ((uint32_t)(now_ms - g_board_link_last_tx_ms) < BOARD_LINK_TX_INTERVAL_MS)
    {
        g_board_link_tx_debug.rate_limited_count++;
        return;
    }
    g_board_link_last_tx_ms = now_ms;

    g_board_link_txbuf[0] = BOARD_LINK_HDR;
    g_board_link_txbuf[1] = len;
    const uint8_t seq     = g_board_link_tx_seq++;
    g_board_link_txbuf[2] = seq;
    memcpy(&g_board_link_txbuf[3], payload, len);
    g_board_link_txbuf[3U + len] = BOARD_LINK_TAIL;

    const uint32_t start_us    = (uint32_t)BSP_DWT_GetTimeline_us();
    const uint32_t interval_us = (g_board_link_tx_debug.attempt_count == 0U) ? 0U : (uint32_t)(start_us - g_board_link_tx_debug.last_attempt_us);
    const uint32_t trace_index = g_board_link_tx_debug.write_index;
    volatile Board_Link_Tx_Trace_t *trace = &g_board_link_tx_trace[trace_index];

    trace->attempt_no        = 0U;
    trace->call_no           = g_board_link_tx_debug.call_count;
    trace->start_us          = start_us;
    trace->interval_us       = interval_us;
    trace->seq               = seq;
    trace->length            = len;
    trace->uart_state_before = (uint8_t)BOARD_LINK_UART.gState;
    trace->dma_state_before  = (BOARD_LINK_UART.hdmatx != NULL) ? (uint8_t)BOARD_LINK_UART.hdmatx->State : 0U;
    trace->dma_ndtr_before   = (BOARD_LINK_UART.hdmatx != NULL) ? (uint16_t)__HAL_DMA_GET_COUNTER(BOARD_LINK_UART.hdmatx) : 0U;

    const int      result      = BSP_UART_Send(g_board_link_uart, g_board_link_txbuf, 4U + len, BOARD_LINK_TX_TIMEOUT_MS);
    const uint32_t end_us      = (uint32_t)BSP_DWT_GetTimeline_us();
    const uint32_t duration_us = (uint32_t)(end_us - start_us);
    const uint32_t attempt_no  = g_board_link_tx_debug.attempt_count + 1U;

    trace->duration_us      = duration_us;
    trace->result           = result;
    trace->uart_error       = BOARD_LINK_UART.ErrorCode;
    trace->dma_error        = (BOARD_LINK_UART.hdmatx != NULL) ? BOARD_LINK_UART.hdmatx->ErrorCode : 0U;
    trace->uart_state_after = (uint8_t)BOARD_LINK_UART.gState;
    trace->dma_state_after  = (BOARD_LINK_UART.hdmatx != NULL) ? (uint8_t)BOARD_LINK_UART.hdmatx->State : 0U;
    trace->dma_ndtr_after   = (BOARD_LINK_UART.hdmatx != NULL) ? (uint16_t)__HAL_DMA_GET_COUNTER(BOARD_LINK_UART.hdmatx) : 0U;
    trace->attempt_no       = attempt_no;

    g_board_link_tx_debug.attempt_count = attempt_no;
    g_board_link_tx_debug.write_index   = (trace_index + 1U) % BOARD_LINK_TX_TRACE_COUNT;
    if (g_board_link_tx_debug.record_count < BOARD_LINK_TX_TRACE_COUNT) g_board_link_tx_debug.record_count++;
    g_board_link_tx_debug.last_attempt_us  = start_us;
    g_board_link_tx_debug.last_interval_us = interval_us;
    g_board_link_tx_debug.last_duration_us = duration_us;
    g_board_link_tx_debug.last_result      = result;
    g_board_link_tx_debug.last_uart_error  = trace->uart_error;
    g_board_link_tx_debug.last_dma_error   = trace->dma_error;
    g_board_link_tx_debug.last_seq         = seq;
    g_board_link_tx_debug.last_length      = len;

    if (attempt_no == 1U)
    {
        g_board_link_tx_debug.min_duration_us = duration_us;
        g_board_link_tx_debug.max_duration_us = duration_us;
    }
    else
    {
        if (interval_us < g_board_link_tx_debug.min_interval_us || g_board_link_tx_debug.min_interval_us == 0U)
            g_board_link_tx_debug.min_interval_us = interval_us;
        if (interval_us > g_board_link_tx_debug.max_interval_us) g_board_link_tx_debug.max_interval_us = interval_us;
        if (duration_us < g_board_link_tx_debug.min_duration_us) g_board_link_tx_debug.min_duration_us = duration_us;
        if (duration_us > g_board_link_tx_debug.max_duration_us) g_board_link_tx_debug.max_duration_us = duration_us;
    }

    if (result == (int)(4U + len))
        g_board_link_tx_debug.success_count++;
    else
        g_board_link_tx_debug.failure_count++;

    if ((uint32_t)(now_ms - g_board_link_last_log_ms) >= 1000U)
    {
        g_board_link_last_log_ms = now_ms;
        LOG_I("tx stats: call=%lu limited=%lu attempt=%lu ok=%lu fail=%lu dt_us=%lu/%lu/%lu dur_us=%lu/%lu result=%d seq=%u err=0x%08lX",
              (unsigned long)g_board_link_tx_debug.call_count, (unsigned long)g_board_link_tx_debug.rate_limited_count,
              (unsigned long)g_board_link_tx_debug.attempt_count, (unsigned long)g_board_link_tx_debug.success_count,
              (unsigned long)g_board_link_tx_debug.failure_count, (unsigned long)g_board_link_tx_debug.last_interval_us,
              (unsigned long)g_board_link_tx_debug.min_interval_us, (unsigned long)g_board_link_tx_debug.max_interval_us,
              (unsigned long)g_board_link_tx_debug.last_duration_us, (unsigned long)g_board_link_tx_debug.max_duration_us, result, (unsigned)seq,
              (unsigned long)BOARD_LINK_UART.ErrorCode);
    }
}
#endif
