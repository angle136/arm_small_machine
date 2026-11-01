#include "board_link.h"

#include "bsp_def.h"
#include "bsp_dwt.h"
#include "bsp_uart.h"
#include "module_offline.h"
#include "usart.h"

#include <string.h>

#define LOG_TAG "board_link"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

typedef enum
{
    BOARD_LINK_SEND_INVALID = 0,
    BOARD_LINK_SEND_RATE_LIMITED,
    BOARD_LINK_SEND_SUCCESS,
    BOARD_LINK_SEND_FAILURE,
} BoardLink_Send_Result_t;

struct BoardLink
{
    UART_HandleTypeDef *huart;
    const char         *name;
    UART_Device        *uart;
    BoardLinkParser_t   parser;
    uint8_t             chunk[BOARD_LINK_MAX_RX_BUF];
    void               *rx_buffer;
    uint16_t            rx_buffer_len;
    Offline_Device     *offline_dev;
    uint8_t             frame_seen;
    uint8_t             last_seq;
    uint8_t             rx_enable;
    uint8_t             tx_enable;
    uint8_t            *tx_buf;
    uint8_t             tx_seq;
    uint16_t            tx_interval_ms;
    uint16_t            tx_timeout_ms;
    uint32_t            last_tx_ms;
    uint32_t            last_attempt_us;
};

static BoardLink s_links[BOARD_LINK_MAX_DEVICES];
static uint8_t   s_link_used[BOARD_LINK_MAX_DEVICES];

BUFFER_SECTION static uint8_t s_rxbuf[BOARD_LINK_MAX_DEVICES][BOARD_LINK_MAX_RX_BUF];
BUFFER_SECTION static uint8_t s_txbuf[BOARD_LINK_MAX_DEVICES][3U + BOARD_LINK_MAX_PAYLOAD + 1U];

static BoardLink *s_default_link;
static uint32_t   s_last_init_state;
static uint32_t   s_default_last_log_ms;

volatile Board_Link_Tx_Debug_t g_board_link_tx_debug;
volatile Board_Link_Tx_Trace_t g_board_link_tx_trace[BOARD_LINK_TX_TRACE_COUNT];

void BoardLinkParser_Reset(BoardLinkParser_t *parser)
{
    if (parser == NULL) return;
    memset(parser, 0, sizeof(*parser));
}

uint8_t BoardLinkParser_Feed(BoardLinkParser_t *parser, uint8_t byte, uint8_t *out_len)
{
    if (parser == NULL) return 0U;

    switch (parser->state)
    {
    case 0U:
        if (byte == BOARD_LINK_HDR) parser->state = 1U;
        break;

    case 1U:
        if (byte == 0U || byte > BOARD_LINK_MAX_PAYLOAD)
        {
            parser->state = (byte == BOARD_LINK_HDR) ? 1U : 0U;
        }
        else
        {
            parser->length = byte;
            parser->index  = 0U;
            parser->state  = 2U;
        }
        break;

    case 2U:
        parser->seq   = byte;
        parser->state = 3U;
        break;

    case 3U:
        parser->data[parser->index++] = byte;
        if (parser->index >= parser->length) parser->state = 4U;
        break;

    case 4U:
        parser->state = (byte == BOARD_LINK_HDR) ? 1U : 0U;
        if (byte == BOARD_LINK_TAIL)
        {
            if (out_len != NULL) *out_len = parser->length;
            return 1U;
        }
        break;

    default:
        BoardLinkParser_Reset(parser);
        break;
    }

    return 0U;
}

BoardLink *BoardLink_Init(const BoardLink_Config_t *config)
{
    s_last_init_state = 1U;
    if (config == NULL || config->huart == NULL)
    {
        LOG_E("invalid config");
        return NULL;
    }

    UART_HandleTypeDef *huart = (UART_HandleTypeDef *)config->huart;
    for (uint32_t i = 0U; i < BOARD_LINK_MAX_DEVICES; ++i)
    {
        if (s_link_used[i] != 0U && s_links[i].huart == huart)
        {
            LOG_E("uart already registered by [%s]", s_links[i].name != NULL ? s_links[i].name : "board_link");
            return NULL;
        }
    }

    uint32_t slot = BOARD_LINK_MAX_DEVICES;
    for (uint32_t i = 0U; i < BOARD_LINK_MAX_DEVICES; ++i)
    {
        if (s_link_used[i] == 0U)
        {
            slot = i;
            break;
        }
    }
    if (slot == BOARD_LINK_MAX_DEVICES)
    {
        LOG_E("no free device slot");
        return NULL;
    }

    BoardLink *link = &s_links[slot];
    memset(link, 0, sizeof(*link));
    link->huart = huart;
    link->name  = config->name;

    huart->Init.BaudRate = config->baud;
    if (HAL_UART_Init(huart) != HAL_OK)
    {
        LOG_E("uart init failed for [%s]", config->name != NULL ? config->name : "board_link");
        return NULL;
    }

    link->rx_enable      = config->rx_enable;
    link->tx_enable      = config->tx_enable;
    link->tx_buf         = s_txbuf[slot];
    link->tx_interval_ms = config->tx_interval_ms;
    link->tx_timeout_ms  = config->tx_timeout_ms;

    if (link->rx_enable != 0U && huart->hdmarx == NULL)
    {
        LOG_E("UART RX DMA not configured for [%s]", config->name != NULL ? config->name : "board_link");
        return NULL;
    }

    uint16_t rx_size = config->rx_buf_size;
    if (rx_size == 0U || rx_size > BOARD_LINK_MAX_RX_BUF) rx_size = BOARD_LINK_MAX_RX_BUF;
    if (link->rx_enable != 0U && (rx_size < 2U || (rx_size & (rx_size - 1U)) != 0U))
    {
        LOG_E("RX buffer size must be a power of 2 for [%s]", config->name != NULL ? config->name : "board_link");
        return NULL;
    }

    UART_Device_init_config uart_config = {
        .huart           = huart,
        .rx_buf          = (link->rx_enable != 0U) ? s_rxbuf[slot] : NULL,
        .rx_buf_size     = (link->rx_enable != 0U) ? rx_size : 0U,
        .expected_rx_len = 0U,
        .rx_mode         = (link->rx_enable != 0U) ? UART_MODE_DMA : UART_MODE_BLOCKING,
        .tx_mode         = (huart->hdmatx != NULL) ? UART_MODE_DMA : UART_MODE_BLOCKING,
    };
    link->uart = BSP_UART_Device_Init(&uart_config);
    if (link->uart == NULL)
    {
        s_last_init_state = 2U;
        LOG_E("bsp uart device init failed for [%s]", config->name != NULL ? config->name : "board_link");
        return NULL;
    }

    if (link->rx_enable != 0U && config->offline_enable != 0U)
    {
        const Offline_Init_config_t offline_config = {
            .name       = (config->name != NULL) ? config->name : "board_link",
            .timeout_ms = config->offline_timeout_ms,
            .beep_times = config->offline_beep_times,
            .enable     = 1U,
        };
        link->offline_dev = Module_Offline_register(&offline_config);
        if (link->offline_dev == NULL)
        {
            LOG_E("offline register failed for [%s]", config->name != NULL ? config->name : "board_link");
        }
    }

    BoardLinkParser_Reset(&link->parser);
    s_link_used[slot] = 1U;
    s_last_init_state = 3U;

    LOG_I("link [%s] ready: %lu baud, tx=%u rx=%u dma_tx=%u dma_rx=%u", config->name != NULL ? config->name : "board_link",
          (unsigned long)config->baud, (unsigned)link->tx_enable, (unsigned)link->rx_enable, huart->hdmatx != NULL, huart->hdmarx != NULL);
    return link;
}

void BoardLink_SetRxBuffer(BoardLink *link, void *buf, uint16_t len)
{
    if (link == NULL) return;
    if (buf == NULL || len == 0U || len > BOARD_LINK_MAX_PAYLOAD)
    {
        link->rx_buffer     = NULL;
        link->rx_buffer_len = 0U;
        return;
    }

    link->rx_buffer     = buf;
    link->rx_buffer_len = len;
}

uint32_t BoardLink_Poll(BoardLink *link)
{
    if (link == NULL || link->uart == NULL || link->rx_enable == 0U) return 0U;

    uint32_t accepted_frames = 0U;
    uint32_t chunk_len       = 0U;
    while (BSP_UART_Read(link->uart, link->chunk, sizeof(link->chunk), &chunk_len, 0U) > 0)
    {
        for (uint32_t i = 0U; i < chunk_len; ++i)
        {
            uint8_t frame_len = 0U;
            if (BoardLinkParser_Feed(&link->parser, link->chunk[i], &frame_len) == 0U) continue;
            if (link->rx_buffer == NULL || frame_len != link->rx_buffer_len) continue;

            memcpy(link->rx_buffer, link->parser.data, frame_len);
            link->last_seq   = link->parser.seq;
            link->frame_seen = 1U;
            if (link->offline_dev != NULL) Module_Offline_device_update(link->offline_dev);
            accepted_frames++;
        }
    }

    return accepted_frames;
}

uint8_t BoardLink_GetStatus(BoardLink *link)
{
    if (link == NULL || link->frame_seen == 0U) return STATE_OFFLINE;
    if (link->offline_dev == NULL) return STATE_ONLINE;
    return Module_Offline_get_device_status(link->offline_dev);
}

uint8_t BoardLink_GetSeq(BoardLink *link) { return (link != NULL) ? link->last_seq : 0U; }

static BoardLink_Send_Result_t board_link_send(BoardLink *link, const void *data, uint8_t len, volatile Board_Link_Tx_Trace_t *trace)
{
    if (link == NULL || link->uart == NULL || link->tx_enable == 0U || data == NULL || len == 0U || len > BOARD_LINK_MAX_PAYLOAD)
    {
        return BOARD_LINK_SEND_INVALID;
    }

    const uint32_t now_ms = (uint32_t)BSP_DWT_GetTimeline_ms();
    if ((uint32_t)(now_ms - link->last_tx_ms) < link->tx_interval_ms) return BOARD_LINK_SEND_RATE_LIMITED;
    link->last_tx_ms = now_ms;

    const uint8_t seq = link->tx_seq++;
    link->tx_buf[0]   = BOARD_LINK_HDR;
    link->tx_buf[1]   = len;
    link->tx_buf[2]   = seq;
    memcpy(&link->tx_buf[3], data, len);
    link->tx_buf[3U + len] = BOARD_LINK_TAIL;

    const uint32_t start_us    = (uint32_t)BSP_DWT_GetTimeline_us();
    const uint32_t interval_us = (link->last_attempt_us == 0U) ? 0U : (uint32_t)(start_us - link->last_attempt_us);
    link->last_attempt_us      = start_us;

    if (trace != NULL)
    {
        memset((void *)trace, 0, sizeof(*trace));
        trace->start_us          = start_us;
        trace->interval_us       = interval_us;
        trace->seq               = seq;
        trace->length            = len;
        trace->uart_state_before = (uint8_t)link->huart->gState;
        trace->dma_state_before  = (link->huart->hdmatx != NULL) ? (uint8_t)link->huart->hdmatx->State : 0U;
        trace->dma_ndtr_before   = (link->huart->hdmatx != NULL) ? (uint16_t)__HAL_DMA_GET_COUNTER(link->huart->hdmatx) : 0U;
    }

    const int      result      = BSP_UART_Send(link->uart, link->tx_buf, 4U + len, link->tx_timeout_ms);
    const uint32_t end_us      = (uint32_t)BSP_DWT_GetTimeline_us();
    const uint32_t duration_us = (uint32_t)(end_us - start_us);

    if (trace != NULL)
    {
        trace->duration_us      = duration_us;
        trace->result           = result;
        trace->uart_error       = link->huart->ErrorCode;
        trace->dma_error        = (link->huart->hdmatx != NULL) ? link->huart->hdmatx->ErrorCode : 0U;
        trace->uart_state_after = (uint8_t)link->huart->gState;
        trace->dma_state_after  = (link->huart->hdmatx != NULL) ? (uint8_t)link->huart->hdmatx->State : 0U;
        trace->dma_ndtr_after   = (link->huart->hdmatx != NULL) ? (uint16_t)__HAL_DMA_GET_COUNTER(link->huart->hdmatx) : 0U;
    }

    return (result == (int)(4U + len)) ? BOARD_LINK_SEND_SUCCESS : BOARD_LINK_SEND_FAILURE;
}

uint8_t BoardLink_Tx(BoardLink *link, const void *data, uint8_t len)
{
    return (board_link_send(link, data, len, NULL) == BOARD_LINK_SEND_SUCCESS) ? 1U : 0U;
}

void board_link_init(void)
{
    memset((void *)&g_board_link_tx_debug, 0, sizeof(g_board_link_tx_debug));
    memset((void *)g_board_link_tx_trace, 0, sizeof(g_board_link_tx_trace));

    const BoardLink_Config_t config = {
        .huart              = &BOARD_LINK_UART,
        .baud               = BOARD_LINK_BAUD,
        .rx_enable          = BOARD_LINK_RX_ENABLE,
        .tx_enable          = BOARD_LINK_TX_ENABLE,
        .tx_interval_ms     = BOARD_LINK_TX_INTERVAL_MS,
        .tx_timeout_ms      = BOARD_LINK_TX_TIMEOUT_MS,
        .rx_buf_size        = BOARD_LINK_RX_BUF_SIZE,
        .name               = "arm_board_link",
        .offline_timeout_ms = BOARD_LINK_OFFLINE_TIMEOUT_MS,
        .offline_enable     = BOARD_LINK_OFFLINE_ENABLE,
        .offline_beep_times = BOARD_LINK_OFFLINE_BEEP_TIMES,
    };

    s_default_link                   = BoardLink_Init(&config);
    g_board_link_tx_debug.init_state = s_last_init_state;
    if (s_default_link != NULL)
    {
        g_board_link_tx_debug.dma_tx_configured = s_default_link->huart->hdmatx != NULL;
    }
}

uint8_t board_link_tx(const void *data, uint8_t len)
{
    g_board_link_tx_debug.call_count++;
    if (s_default_link == NULL || data == NULL || len == 0U || len > BOARD_LINK_PAYLOAD_MAX)
    {
        g_board_link_tx_debug.invalid_call_count++;
        return 0U;
    }

    const uint32_t                  trace_index = g_board_link_tx_debug.write_index;
    volatile Board_Link_Tx_Trace_t *trace       = &g_board_link_tx_trace[trace_index];

    const BoardLink_Send_Result_t send_result = board_link_send(s_default_link, data, len, trace);
    if (send_result == BOARD_LINK_SEND_INVALID)
    {
        g_board_link_tx_debug.invalid_call_count++;
        return 0U;
    }
    if (send_result == BOARD_LINK_SEND_RATE_LIMITED)
    {
        g_board_link_tx_debug.rate_limited_count++;
        return 0U;
    }

    const uint32_t attempt_no = g_board_link_tx_debug.attempt_count + 1U;
    trace->call_no            = g_board_link_tx_debug.call_count;
    trace->attempt_no         = attempt_no;

    g_board_link_tx_debug.attempt_count    = attempt_no;
    g_board_link_tx_debug.write_index      = (trace_index + 1U) % BOARD_LINK_TX_TRACE_COUNT;
    g_board_link_tx_debug.last_attempt_us  = trace->start_us;
    g_board_link_tx_debug.last_interval_us = trace->interval_us;
    g_board_link_tx_debug.last_duration_us = trace->duration_us;
    g_board_link_tx_debug.last_result      = trace->result;
    g_board_link_tx_debug.last_uart_error  = trace->uart_error;
    g_board_link_tx_debug.last_dma_error   = trace->dma_error;
    g_board_link_tx_debug.last_seq         = trace->seq;
    g_board_link_tx_debug.last_length      = trace->length;
    if (g_board_link_tx_debug.record_count < BOARD_LINK_TX_TRACE_COUNT) g_board_link_tx_debug.record_count++;

    if (attempt_no == 1U)
    {
        g_board_link_tx_debug.min_duration_us = trace->duration_us;
        g_board_link_tx_debug.max_duration_us = trace->duration_us;
    }
    else
    {
        if (trace->interval_us < g_board_link_tx_debug.min_interval_us || g_board_link_tx_debug.min_interval_us == 0U)
            g_board_link_tx_debug.min_interval_us = trace->interval_us;
        if (trace->interval_us > g_board_link_tx_debug.max_interval_us) g_board_link_tx_debug.max_interval_us = trace->interval_us;
        if (trace->duration_us < g_board_link_tx_debug.min_duration_us) g_board_link_tx_debug.min_duration_us = trace->duration_us;
        if (trace->duration_us > g_board_link_tx_debug.max_duration_us) g_board_link_tx_debug.max_duration_us = trace->duration_us;
    }

    if (send_result == BOARD_LINK_SEND_SUCCESS)
        g_board_link_tx_debug.success_count++;
    else
        g_board_link_tx_debug.failure_count++;

    const uint32_t now_ms = (uint32_t)BSP_DWT_GetTimeline_ms();
    if ((uint32_t)(now_ms - s_default_last_log_ms) >= 1000U)
    {
        s_default_last_log_ms = now_ms;
        LOG_I("tx stats: call=%lu limited=%lu attempt=%lu ok=%lu fail=%lu dt_us=%lu/%lu/%lu dur_us=%lu/%lu result=%ld seq=%u err=0x%08lX",
              (unsigned long)g_board_link_tx_debug.call_count, (unsigned long)g_board_link_tx_debug.rate_limited_count,
              (unsigned long)g_board_link_tx_debug.attempt_count, (unsigned long)g_board_link_tx_debug.success_count,
              (unsigned long)g_board_link_tx_debug.failure_count, (unsigned long)g_board_link_tx_debug.last_interval_us,
              (unsigned long)g_board_link_tx_debug.min_interval_us, (unsigned long)g_board_link_tx_debug.max_interval_us,
              (unsigned long)g_board_link_tx_debug.last_duration_us, (unsigned long)g_board_link_tx_debug.max_duration_us,
              (long)g_board_link_tx_debug.last_result, (unsigned)g_board_link_tx_debug.last_seq,
              (unsigned long)g_board_link_tx_debug.last_uart_error);
    }

    return (send_result == BOARD_LINK_SEND_SUCCESS) ? 1U : 0U;
}

void board_link_set_rx_buffer(void *buf, uint16_t len) { BoardLink_SetRxBuffer(s_default_link, buf, len); }

uint32_t board_link_poll(void) { return BoardLink_Poll(s_default_link); }

uint8_t board_link_get_status(void) { return BoardLink_GetStatus(s_default_link); }

uint8_t board_link_get_seq(void) { return BoardLink_GetSeq(s_default_link); }
