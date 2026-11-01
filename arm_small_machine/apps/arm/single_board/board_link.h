/*
 * 机械臂板间 UART 通信层。
 *
 * 线协议固定为 0xAA | LEN | SEQ | payload | 0xFF。本模块不解释 payload，
 * 同时提供可注册多链路的通用接口和保持现有 APP 调用不变的默认链路兼容接口。
 */
#ifndef _ARM_BOARD_LINK_H_
#define _ARM_BOARD_LINK_H_

#include <stdint.h>

#define BOARD_LINK_HDR  0xAAU
#define BOARD_LINK_TAIL 0xFFU

#ifndef BOARD_LINK_MAX_DEVICES
#define BOARD_LINK_MAX_DEVICES 4U
#endif
#ifndef BOARD_LINK_MAX_PAYLOAD
#define BOARD_LINK_MAX_PAYLOAD 64U
#endif
#ifndef BOARD_LINK_MAX_RX_BUF
#define BOARD_LINK_MAX_RX_BUF 256U
#endif

/* 默认机械臂链路配置；由 CMake 生成的 module_config.h 优先覆盖。 */
#ifndef BOARD_LINK_UART
#define BOARD_LINK_UART huart6
#endif
#ifndef BOARD_LINK_BAUD
#define BOARD_LINK_BAUD 921600U
#endif
#ifndef BOARD_LINK_PAYLOAD_MAX
#define BOARD_LINK_PAYLOAD_MAX 36U
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
#ifndef BOARD_LINK_RX_BUF_SIZE
#define BOARD_LINK_RX_BUF_SIZE 256U
#endif
#ifndef BOARD_LINK_OFFLINE_ENABLE
#define BOARD_LINK_OFFLINE_ENABLE 1
#endif
#ifndef BOARD_LINK_OFFLINE_TIMEOUT_MS
#define BOARD_LINK_OFFLINE_TIMEOUT_MS 100U
#endif
#ifndef BOARD_LINK_OFFLINE_BEEP_TIMES
#define BOARD_LINK_OFFLINE_BEEP_TIMES 0U
#endif
#ifndef BOARD_LINK_TX_TRACE_COUNT
#define BOARD_LINK_TX_TRACE_COUNT 256U
#endif

#if BOARD_LINK_PAYLOAD_MAX > BOARD_LINK_MAX_PAYLOAD
#error "BOARD_LINK_PAYLOAD_MAX exceeds BOARD_LINK_MAX_PAYLOAD"
#endif
#if BOARD_LINK_RX_BUF_SIZE > BOARD_LINK_MAX_RX_BUF
#error "BOARD_LINK_RX_BUF_SIZE exceeds BOARD_LINK_MAX_RX_BUF"
#endif

/** 通用逐字节帧解析状态。 */
typedef struct
{
    uint8_t state;  /**< 0=帧头，1=LEN，2=SEQ，3=payload，4=帧尾。 */
    uint8_t length; /**< 当前帧 payload 长度。 */
    uint8_t index;  /**< 当前 payload 写入下标。 */
    uint8_t seq;    /**< 当前帧序号。 */
    uint8_t data[BOARD_LINK_MAX_PAYLOAD];
} BoardLinkParser_t;

/** 通用链路运行时配置。 */
typedef struct
{
    void       *huart;              /**< UART_HandleTypeDef *。 */
    uint32_t    baud;               /**< UART 波特率。 */
    uint8_t     rx_enable;          /**< 1=启用 DMA 接收。 */
    uint8_t     tx_enable;          /**< 1=启用发送。 */
    uint16_t    tx_interval_ms;     /**< 最小发送间隔。 */
    uint16_t    tx_timeout_ms;      /**< 单次发送超时。 */
    uint16_t    rx_buf_size;        /**< DMA 接收缓冲大小，0 使用最大值。 */
    const char *name;               /**< 日志和离线设备名称。 */
    uint32_t    offline_timeout_ms; /**< 接收离线超时。 */
    uint8_t     offline_enable;     /**< 1=注册 Offline 设备。 */
    uint8_t     offline_beep_times; /**< 掉线蜂鸣次数。 */
} BoardLink_Config_t;

typedef struct BoardLink BoardLink;

/** 单次 UART 实际发送尝试的环形诊断记录。 */
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

/** 默认机械臂链路的发送累计诊断快照。 */
typedef struct
{
    uint32_t init_state; /**< 0=未初始化，1=HAL/配置失败，2=BSP 失败，3=就绪。 */
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

/** 注册一条 UART 链路；同一 UART 只能注册一次。 */
BoardLink *BoardLink_Init(const BoardLink_Config_t *config);

/** 设置接收目标；仅当完整帧 payload 长度等于 len 时更新目标。 */
void BoardLink_SetRxBuffer(BoardLink *link, void *buf, uint16_t len);

/** 非阻塞读取 DMA 数据并解析；返回本次写入接收目标的完整帧数量。 */
uint32_t BoardLink_Poll(BoardLink *link);

/** 未收到首帧或离线时返回 STATE_OFFLINE，否则返回 STATE_ONLINE。 */
uint8_t BoardLink_GetStatus(BoardLink *link);

/** 返回最近一帧成功写入接收目标的序号。 */
uint8_t BoardLink_GetSeq(BoardLink *link);

/** 发送一帧；实际完整发送返回 1，参数无效、限频或发送失败返回 0。 */
uint8_t BoardLink_Tx(BoardLink *link, const void *data, uint8_t len);

void    BoardLinkParser_Reset(BoardLinkParser_t *parser);
uint8_t BoardLinkParser_Feed(BoardLinkParser_t *parser, uint8_t byte, uint8_t *out_len);

/* 现有 arm APP 默认链路兼容接口。 */
void     board_link_init(void);
uint8_t  board_link_tx(const void *data, uint8_t len);
void     board_link_set_rx_buffer(void *buf, uint16_t len);
uint32_t board_link_poll(void);
uint8_t  board_link_get_status(void);
uint8_t  board_link_get_seq(void);

#endif /* _ARM_BOARD_LINK_H_ */
