/*
 * 机械臂控制板到接收板的单向 UART 帧传输层。
 *
 * 本模块只负责 0xAA | LEN | SEQ | payload | 0xFF 封帧、限频、发送和诊断，
 * 不解释 payload 内的关节位置或夹爪速度语义。
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
#ifndef BOARD_LINK_TX_TRACE_COUNT
#define BOARD_LINK_TX_TRACE_COUNT 256U
#endif

#if BOARD_LINK_TX_ENABLE
/** 单次 UART 实际发送尝试的环形诊断记录。 */
typedef struct
{
    uint32_t call_no;           /**< 对应 board_link_tx() 调用序号。 */
    uint32_t attempt_no;        /**< 通过参数及限频检查后的发送尝试序号。 */
    uint32_t start_us;          /**< BSP_UART_Send() 调用前的时间戳，单位 us。 */
    uint32_t interval_us;       /**< 与上一次实际发送尝试的间隔，单位 us。 */
    uint32_t duration_us;       /**< BSP_UART_Send() 调用耗时，单位 us。 */
    int32_t  result;            /**< BSP_UART_Send() 返回值。 */
    uint32_t uart_error;        /**< 发送完成后的 HAL UART 错误码。 */
    uint32_t dma_error;         /**< 发送完成后的 HAL DMA 错误码；无 DMA 时为 0。 */
    uint16_t dma_ndtr_before;   /**< 发送前 DMA 剩余传输计数。 */
    uint16_t dma_ndtr_after;    /**< 发送后 DMA 剩余传输计数。 */
    uint8_t  seq;               /**< 本次帧序号。 */
    uint8_t  length;            /**< payload 长度，单位 byte。 */
    uint8_t  uart_state_before; /**< 发送前 HAL UART 状态。 */
    uint8_t  uart_state_after;  /**< 发送后 HAL UART 状态。 */
    uint8_t  dma_state_before;  /**< 发送前 HAL DMA 状态。 */
    uint8_t  dma_state_after;   /**< 发送后 HAL DMA 状态。 */
} Board_Link_Tx_Trace_t;

/** 板间 UART 初始化、限频和发送结果的累计诊断快照。 */
typedef struct
{
    uint32_t init_state;         /**< 0=未调用，1=HAL 初始化失败，2=BSP 注册失败，3=就绪。 */
    uint32_t call_count;         /**< board_link_tx() 总调用次数。 */
    uint32_t invalid_call_count; /**< 参数或初始化状态无效的调用次数。 */
    uint32_t rate_limited_count; /**< 被最小发送间隔拒绝的调用次数。 */
    uint32_t attempt_count;      /**< 实际调用 BSP_UART_Send() 的次数。 */
    uint32_t success_count;      /**< 完整写出一帧的次数。 */
    uint32_t failure_count;      /**< 实际发送但未完整写出的次数。 */
    uint32_t write_index;        /**< 下一条环形记录写入下标。 */
    uint32_t record_count;       /**< 当前有效环形记录数量。 */
    uint32_t last_attempt_us;    /**< 最近发送尝试的起始时间戳，单位 us。 */
    uint32_t last_interval_us;   /**< 最近两次发送尝试间隔，单位 us。 */
    uint32_t min_interval_us;    /**< 已观测最小发送尝试间隔，单位 us。 */
    uint32_t max_interval_us;    /**< 已观测最大发送尝试间隔，单位 us。 */
    uint32_t last_duration_us;   /**< 最近发送尝试耗时，单位 us。 */
    uint32_t min_duration_us;    /**< 已观测最小发送尝试耗时，单位 us。 */
    uint32_t max_duration_us;    /**< 已观测最大发送尝试耗时，单位 us。 */
    int32_t  last_result;        /**< 最近一次 BSP_UART_Send() 返回值。 */
    uint32_t last_uart_error;    /**< 最近一次尝试后的 HAL UART 错误码。 */
    uint32_t last_dma_error;     /**< 最近一次尝试后的 HAL DMA 错误码。 */
    uint8_t  last_seq;           /**< 最近实际发送帧的序号。 */
    uint8_t  last_length;        /**< 最近实际发送 payload 长度，单位 byte。 */
    uint8_t  dma_tx_configured;  /**< UART 配置了 TX DMA 时为 1。 */
    uint8_t  reserved;           /**< 对齐保留字段。 */
} Board_Link_Tx_Debug_t;

/** Ozone 观察用的板间发送累计诊断。 */
extern volatile Board_Link_Tx_Debug_t g_board_link_tx_debug;
/** Ozone 观察用的最近 BOARD_LINK_TX_TRACE_COUNT 次实际发送记录。 */
extern volatile Board_Link_Tx_Trace_t g_board_link_tx_trace[BOARD_LINK_TX_TRACE_COUNT];
#endif

/**
 * @brief 初始化板间 UART 及发送诊断状态。
 *
 * @param 无。
 *
 * @retval 无。
 *
 * @note 调用关系：由 robot_control_init() 在线程创建前调用；不主动发送帧。
 */
void board_link_init(void);

#if BOARD_LINK_TX_ENABLE
/**
 * @brief 封装并发送一帧板间 payload。
 *
 * @param data 输入，payload 首地址；函数返回前已复制到模块发送缓冲区。
 * @param len  输入，payload 字节数，范围 1~BOARD_LINK_PAYLOAD_MAX。
 *
 * @return uint8_t 完整写出帧头、payload 和帧尾返回 1；参数无效、被限频或写失败返回 0。
 *
 * @note 调用关系：由 arm_link_tx_task() 在互斥锁外调用。
 * @note 安全说明：函数不解释 payload，不改变机械臂控制状态；失败详情写入诊断快照。
 */
uint8_t board_link_tx(const void *data, uint8_t len);
#endif

#endif /* _ARM_BOARD_LINK_H_ */
