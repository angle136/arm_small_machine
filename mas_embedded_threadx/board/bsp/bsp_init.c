/*
 * @Author: laladuduqq 17503181697@163.com
 * @Date: 2026-04-20 07:56:26
 * @LastEditors: laladuduqq 17503181697@163.com
 * @LastEditTime: 2026-04-21 13:14:04
 * @FilePath: \mas_embedded_threadx\board\bsp\bsp_init.c
 * @Description:
 */
#include "bsp_init.h"

#include "bsp_dwt.h"
#include "usbd_cdc_acm_user.h"
#include "bsp_led.h"
#include "bsp_beep.h"
#include "bsp_can_task.h"
    
#include "gpio.h"


#define LOG_LVL LOG_LVL_INFO
#define LOG_TAG "Robot_Init"
#include "ulog_def.h"

void BSP_Init(void)
{
#if defined(STM32H723xx)
    // POWER_24V 和 POWER_5V 电源使能
    HAL_GPIO_WritePin(POWER_24V_1_GPIO_Port, POWER_24V_1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(POWER_24V_2_GPIO_Port, POWER_24V_2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(POWER_5V_GPIO_Port, POWER_5V_Pin, GPIO_PIN_SET);

    BSP_DWT_Init(480);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
    BSP_DWT_Delay(0.1);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
    BSP_DWT_Delay(0.1);
    cdc_acm_init(0, USB_OTG_HS_PERIPH_BASE);
#elif defined(STM32F407xx)
    BSP_DWT_Init(168);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
    BSP_DWT_Delay(0.1);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
    BSP_DWT_Delay(0.1);
    cdc_acm_init(0, USB_OTG_FS_PERIPH_BASE);
#endif

    BSP_LED_Init();
    BSP_BEEP_Init();
    BSP_CAN_TaskInit();

    LOG_I("BSP Init finish");
}
