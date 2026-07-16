/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2026-07-13 13:11:50
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2026-07-13 14:39:14
 * @FilePath: \mas_embedded_threadx\apps\app_init.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "app_init.h"
#include "tx_api.h"

#define LOG_LVL LOG_LVL_INFO
#define LOG_TAG "APP_Init"
#include "ulog_def.h"

#include "robot_control.h"

void APP_Init(void)
{
    robot_control_init();

    LOG_I("APP init finished");
}