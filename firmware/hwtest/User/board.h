/* 板级定义：F407VET6 黑色最小系统板（资料/数据手册/开发板原理图_stm32f407v.pdf） */
#ifndef BOARD_H
#define BOARD_H

#include "stm32f4xx.h"

#define FW_NAME "duct-anc-hwtest-0.1" /* 不含空格，INFO 行按 key=value 解析 */

#ifndef BT_BAUD
#define BT_BAUD 9600
#endif
#ifndef FS_HZ
#define FS_HZ 8000
#endif

/* LED：VCC - LED - 510R - 引脚，低电平点亮 */
#define LED_PORT  GPIOA
#define LED_D2    GPIO_Pin_6 /* 心跳 */
#define LED_D3    GPIO_Pin_7 /* 扬声器输出中 */

/* 按键：K0/K1 对地低有效，K_UP 接 VCC 高有效 */
#define KEY0_PORT GPIOE
#define KEY0_PIN  GPIO_Pin_4
#define KEY1_PORT GPIOE
#define KEY1_PIN  GPIO_Pin_3
#define KEYUP_PORT GPIOA
#define KEYUP_PIN GPIO_Pin_0

/* 蓝牙 JDY-31：J6 排针 5V/GND/PA10/PA9，模块 TX→PA10，模块 RX→PA9 */
/* 麦克风 MAX4466：参考麦 PC0 (ADC123_IN10)，误差麦 PC1 (ADC123_IN11) */
/* 扬声器：PA4 DAC_OUT1 → 隔直 → PAM8403 */

/* NVIC 抢占优先级（分组 4，0 最高）。调用 FromISR API 的中断必须 >= 5 */
#define IRQ_PRIO_DMA_AUDIO 5
#define IRQ_PRIO_ADC       5
#define IRQ_PRIO_USART     6

#endif
