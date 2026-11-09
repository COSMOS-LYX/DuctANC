/*
 * 管道 ANC 基础硬件测试固件
 * - 蓝牙 JDY-31（USART1）命令行与上位机通信
 * - 扬声器：DAC PA4 输出单音 / 白噪声 / 扫频
 * - 麦克风：ADC1 PC0（参考）/ PC1（误差）电平统计与原始数据采集
 */
#include "FreeRTOS.h"
#include "task.h"

#include "app.h"
#include "audio.h"
#include "bsp.h"
#include "bt_uart.h"

void fault_trap(void)
{
    __disable_irq();
    led_set(LED_D2 | LED_D3, true);
    for (;;) {
    }
}

void vAssertCalled(const char *file, int line)
{
    (void)file;
    (void)line;
    fault_trap();
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *name)
{
    (void)task;
    (void)name;
    fault_trap();
}

void vApplicationMallocFailedHook(void)
{
    fault_trap();
}

int main(void)
{
    bsp_init();
    bt_init();
    audio_init();
    app_start();
    audio_start_task();
    vTaskStartScheduler();
    fault_trap();
}
