/* 蓝牙串口：USART1 透传到 JDY-31，收发都走 FreeRTOS 流缓冲 */
#ifndef BT_UART_H
#define BT_UART_H

#include <stddef.h>
#include <stdint.h>

void bt_init(void);
void bt_write(const void *data, size_t len);
void bt_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
/* 多段输出（如二进制转储）期间独占串口，可嵌套 bt_printf */
void bt_lock(void);
void bt_unlock(void);
#define BT_WAIT_FOREVER 0xFFFFFFFFu
/* 读一行（去掉 \r\n），超时返回 -1，否则返回长度 */
int bt_getline(char *buf, size_t size, uint32_t timeout_ms);
uint32_t bt_rx_overflow(void);

#endif
