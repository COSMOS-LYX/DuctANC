#include "bt_uart.h"

#include <stdarg.h>
#include <stdio.h>

#include "board.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "stream_buffer.h"

#define RX_SIZE 256
#define TX_SIZE 1024
#define TX_CHUNK 64

static StreamBufferHandle_t rx_sb, tx_sb;
static SemaphoreHandle_t tx_mutex;
static volatile uint32_t rx_overflow;

void bt_init(void)
{
    GPIO_InitTypeDef g;
    USART_InitTypeDef u;
    NVIC_InitTypeDef n;

    rx_sb = xStreamBufferCreate(RX_SIZE, 1);
    tx_sb = xStreamBufferCreate(TX_SIZE, 1);
    tx_mutex = xSemaphoreCreateRecursiveMutex();
    configASSERT(rx_sb && tx_sb && tx_mutex);

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
    g.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
    g.GPIO_Mode = GPIO_Mode_AF;
    g.GPIO_OType = GPIO_OType_PP;
    g.GPIO_Speed = GPIO_Speed_25MHz;
    g.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &g);

    u.USART_BaudRate = BT_BAUD;
    u.USART_WordLength = USART_WordLength_8b;
    u.USART_StopBits = USART_StopBits_1;
    u.USART_Parity = USART_Parity_No;
    u.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    u.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &u);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    n.NVIC_IRQChannel = USART1_IRQn;
    n.NVIC_IRQChannelPreemptionPriority = IRQ_PRIO_USART;
    n.NVIC_IRQChannelSubPriority = 0;
    n.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&n);
    USART_Cmd(USART1, ENABLE);
}

void bt_write(const void *data, size_t len)
{
    const uint8_t *p = data;

    bt_lock(); /* 流缓冲只允许一个写者 */
    while (len) {
        size_t chunk = len > TX_CHUNK ? TX_CHUNK : len;
        /* 先开 TXE 再可能阻塞，保证缓冲满时 ISR 在排空 */
        USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
        size_t sent = xStreamBufferSend(tx_sb, p, chunk, portMAX_DELAY);
        USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
        p += sent;
        len -= sent;
    }
    bt_unlock();
}

void bt_lock(void)
{
    xSemaphoreTakeRecursive(tx_mutex, portMAX_DELAY);
}

void bt_unlock(void)
{
    xSemaphoreGiveRecursive(tx_mutex);
}

void bt_printf(const char *fmt, ...)
{
    static char buf[256];
    va_list ap;
    int n;

    bt_lock();
    va_start(ap, fmt);
    n = vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    if (n > (int)sizeof buf - 1)
        n = sizeof buf - 1;
    if (n > 0)
        bt_write(buf, (size_t)n);
    bt_unlock();
}

int bt_getline(char *buf, size_t size, uint32_t timeout_ms)
{
    TickType_t to = timeout_ms == BT_WAIT_FOREVER ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    size_t len = 0;
    char c;

    for (;;) {
        if (!xStreamBufferReceive(rx_sb, &c, 1, to))
            return -1;
        if (c == '\n' || c == '\r') {
            if (len == 0)
                continue;
            buf[len] = '\0';
            return (int)len;
        }
        if (len < size - 1)
            buf[len++] = c;
    }
}

uint32_t bt_rx_overflow(void)
{
    return rx_overflow;
}

void USART1_IRQHandler(void)
{
    BaseType_t woken = pdFALSE;
    uint16_t sr = USART1->SR;

    if (sr & (USART_SR_RXNE | USART_SR_ORE)) {
        uint8_t c = (uint8_t)USART1->DR; /* 读 SR 再读 DR 同时清 ORE */
        if (!xStreamBufferSendFromISR(rx_sb, &c, 1, &woken))
            rx_overflow++;
    }
    if ((sr & USART_SR_TXE) && (USART1->CR1 & USART_CR1_TXEIE)) {
        uint8_t c;
        if (xStreamBufferReceiveFromISR(tx_sb, &c, 1, &woken))
            USART1->DR = c;
        else
            USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
    }
    portYIELD_FROM_ISR(woken);
}
