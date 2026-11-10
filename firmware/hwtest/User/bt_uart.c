#include "bt_uart.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "board.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "stream_buffer.h"
#include "task.h"

#define RX_SIZE 256
#define TX_SIZE 1024
#define TX_CHUNK 64

static StreamBufferHandle_t rx_sb, tx_sb;
/* getline 遇到 \r\n 时，另一半可能还要再读一个字节才能确认。读到的不是配对换行就退回来，避免二进制命令把这个字节吃掉。 */
static int rx_push = -1;
static SemaphoreHandle_t tx_mutex;
static volatile uint32_t rx_overflow;
static uint32_t cur_baud = BT_BAUD;
static const char *jdy_status = "-";

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

static int rx_get(TickType_t to)
{
    uint8_t c;

    if (rx_push >= 0) {
        c = (uint8_t)rx_push;
        rx_push = -1;
        return c;
    }
    if (!xStreamBufferReceive(rx_sb, &c, 1, to))
        return -1;
    return c;
}

int bt_getline(char *buf, size_t size, uint32_t timeout_ms)
{
    TickType_t to = timeout_ms == BT_WAIT_FOREVER ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    size_t len = 0;

    for (;;) {
        int ci = rx_get(to);
        char c;
        if (ci < 0)
            return -1;
        c = (char)ci;
        if (c == '\n' || c == '\r') {
            int extra;
            if (len == 0)
                continue;
            buf[len] = '\0';
            /* \r\n 的第二个字节通常已经在缓冲里。2 ms 只是等它进中断。 */
            extra = rx_get(pdMS_TO_TICKS(2));
            if (extra >= 0 && !((c == '\r' && extra == '\n') || (c == '\n' && extra == '\r')))
                rx_push = extra;
            return (int)len;
        }
        if (len < size - 1)
            buf[len++] = c;
    }
}

int bt_read(void *data, size_t len, uint32_t timeout_ms)
{
    uint8_t *p = data;
    TickType_t start = xTaskGetTickCount();
    TickType_t to = pdMS_TO_TICKS(timeout_ms);
    size_t got = 0;

    if (rx_push >= 0 && len > 0) {
        p[0] = (uint8_t)rx_push;
        rx_push = -1;
        got = 1;
    }
    while (got < len) {
        TickType_t elapsed = xTaskGetTickCount() - start;
        if (elapsed >= to)
            return -1;
        size_t n = xStreamBufferReceive(rx_sb, p + got, len - got, to - elapsed);
        if (n == 0)
            return -1;
        got += n;
    }
    return (int)got;
}

uint32_t bt_rx_overflow(void)
{
    return rx_overflow;
}

uint32_t bt_baud(void)
{
    return cur_baud;
}

const char *bt_jdy_status(void)
{
    return jdy_status;
}

/* 等发送缓冲排空后切换 USART1 波特率，丢弃已收到的字节 */
static void bt_set_baud(uint32_t baud)
{
    USART_InitTypeDef u;

    while (!xStreamBufferIsEmpty(tx_sb) || USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
        vTaskDelay(1);
    USART_Cmd(USART1, DISABLE);
    USART_StructInit(&u);
    u.USART_BaudRate = baud;
    USART_Init(USART1, &u);
    USART_Cmd(USART1, ENABLE);
    xStreamBufferReset(rx_sb);
    rx_push = -1;
    cur_baud = baud;
}

/* 发一条 AT 指令，等以 prefix 开头的回复行；reply 可为 NULL */
static bool at_cmd(const char *cmd, const char *prefix, char *reply, size_t size, uint32_t timeout_ms)
{
    char line[48];
    TickType_t end = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);

    xStreamBufferReset(rx_sb);
    rx_push = -1;
    bt_printf("%s\r\n", cmd);
    for (;;) {
        int32_t left = (int32_t)(end - xTaskGetTickCount());
        if (left <= 0 || bt_getline(line, sizeof line, (uint32_t)left * 1000u / configTICK_RATE_HZ) < 0)
            return false;
        if (!strncmp(line, prefix, strlen(prefix))) {
            if (reply)
                snprintf(reply, size, "%s", line);
            return true;
        }
    }
}

/*
 * JDY-31 只在没有蓝牙连接时响应 AT 指令（连接后串口是透传）。开机时模块刚上电、还没被连上，
 * 先按 BT_BAUD 查询；不通再按出厂 9600 查询，通了就发 AT+BAUDn 改成 BT_BAUD 并重启模块。
 * 设置保存在模块里，只需成功一次。两种速率都没回复（多半是电脑还连着），按 BT_BAUD 继续。
 */
const char *bt_jdy_setup(void)
{
    static const struct { uint32_t baud; char code; } tab[] = {
        {9600, '4'}, {19200, '5'}, {38400, '6'}, {57600, '7'}, {115200, '8'}, {128000, '9'}};
    char want = 0, cmd[16], reply[48];

    for (size_t i = 0; i < sizeof tab / sizeof tab[0]; i++)
        if (tab[i].baud == BT_BAUD)
            want = tab[i].code;
    configASSERT(want);
    snprintf(cmd, sizeof cmd, "AT+BAUD%c", want);

    bt_lock();
    vTaskDelay(pdMS_TO_TICKS(800)); /* 等模块启动 */
    bt_set_baud(BT_BAUD);
    if (at_cmd("AT+BAUD", "+BAUD=", reply, sizeof reply, 300)) {
        jdy_status = "ok";
    } else if (BT_BAUD != BT_BAUD_FACTORY && (bt_set_baud(BT_BAUD_FACTORY),
               at_cmd("AT+BAUD", "+BAUD=", reply, sizeof reply, 300))) {
        at_cmd(cmd, "+", NULL, 0, 500);
        at_cmd("AT+RESET", "+", NULL, 0, 500); /* 部分版本改完立即生效，收不到回复也无妨 */
        vTaskDelay(pdMS_TO_TICKS(1000));
        bt_set_baud(BT_BAUD);
        jdy_status = at_cmd("AT+BAUD", "+BAUD=", reply, sizeof reply, 500) ? "set" : "set_fail";
    } else {
        bt_set_baud(BT_BAUD);
        jdy_status = "noreply";
    }
    bt_unlock();
    return jdy_status;
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
