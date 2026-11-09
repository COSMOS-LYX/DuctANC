#include "bsp.h"

void bsp_init(void)
{
    GPIO_InitTypeDef g;

    SystemCoreClockUpdate();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOE, ENABLE);

    GPIO_SetBits(LED_PORT, LED_D2 | LED_D3);
    g.GPIO_Pin = LED_D2 | LED_D3;
    g.GPIO_Mode = GPIO_Mode_OUT;
    g.GPIO_OType = GPIO_OType_PP;
    g.GPIO_Speed = GPIO_Speed_2MHz;
    g.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(LED_PORT, &g);

    g.GPIO_Mode = GPIO_Mode_IN;
    g.GPIO_PuPd = GPIO_PuPd_UP;
    g.GPIO_Pin = KEY0_PIN | KEY1_PIN;
    GPIO_Init(KEY0_PORT, &g);
    g.GPIO_PuPd = GPIO_PuPd_DOWN;
    g.GPIO_Pin = KEYUP_PIN;
    GPIO_Init(KEYUP_PORT, &g);
}

bool bsp_clock_ok(void)
{
    return SystemCoreClock == 168000000u && RCC_GetSYSCLKSource() == 0x08;
}

void led_set(uint16_t pin, bool on)
{
    if (on)
        GPIO_ResetBits(LED_PORT, pin);
    else
        GPIO_SetBits(LED_PORT, pin);
}

void led_toggle(uint16_t pin)
{
    GPIO_ToggleBits(LED_PORT, pin);
}

key_id_t key_poll(void)
{
    static uint8_t last, cnt;
    uint8_t now = 0;

    if (!GPIO_ReadInputDataBit(KEY0_PORT, KEY0_PIN)) now = KEY_K0;
    else if (!GPIO_ReadInputDataBit(KEY1_PORT, KEY1_PIN)) now = KEY_K1;
    else if (GPIO_ReadInputDataBit(KEYUP_PORT, KEYUP_PIN)) now = KEY_UP;

    if (now != last) {
        last = now;
        cnt = 0;
        return KEY_NONE;
    }
    if (cnt < 3 && ++cnt == 3)
        return (key_id_t)now;
    return KEY_NONE;
}
