#ifndef BSP_H
#define BSP_H

#include <stdbool.h>
#include "board.h"

typedef enum { KEY_NONE = 0, KEY_K0, KEY_K1, KEY_UP } key_id_t;

void bsp_init(void);
bool bsp_clock_ok(void);
void led_set(uint16_t pin, bool on);
void led_toggle(uint16_t pin);
/* 每 10 ms 调用一次，返回新按下的键（已消抖） */
key_id_t key_poll(void);

#endif
