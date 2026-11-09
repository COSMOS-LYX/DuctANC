/* 异常处理。SVC / PendSV / SysTick 由 FreeRTOS 接管（见 FreeRTOSConfig.h） */
#include "stm32f4xx.h"

void fault_trap(void);

void NMI_Handler(void) { fault_trap(); }
void HardFault_Handler(void) { fault_trap(); }
void MemManage_Handler(void) { fault_trap(); }
void BusFault_Handler(void) { fault_trap(); }
void UsageFault_Handler(void) { fault_trap(); }
void DebugMon_Handler(void) {}
