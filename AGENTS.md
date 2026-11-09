# AGENTS.md

管道 ANC 毕业设计的讨论与开发目录。文档用中文。

## 硬件
- STM32F407VET6 黑色最小系统板，8 MHz HSE，原理图 `资料/数据手册/开发板原理图_stm32f407v.pdf`
- 蓝牙 JDY-31（SPP）接 USART1 PA9/PA10，默认 9600；麦 MAX4466 → PC0（参考）/PC1（误差）；DAC PA4 → PAM8403

## 固件 firmware/hwtest（StdPeriph V1.9.0 + FreeRTOS V10.3.1）
- 工具链：STM32CubeCLT（`D:\Tools\Stm32CubeCLT\STM32CubeCLT_1.22.0`，已在 PATH）
- 编译：`cmake --preset debug && cmake --build --preset debug`（在 `firmware/hwtest` 下）
- 烧录：`cmake --build --preset debug --target flash`（ST-Link SWD）
- 编译要求零告警；厂商库按 SYSTEM include 处理
- 库来源：StdPeriph 取自 `D:\Work\stm32f407\stsw-stm32065_v1-9-0`，FreeRTOS 取自 `~/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.3`
- `User/system_stm32f4xx.c` 已改 `PLL_M = 8`

## 上位机 host/
- `python -m pytest -q`（在 `host` 下）：用模拟固件离线测试协议和自检流程
- 协议改动时同步修改 `User/app.c`、`host/hwtest.py` 和 `host/test_hwtest.py` 里的 FakeFirmware
