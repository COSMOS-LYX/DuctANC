cmake_minimum_required(VERSION 3.20)

if(NOT CMAKE_TOOLCHAIN_FILE)
  set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/cmake/arm-none-eabi.cmake)
endif()

project(hwtest C ASM)

set(CMAKE_C_STANDARD 11)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()
set(CMAKE_C_FLAGS_DEBUG "-Og -g3")
set(CMAKE_C_FLAGS_RELEASE "-O2 -g")

# 蓝牙模块波特率（JDY-31 出厂 9600，固件开机时用 AT+BAUDn 自动改成这里的值）
set(BT_BAUD 115200 CACHE STRING "USART1 baud rate for the Bluetooth module")
# 采样率，ADC 与 DAC 共用 TIM2 触发
set(FS_HZ 8000 CACHE STRING "Sample rate in Hz")

set(LIB ${CMAKE_CURRENT_SOURCE_DIR}/Libraries)
set(SPL ${LIB}/STM32F4xx_StdPeriph_Driver)
set(RTOS ${LIB}/FreeRTOS)

file(GLOB USER_SRC ${CMAKE_CURRENT_SOURCE_DIR}/User/*.c)
file(GLOB SPL_SRC ${SPL}/src/*.c)
set(RTOS_SRC
  ${RTOS}/tasks.c ${RTOS}/queue.c ${RTOS}/list.c ${RTOS}/timers.c
  ${RTOS}/stream_buffer.c ${RTOS}/event_groups.c
  ${RTOS}/portable/GCC/ARM_CM4F/port.c
  ${RTOS}/portable/MemMang/heap_4.c)

add_executable(${PROJECT_NAME}
  ${USER_SRC} ${SPL_SRC} ${RTOS_SRC}
  ${CMAKE_CURRENT_SOURCE_DIR}/startup/startup_stm32f40_41xxx.s)

target_include_directories(${PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/User)
# 厂商库头文件按系统头处理，屏蔽其自带的重定义告警
target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE
  ${LIB}/CMSIS/Include
  ${LIB}/CMSIS/Device
  ${SPL}/inc
  ${RTOS}/include
  ${RTOS}/portable/GCC/ARM_CM4F)

target_compile_definitions(${PROJECT_NAME} PRIVATE
  STM32F40_41xxx USE_STDPERIPH_DRIVER HSE_VALUE=8000000
  BT_BAUD=${BT_BAUD} FS_HZ=${FS_HZ})

set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/User/app.c
  ${CMAKE_CURRENT_SOURCE_DIR}/User/anc.c
  ${CMAKE_CURRENT_SOURCE_DIR}/User/audio.c
  ${CMAKE_CURRENT_SOURCE_DIR}/User/bsp.c
  ${CMAKE_CURRENT_SOURCE_DIR}/User/bt_uart.c
  ${CMAKE_CURRENT_SOURCE_DIR}/User/main.c
  ${CMAKE_CURRENT_SOURCE_DIR}/User/syscalls.c
  PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra")

target_link_options(${PROJECT_NAME} PRIVATE
  -T${CMAKE_CURRENT_SOURCE_DIR}/ld/STM32F407VETx_FLASH.ld
  -Wl,-Map=${PROJECT_NAME}.map -Wl,--print-memory-usage -Wl,--no-warn-rwx-segments)
target_link_libraries(${PROJECT_NAME} PRIVATE m)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.bin
  COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.hex
  COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}>)

# 通过 ST-Link (SWD) 烧录，需要 STM32CubeProgrammer 在 PATH 中
add_custom_target(flash
  COMMAND STM32_Programmer_CLI -c port=SWD -w $<TARGET_FILE:${PROJECT_NAME}> -v -rst
  DEPENDS ${PROJECT_NAME})
