project(board)
cmake_minimum_required(VERSION 3.16)
enable_language(ASM)

set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/STM32G0B1RCTx_FLASH.ld PARENT_SCOPE)

#############
# STM32CUBE #
#############

list(APPEND ${PROJECT_NAME}_SOURCE_FILES
    startup_stm32g0b1xx.s
    Core/Src/main.c
    Core/Src/app_freertos.c
    Core/Src/stm32g0xx_it.c
    Core/Src/stm32g0xx_hal_msp.c
    Core/Src/stm32g0xx_hal_timebase_tim.c
    Core/Src/system_stm32g0xx.c
)

list(APPEND ${PROJECT_NAME}_INCLUDE_DIRS Core/Inc)

#############
# STM32 HAL #
#############

set(STM32HAL_DIR Drivers/STM32G0xx_HAL_Driver CACHE PATH "STM32HAL directory")

list(APPEND ${PROJECT_NAME}_SOURCE_FILES
    ${STM32HAL_DIR}/Src/stm32g0xx_hal_gpio.c
    ${STM32HAL_DIR}/Src/stm32g0xx_hal_adc.c
    ${STM32HAL_DIR}/Src/stm32g0xx_hal_adc_ex.c
    ${STM32HAL_DIR}/Src/stm32g0xx_ll_adc.c
    ${STM32HAL_DIR}/Src/stm32g0xx_hal_rcc.c
    ${STM32HAL_DIR}/Src/stm32g0xx_hal_rcc_ex.c
    ${STM32HAL_DIR}/Src/stm32g0xx_ll_rcc.c
    ${STM32HAL_DIR}/Src/stm32g0xx_hal_flash.c
    ${STM32HAL_DIR}/Src/stm32g0xx_hal_flash_ex.c
    ${STM32HAL_DIR}/Src/stm32g0xx_hal_dma.c
    ${STM32HAL_DIR}/Src/stm32g0xx_hal_dma_ex.c
    ${STM32HAL_DIR}/Src/stm32g0xx_hal_pwr.c
    ${STM32HAL_DIR}/Src/stm32g0xx_hal_pwr_ex.c
    ${STM32HAL_DIR}/Src/stm32g0xx_hal_cortex.c
    ${STM32HAL_DIR}/Src/stm32g0xx_hal.c
    ${STM32HAL_DIR}/Src/stm32g0xx_hal_exti.c
    ${STM32HAL_DIR}/Src/stm32g0xx_hal_dac.c
    ${STM32HAL_DIR}/Src/stm32g0xx_hal_dac_ex.c
    ${STM32HAL_DIR}/Src/stm32g0xx_hal_i2c.c
    ${STM32HAL_DIR}/Src/stm32g0xx_hal_i2c_ex.c
    ${STM32HAL_DIR}/Src/stm32g0xx_hal_rtc.c
    ${STM32HAL_DIR}/Src/stm32g0xx_hal_rtc_ex.c
    ${STM32HAL_DIR}/Src/stm32g0xx_hal_spi.c
    ${STM32HAL_DIR}/Src/stm32g0xx_hal_spi_ex.c
    ${STM32HAL_DIR}/Src/stm32g0xx_hal_tim.c
    ${STM32HAL_DIR}/Src/stm32g0xx_hal_tim_ex.c
    ${STM32HAL_DIR}/Src/stm32g0xx_hal_uart.c
    ${STM32HAL_DIR}/Src/stm32g0xx_hal_uart_ex.c
)

list(APPEND ${PROJECT_NAME}_INCLUDE_DIRS
    ${STM32HAL_DIR}/Inc
    ${STM32HAL_DIR}/Inc/Legacy
)

#########
# CMSIS #
#########

set(CMSIS_DIR Drivers/CMSIS CACHE PATH "CMSIS device headers directory")

list(APPEND ${PROJECT_NAME}_INCLUDE_DIRS
    ${CMSIS_DIR}/Include
    ${CMSIS_DIR}/Device/ST/STM32G0xx/Include
)

############
# FreeRTOS #
############

set(FREERTOS_DIR Middlewares/Third_Party/FreeRTOS CACHE PATH "FreeRTOS source directory")

list(APPEND ${PROJECT_NAME}_SOURCE_FILES
    ${FREERTOS_DIR}/Source/croutine.c
    ${FREERTOS_DIR}/Source/event_groups.c
    ${FREERTOS_DIR}/Source/list.c
    ${FREERTOS_DIR}/Source/queue.c
    ${FREERTOS_DIR}/Source/stream_buffer.c
    ${FREERTOS_DIR}/Source/tasks.c
    ${FREERTOS_DIR}/Source/timers.c
    ${FREERTOS_DIR}/Source/CMSIS_RTOS_V2/cmsis_os2.c
    ${FREERTOS_DIR}/Source/portable/MemMang/heap_4.c
    ${FREERTOS_DIR}/Source/portable/GCC/ARM_CM0/port.c
)

list(APPEND ${PROJECT_NAME}_INCLUDE_DIRS
    ${FREERTOS_DIR}/Source/include
    ${FREERTOS_DIR}/Source/CMSIS_RTOS_V2
    ${FREERTOS_DIR}/Source/portable/GCC/ARM_CM0
)

###########
# Library #
###########

add_library(${PROJECT_NAME} OBJECT EXCLUDE_FROM_ALL
    ${${PROJECT_NAME}_SOURCE_FILES}
)

target_compile_definitions(${PROJECT_NAME} PUBLIC
    USE_HAL_DRIVER
    STM32G0B1xx
    CMSIS_device_header=<stm32g0xx.h>
)

target_include_directories(${PROJECT_NAME} PUBLIC
    ${${PROJECT_NAME}_INCLUDE_DIRS}
)
