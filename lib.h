
#pragma once

/*
 HAL_StatusTypeDef:
     HAL_OK       = 0x00U,
     HAL_ERROR    = 0x01U,
     HAL_BUSY     = 0x02U,
     HAL_TIMEOUT  = 0x03U
*/

#include "py32f0xx_ll_rcc.h"
#include "py32f0xx_ll_bus.h"
#include "py32f0xx_ll_system.h"
#include "py32f0xx_ll_cortex.h"
#include "py32f0xx_ll_utils.h"
#include "py32f0xx_ll_iwdg.h"
#include "py32f0xx_ll_usart.h"
#include "py32f0xx_ll_gpio.h"

#include "rtt.h"

typedef unsigned int uint;
typedef unsigned char uchar;
typedef uint8_t u8;
typedef int16_t s16;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint16_t ushort;
typedef uint64_t u64;

extern void panic(int);

extern void board_init();
extern void hard_delay(u32 d);
extern void _1_ms_tick();
extern void _1_sec_tick();
extern void _100_ms_tick();
extern void wdt_reset();
extern void set_led(u8);
extern void blink(u16);
extern int sprintf(char * buf, const char * fmt, ...);
const char *strtime();
extern void uart_send(const char *b, uint sz);
extern void clear_uart_rx_buf();

#define UART_BAUDRATE           115200
#define UART_RX_CMD_SIZE        3
#define MAX_CMD_DATA_SIZE       64
#define UART_RX_BUF_SIZE        (UART_RX_CMD_SIZE + 1 + MAX_CMD_DATA_SIZE) // // *cXN<data>

extern u8 uart_rx_buf[UART_RX_BUF_SIZE];

#define ms_delay(x)		LL_mDelay((x)-1)

// must be no more than 0xFF types
typedef enum oops_t {
    NMI_OOPS,                   // 0
    HARDFAULT_OOPS,             // 1
    MEMMANAGE_OOPS,             // 2
    BUSFAULT_OOPS,              // 3
    USAGEFAULT_OOPS,            // 4
    DEBUGMONITOR_OOPS,          // 5
    SVC_OOPS,                   // 6
    PEND_SVC_OOPS,              // 7
    WWDG_OOPS,                  // 8
    PVD_OOPS,                   // 9
    FLASH_OOPS,                 // 10
    RCC_OOPS,                   // 11
    SYSTICK_OOPS,               // 12
    SYSTEMCLOCK_OOPS,           // 13
    WDT_OOPS,                   // 14
    RTC_OOPS,                   // 15
    UART_OOPS,                  // 16
    FLASH_PROTECTION_OOPS,      // 17
    ADC_INIT_OOPS,              // 18
    TEMP_SENSOR_OOPS,           // 19
    GPIO_OOPS                   // 20
} oops_t;

#define LED_PIN                LL_GPIO_PIN_5
#define LED_GPIO_PORT          GPIOB
