
#include <app.h>
#include <string.h>

#define OB_RDP_LEVEL_0  0x000000AAU
#define OB_RDP_LEVEL_1  0x000000BBU
#define OB_RDP_LEVEL_2  0x000000CCU

uint8_t rdp_enabled() {
 uint32_t rdplvl = READ_BIT(FLASH->OPTR, FLASH_OPTR_RDP);
 if ((rdplvl != OB_RDP_LEVEL_0) && (rdplvl != OB_RDP_LEVEL_2))
   rdplvl = OB_RDP_LEVEL_1;
 return (rdplvl != OB_RDP_LEVEL_0);
}

void set_led(u8 on) {
 if (on)
  LL_GPIO_ResetOutputPin(LED_GPIO_PORT, LED_PIN);
 else
  LL_GPIO_SetOutputPin(LED_GPIO_PORT, LED_PIN);
}

void blink(u16 period) {
 status.blink = 1;
 status.blink_delay = period;
}

#define CYCLES_PER_LOOP         3

void wait_cycles(uint32_t n) {
 uint32_t l = n/CYCLES_PER_LOOP;
 while(l--)
  asm volatile("nop;");
}

void hard_delay(uint32_t d) { wait_cycles(d); }

void panic_delay() {
 // hard_delay(0x4FFFFF);
 ms_delay(2000);
}

void panic(int i) {
 if (status.panic)
   return;
 status.panic = 1;
 debug("%s%sOops [%d] !%s", strtime(), RTT_CTRL_BG_BRIGHT_RED, i, RTT_CTRL_RESET);
 u8 on = 1;
 while(1) {
   set_led(on);
   panic_delay();
   on = on ? 0 : 1;
 }
}

void clock_init() {
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
  LL_RCC_HSI_Enable();
  while(LL_RCC_HSI_IsReady() != 1);

  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSISYS);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSISYS);

  LL_RCC_LSI_Enable();
  while(LL_RCC_LSI_IsReady() != 1);
  
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_Init1msTick(HSI_VALUE);
  LL_SetSystemCoreClock(HSI_VALUE);
  NVIC_SetPriority(SysTick_IRQn, 0);
  LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);
  LL_SYSTICK_EnableIT();
}

void led_init() {
 LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
 LL_GPIO_InitTypeDef gpio = { 0 };
 gpio.Pin = LED_PIN;
 gpio.Mode = LL_GPIO_MODE_OUTPUT;
 gpio.Speed = LL_GPIO_SPEED_FREQ_LOW;
 gpio.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
 gpio.Pull = LL_GPIO_PULL_UP;
 LL_GPIO_Init(LED_GPIO_PORT, &gpio);
 set_led(0); // turn led off
}

void wdt_init() {
 LL_IWDG_Enable(IWDG);
 LL_IWDG_EnableWriteAccess(IWDG);
 LL_IWDG_SetPrescaler(IWDG, LL_IWDG_PRESCALER_8); // ~1 sec
 LL_IWDG_SetReloadCounter(IWDG, 4078);
 while (LL_IWDG_IsReady(IWDG) != 1);
 LL_IWDG_ReloadCounter(IWDG);
}

u8 uart_rx_buf[UART_RX_BUF_SIZE], uart_rx_idx = 0;
u64 last_uart_rx = 0;

void clear_uart_rx_buf() {
 memset(uart_rx_buf, 0, sizeof(uart_rx_buf));
 uart_rx_idx = 0;
 last_uart_rx = 0;
}

void uart_init() {
 clear_uart_rx_buf();
 // clocks
 LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
 LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_USART1);
 // rx
 LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_2, LL_GPIO_MODE_ALTERNATE);
 LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_2, LL_GPIO_SPEED_FREQ_HIGH);
 LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_2, LL_GPIO_PULL_UP);
 LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_2, LL_GPIO_AF_1);
 // tx
 LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_3, LL_GPIO_MODE_ALTERNATE);
 LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_3, LL_GPIO_SPEED_FREQ_HIGH);
 LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_3, LL_GPIO_PULL_UP);
 LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_3, LL_GPIO_AF_1);
 // uart
 LL_USART_SetBaudRate(USART1, HSI_VALUE, LL_USART_OVERSAMPLING_16, UART_BAUDRATE);
 LL_USART_SetDataWidth(USART1, LL_USART_DATAWIDTH_8B);
 LL_USART_SetStopBitsLength(USART1, LL_USART_STOPBITS_1);
 LL_USART_SetParity(USART1, LL_USART_PARITY_NONE);
 LL_USART_SetHWFlowCtrl(USART1, LL_USART_HWCONTROL_NONE);
 LL_USART_SetTransferDirection(USART1, LL_USART_DIRECTION_TX_RX);
 LL_USART_Enable(USART1);
 LL_USART_ClearFlag_TC(USART1);
 LL_USART_Enable(USART1);
 LL_USART_EnableIT_ERROR(USART1);
 LL_USART_EnableIT_RXNE(USART1);
 NVIC_SetPriority(USART1_IRQn, 1);
 NVIC_EnableIRQ(USART1_IRQn);
}

void clear_uart_error(uint32_t isr) {
  WRITE_REG(USART1->SR, USART_SR_FE | USART_SR_NE | USART_SR_ORE | USART_SR_PE);
}

void uart_rx() {
__IO uint32_t isr = LL_USART_ReadReg(USART1, SR);
 u8 v = LL_USART_ReceiveData8(USART1);
 char err[16] = { 0 };
 if (isr & USART_SR_FE)
  strcat(err, "FE");
 if (isr & USART_SR_NE)
  strcat(err, "NE");
 if (isr & USART_SR_ORE)
  strcat(err, "ORE");
 if (err[0]) {
   debug("%s*** uart_rx hw error %s", strtime(), err);
   clear_uart_error(isr);
   clear_uart_rx_buf();
   return;
 } else {
    if (!last_uart_rx)
      last_uart_rx = status.seconds;
    if ((status.seconds - last_uart_rx) > 2) // rx timeout, begin all again
     clear_uart_rx_buf();
    last_uart_rx = status.seconds;
    if (status.cmd) { // previous command in progress
     err[0] = 2;
     goto err;
    }
    // uart_rx_buf[uart_rx_idx] = v;
    debug("%s*** uart_rx <== 0x%x, idx %d", strtime(), v, uart_rx_idx);
    // uart_rx_idx++;
    return;
 }
err:
 debug("%s*** uart_rx err %d", strtime(), err[0]);
 clear_uart_rx_buf();
}

void uart_error() {
__IO uint32_t isr = LL_USART_ReadReg(USART1, SR);
 char err[32] = { 0 };
 if (isr & USART_SR_FE)
  strcat(err, "FE");
 if (isr & USART_SR_NE)
  strcat(err, "NE");
 if (isr & USART_SR_ORE)
  strcat(err, "ORE");
 if (err[0])
  debug("%suart_error %s [0x%x]", strtime(), err, isr);
 clear_uart_error(isr);
}

volatile const char *uart_tx_buf;
volatile s16 txpos = -1, txlen;

void uart_irq() {
 if (LL_USART_IsActiveFlag_RXNE(USART1) && LL_USART_IsEnabledIT_RXNE(USART1))
  return uart_rx();
 if (LL_USART_IsEnabledIT_TXE(USART1) && LL_USART_IsActiveFlag_TXE(USART1)) {
  if (++txpos == (txlen - 1)) {
    LL_USART_DisableIT_TXE(USART1);
    LL_USART_EnableIT_TC(USART1);
  }
  LL_USART_TransmitData8(USART1, uart_tx_buf[txpos]);
  return;
 }
 if (LL_USART_IsEnabledIT_TC(USART1) && LL_USART_IsActiveFlag_TC(USART1)) {
  txpos = -1;
  LL_USART_ClearFlag_TC(USART1);
  return LL_USART_DisableIT_TC(USART1);
 }
 if (LL_USART_IsEnabledIT_ERROR(USART1))
  uart_error();
}

void board_init() {
 memset(&status, 0, sizeof(status));
 debug("%sFirmware v%d.%d started", strtime(), APP_FW_VERSION >> 4, APP_FW_VERSION & 0xF);
 clock_init();
 led_init();
 uart_init();
 blink(12);
#ifdef RELEASE_VERSION
 u8 flash_protected = rdp_enabled();
 debug("%sFlash is %sprotected !", strtime(), flash_protected ? "" : "not ");
 if (!flash_protected)
  panic(FLASH_PROTECTION_OOPS);
#endif
 wdt_init();
 blink(3); // normal operation
}

void uart_send(const char *b, uint sz) {
 if (txpos >= 0)
  return;
 txlen = sz ? sz : strlen(b);
 uart_tx_buf = b;
 txpos = 0;
 LL_USART_ClearFlag_TC(USART1);
 LL_USART_TransmitData8(USART1, b[txpos]);
 LL_USART_EnableIT_TXE(USART1);
 while(txpos >= 0)
  wdt_reset();
}

const char *strtime() {
 static char b[32];
 uchar days = (status.seconds / 86400);
 uchar hours = (status.seconds / 3600) % 24;
 uchar minutes = (status.seconds / 60) % 60;
 uchar seconds = status.seconds % 60;
 sprintf(b, "[%04d:%02d:%02d:%02d.%03d] ", days, hours, minutes, seconds, status.milliseconds);
 return b;
}

void wdt_reset() { LL_IWDG_ReloadCounter(IWDG); }
void _1_ms_tick() { }
