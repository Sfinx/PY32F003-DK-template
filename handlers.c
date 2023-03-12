
#include "app.h"

void NMI_Handler() { panic(NMI_OOPS); }
void HardFault_Handler() { panic(HARDFAULT_OOPS); }
void MemManage_Handler() { panic(MEMMANAGE_OOPS); }
void BusFault_Handler() { panic(BUSFAULT_OOPS); }
void UsageFault_Handler() { panic(USAGEFAULT_OOPS); }
void SVC_Handler() { panic(SVC_OOPS); }
void DebugMon_Handler() { panic(DEBUGMONITOR_OOPS); }
void PendSV_Handler() { panic(PEND_SVC_OOPS); }

// happens each 1ms
void SysTick_Handler() {
 if (status.panic)
   return;
 _1_ms_tick();
 if (!(status.milliseconds++ % 100))
   _100_ms_tick();
 if (status.milliseconds > 999) {
   status.milliseconds = 0;
   _1_sec_tick();
   status.seconds++;
 }
}

extern void uart_irq();
void USART1_IRQHandler() { uart_irq(); }
