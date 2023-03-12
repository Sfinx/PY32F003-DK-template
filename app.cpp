
#include "app.h"
#include <string.h>

void main() {
 board_init();
 status.start = 1;
 char b[64];
 sprintf(b, "Firmware v%d.%d started", APP_FW_VERSION >> 4, APP_FW_VERSION & 0xF);
 uart_send(b);
 while(1) {
  debug("%sapp...", strtime());
  ms_delay(900);
  wdt_reset();
 }
}

// called from systick interrupt
void _1_sec_tick() {
}

// called from systick interrupt
void _100_ms_tick() {
 static char ledv;
 static short t;
 if (status.blink && (++t >= status.blink_delay)) {
   t = 0;
   set_led(ledv);
   ledv = ledv ? 0 : 1;
 }
}

status_t status;
