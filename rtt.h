#pragma once

#ifdef RELEASE_VERSION
#define SEGGER_RTT_printf(x...)
#define debug(x...)
#else
#include <SEGGER_RTT.h>
#define debug(x...)	{ SEGGER_RTT_printf(0, x); SEGGER_RTT_printf(0, "\n"); }
#endif
