# PY32F003 template project

 * Linux dev environment
 * C++ support
 * RTT debug support (set RELEASE to 0 in makefile)
 * uart supported using IT RX/TX mode
 * jlink programmer supported

## Usage

 * link/copy the SDK CMSIS dir to lib/CMSIS
 * link/copy the PY32F0xx_LL_Driver to lib/drivers
 * link/copy the JLink to ../jlink and untar ../jlink/Samples/RTT/SEGGER_RTT_* to ./jlink/Samples/RTT/SEGGER_RTT
 * study Makefile & enjoy !
