
DFUUTIL	    = /usr/local/bin/dfu-util
JLINKDIR    = ../jlink
JLINKSPEED  = 5000
# PY32F003F16P6
JLINKDEVICE = PY32F003X6
JLINKCMD    = $(JLINKDIR)/JLinkExe -if SWD -speed $(JLINKSPEED) -device $(JLINKDEVICE) -AutoConnect 1 -NoGui 1
JUNLOCK	    = $(JLINKDIR)/JLinkSTM32
MCU  = cortex-m0

TRGT = arm-none-eabi-
CC   = $(TRGT)gcc
CXX   = $(TRGT)g++
CP   = $(TRGT)objcopy
AS   = $(TRGT)gcc -x assembler-with-cpp
HEX  = $(CP) -O ihex 
BIN  = $(CP) -O binary

RUN_FROM_FLASH = 1
RELEASE = 0

# List all default C defines here, like -D_DEBUG=1
# set the DDEFS CPU & HAL usage
DDEFS = -DPY32F003x6

# List all default ASM defines here, like -D_DEBUG=1
DADEFS = 

# List all default directories to look for include files here
DINCDIR = .

# List the default directory to look for the libraries here
DLIBDIR =

# List all default libraries here
DLIBS = 

# Define project name and Ram/Flash mode here
PROJECT        = fw

# List all user C define here, like -D_DEBUG=1
UDEFS =

# Define ASM defines here
UADEFS = 

LIBSDIR      	= ./lib
DRIVERS      	= $(LIBSDIR)/drivers
DRIVERSINC   	= $(LIBSDIR)/drivers/Inc
DRIVERSRC    	= $(LIBSDIR)/drivers/Src
CMSISDIR     	= $(LIBSDIR)/CMSIS
CMSISDIRINC  	= $(CMSISDIR)/Include
CMSISDEVDIR     = $(CMSISDIR)/Device/PY32F0xx
CMSISDEVDIRINC  = $(CMSISDEVDIR)/Include
BOARD        	= $(LIBSDIR)/board
BOARDINC     	= $(BOARD)/inc
STARTUP      	= $(BOARD)/startup
LINKER       	= $(BOARD)

# source list

CPPSRC  = app.cpp

SRC 	= lib.c
SRC 	+= handlers.c
SRC 	+= printf.c

SRC += $(STARTUP)/system_py32f0xx.c

SRC += $(DRIVERSRC)/py32f0xx_ll_gpio.c
SRC += $(DRIVERSRC)/py32f0xx_ll_rcc.c
SRC += $(DRIVERSRC)/py32f0xx_ll_usart.c
SRC += $(DRIVERSRC)/py32f0xx_ll_utils.c

DINCDIR += $(CMSISDIR)/Core/Include

ifeq ($(RELEASE), 0)
DDEFS   += -DDEBUG_VERSION
DINCDIR += rtt/RTT rtt/Config
SRC     += rtt/Syscalls/SEGGER_RTT_Syscalls_GCC.c
SRC     += rtt/RTT/SEGGER_RTT.c
SRC     += rtt/RTT/SEGGER_RTT_printf.c
else
DDEFS   += -DRELEASE_VERSION 
endif

DDEFS   += -DUSE_FULL_LL_DRIVER

# List ASM source files here
ASRC = $(CMSISDEVDIR)/Source/gcc/startup_py32f003.s

# List all user directories here
UINCDIR  = $(CMSISDEVDIRINC)
UINCDIR += $(CMSISDIRINC)
UINCDIR += $(DRIVERSINC)
UINCDIR += $(BOARDINC)
# List the user directory to look for the libraries here
ULIBDIR =

# List all user libraries here
ULIBS =
BUFSIZE = 256
RTT_PRINTF_BUFSIZE = 512

# Define optimisation level here
# -mfpu=vfp -msoft-float -fno-builtin
OPT = -DPREFER_SIZE_OVER_SPEED -D__OPTIMIZE_SIZE__ -D__thumb2__ -D__BUFSIZ__=$(BUFSIZE) -Os -DSEGGER_RTT_PRINTF_BUFFER_SIZE=$(RTT_PRINTF_BUFSIZE)

ifeq ($(RUN_FROM_FLASH), 0)
LDSCRIPT = $(LINKER)/ram.ld
else
LDSCRIPT = $(LINKER)/flash.ld
endif

INCDIR  = $(patsubst %,-I%,$(DINCDIR) $(UINCDIR))
LIBDIR  = $(patsubst %,-L%,$(DLIBDIR) $(ULIBDIR))

ifeq ($(RUN_FROM_FLASH), 0)
DEFS    = $(DDEFS) $(UDEFS) -DRUN_FROM_FLASH=0 -DVECT_TAB_SRAM
else
DEFS    = $(DDEFS) $(UDEFS) -DRUN_FROM_FLASH=1
endif

ADEFS   = $(DADEFS) $(UADEFS)
OBJS    = $(ASRC:.s=.o) $(SRC:.c=.o) $(CPPSRC:.cpp=.o)
LIBS    = $(DLIBS) $(ULIBS)
MCFLAGS = -mcpu=$(MCU) -mthumb -ffreestanding
# Dangerous while developing !!! set only for production release
MCFLAGS += -flto


#LSTASFLAGS = -g3 -Wa,-ahlms=$(<:.s=.lst)
ASFLAGS = $(MCFLAGS) -D__thumb2__ $(ADEFS) $(LSTASFLAGS)
# -Wextra -Wunused-variable -Wunused-but-set-variable
CFLAGS = $(MCFLAGS) -Wall -Wmaybe-uninitialized -Wuninitialized $(OPT) -Werror \
	-fomit-frame-pointer -falign-functions=16 -ffunction-sections -fdata-sections \
	-fverbose-asm $(DEFS)
CFLAGS += -MD -MP -MF .dep/$(@F).d
# Generate dependency information
#LSTCFLAGS = -g3 -Wa,-ahlms=$(<:.c=.lst)
#LSTCXXFLAGS = -g3 -Wa,-ahlms=$(<:.cpp=.lst)
CXXFLAGS = $(CFLAGS) -std=gnu++17 $(LSTCXXFLAGS)
CPFLAGS = $(CFLAGS) -std=gnu2x $(LSTCFLAGS)
LDFLAGS = $(MCFLAGS) -fwhole-program -T$(LDSCRIPT) -Wl,--gc-sections,-Map=$(PROJECT).map,--cref,--no-warn-mismatch $(LIBDIR)

#
# makefile rules
#
all:	$(PROJECT)

$(PROJECT): $(OBJS) $(PROJECT).elf
	$(BIN) $(PROJECT).elf $(PROJECT).bin
	@$(TRGT)size $(PROJECT).elf
	@md5sum $(PROJECT).bin

%.o : %.c
	$(CC) -c $(CPFLAGS) $(INCDIR) $< -o $@

%.o : %.cpp
	$(CXX) -c $(CXXFLAGS) $(INCDIR) $< -o $@

%.o : %.s
	$(AS) -c $(ASFLAGS) $< -o $@

%.elf: $(OBJS)
	$(CXX) $(OBJS) $(LDFLAGS) $(LIBS) -o $@

%.hex: %.elf
	$(HEX) $< $@

%.bin: %.elf
	$(BIN) $< $@

clean:
	@rm -f $(OBJS)
	@rm -f *.bin *.elf *.map *.hex JLink.log tty *.bak *.lst *.crypted .gdb_history
	@rm -fR .dep
	@echo Clean done

todo:
	@cat TODO
	@echo

jlink:
	@$(JLINKCMD) -CommandFile halt.cmd

jlinkprg:
	@$(JLINKCMD) -CommandFile prg.cmd

view:
	@$(JLINKDIR)/JLinkRTTClient

-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)
