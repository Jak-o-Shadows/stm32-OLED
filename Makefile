#this makefile is helped in part by https://github.com/libopencm3/libopencm3-examples/blob/master/examples/Makefile.rules

#My includes/own library location
ROOT =
SUBHIGH =
SUBMID =
#Use a separate file to keep track of own includes
INCLUDE_PATHS = -I .
HL_LIBS_C =
HL_LIBS_CPP =
include Makefile.config

#LibopenCM3 location
libopenCM3Path = ./libopencm3
SRCLIBDIR = $(libopenCM3Path)
# and has gotten increasingly opinionated
OPENCM3_DIR = $(libopenCM3Path)

INCLUDE_DIR = $(libopenCM3Path)/include
LIB_DIR = $(libopenCM3Path)/lib
SCRIPT_DIR = $(libopenCM3)/scripts

######Device specific things
DEVICE = stm32f103c8t6
ARCH_FLAGS =


#######  LD Script #################
# libopencm3 makefile that does the ldscript
#uses libopencm3 to make a .ld file for the specified device
LDFLAGS		?=
LDFLAGS		+= --static -nostartfiles
LDFLAGS		+= -L$(LIB_DIR)
LDFLAGS		+= -T$(LDSCRIPT)
LDFLAGS		+= -Wl,-Map=$(*).map
LDFLAGS		+= -Wl,--gc-sections
LDFLAGS     += --specs=nosys.specs

LIBNAME = opencm3_stm32f1
#LIBNAME = opencm3_stm32f4

# FreeRTOS

freertosPath = ./freertos
freertosKernelPath = $(freertosPath)/FreeRTOS/FreeRTOS-Kernel
# Generic
INCLUDE_PATHS += -I $(freertosKernelPath)/include
HL_LIBS_C += $(freertosKernelPath)/tasks.c
HL_LIBS_C += $(freertosKernelPath)/list.c
HL_LIBS_C += $(freertosKernelPath)/queue.c
HL_LIBS_C += $(freertosKernelPath)/timers.c
HL_LIBS_C += $(freertosKernelPath)/event_groups.c
HL_LIBS_C += $(freertosKernelPath)/stream_buffer.c
HL_LIBS_C += $(freertosKernelPath)/croutine.c
# Platform Specific
INCLUDE_PATHS += -I $(freertosKernelPath)/portable/GCC/ARM_CM3
HL_LIBS_C += $(freertosKernelPath)/portable/GCC/ARM_CM3/port.c
# Malloc Behaviour
HL_LIBS_C += $(freertosKernelPath)/portable/MemMang/heap_4.c







#what is the main file
BINARY = main
OBJS += $(HL_LIBS_C:.c=.o) $(HL_LIBS_CPP:.cpp=.o)



toolchainPath ?=

PREFIX		?= $(toolchainPath)arm-none-eabi

CC		:= $(PREFIX)-gcc
LD		:= $(PREFIX)-g++
CXX		:= $(PREFIX)-g++
AR		:= $(PREFIX)-ar
AS		:= $(PREFIX)-as
OBJCOPY		:= $(PREFIX)-objcopy
OBJDUMP		:= $(PREFIX)-objdump
GDB		:= $(PREFIX)-gdb



CFLAGS =
#with thanks to libopencm3 makefile
CFLAGS += -g
CFLAGS += -Wextra -Wshadow -Wimplicit-function-declaration
CFLAGS += -Wredundant-decls -Wmissing-prototypes -Wstrict-prototypes
CFLAGS += -fno-common -ffunction-sections -fdata-sections

CFLAGS += -MD
CFLAGS += -Wall -Wundef
CFLAGS += -mcpu=cortex-m3 -mthumb


CFLAGS += -T $(LDSCRIPT)

#because libopencm3 is slightly broken
CFLAGS += -DSTM32F1

CPPFLAGS =
CPPFLAGS += -g
CPPFLAGS += -fno-common -ffunction-sections -fdata-sections
CPPFLAGS += -MD
CPPFLAGS += -Wall -Wundef
CPPFLAGS += -mcpu=cortex-m3
CPPFLAGS += -T $(LDSCRIPT)
#because libopencm3 is slightly broken
CPPFLAGS += -DSTM32F1



#Includes
INCLUDE_PATHS += -I $(INCLUDE_DIR)
INCLUDE_PATHS += $(SUBHIGH)







LDLIBS		+= -l$(LIBNAME)
LDLIBS		+= -Wl,--start-group -lc -lgcc -lnosys -Wl,--end-group
#include <math.h>
LDLIBS      += -lm

include tests.mk


.SUFFIXES: .elf .bin .hex .srec .list .map .images
.SECONDEXPANSION:
.SECONDARY:

all: elf bin

include $(libopenCM3Path)/ld/Makefile.linker

elf: $(BINARY).elf
bin: $(BINARY).bin
hex: $(BINARY).hex
srec: $(BINARY).srec
list: $(BINARY).list

images: $(BINARY).images
flash: $(BINARY).flash

%.images: %.bin %.hex %.srec %.list %.map
	@#printf "*** $* images generated ***\n"

%.bin: %.elf
	@#printf "  OBJCOPY $(*).bin\n"
	$(Q)$(OBJCOPY) -Obinary $(*).elf $(*).bin

%.hex: %.elf
	@#printf "  OBJCOPY $(*).hex\n"
	$(Q)$(OBJCOPY) -Oihex $(*).elf $(*).hex

%.srec: %.elf
	@#printf "  OBJCOPY $(*).srec\n"
	$(OBJCOPY) -Osrec $(*).elf $(*).srec

%.list: %.elf
	@#printf "  OBJDUMP $(*).list\n"
	$(OBJDUMP) -S $(*).elf > $(*).list

#$.elf %.map: $(OBJS) $(LDSCRIPT)
#	@#printf "  LD      $(*).elf\n"
#	$(LD) $(LDFLAGS) $(ARCH_FLAGS) $(OBJS) $(LDLIBS) -o $(*).elf

$(BINARY).elf: $(OBJS)
	$(LD) $(LDFLAGS) $(ARCH_FLAGS) $(OBJS) $(LDLIBS) -o $(BINARY).elf

%.o: %.c
	@#printf "  CC      $(*).c\n"
	$(Q) $(CC) $(CFLAGS) $(INCLUDE_PATHS) $(ARCH_FLAGS) -o $(*).o -c $(*).c

%.o: %.cpp
	@#printf "  CXX      $(*).cpp\n"
	$(Q) $(CXX) $(CPPFLAGS) $(INCLUDE_PATHS) $(ARCH_FLAGS) -o $(*).o -c $(*).cpp

clean:
	@#printf "  CLEAN\n"
	$(Q)$(RM) $(OBJS)
	$(Q)$(RM) *.o *.d *.elf *.bin *.hex *.srec *.list *.map





openocd:;
	openocd -s "E:\Tools\arm-none-eabi\8 2019-q3-update\bin\scripts" -f "openocd.cfg" -c "init" -c "halt" -c "reset halt"

debug: main.elf main.bin;
	$(GDB) --eval-command="target ext:3335"  main.elf

telnet:;
	telnet localhost 4446

testecho:;
	$(Q) echo $(HL_LIBS_C)
	$(Q) echo $(HL_LIBS_CPP)
	$(Q) echo $(OBJS)
#	echo $()



tests: test.exe
	./test.exe

%.to: %.c
	$(Q) $(TEST_CC) $(TEST_CFLAGS) -o $(*).to -c $(*).c

%.to: %.cpp
	$(Q) $(TEST_CXX) $(TEST_CXXFLAGS) -o $(*).to -c $(*).cpp
	$(Q) $(TEST_CXX) -c -o $@ $< $(TEST_CXXFLAGS)

test.exe: ${TEST_OBJS}
	$(TEST_LD) -o $@ $^ $(TEST_CXXFLAGS) $(TEST_LDFLAGS)

testtestecho:
	echo $(TEST_CXXFLAGS)


