# Tiva Makefile
#######################################
# user configuration:
#######################################
# TARGET: name of the output file
TARGET = project
# MCU: part number to build for
MCU = TM4C123GH6PM
# SOURCES: list of input source sources
SOURCES = main.c startup_gcc.c can0.c uartstdio.c
ifeq ($(I2C_RTC),1)
SOURCES += i2c_rtc.c
endif
# OUTDIR: directory to use for output
OUTDIR = build

ROOT = .

# LD_SCRIPT: linker script
LD_SCRIPT = $(MCU).lds

# define flags
CFLAGS = -g -mthumb -march=armv7e-m+fp -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard
CFLAGS += -O0 -ffunction-sections -fdata-sections -MD -std=c99 -Wall
CFLAGS += -pedantic -DPART_$(MCU) -c -I $(ROOT)
CFLAGS += -DTARGET_IS_TM4C123_RB1 -D gcc 
LIBS = -lc -ldriver
LDFLAGS = -mthumb -march=armv7e-m+fp -mfloat-abi=hard -Wl,-Map=build/$(TARGET).map  -Wl,-T $(LD_SCRIPT) --entry ResetISR -Wl,--gc-sections -Wl,--start-group $(LIBS) -Wl,--end-group 

DRIVERLIB = $(ROOT)/driverlib
LIBDIR = $(ROOT)/lib
DRIVER_CFILES = $(wildcard $(DRIVERLIB)/*.c)
DRIVER_OBJECTS = $(DRIVER_CFILES:%.c=%.o)

#######################################
# end of user configuration
#######################################
#
#######################################
# binaries
#######################################
PREFIX = arm-none-eabi
CC =  ${PREFIX}-gcc
AR =  ${PREFIX}-ar
RM      = rm -f
MKDIR	= mkdir -p
#######################################

# list of object files, placed in the build directory regardless of source path
OBJECTS = $(addprefix $(OUTDIR)/,$(notdir $(SOURCES:.c=.o)))

# default: build bin
all: $(OUTDIR)/$(TARGET).elf

$(LIBDIR)/libdriver.a: $(DRIVER_OBJECTS) | $(LIBDIR)
	${AR} -cr ${@} ${^}
	$(RM) $(DRIVERLIB)/*.o $(DRIVERLIB)/*.d

$(OUTDIR)/%.o: src/%.c | $(OUTDIR)
	$(CC) -o $@ $^ $(CFLAGS)

$(OUTDIR)/$(TARGET).elf: $(OBJECTS) $(LIBDIR)/libdriver.a
	$(CC) -o $@ $^ $(LDFLAGS) 

# create the output directory
$(OUTDIR):
	$(MKDIR) $(OUTDIR)

$(LIBDIR):
	$(MKDIR) $(LIBDIR)
clean:
	$(RM) -r $(OUTDIR) $(LIBDIR)	

.PHONY: all clean
