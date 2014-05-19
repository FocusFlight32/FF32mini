###############################################################################
# "THE BEER-WARE LICENSE" (Revision 42):
# <msmith@FreeBSD.ORG> wrote this file. As long as you retain this notice you
# can do whatever you want with this stuff. If we meet some day, and you think
# this stuff is worth it, you can buy me a beer in return
###############################################################################
#
# Makefile for building the firmware.
#
# Invoke this with 'make help' to see the list of supported targets.
# 

###############################################################################
# Things that the user might override on the commandline
#

# The target to build, must be one of NAZEPRO
TARGET		?= NAZEPRO

# Compile-time options
OPTIONS		?=

# Debugger optons, must be empty or GDB
DEBUG ?=

# Serial port/Device for flashing
SERIAL_DEVICE	?= /dev/ttyUSB0

###############################################################################
# Things that need to be maintained as the source changes
#

FORKNAME			 = ff32mini

VALID_TARGETS	 = NAZEPRO

# Working directories
ROOT		 = $(dir $(lastword $(MAKEFILE_LIST)))
SRC_DIR		 = $(ROOT)/src
OBJECT_DIR	 = $(ROOT)/obj
BIN_DIR		 = $(ROOT)/obj
CMSIS_DIR	 = $(ROOT)/lib/CMSIS
INCLUDE_DIRS = $(SRC_DIR)

# Search path for sources
VPATH		:= $(SRC_DIR):$(SRC_DIR)/startup


ifeq ($(TARGET),$(filter $(TARGET),NAZEPRO))

STDPERIPH_DIR	 = $(ROOT)/lib/STM32F30x_StdPeriph_Driver
USBPERIPH_DIR	 = $(ROOT)/lib/STM32_USB-FS-Device_Driver

VPATH		:= $(VPATH):$(CMSIS_DIR)/CM1/CoreSupport:$(CMSIS_DIR)/CM1/DeviceSupport/ST/STM32F30x
CMSIS_SRC	 = $(notdir $(wildcard $(CMSIS_DIR)/CM1/CoreSupport/*.c \
			   $(CMSIS_DIR)/CM1/DeviceSupport/ST/STM32F30x/*.c))

INCLUDE_DIRS := $(INCLUDE_DIRS) \
		   $(STDPERIPH_DIR)/inc \
		   $(USBPERIPH_DIR)/inc \
		   $(CMSIS_DIR)/CM1/CoreSupport \
		   $(CMSIS_DIR)/CM1/DeviceSupport/ST/STM32F30x \

LD_SCRIPT	 = $(ROOT)/stm32_flash_f303.ld

ARCH_FLAGS	 = -mthumb -mcpu=cortex-m4
DEVICE_FLAGS = -DSTM32F303xC
TARGET_FLAGS = -D$(TARGET)

# keep me, but not too long
else

STDPERIPH_DIR	 = $(ROOT)/lib/STM32F10x_StdPeriph_Driver

# Search path and source files for the CMSIS sources
VPATH		:= $(VPATH):$(CMSIS_DIR)/CM3/CoreSupport:$(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x
CMSIS_SRC	 = $(notdir $(wildcard $(CMSIS_DIR)/CM3/CoreSupport/*.c \
			   $(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x/*.c))

INCLUDE_DIRS := $(INCLUDE_DIRS) \
		   $(STDPERIPH_DIR)/inc \
		   $(CMSIS_DIR)/CM3/CoreSupport \
		   $(CMSIS_DIR)/CM3/DeviceSupport/ST/STM32F10x \

LD_SCRIPT	 = $(ROOT)/stm32_flash_f103.ld

ARCH_FLAGS	 = -mthumb -mcpu=cortex-m3
TARGET_FLAGS = -D$(TARGET)  
DEVICE_FLAGS = -DSTM32F10X_MD 

endif

COMMON_SRC	 = startup/startup_stm32f30x.s \
		 arm_mat_mult_f32.c \
		 arm_mat_init_f32.c \
		 batMon.c \
		 coordinateTransforms.c \
		 evrStringTable.c \
		 gps.c \
		 mavlinkStrings.c \
		 utilities.c \
		 computeAxisCommands.c \
		 eeprom.c \
		 firstOrderFilter.c \
		 main.c \
		 mixer.c \
		 stm32_it.c \
 		 vertCompFilter.c \
		 evr.c \
		 flightCommand.c \
		 MargAHRS.c \
		 pid.c \
		 system_stm32f30x.c \
		 watchdogs.c \
		 calibration/accelCalibrationMPU.c \
		 calibration/escCalibration.c \
		 calibration/magCalibration.c \
		 calibration/mpu6000Calibration.c \
		 cli/cli.c \
		 cli/cliEEPROM.c \
		 cli/cliMixer.c \
		 cli/cliReceiver.c \
		 cli/cliSensor.c \
		 cli/cliTelemetry.c \
		 drv/drv_adc.c \
		 drv/drv_gpio.c \
		 drv/drv_ppmRx.c \
		 drv/drv_pwmServo.c \
		 drv/drv_spi.c \
		 drv/drv_agl.c \
		 drv/drv_crc.c \
		 drv/drv_pwmEsc.c \
		 drv/drv_uart1.c \
		 drv/drv_uart2.c \
		 drv/drv_usb.c \
		 drv/drv_spektrum.c \
		 drv/drv_system.c \
		 drv/drv_timingFunctions.c \
		 sensors/hmc5983.c \
		 sensors/mpu6000.c \
		 sensors/ms5611_SPI.c \
		 vcp/hw_config.c \
		 vcp/usb_desc.c \
		 vcp/usb_endp.c \
		 vcp/usb_istr.c \
		 vcp/usb_prop.c \
		 vcp/usb_pwr.c \
		 wmm/geoMagElements.c \
		 wmm/GeomagnetismLibrary.c \
		 $(CMSIS_SRC) \
		 $(STDPERIPH_SRC) \
		 $(USBPERIPH_SRC)

NAZEPRO_SRC	 = $(COMMON_SRC)



# In some cases, %.s regarded as intermediate file, which is actually not.
# This will prevent accidental deletion of startup code.
.PRECIOUS: %.s

# Search path and source files for the ST stdperiph library
VPATH		:= $(VPATH):$(STDPERIPH_DIR)/src:$(USBPERIPH_DIR)/src
USBPERIPH_SRC	 = $(notdir $(wildcard $(USBPERIPH_DIR)/src/*.c))
STDPERIPH_SRC	 = $(notdir $(wildcard $(STDPERIPH_DIR)/src/*.c))

###############################################################################
# Things that might need changing to use different tools
#

# Tool names
CC		 = arm-none-eabi-gcc
OBJCOPY		 = arm-none-eabi-objcopy

#
# Tool options.
#

ARCH_FLAGS	 = -mthumb -mcpu=cortex-m3
BASE_CFLAGS	 = $(ARCH_FLAGS) \
		   $(addprefix -D,$(OPTIONS)) \
		   $(addprefix -I,$(INCLUDE_DIRS)) \
		   -Wall \
		   -ffunction-sections \
		   -fdata-sections \
		   $(DEVICE_FLAGS) \
		   -DUSE_STDPERIPH_DRIVER \
		   $(TARGET_FLAGS) \
		   -D'__FORKNAME__="$(FORKNAME)"' \
		   -DARM_MATH_CM4 \
		   -I$(SRC_DIR)/mavlink/common 

ASFLAGS		 = $(ARCH_FLAGS) \
		   -x assembler-with-cpp \
		   $(addprefix -I,$(INCLUDE_DIRS))

# XXX Map/crossref output?
LDFLAGS		 = -lm -lc -lnosys \
		   $(ARCH_FLAGS) \
		   -static \
		   -Wl,-gc-sections,-Map,$(TARGET_MAP) \
		   -T$(LD_SCRIPT)

###############################################################################
# No user-serviceable parts below
###############################################################################

#
# Things we will build
#
ifeq ($(filter $(TARGET),$(VALID_TARGETS)),)
$(error Target '$(TARGET)' is not valid, must be one of $(VALID_TARGETS))
endif

ifeq ($(DEBUG),GDB)
CFLAGS = $(BASE_CFLAGS) \
	-ggdb \
	-O0
else
CFLAGS = $(BASE_CFLAGS) \
	-Os
endif


TARGET_HEX	 = $(BIN_DIR)/$(FORKNAME)_$(TARGET).hex
TARGET_ELF	 = $(BIN_DIR)/$(FORKNAME)_$(TARGET).elf
TARGET_OBJS	 = $(addsuffix .o,$(addprefix $(OBJECT_DIR)/$(TARGET)/,$(basename $($(TARGET)_SRC))))
TARGET_MAP   = $(OBJECT_DIR)/$(FORKNAME)_$(TARGET).map

# List of buildable ELF files and their object dependencies.
# It would be nice to compute these lists, but that seems to be just beyond make.

$(TARGET_HEX): $(TARGET_ELF)
	$(OBJCOPY) -O ihex --set-start 0x8000000 $< $@

$(TARGET_ELF):  $(TARGET_OBJS)
	$(CC) -o $@ $^ $(LDFLAGS)

# Compile
$(OBJECT_DIR)/$(TARGET)/%.o: %.c
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(CFLAGS) $<

# Assemble
$(OBJECT_DIR)/$(TARGET)/%.o: %.s
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(ASFLAGS) $< 
$(OBJECT_DIR)/$(TARGET)/%.o): %.S
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(ASFLAGS) $< 

clean:
	rm -f $(TARGET_HEX) $(TARGET_ELF) $(TARGET_OBJS) $(TARGET_MAP)
	rm -rf $(OBJECT_DIR)/$(TARGET)

flash_$(TARGET): $(TARGET_HEX)
	stty -F $(SERIAL_DEVICE) raw speed 115200 -crtscts cs8 -parenb -cstopb -ixon
	echo -n 'R' >$(SERIAL_DEVICE)
	stm32flash -w $(TARGET_HEX) -v -g 0x0 -b 115200 $(SERIAL_DEVICE)

flash: flash_$(TARGET)


unbrick_$(TARGET): $(TARGET_HEX)
	stty -F $(SERIAL_DEVICE) raw speed 115200 -crtscts cs8 -parenb -cstopb -ixon
	stm32flash -w $(TARGET_HEX) -v -g 0x0 -b 115200 $(SERIAL_DEVICE)

unbrick: unbrick_$(TARGET)

help:
	@echo ""
	@echo "Makefile for the $(FORKNAME) firmware"
	@echo ""
	@echo "Usage:"
	@echo "        make [TARGET=<target>] [OPTIONS=\"<options>\"]"
	@echo ""
	@echo "Valid TARGET values are: $(VALID_TARGETS)"
	@echo ""
