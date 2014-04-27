# System configuration - UNCOMMENT AS DESIRED

PROJECT=FF32lite

CC = arm-none-eabi-gcc
OPT = -O2
#OPT = -DDEBUG -std=c99
OBJCOPY = arm-none-eabi-objcopy

RM = rm -rf

# Define output directory
OBJECT_DIR = obj
BIN_DIR = $(OBJECT_DIR)
LINK_SCRIPT=stm32_flash.ld

# Assembler, Compiler and Linker flags and linker script settings
LINKER_FLAGS=-lm -mthumb -mcpu=cortex-m3 -Wl,--gc-sections -T$(LINK_SCRIPT) -static -Wl,-cref "-Wl,-Map=$(BIN_DIR)/$(PROJECT).map" -Wl,--defsym=malloc_getpagesize_P=0x1000
ASSEMBLER_FLAGS=-c -g -mthumb -mcpu=cortex-m3 $(OPT) -x assembler-with-cpp -Isrc -ILibraries/STM32F10x_StdPeriph_Driver/inc -ILibraries/CMSIS/Include -ILibraries/CMSIS/Device/ST/STM32F10x
COMPILER_FLAGS=-c -g -mthumb -mcpu=cortex-m3   $(OPT) -Wall -ffunction-sections -fdata-sections -mthumb -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER -D"ARM_MATH_CM3" -Isrc  -Isrc/drv -Isrc/calibration -Isrc/cli -Isrc/mavlink/common -Isrc/sensors -ILibraries/STM32F10x_StdPeriph_Driver/inc  -ILibraries/CMSIS/Device/ST/STM32F10x/Include -ILibraries/CMSIS/Include

#LINKER_FLAGS=-lm -mthumb -mcpu=cortex-m3 -Wl,--gc-sections -T$(LINK_SCRIPT) -static  -Wl,-cref "-Wl,-Map=$(BIN_DIR)/$(PROJECT).map" -Wl,--defsym=malloc_getpagesize_P=0x1000
#LINK_SCRIPT="stm32_flash.ld"
#ASSEMBLER_FLAGS=-c -g -O0 -mcpu=cortex-m3 -mthumb  -x assembler-with-cpp  -Isrc -ILibraries\STM32F10x_StdPeriph_Driver\inc -ILibraries\CMSIS\Device\ST\STM32F10x\Include -ILibraries\CMSIS\Include
#COMPILER_FLAGS=-c -g -mcpu=cortex-m3 -O0 -Wall -ffunction-sections -fdata-sections -mthumb -D"STM32F10X_MD" -D"USE_STDPERIPH_DRIVER"   -Isrc -ILibraries\STM32F10x_StdPeriph_Driver\inc -ILibraries\CMSIS\Device\ST\STM32F10x\Include -ILibraries\CMSIS\CM3/CoreSupport 

# Define sources and objects
#
SRC := 	$(wildcard Librarties/*/*/*/*/*/*.c) \
	$(wildcard Libraries/*/*/*/*/*.c) \
	$(wildcard Libraries/*/*/*/*.c) \
	$(wildcard Libraries/*/*/*.c) \
	$(wildcard Libraries/*/*.c) \
	$(wildcard src/*/*/*/c.c) \
	$(wildcard src/*/*/*.c) \
	$(wildcard src/*/*.c) \
	$(wildcard src/*.c)

SRCSASM := $(wildcard src/*/startup_stm32f10x_md.S)

OBJS := $(SRC:%.c=$(OBJECT_DIR)/%.o) $(SRCSASM:%.S=$(OBJECT_DIR)/%.o)
OBJS := $(OBJS:%.S=$(OBJECT_DIR)/%.o)

all: buildelf
	$(OBJCOPY) -O ihex "$(BIN_DIR)/$(PROJECT).elf" "$(BIN_DIR)/$(PROJECT).hex"

buildelf: $(OBJS) 
	$(CC) -o "$(BIN_DIR)/$(PROJECT).elf" $(OBJS) $(LINKER_FLAGS)

clean:
	$(RM) $(OBJS) "$(BIN_DIR)/*.*"
#	$(RM) $(OBJS) "$(BIN_DIR)/$(PROJECT).elf" "$(BIN_DIR)/$(PROJECT).map" "$(BIN_DIR)/src/*.*" "$(BIN_DIR)/lib/*.*"

$(OBJECT_DIR)/main.o: main.c
	@mkdir -p $(dir $@) 
	$(CC) $(COMPILER_FLAGS) main.c -o $(OBJECT_DIR)/main.o 

$(OBJECT_DIR)/%.o: %.c
	@mkdir -p $(dir $@) 
	$(CC) $(COMPILER_FLAGS) $< -o $@

$(OBJECT_DIR)/%.o: %.s
	@mkdir -p $(dir $@)
	$(CC) $(ASSEMBLER_FLAGS) $< -o $@
	
$(OBJECT_DIR)/%.o: %.S
	@mkdir -p $(dir $@) 
	$(CC) $(ASSEMBLER_FLAGS) $< -o $@
