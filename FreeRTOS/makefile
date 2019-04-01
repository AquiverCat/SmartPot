################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/TI/ccs_v6/ccsv6/tools/compiler/ti-cgt-arm_5.2.2

ORDERED_OBJS += \
"./button_if.obj" \
"./gpio_if.obj" \
"./main.obj" \
"./network_common.obj" \
"./network_if.obj" \
"./pinmux.obj" \
"./startup_ccs.obj" \
"./temp.obj" \
"./timer_if.obj" \
"./uart_if.obj" \
"../cc3200v1p32.cmd" \
$(GEN_CMDS__FLAG) \
-l"libc.a" \
-l"C:/TI/CC3200SDK_1.2.0/cc3200-sdk/example/mqtt_client/ccs/../../../oslib/ccs/free_rtos/free_rtos.a" \
-l"C:/TI/CC3200SDK_1.2.0/cc3200-sdk/example/mqtt_client/ccs/../../../simplelink/ccs/OS/simplelink.a" \
-l"C:/TI/CC3200SDK_1.2.0/cc3200-sdk/example/mqtt_client/ccs/../../../netapps/mqtt/ccs/client/mqtt.a" \
-l"C:/TI/CC3200SDK_1.2.0/cc3200-sdk/example/mqtt_client/ccs/../../../driverlib/ccs/Release/driverlib.a" \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
SmartPot.out \

EXE_OUTPUTS__QUOTED += \
"SmartPot.out" \

BIN_OUTPUTS += \
SmartPot.bin \

BIN_OUTPUTS__QUOTED += \
"SmartPot.bin" \


# All Target
all: SmartPot.out

# Tool invocations
SmartPot.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@echo 'Building target: $@'
	@echo 'Invoking: ARM Linker'
	"C:/TI/ccs_v6/ccsv6/tools/compiler/ti-cgt-arm_5.2.2/bin/armcl" -mv7M4 --code_state=16 --float_support=fpalib --abi=eabi -me -g --gcc --define=ccs --define=USE_FREERTOS --define=SL_PLATFORM_MULTI_THREADED --define=cc3200 --display_error_number --diag_warning=225 --diag_wrap=off -z -m"SmartPot.map" --heap_size=0x8000 --stack_size=0x5000 -i"C:/TI/ccs_v6/ccsv6/tools/compiler/ti-cgt-arm_5.2.2/lib" -i"C:/TI/ccs_v6/ccsv6/tools/compiler/ti-cgt-arm_5.2.2/include" --reread_libs --warn_sections --display_error_number --diag_wrap=off --xml_link_info="SmartPot_linkInfo.xml" --rom_model -o "SmartPot.out" $(ORDERED_OBJS)
	@echo 'Finished building target: $@'
	@echo ' '
	@$(MAKE) --no-print-directory post-build

SmartPot.bin: $(EXE_OUTPUTS)
	@echo 'Invoking: ARM Hex Utility'
	"C:/TI/ccs_v6/ccsv6/tools/compiler/ti-cgt-arm_5.2.2/bin/armhex"  -o "SmartPot.bin" $(EXE_OUTPUTS__QUOTED)
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXE_OUTPUTS__QUOTED)$(BIN_OUTPUTS__QUOTED)
	-$(RM) "button_if.pp" "gpio_if.pp" "main.pp" "network_common.pp" "network_if.pp" "pinmux.pp" "startup_ccs.pp" "temp.pp" "timer_if.pp" "uart_if.pp" 
	-$(RM) "button_if.obj" "gpio_if.obj" "main.obj" "network_common.obj" "network_if.obj" "pinmux.obj" "startup_ccs.obj" "temp.obj" "timer_if.obj" "uart_if.obj" 
	-@echo 'Finished clean'
	-@echo ' '

post-build:
	-"C:/TI/ccs_v6/ccsv6/utils/tiobj2bin/tiobj2bin" "SmartPot.out" "SmartPot.bin" "C:/TI/ccs_v6/ccsv6/tools/compiler/ti-cgt-arm_5.2.2/bin/armofd" "C:/TI/ccs_v6/ccsv6/tools/compiler/ti-cgt-arm_5.2.2/bin/armhex" "C:/TI/ccs_v6/ccsv6/utils/tiobj2bin/mkhex4bin"
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets

