################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/ti/ccs1281/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS

GEN_OPTS__FLAG := 
GEN_CMDS__FLAG := 

ORDERED_OBJS += \
"../source/F28034.cmd" \
"./source/DCDC50A_Isr.obj" \
"./source/DSP2803x_Adc.obj" \
"./source/DSP2803x_CSMPasswords.obj" \
"./source/DSP2803x_CodeStartBranch.obj" \
"./source/DSP2803x_Comp.obj" \
"./source/DSP2803x_CpuTimers.obj" \
"./source/DSP2803x_DBGIER.obj" \
"./source/DSP2803x_DefaultIsr.obj" \
"./source/DSP2803x_DisInt.obj" \
"./source/DSP2803x_ECan.obj" \
"./source/DSP2803x_EPwm.obj" \
"./source/DSP2803x_GlobalVariableDefs.obj" \
"./source/DSP2803x_Gpio.obj" \
"./source/DSP2803x_I2C.obj" \
"./source/DSP2803x_MemCopy.obj" \
"./source/DSP2803x_PieCtrl.obj" \
"./source/DSP2803x_PieVect.obj" \
"./source/DSP2803x_Sci.obj" \
"./source/DSP2803x_Spi.obj" \
"./source/DSP2803x_SysCtrl.obj" \
"./source/DSP2803x_usDelay.obj" \
"./source/Main.obj" \
"../source/DSP2803x_Headers_nonBIOS.cmd" \
$(GEN_CMDS__FLAG) \
-llibc.a \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include source/subdir_vars.mk
-include source/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(SV7A_DEPS)),)
-include $(SV7A_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
24100-1V0.1-ZV001.out 

EXE_OUTPUTS__QUOTED += \
"24100-1V0.1-ZV001.out" 


# All Target
all: $(OBJS) $(CMD_SRCS) $(LIB_SRCS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "24100-1V0.1-ZV001.out"

# Tool invocations
24100-1V0.1-ZV001.out: $(OBJS) $(CMD_SRCS) $(LIB_SRCS) $(GEN_CMDS)
	@echo 'Building target: "$@"'
	@echo 'Invoking: C2000 Linker'
	"C:/ti/ccs1281/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --advice:performance=all --define=_DEBUG --define=LARGE_MODEL -g --diag_warning=225 --diag_wrap=off --display_error_number --abi=coffabi -z -m"24100-1V0.1-ZV001.map" --stack_size=0X380 --warn_sections -i"C:/ti/ccs1281/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/lib" -i"E:/Work/WuYang/WYBX3C-24100/Software/24100-1V0.1-ZV001" -i"C:/ti/ccs1281/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/include" --reread_libs --diag_wrap=off --display_error_number --xml_link_info="24100-1V0.1-ZV001_linkInfo.xml" --rom_model -o "24100-1V0.1-ZV001.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXE_OUTPUTS__QUOTED)
	-$(RM) "source\DCDC50A_Isr.obj" "source\DSP2803x_Adc.obj" "source\DSP2803x_CSMPasswords.obj" "source\DSP2803x_CodeStartBranch.obj" "source\DSP2803x_Comp.obj" "source\DSP2803x_CpuTimers.obj" "source\DSP2803x_DBGIER.obj" "source\DSP2803x_DefaultIsr.obj" "source\DSP2803x_DisInt.obj" "source\DSP2803x_ECan.obj" "source\DSP2803x_EPwm.obj" "source\DSP2803x_GlobalVariableDefs.obj" "source\DSP2803x_Gpio.obj" "source\DSP2803x_I2C.obj" "source\DSP2803x_MemCopy.obj" "source\DSP2803x_PieCtrl.obj" "source\DSP2803x_PieVect.obj" "source\DSP2803x_Sci.obj" "source\DSP2803x_Spi.obj" "source\DSP2803x_SysCtrl.obj" "source\DSP2803x_usDelay.obj" "source\Main.obj" 
	-$(RM) "source\DCDC50A_Isr.d" "source\DSP2803x_Adc.d" "source\DSP2803x_Comp.d" "source\DSP2803x_CpuTimers.d" "source\DSP2803x_DefaultIsr.d" "source\DSP2803x_ECan.d" "source\DSP2803x_EPwm.d" "source\DSP2803x_GlobalVariableDefs.d" "source\DSP2803x_Gpio.d" "source\DSP2803x_I2C.d" "source\DSP2803x_MemCopy.d" "source\DSP2803x_PieCtrl.d" "source\DSP2803x_PieVect.d" "source\DSP2803x_Sci.d" "source\DSP2803x_Spi.d" "source\DSP2803x_SysCtrl.d" "source\Main.d" 
	-$(RM) "source\DSP2803x_CSMPasswords.d" "source\DSP2803x_CodeStartBranch.d" "source\DSP2803x_DBGIER.d" "source\DSP2803x_DisInt.d" "source\DSP2803x_usDelay.d" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

