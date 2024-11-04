################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Add inputs and outputs from these tool invocations to the build variables 
CMD_SRCS += \
../source/DSP2803x_Headers_nonBIOS.cmd \
../source/F28034.cmd 

ASM_SRCS += \
../source/DSP2803x_CSMPasswords.asm \
../source/DSP2803x_CodeStartBranch.asm \
../source/DSP2803x_DBGIER.asm \
../source/DSP2803x_DisInt.asm \
../source/DSP2803x_usDelay.asm 

C_SRCS += \
../source/DCDC50A_Isr.c \
../source/DSP2803x_Adc.c \
../source/DSP2803x_Comp.c \
../source/DSP2803x_CpuTimers.c \
../source/DSP2803x_DefaultIsr.c \
../source/DSP2803x_ECan.c \
../source/DSP2803x_EPwm.c \
../source/DSP2803x_GlobalVariableDefs.c \
../source/DSP2803x_Gpio.c \
../source/DSP2803x_I2C.c \
../source/DSP2803x_MemCopy.c \
../source/DSP2803x_PieCtrl.c \
../source/DSP2803x_PieVect.c \
../source/DSP2803x_Sci.c \
../source/DSP2803x_Spi.c \
../source/DSP2803x_SysCtrl.c \
../source/Main.c 

C_DEPS += \
./source/DCDC50A_Isr.d \
./source/DSP2803x_Adc.d \
./source/DSP2803x_Comp.d \
./source/DSP2803x_CpuTimers.d \
./source/DSP2803x_DefaultIsr.d \
./source/DSP2803x_ECan.d \
./source/DSP2803x_EPwm.d \
./source/DSP2803x_GlobalVariableDefs.d \
./source/DSP2803x_Gpio.d \
./source/DSP2803x_I2C.d \
./source/DSP2803x_MemCopy.d \
./source/DSP2803x_PieCtrl.d \
./source/DSP2803x_PieVect.d \
./source/DSP2803x_Sci.d \
./source/DSP2803x_Spi.d \
./source/DSP2803x_SysCtrl.d \
./source/Main.d 

OBJS += \
./source/DCDC50A_Isr.obj \
./source/DSP2803x_Adc.obj \
./source/DSP2803x_CSMPasswords.obj \
./source/DSP2803x_CodeStartBranch.obj \
./source/DSP2803x_Comp.obj \
./source/DSP2803x_CpuTimers.obj \
./source/DSP2803x_DBGIER.obj \
./source/DSP2803x_DefaultIsr.obj \
./source/DSP2803x_DisInt.obj \
./source/DSP2803x_ECan.obj \
./source/DSP2803x_EPwm.obj \
./source/DSP2803x_GlobalVariableDefs.obj \
./source/DSP2803x_Gpio.obj \
./source/DSP2803x_I2C.obj \
./source/DSP2803x_MemCopy.obj \
./source/DSP2803x_PieCtrl.obj \
./source/DSP2803x_PieVect.obj \
./source/DSP2803x_Sci.obj \
./source/DSP2803x_Spi.obj \
./source/DSP2803x_SysCtrl.obj \
./source/DSP2803x_usDelay.obj \
./source/Main.obj 

ASM_DEPS += \
./source/DSP2803x_CSMPasswords.d \
./source/DSP2803x_CodeStartBranch.d \
./source/DSP2803x_DBGIER.d \
./source/DSP2803x_DisInt.d \
./source/DSP2803x_usDelay.d 

OBJS__QUOTED += \
"source\DCDC50A_Isr.obj" \
"source\DSP2803x_Adc.obj" \
"source\DSP2803x_CSMPasswords.obj" \
"source\DSP2803x_CodeStartBranch.obj" \
"source\DSP2803x_Comp.obj" \
"source\DSP2803x_CpuTimers.obj" \
"source\DSP2803x_DBGIER.obj" \
"source\DSP2803x_DefaultIsr.obj" \
"source\DSP2803x_DisInt.obj" \
"source\DSP2803x_ECan.obj" \
"source\DSP2803x_EPwm.obj" \
"source\DSP2803x_GlobalVariableDefs.obj" \
"source\DSP2803x_Gpio.obj" \
"source\DSP2803x_I2C.obj" \
"source\DSP2803x_MemCopy.obj" \
"source\DSP2803x_PieCtrl.obj" \
"source\DSP2803x_PieVect.obj" \
"source\DSP2803x_Sci.obj" \
"source\DSP2803x_Spi.obj" \
"source\DSP2803x_SysCtrl.obj" \
"source\DSP2803x_usDelay.obj" \
"source\Main.obj" 

C_DEPS__QUOTED += \
"source\DCDC50A_Isr.d" \
"source\DSP2803x_Adc.d" \
"source\DSP2803x_Comp.d" \
"source\DSP2803x_CpuTimers.d" \
"source\DSP2803x_DefaultIsr.d" \
"source\DSP2803x_ECan.d" \
"source\DSP2803x_EPwm.d" \
"source\DSP2803x_GlobalVariableDefs.d" \
"source\DSP2803x_Gpio.d" \
"source\DSP2803x_I2C.d" \
"source\DSP2803x_MemCopy.d" \
"source\DSP2803x_PieCtrl.d" \
"source\DSP2803x_PieVect.d" \
"source\DSP2803x_Sci.d" \
"source\DSP2803x_Spi.d" \
"source\DSP2803x_SysCtrl.d" \
"source\Main.d" 

ASM_DEPS__QUOTED += \
"source\DSP2803x_CSMPasswords.d" \
"source\DSP2803x_CodeStartBranch.d" \
"source\DSP2803x_DBGIER.d" \
"source\DSP2803x_DisInt.d" \
"source\DSP2803x_usDelay.d" 

C_SRCS__QUOTED += \
"../source/DCDC50A_Isr.c" \
"../source/DSP2803x_Adc.c" \
"../source/DSP2803x_Comp.c" \
"../source/DSP2803x_CpuTimers.c" \
"../source/DSP2803x_DefaultIsr.c" \
"../source/DSP2803x_ECan.c" \
"../source/DSP2803x_EPwm.c" \
"../source/DSP2803x_GlobalVariableDefs.c" \
"../source/DSP2803x_Gpio.c" \
"../source/DSP2803x_I2C.c" \
"../source/DSP2803x_MemCopy.c" \
"../source/DSP2803x_PieCtrl.c" \
"../source/DSP2803x_PieVect.c" \
"../source/DSP2803x_Sci.c" \
"../source/DSP2803x_Spi.c" \
"../source/DSP2803x_SysCtrl.c" \
"../source/Main.c" 

ASM_SRCS__QUOTED += \
"../source/DSP2803x_CSMPasswords.asm" \
"../source/DSP2803x_CodeStartBranch.asm" \
"../source/DSP2803x_DBGIER.asm" \
"../source/DSP2803x_DisInt.asm" \
"../source/DSP2803x_usDelay.asm" 


