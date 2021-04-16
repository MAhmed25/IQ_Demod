################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
ASM_SRCS += \
../src/sinegen.asm 

SRC_OBJS += \
./src/sinegen.doj 

ASM_DEPS += \
./src/sinegen.d 


# Each subdirectory must supply rules for building sources it contributes
src/sinegen.doj: ../src/sinegen.asm
	@echo 'Building file: $<'
	@echo 'Invoking: CrossCore Blackfin Assembler'
	easmblkfn.exe -file-attr ProjectName="sinegen" -proc ADSP-BF706 -si-revision 1.0 -DCORE0 -DNDEBUG -i"C:\Users\Patrick\OneDrive\Signal Wizard Blackfin\Assembly test projects\sinegen\system" -gnu-style-dependencies -MM -Mo "src/sinegen.d" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


