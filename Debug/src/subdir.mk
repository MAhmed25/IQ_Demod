################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
ASM_SRCS += \
../src/IQ_Audio_Demod.asm 

SRC_OBJS += \
./src/IQ_Audio_Demod.doj 

ASM_DEPS += \
./src/IQ_Audio_Demod.d 


# Each subdirectory must supply rules for building sources it contributes
src/IQ_Audio_Demod.doj: ../src/IQ_Audio_Demod.asm
	@echo 'Building file: $<'
	@echo 'Invoking: CrossCore Blackfin Assembler'
	easmblkfn -file-attr ProjectName="IQ_Audio_Demod" -proc ADSP-BF706 -si-revision 1.1 -g -D_DEBUG -DCORE0 @includes-59d032f4c97d40ffd2dd73f1b7dba989.txt -gnu-style-dependencies -MM -Mo "src/IQ_Audio_Demod.d" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


