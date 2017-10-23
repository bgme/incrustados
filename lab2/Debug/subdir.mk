################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../LED.cpp \
../Scheduler.cpp \
../Task.cpp \
../main.cpp 

C_SRCS += \
../startup_msp432p401r_ccs.c \
../system_msp432p401r.c 

OBJS += \
./LED.o \
./Scheduler.o \
./Task.o \
./main.o \
./startup_msp432p401r_ccs.o \
./system_msp432p401r.o 

C_DEPS += \
./startup_msp432p401r_ccs.d \
./system_msp432p401r.d 

CPP_DEPS += \
./LED.d \
./Scheduler.d \
./Task.d \
./main.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

%.o: ../%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


