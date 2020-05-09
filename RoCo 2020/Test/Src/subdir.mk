################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Src/IOBus.cpp \
../Src/MessageBus.cpp \
../Src/NetworkBus.cpp \
../Src/NetworkClientIO.cpp \
../Src/NetworkServerIO.cpp 

OBJS += \
./Src/IOBus.o \
./Src/MessageBus.o \
./Src/NetworkBus.o \
./Src/NetworkClientIO.o \
./Src/NetworkServerIO.o 

CPP_DEPS += \
./Src/IOBus.d \
./Src/MessageBus.d \
./Src/NetworkBus.d \
./Src/NetworkClientIO.d \
./Src/NetworkServerIO.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -std=c++17 -DBUILD_FOR_TESTING -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


