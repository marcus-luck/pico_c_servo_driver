cmake_minimum_required(VERSION 3.13)

# Include build functions from Pico SDK
include(pico_sdk_import.cmake)
# include(FreeRTOS_Kernel_import.cmake)
message(STATUS "FreeRTOS_Kernel_import.cmake: ${FREERTOS_KERNEL_PATH}")
# include(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

# Set name of project (as PROJECT_NAME) and C/C   standards
project(servo_pwm C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
# include(pico_sdk_import.cmake)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# rest of your project

# Tell CMake where to find the executable source file
add_executable(${PROJECT_NAME} 
    src/servo_pwm.c
    src/uart.c
)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(${PROJECT_NAME})

# Link to pico_stdlib (gpio, time, etc. functions)
target_link_libraries(${PROJECT_NAME} 
    pico_stdlib
    hardware_pwm
)

add_library(FreeRTOS STATIC 
    FreeRTOS-Kernel/croutine.c
    FreeRTOS-Kernel/event_groups.c
    FreeRTOS-Kernel/list.c
    FreeRTOS-Kernel/queue.c
    FreeRTOS-Kernel/stream_buffer.c
    FreeRTOS-Kernel/tasks.c
    FreeRTOS-Kernel/timers.c 
    FreeRTOS-Kernel/portable/GCC/ARM_CM0/port.c 
    FreeRTOS-Kernel/portable/MemMang/heap_4.c 
    src/FreeRTOS_Support_Files/IdleMemory.c
)

target_include_directories(FreeRTOS PUBLIC 
    FreeRTOS-Kernel/include 
    include/ 
    FreeRTOS-Kernel/portable/GCC/ARM_CM0 
)

target_link_libraries(${PROJECT_NAME} 
    FreeRTOS
    # FreeRTOS-Kernel
    # FreeRTOS-Kernel-Heap4
) # any 3rd party here?

# Enable usb output, disable uart output
pico_enable_stdio_usb(${PROJECT_NAME} 0)
pico_enable_stdio_uart(${PROJECT_NAME} 1)
